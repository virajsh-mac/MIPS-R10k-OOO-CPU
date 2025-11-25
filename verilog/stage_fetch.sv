`include "sys_defs.svh"
`include "ISA.svh"

module stage_fetch (
    input logic                       clock,
    input logic                       reset,

    // icache_subsystem
    output I_ADDR_PACKET [1:0]        read_addrs,
    input CACHE_DATA     [1:0]        cache_data,

    // branch predictor
    output BP_PREDICT_REQUEST         bp_request,
    input  BP_PREDICT_RESPONSE        bp_response,

    // retire when mispredict
    input I_ADDR_PACKET               correct_branch_target,

    // instruction buffer
    input logic          [`IB_IDX_BITS:0]   ib_free_slots,
    output FETCH_PACKET  [3:0]                fetch_packet
);
    // Internal state
    ADDR PC, PC_next;
    
    // Derived signals
    INST [3:0] insts;
    logic [3:0] cache_valid_bits;
    logic [3:0] align_valid_bits;
    logic [3:0] final_valid_bits;
    
    logic       stall;
    ADDR        PC_aligned;
    
    // Mispredict handling
    logic       mispredict;
    ADDR        mispredict_target;
    
    // Cache ready check - wait for all cache data to be valid
    logic       icache_ready;
    
    // Control flow detection
    logic       found_branch;
    logic [1:0] first_branch_idx;
    logic       found_jump;
    logic [1:0] first_jump_idx;
    logic       found_control_flow;
    logic [1:0] first_control_flow_idx;
    logic       is_first_jump;
    
    ADDR        branch_pc;
    ADDR        jal_target;
    logic       pred_taken;
    logic [2:0] valid_count;
    logic [$clog2(`IB_PUSH_WIDTH+1)-1:0] num_pushes;

    // Helper: conditional branch
    function automatic logic is_conditional_branch(INST instr);
        return instr.r.opcode == `RV32_BRANCH;
    endfunction

    // Helper: unconditional jump (JAL/JALR)
    function automatic logic is_unconditional_jump(INST instr);
        return (instr.r.opcode == `RV32_JAL_OP) || (instr.r.opcode == `RV32_JALR_OP);
    endfunction

    // Helper: compute JAL target
    function automatic ADDR compute_jal_target(ADDR pc, INST instr);
        DATA imm;
        imm = `RV32_signext_Jimm(instr);
        return ADDR'(signed'(pc) + signed'(imm));
    endfunction

    // Helper: get instruction type name
    function automatic string get_inst_type(INST instr);
        case (instr.r.opcode)
            `RV32_LOAD:     return "LOAD";
            `RV32_STORE:    return "STORE";
            `RV32_BRANCH:   return "BRANCH";
            `RV32_JALR_OP:  return "JALR";
            `RV32_JAL_OP:   return "JAL";
            `RV32_OP_IMM:   return "OP_IMM";
            `RV32_OP:       return "OP";
            `RV32_SYSTEM:   return "SYSTEM";
            `RV32_AUIPC_OP: return "AUIPC";
            `RV32_LUI_OP:   return "LUI";
            `RV32_FENCE:    return "FENCE";
            `RV32_AMO:      return "AMO";
            default:        return "UNKNOWN";
        endcase
    endfunction

    // Unpack Inputs
    assign mispredict        = correct_branch_target.valid;
    assign mispredict_target = ADDR'(correct_branch_target.addr);

    // PC alignment (rounded down to 8-byte boundary)
    assign PC_aligned = {PC[31:3], 3'b000};

    // Extract instructions and valid bits from cache lines
    assign insts[0] = cache_data[0].data.word_level[0];
    assign insts[1] = cache_data[0].data.word_level[1];
    assign insts[2] = cache_data[1].data.word_level[0];
    assign insts[3] = cache_data[1].data.word_level[1];

    assign cache_valid_bits[0] = cache_data[0].valid;
    assign cache_valid_bits[1] = cache_data[0].valid;
    assign cache_valid_bits[2] = cache_data[1].valid;
    assign cache_valid_bits[3] = cache_data[1].valid;

    // Wait for all cache data to be valid
    assign icache_ready = &cache_valid_bits;

    // Alignment Logic - invalidate misaligned instructions
    // PC[2] determines if we start at the 1st or 2nd word of the 8-byte aligned block
    // PC[2]==0 -> start at offset 0 (index 0)
    // PC[2]==1 -> start at offset 4 (index 1)
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            align_valid_bits[i] = cache_valid_bits[i] && (i >= {1'b0, PC[2]});
        end
    end

    // Search for first valid branch or JAL/JALR instruction
    always_comb begin
        found_branch = 1'b0;
        first_branch_idx = '0;
        found_jump = 1'b0;
        first_jump_idx = '0;
        found_control_flow = 1'b0;
        first_control_flow_idx = '0;
        is_first_jump = 1'b0;
        jal_target = '0;
        
        // Single pass: find first branch or jump
        for (int i = 0; i < 4; i++) begin
            if (!found_control_flow && align_valid_bits[i]) begin
                // Check for jump first (higher priority)
                if (is_unconditional_jump(insts[i])) begin
                    found_jump = 1'b1;
                    first_jump_idx = 2'(i);
                    found_control_flow = 1'b1;
                    first_control_flow_idx = 2'(i);
                    is_first_jump = 1'b1;
                    // Compute JAL target if it's a JAL
                    if (insts[i].r.opcode == `RV32_JAL_OP) begin
                        jal_target = compute_jal_target(PC_aligned + (ADDR'(i) << 2), insts[i]);
                    end
                end
                // Check for branch (only if no jump found yet)
                else if (is_conditional_branch(insts[i])) begin
                    found_branch = 1'b1;
                    first_branch_idx = 2'(i);
                    found_control_flow = 1'b1;
                    first_control_flow_idx = 2'(i);
                    is_first_jump = 1'b0;
                end
            end
        end
    end

    assign branch_pc = PC_aligned + (ADDR'(first_branch_idx) << 2);

    // Branch Prediction Request - only if branch is found and it's the first control flow
    always_comb begin
        bp_request.valid = 1'b0;
        bp_request.pc    = '0;

        // Only request if we found a branch, it's the first control flow (not after a jump),
        // cache is ready, and not mispredicting
        if (icache_ready && !mispredict && found_branch && found_control_flow && !is_first_jump) begin
            bp_request.valid = 1'b1;
            bp_request.pc    = branch_pc;
        end
    end
    
    
    // Infer taken status from target (if target != branch_pc+4, it's taken)
    assign pred_taken = found_branch && (bp_response.target != (branch_pc + 4));

    // Final Valid Logic - invalidate instructions after control flow
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            // Start with aligned valid
            final_valid_bits[i] = align_valid_bits[i];
            
            // If we have a JAL/JALR that's first, invalidate everything after it
            if (found_control_flow && is_first_jump && (2'(i) > first_control_flow_idx)) begin
                final_valid_bits[i] = 1'b0;
            end
            
            // If we have a branch that's first and predicted taken, invalidate everything after it
            if (found_control_flow && !is_first_jump && pred_taken && (2'(i) > first_control_flow_idx)) begin
                final_valid_bits[i] = 1'b0;
            end
        end
    end
    
    assign valid_count = 3'($countones(final_valid_bits));
    assign num_pushes = stall ? '0 : ($clog2(`IB_PUSH_WIDTH+1))'(valid_count);

    // Stall Logic - wait for cache and check IB space
    assign stall = !icache_ready || (ib_free_slots < valid_count) || mispredict;

    // Output Packets (to IB)
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            fetch_packet[i].pc              = PC_aligned + (ADDR'(i) << 2);
            fetch_packet[i].inst            = insts[i];
            // Only valid if not stalled and passed all masks
            fetch_packet[i].valid           = !stall && final_valid_bits[i];
            
            // Default branch metadata
            fetch_packet[i].is_branch       = 1'b0;
            fetch_packet[i].bp_pred_taken   = 1'b0;
            fetch_packet[i].bp_pred_target  = '0;
            fetch_packet[i].bp_ghr_snapshot = '0;
        end

        // Mark branch metadata if we have a valid branch
        if (!stall && found_branch && !is_first_jump && final_valid_bits[first_branch_idx]) begin
            fetch_packet[first_branch_idx].is_branch       = 1'b1;
            fetch_packet[first_branch_idx].bp_pred_taken   = pred_taken;
            fetch_packet[first_branch_idx].bp_pred_target  = bp_response.target;
            fetch_packet[first_branch_idx].bp_ghr_snapshot = bp_response.ghr_snapshot;
        end
    end

    // Next PC Logic
    always_comb begin
        PC_next = PC; // Default: hold current PC (stall)

        if (mispredict) begin
            // Recovery: go to correct target immediately
            PC_next = mispredict_target;
        end else if (!stall) begin
            // Priority: JAL/JALR > taken branch > not taken branch / no branch
            if (found_control_flow && is_first_jump) begin
                // JAL/JALR is first: jump to target
                if (insts[first_control_flow_idx].r.opcode == `RV32_JAL_OP) begin
                    PC_next = jal_target;
                end else begin
                    // JALR target is resolved in execute stage, handled by mispredict recovery
                    // For now, advance PC (mispredict will handle recovery)
                    PC_next = PC_aligned + 16;
                end
            end else if (found_control_flow && !is_first_jump && pred_taken) begin
                // Branch is first and predicted taken: jump to target
                PC_next = bp_response.target;
            end else begin
                // Predict not taken or no branch: advance by bundle size
                PC_next = PC_aligned + 16;
            end
        end
    end

    // DEBUG: Track previous values to avoid duplicate prints
    logic prev_bp_request_valid;
    logic prev_icache_ready;
    ADDR prev_PC;
    integer cycle_count;
    
    // DEBUG: Branch prediction and fetched instructions (print once per cycle)
    always_ff @(posedge clock) begin
        if (reset) begin
            cycle_count <= 0;
            prev_bp_request_valid <= 1'b0;
            prev_icache_ready <= 1'b0;
            prev_PC <= '0;
        end else begin
            cycle_count <= cycle_count + 1;
            
            // Print fetched instructions when cache becomes ready (only once per PC)
            if (icache_ready && !mispredict && (PC != prev_PC || !prev_icache_ready)) begin
                $display("[FETCH] C%0d | PC=0x%08h (aligned=0x%08h, PC[2]=%b) | Inst[0]=%s(0x%08h)[cache=%b,align=%b,final=%b] [1]=%s(0x%08h)[cache=%b,align=%b,final=%b] [2]=%s(0x%08h)[cache=%b,align=%b,final=%b] [3]=%s(0x%08h)[cache=%b,align=%b,final=%b]",
                         cycle_count, PC, PC_aligned, PC[2],
                         get_inst_type(insts[0]), insts[0], cache_valid_bits[0], align_valid_bits[0], final_valid_bits[0],
                         get_inst_type(insts[1]), insts[1], cache_valid_bits[1], align_valid_bits[1], final_valid_bits[1],
                         get_inst_type(insts[2]), insts[2], cache_valid_bits[2], align_valid_bits[2], final_valid_bits[2],
                         get_inst_type(insts[3]), insts[3], cache_valid_bits[3], align_valid_bits[3], final_valid_bits[3]);
            end
            
            // Print branch prediction request (only when it becomes valid)
            if (bp_request.valid && !prev_bp_request_valid) begin
                $display("[FETCH] C%0d | BP Request  | PC=0x%08h | branch_idx=%0d | branch_pc=0x%08h",
                         cycle_count, PC, first_branch_idx, branch_pc);
            end
            
            // Print branch prediction response (only when response is used)
            if (found_branch) begin
                $display("[FETCH] C%0d | BP Response | pred_target=0x%08h | pred_taken=%b | fallthrough=0x%08h",
                         cycle_count, bp_response.target, pred_taken, branch_pc + 4);
            end
            
            // DEBUG: Print what's being sent to IB
            if (!stall && valid_count > 0) begin
                $write("[FETCH] C%0d | TO_IB: valid_count=%0d | final_valid=%b%b%b%b | ", 
                       cycle_count, valid_count, 
                       final_valid_bits[3], final_valid_bits[2], final_valid_bits[1], final_valid_bits[0]);
                for (int i = 0; i < 4; i++) begin
                    if (final_valid_bits[i]) begin
                        $write("packet[%0d]: PC=0x%08h(%s) valid=%b ", 
                               i, fetch_packet[i].pc, get_inst_type(fetch_packet[i].inst), fetch_packet[i].valid);
                    end
                end
                $display("");
            end
            
            // Update previous values
            prev_bp_request_valid <= bp_request.valid;
            prev_icache_ready <= icache_ready;
            prev_PC <= PC;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            PC <= '0;
        end else begin
            PC <= PC_next;
            // DEBUG: Print PC update each cycle with stall breakdown
            if (stall) begin
                $display("[FETCH] C%0d | PC Update    | PC=0x%08h -> PC_next=0x%08h | STALL: icache_ready=%b ib_free_slots=%0d valid_count=%0d mispredict=%b",
                         cycle_count, PC, PC_next, icache_ready, ib_free_slots, valid_count, mispredict);
                // Print cache miss details when stalled due to cache
                if (!icache_ready) begin
                    $display("[FETCH] C%0d | CACHE MISS: cache_data[0].valid=%b cache_data[1].valid=%b | read_addrs[0]=0x%08h read_addrs[1]=0x%08h",
                             cycle_count, cache_data[0].valid, cache_data[1].valid, 
                             read_addrs[0].addr, read_addrs[1].addr);
                end
            end else begin
                $display("[FETCH] C%0d | PC Update    | PC=0x%08h -> PC_next=0x%08h | stall=0 | mispredict=%b",
                         cycle_count, PC, PC_next, mispredict);
            end
        end
    end

    // ICache Read Addresses
    always_comb begin
        // Prepare read requests for current PC
        read_addrs[0].addr  = I_ADDR'(PC_aligned);
        read_addrs[1].addr  = I_ADDR'(PC_aligned + 8);
        
        // Always request (even if stalled, to maintain request for cache misses)
        read_addrs[0].valid = 1'b1;
        read_addrs[1].valid = 1'b1;
    end

endmodule

