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
    output logic                      bp_response_used,

    // retire when mispredict
    input I_ADDR_PACKET               correct_branch_target,

    // instruction buffer
    input logic          [`IB_IDX_BITS:0]   ib_free_slots,
    output FETCH_PACKET  [3:0]                fetch_packet,
    output logic [$clog2(`IB_PUSH_WIDTH+1)-1:0] num_pushes
);

    // Internal state
    ADDR PC, PC_next;
    
    // Derived signals
    INST [3:0] insts;
    logic [3:0] cache_valid_bits;
    logic [3:0] align_valid_bits;
    logic [3:0] final_valid_bits;
    logic [3:0] is_branch_bits;
    
    logic       found_branch;
    logic [1:0] first_branch_idx;
    logic       stall;
    
    // Mispredict handling
    logic       mispredict;
    ADDR        mispredict_target;
    ADDR        branch_pc;
    ADDR        PC_aligned;
    logic       pred_taken;
    
    logic       icache_ready;
    logic [2:0] valid_count;

    // Helper: conditional branch
    function automatic logic is_conditional_branch(INST instr);
        return instr.r.opcode == `RV32_BRANCH;
    endfunction

    // Helper: unconditional jump (JAL/JALR)
    function automatic logic is_unconditional_jump(INST instr);
        return (instr.r.opcode == `RV32_JAL_OP) || (instr.r.opcode == `RV32_JALR_OP);
    endfunction

    // Helper: extract JAL immediate and compute target
    function automatic ADDR compute_jal_target(ADDR pc, INST instr);
        DATA imm;
        // Sign-extend JAL immediate: {imm[20], imm[10:1], imm[11], imm[19:12], 1'b0}
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

    assign icache_ready = &cache_valid_bits;

    // Alignment Logic
    // PC[2] determines if we start at the 1st or 2nd word of the 8-byte aligned block
    // PC[2]==0 -> start at offset 0 (index 0)
    // PC[2]==1 -> start at offset 4 (index 1)
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            align_valid_bits[i] = cache_valid_bits[i] && (i >= {1'b0, PC[2]});
        end
    end

    // Branch and Jump Detection
    logic [3:0] is_jump_bits;
    logic       found_jump;
    logic [1:0] first_jump_idx;
    ADDR        jal_target;
    
    always_comb begin
        found_branch     = 1'b0;
        first_branch_idx = '0;
        found_jump       = 1'b0;
        first_jump_idx   = '0;
        jal_target       = '0;
        
        for (int i = 0; i < 4; i++) begin
            is_branch_bits[i] = align_valid_bits[i] && is_conditional_branch(insts[i]);
            is_jump_bits[i]   = align_valid_bits[i] && is_unconditional_jump(insts[i]);
        end

        // Priority encoder to find the first VALID branch in the bundle
        for (int i = 0; i < 4; i++) begin
            if (!found_branch && is_branch_bits[i]) begin
                found_branch     = 1'b1;
                first_branch_idx = 2'(i);
            end
        end
        
        // Priority encoder to find the first VALID jump (JAL/JALR) in the bundle
        for (int i = 0; i < 4; i++) begin
            if (!found_jump && is_jump_bits[i]) begin
                found_jump     = 1'b1;
                first_jump_idx = 2'(i);
                // Compute JAL target if it's a JAL (JALR target needs rs1, handled later)
                if (insts[i].r.opcode == `RV32_JAL_OP) begin
                    jal_target = compute_jal_target(PC_aligned + (ADDR'(i) << 2), insts[i]);
                end
            end
        end
    end

    assign branch_pc = PC_aligned + (ADDR'(first_branch_idx) << 2); // first_branch_idx * 4

    // Branch Prediction Request
    always_comb begin
        bp_request.valid = 1'b0;
        bp_request.pc    = '0;

        // Only request if we found a valid branch and we aren't stalled by cache miss/mispredict
        if (icache_ready && !mispredict && found_branch) begin
            bp_request.valid = 1'b1;
            bp_request.pc    = branch_pc;
        end
    end
    
    assign bp_response_used = bp_request.valid;
    
    // Infer taken status from target (if target != branch_pc+4, it's taken)
    assign pred_taken = (bp_response.target != (branch_pc + 4));
    
    // DEBUG: Track previous values to avoid duplicate prints
    logic prev_bp_request_valid;
    logic prev_bp_response_used;
    logic prev_icache_ready;
    ADDR prev_PC;
    logic [$clog2(`IB_PUSH_WIDTH+1)-1:0] prev_num_pushes;
    
    // DEBUG: Cycle counter for prints
    integer cycle_count;
    
    // DEBUG: Branch prediction and fetched instructions (print once per cycle)
    always_ff @(posedge clock) begin
        if (reset) begin
            cycle_count <= 0;
            prev_bp_request_valid <= 1'b0;
            prev_bp_response_used <= 1'b0;
            prev_icache_ready <= 1'b0;
            prev_PC <= '0;
            prev_num_pushes <= '0;
        end else begin
            cycle_count <= cycle_count + 1;
            
            // Print fetched instructions when cache becomes ready (only once per PC)
            if (icache_ready && !mispredict && (PC != prev_PC || !prev_icache_ready)) begin
                $display("[FETCH] C%0d | PC=%0d (aligned=%0d, PC[2]=%b) | Inst[0]=%s(0x%08h)[cache=%b,align=%b,final=%b] [1]=%s(0x%08h)[cache=%b,align=%b,final=%b] [2]=%s(0x%08h)[cache=%b,align=%b,final=%b] [3]=%s(0x%08h)[cache=%b,align=%b,final=%b]",
                         cycle_count, PC, PC_aligned, PC[2],
                         get_inst_type(insts[0]), insts[0], cache_valid_bits[0], align_valid_bits[0], final_valid_bits[0],
                         get_inst_type(insts[1]), insts[1], cache_valid_bits[1], align_valid_bits[1], final_valid_bits[1],
                         get_inst_type(insts[2]), insts[2], cache_valid_bits[2], align_valid_bits[2], final_valid_bits[2],
                         get_inst_type(insts[3]), insts[3], cache_valid_bits[3], align_valid_bits[3], final_valid_bits[3]);
            end
            
            // Print branch prediction request (only when it becomes valid)
            if (bp_request.valid && !prev_bp_request_valid) begin
                $display("[FETCH] C%0d | BP Request  | PC=%0d | branch_idx=%0d | branch_pc=%0d",
                         cycle_count, PC, first_branch_idx, branch_pc);
            end
            
            // Print branch prediction response (only when response is used)
            if (bp_response_used && found_branch && !prev_bp_response_used) begin
                $display("[FETCH] C%0d | BP Response | pred_target=%0d | pred_taken=%b | fallthrough=%0d",
                         cycle_count, bp_response.target, pred_taken, branch_pc + 4);
            end
            
            // DEBUG: Print what's being sent to IB (when num_pushes changes or when we have pushes)
            if (num_pushes > 0 && num_pushes != prev_num_pushes) begin
                $write("[FETCH] C%0d | TO_IB: num_pushes=%0d | valid_count=%0d | stall=%b | final_valid=%b%b%b%b | ", 
                       cycle_count, num_pushes, valid_count, stall, 
                       final_valid_bits[3], final_valid_bits[2], final_valid_bits[1], final_valid_bits[0]);
                for (int i = 0; i < 4; i++) begin
                    if (i < num_pushes) begin
                        $write("packet[%0d]: PC=%0d(%s) valid=%b ", 
                               i, fetch_packet[i].pc, get_inst_type(fetch_packet[i].inst), fetch_packet[i].valid);
                    end
                end
                $display("");
            end
            
            // Update previous values
            prev_bp_request_valid <= bp_request.valid;
            prev_bp_response_used <= bp_response_used;
            prev_icache_ready <= icache_ready;
            prev_PC <= PC;
            prev_num_pushes <= num_pushes;
        end
    end

    // Final Valid Logic
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            // Start with aligned valid
            final_valid_bits[i] = align_valid_bits[i];
            
            // If we have a taken branch, mask everything after it
            if (found_branch && pred_taken && (2'(i) > first_branch_idx)) begin
                final_valid_bits[i] = 1'b0;
            end
            
            // If we have an unconditional jump (JAL/JALR), mask everything after it
            // Jumps are always taken, so invalidate all subsequent instructions
            if (found_jump && (2'(i) > first_jump_idx)) begin
                final_valid_bits[i] = 1'b0;
            end
        end
    end
    
    assign valid_count = 3'($countones(final_valid_bits));

    // Stall Logic
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

        if (!stall && found_branch && final_valid_bits[first_branch_idx]) begin
            fetch_packet[first_branch_idx].is_branch       = 1'b1;
            fetch_packet[first_branch_idx].bp_pred_taken   = pred_taken;
            fetch_packet[first_branch_idx].bp_pred_target  = bp_response.target;
            fetch_packet[first_branch_idx].bp_ghr_snapshot = bp_response.ghr_snapshot;
        end
    end

    // Compute num_pushes using $countones
    // Note: Use final_valid_bits, not fetch_packet[].valid, to avoid circular dependency
    // fetch_packet[].valid includes !stall, but num_pushes is used to compute stall condition
    assign num_pushes = stall ? '0 : ($clog2(`IB_PUSH_WIDTH+1))'(valid_count);

    // Next PC Logic
    always_comb begin
        PC_next = PC; // Default: hold current PC (stall)

        if (mispredict) begin
            // Recovery: go to correct target immediately
            PC_next = mispredict_target;
        end else if (!stall) begin
            // Priority: JAL/JALR > taken branch > not taken branch / no branch
            if (found_jump && (insts[first_jump_idx].r.opcode == `RV32_JAL_OP)) begin
                // JAL: jump to computed target
                PC_next = jal_target;
            end else if (found_branch && pred_taken) begin
                // Predict taken: jump to target
                PC_next = bp_response.target;
            end else begin
                // Predict not taken or no branch: advance by bundle size
                // We align to the next 16-byte block boundary
                PC_next = PC_aligned + 16;
            end
            // Note: JALR target is computed in execute stage, handled via mispredict recovery
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            PC <= '0;
        end else begin
            PC <= PC_next;
            // DEBUG: Print PC update each cycle
            $display("[FETCH] C%0d | PC Update    | PC=%0d -> PC_next=%0d | stall=%b | mispredict=%b",
                     cycle_count, PC, PC_next, stall, mispredict);
        end
    end

    // ICache Read Addresses
    always_comb begin
        // Prepare read requests for current PC
        // Note: ICache handles unaligned requests by returning the line containing the address
        read_addrs[0].addr  = I_ADDR'(PC_aligned);
        read_addrs[1].addr  = I_ADDR'(PC_aligned + 8);
        
        // Valid logic:
        // Request if we are not reset. 
        // Even if stalled, we want to maintain the request to get data (if miss) or because we are waiting.
        read_addrs[0].valid = 1'b1;
        read_addrs[1].valid = 1'b1;
    end

endmodule
