`include "sys_defs.svh"
`include "ISA.svh"
// TODO make sure, pb always send back BTB whatever even if predict not taken,
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
    output FETCH_PACKET  [3:0]              fetch_packet
);
    typedef enum logic [2:0] {
        BRANCH_CAT_NONE   = 3'b000,
        BRANCH_CAT_J      = 3'b001,  // J, no writeback, no need for PB
        BRANCH_CAT_JAL    = 3'b010,  // JAL, has writeback, no need for PB
        BRANCH_CAT_JALR   = 3'b011,  // JALR, has writeback, needs PB 
        BRANCH_CAT_BRANCH = 3'b100   // BEQ, BNE, etc.
    } BRANCH_CATEGORY;

    function automatic logic is_branch(INST instr);
        return instr.r.opcode == `RV32_BRANCH || instr.r.opcode == `RV32_JALR_OP || instr.r.opcode == `RV32_JAL_OP;
    endfunction

    function automatic BRANCH_CATEGORY get_branch_category(INST instr);
        case (instr.r.opcode)
            `RV32_JAL_OP: begin
                if (instr.r.rd == `ZERO_REG) begin
                    return BRANCH_CAT_J;
                end else begin
                    return BRANCH_CAT_JAL;
                end
            end
            `RV32_JALR_OP: return BRANCH_CAT_JALR;
            `RV32_BRANCH:  return BRANCH_CAT_BRANCH;
            default:       return BRANCH_CAT_NONE;
        endcase
    endfunction

    // Helper: compute JAL target
    function automatic ADDR compute_jal_target(ADDR pc, INST instr);
        DATA imm;
        imm = `RV32_signext_Jimm(instr);
        return ADDR'(signed'(pc) + signed'(imm));
    endfunction

    // Helper: compute BRANCH target
    function automatic ADDR compute_branch_target(ADDR pc, INST instr);
        DATA imm;
        imm = `RV32_signext_Bimm(instr);
        return ADDR'(signed'(pc) + signed'(imm));
    endfunction

    logic [2:0] first_branch_idx;   // 4, out of bound index, means not found
    logic [2:0] second_branch_idx;  // 4, out of bound index, means not found
    BRANCH_CATEGORY first_branch_cat;
    BRANCH_CATEGORY second_branch_cat;
    logic [3:0] fetch_packet_valid_bits;
    INST  [3:0] insts;
    ADDR PC, PC_next;
    
    // Reusable signals for next PC and other logic
    ADDR PC_aligned;
    ADDR first_branch_pc;
    ADDR second_branch_pc;
    ADDR jal_target;
    ADDR branch_target;  // Computed BRANCH target
    logic first_branch_taken;
    logic [2:0] num_valids;  // Count of valid instructions in fetch_packet (0-4)

    assign insts[0] = cache_data[0].data.word_level[0];
    assign insts[1] = cache_data[0].data.word_level[1];
    assign insts[2] = cache_data[1].data.word_level[0];
    assign insts[3] = cache_data[1].data.word_level[1];

    // Compute PC_aligned and branch PCs
    assign PC_aligned = {PC[31:3], 3'b000};
    assign first_branch_pc = (first_branch_idx != 3'd4) ? (PC_aligned + (ADDR'(first_branch_idx) << 2)) : '0;
    assign second_branch_pc = (second_branch_idx != 3'd4) ? (PC_aligned + (ADDR'(second_branch_idx) << 2)) : '0;
    
    // Compute JAL target if first branch is J or JAL
    always_comb begin
        jal_target = '0;
        if (first_branch_idx != 3'd4 && 
            (first_branch_cat == BRANCH_CAT_J || first_branch_cat == BRANCH_CAT_JAL)) begin
            jal_target = compute_jal_target(first_branch_pc, insts[first_branch_idx]);
        end
    end

    // Compute BRANCH target if first branch is BRANCH type
    always_comb begin
        branch_target = '0;
        if (first_branch_idx != 3'd4 && first_branch_cat == BRANCH_CAT_BRANCH) begin
            branch_target = compute_branch_target(first_branch_pc, insts[first_branch_idx]);
        end
    end

    // branch_idx
    always_comb begin
        first_branch_idx = 3'd4;   // Default: not found
        second_branch_idx = 3'd4;  // Default: not found

        // first_branch_idx - start from index 1 if PC misaligned, otherwise from 0
        for (int i = (PC[2] ? 1 : 0); i < 4; i++) begin
            if (first_branch_idx == 3'd4 && is_branch(insts[i])) begin
                first_branch_idx = 3'(i);
            end
        end

        // second_branch_idx - only search if first was found
        if (first_branch_idx != 3'd4) begin
            for (int i = first_branch_idx + 1; i < 4; i++) begin
                if (second_branch_idx == 3'd4 && is_branch(insts[i])) begin
                    second_branch_idx = 3'(i);
                end
            end
        end
    end

    // branch categorization
    always_comb begin
        first_branch_cat = BRANCH_CAT_NONE;
        second_branch_cat = BRANCH_CAT_NONE;

        if (first_branch_idx != 3'd4) begin
            first_branch_cat = get_branch_category(insts[first_branch_idx]);
        end

        if (second_branch_idx != 3'd4) begin
            second_branch_cat = get_branch_category(insts[second_branch_idx]);
        end
    end

    // Compute first_branch_taken (reusable signal)
    always_comb begin
        first_branch_taken = 1'b0;
        
        if (first_branch_idx != 3'd4) begin
            if (first_branch_cat == BRANCH_CAT_J || 
                first_branch_cat == BRANCH_CAT_JALR || 
                first_branch_cat == BRANCH_CAT_JAL) begin
                // J/JALR/JAL are always taken
                first_branch_taken = 1'b1;
            end else if (first_branch_cat == BRANCH_CAT_BRANCH) begin
                // BRANCH: use taken bit from bp_response
                first_branch_taken = bp_response.taken;
            end
        end
    end

    // bp request
    always_comb begin
        bp_request.valid = 1'b0;
        bp_request.pc    = '0;

        // Only request for JALR or BRANCH type (J and JAL targets computed in fetch)
        if (first_branch_idx != 3'd4) begin
            if (first_branch_cat == BRANCH_CAT_JALR || first_branch_cat == BRANCH_CAT_BRANCH) begin
                bp_request.valid = 1'b1;
                bp_request.pc    = first_branch_pc;
            end
        end
    end

    // fetch_packet valid bits
    always_comb begin
        fetch_packet_valid_bits = 4'b1111;

        // If misaligned, invalidate first instruction
        if (PC[2]) begin
            fetch_packet_valid_bits[0] = 1'b0;
        end

        // Handle first branch
        if (first_branch_idx != 3'd4) begin
            // If first branch is taken, invalidate everything after it
            if (first_branch_taken) begin
                for (int i = first_branch_idx + 1; i < 4; i++) begin
                    fetch_packet_valid_bits[i] = 1'b0;
                end
            end

            // If first branch is J, invalidate itself (no writeback, no need to send to processor)
            if (first_branch_cat == BRANCH_CAT_J) begin
                fetch_packet_valid_bits[first_branch_idx] = 1'b0;
            end
        end

        // Handle second branch - if exists, invalidate itself and everything after it
        if (second_branch_idx != 3'd4) begin
            for (int i = second_branch_idx; i < 4; i++) begin
                fetch_packet_valid_bits[i] = 1'b0;
            end
        end
    end

    // Count number of valid instructions in fetch_packet
    assign num_valids = 3'($countones(fetch_packet_valid_bits));

    // Next PC logic
    always_comb begin
        PC_next = PC; // Default: hold current PC (stall)

        // Handle mispredict recovery
        if (correct_branch_target.valid) begin
            PC_next = ADDR'(correct_branch_target.addr);
        end

        else if (cache_data[0].valid && cache_data[1].valid && num_valids <= ib_free_slots) begin
            if (first_branch_idx != 3'd4) begin
                if (first_branch_cat == BRANCH_CAT_JALR) begin
                    PC_next = bp_response.target;
                end else if (first_branch_cat == BRANCH_CAT_BRANCH) begin
                    if (first_branch_taken) begin
                        // Use computed branch target (correct even when BTB misses)
                        PC_next = branch_target;
                    end else begin
                        if (second_branch_idx != 3'd4) begin
                            PC_next = second_branch_pc;
                        end else begin
                            PC_next = PC_aligned + 16;
                        end
                    end
                end else if (first_branch_cat == BRANCH_CAT_J || first_branch_cat == BRANCH_CAT_JAL) begin
                    PC_next = jal_target;
                end else begin
                    PC_next = PC_aligned + 16;
                end
            end else begin
                PC_next = PC_aligned + 16;
            end
        end
    end

    // cache read
    always_comb begin
        // Send current PC and PC + 8 to cache (each cache line is 2 instructions = 8 bytes)
        read_addrs[0].valid = 1'b1;
        read_addrs[0].addr  = I_ADDR'(PC_aligned);
        
        read_addrs[1].valid = 1'b1;
        read_addrs[1].addr  = I_ADDR'(PC_aligned + 8);
    end
    
    // Fetch packet
    always_comb begin
        logic send_to_ib;
        send_to_ib = !correct_branch_target.valid && 
                    cache_data[0].valid && 
                    cache_data[1].valid && 
                    num_valids <= ib_free_slots;
        
        // Initialize all fetch packets
        for (int i = 0; i < 4; i++) begin
            fetch_packet[i].pc    = PC_aligned + (ADDR'(i) << 2);
            fetch_packet[i].inst  = insts[i];
            fetch_packet[i].valid = send_to_ib ? fetch_packet_valid_bits[i] : 1'b0;
            
            // Default branch metadata
            fetch_packet[i].is_branch       = 1'b0;
            fetch_packet[i].bp_pred_taken   = 1'b0;
            fetch_packet[i].bp_pred_target  = '0;
            fetch_packet[i].bp_ghr_snapshot = '0;
        end
        
        // Set branch metadata for first branch if it's valid and we're send_to_ib
        if (send_to_ib && first_branch_idx != 3'd4 && fetch_packet_valid_bits[first_branch_idx]) begin
            fetch_packet[first_branch_idx].is_branch       = 1'b1;
            fetch_packet[first_branch_idx].bp_pred_taken   = first_branch_taken;
            
            // Set predicted target based on branch category
            if (first_branch_cat == BRANCH_CAT_JALR) begin
                // JALR uses bp_response target (register-based, can't compute in fetch)
                fetch_packet[first_branch_idx].bp_pred_target  = bp_response.target;
                fetch_packet[first_branch_idx].bp_ghr_snapshot = bp_response.ghr_snapshot;
            end else if (first_branch_cat == BRANCH_CAT_BRANCH) begin
                // BRANCH: use computed target when predicted taken, sequential when not taken
                if (first_branch_taken) begin
                    fetch_packet[first_branch_idx].bp_pred_target = branch_target;
                end else begin
                    fetch_packet[first_branch_idx].bp_pred_target = first_branch_pc + 4;
                end
                fetch_packet[first_branch_idx].bp_ghr_snapshot = bp_response.ghr_snapshot;
            end else if (first_branch_cat == BRANCH_CAT_J || first_branch_cat == BRANCH_CAT_JAL) begin
                // J and JAL use computed jal_target
                fetch_packet[first_branch_idx].bp_pred_target  = jal_target;
                fetch_packet[first_branch_idx].bp_ghr_snapshot = '0;  // J/JAL don't use BP
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            PC <= '0;
        end else begin
            PC <= PC_next;
        end
    end
    
endmodule