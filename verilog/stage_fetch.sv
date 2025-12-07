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
    output FETCH_PACKET  [3:0]              fetch_packet
);

    // =========================================================================
    // Helper Functions
    // =========================================================================
    
    function automatic logic is_branch(INST instr);
        return instr.r.opcode == `RV32_JAL_OP  ||
               instr.r.opcode == `RV32_JALR_OP ||
               instr.r.opcode == `RV32_BRANCH;
    endfunction

    function automatic logic is_jal(INST instr);
        return instr.r.opcode == `RV32_JAL_OP;
    endfunction

    function automatic logic is_jalr(INST instr);
        return instr.r.opcode == `RV32_JALR_OP;
    endfunction

    function automatic logic is_cond_branch(INST instr);
        return instr.r.opcode == `RV32_BRANCH;
    endfunction

    function automatic ADDR compute_jal_target(ADDR pc, INST instr);
        return ADDR'(signed'(pc) + signed'(`RV32_signext_Jimm(instr)));
    endfunction

    function automatic ADDR compute_branch_target(ADDR pc, INST instr);
        return ADDR'(signed'(pc) + signed'(`RV32_signext_Bimm(instr)));
    endfunction

    // =========================================================================
    // Signals
    // =========================================================================
    
    INST  [3:0] insts;
    ADDR PC, PC_next, PC_aligned;
    
    // Branch detection
    logic [2:0] first_branch_idx;   // 4 = not found
    logic [2:0] second_branch_idx;  // 4 = not found
    ADDR first_branch_pc;
    
    // Computed targets
    ADDR jal_target, branch_target;
    
    // Control signals
    logic first_is_jal, first_is_jalr, first_is_cond;
    logic first_branch_taken;
    logic [3:0] valid_bits;
    logic [2:0] num_valids;
    logic send_to_ib;
    
    // =========================================================================
    // Instruction Extraction
    // =========================================================================
    
    assign insts[0] = cache_data[0].data.word_level[0];
    assign insts[1] = cache_data[0].data.word_level[1];
    assign insts[2] = cache_data[1].data.word_level[0];
    assign insts[3] = cache_data[1].data.word_level[1];
    
    assign PC_aligned = {PC[31:3], 3'b000};
    assign first_branch_pc = PC_aligned + (ADDR'(first_branch_idx) << 2);

    // =========================================================================
    // Find First and Second Branch
    // =========================================================================
    
    always_comb begin
        first_branch_idx  = 3'd4;
        second_branch_idx = 3'd4;

        for (int i = 0; i < 4; i++) begin
            if (first_branch_idx == 3'd4 && is_branch(insts[i])) begin
                // Skip slot 0 if PC is misaligned
                if (i != 0 || !PC[2]) first_branch_idx = 3'(i);
            end else if (first_branch_idx != 3'd4 && second_branch_idx == 3'd4 && is_branch(insts[i])) begin
                second_branch_idx = 3'(i);
            end
        end
    end

    // =========================================================================
    // Branch Type and Target Computation
    // =========================================================================
    
    always_comb begin
        first_is_jal  = (first_branch_idx != 3'd4) && is_jal(insts[first_branch_idx]);
        first_is_jalr = (first_branch_idx != 3'd4) && is_jalr(insts[first_branch_idx]);
        first_is_cond = (first_branch_idx != 3'd4) && is_cond_branch(insts[first_branch_idx]);
        
        // JAL target (computed in fetch)
        jal_target = first_is_jal ? compute_jal_target(first_branch_pc, insts[first_branch_idx]) : '0;
        
        // Conditional branch target (computed in fetch)
        branch_target = first_is_cond ? compute_branch_target(first_branch_pc, insts[first_branch_idx]) : '0;
        
        // JAL/JALR always taken, conditional uses predictor
        first_branch_taken = first_is_jal || first_is_jalr || (first_is_cond && bp_response.taken) || (branch_target == first_branch_pc + 4);
    end

    // =========================================================================
    // Branch Predictor Request
    // =========================================================================
    
    always_comb begin
        bp_request.valid = first_is_jalr || first_is_cond;
        bp_request.pc    = first_branch_pc;
    end

    // =========================================================================
    // Valid Bits Computation
    // =========================================================================
    
    always_comb begin
        valid_bits = 4'b1111;
        
        // Misaligned: skip first slot
        if (PC[2]) valid_bits[0] = 1'b0;
        
        // Invalidate after taken branch
        if (first_branch_idx != 3'd4 && first_branch_taken) begin
            for (int i = 0; i < 4; i++) begin
                if (i > first_branch_idx) valid_bits[i] = 1'b0;
            end
        end
        
        // Second branch: invalidate it and everything after
        if (second_branch_idx != 3'd4) begin
            for (int i = 0; i < 4; i++) begin
                if (i >= second_branch_idx) valid_bits[i] = 1'b0;
            end
        end
    end
    
    assign num_valids = 3'($countones(valid_bits));
    assign send_to_ib = !correct_branch_target.valid && 
                        cache_data[0].valid && 
                        cache_data[1].valid && 
                        num_valids <= ib_free_slots;

    // =========================================================================
    // Next PC Logic
    // =========================================================================
    
    always_comb begin
        PC_next = PC;  // Default: stall

        if (correct_branch_target.valid) begin
            // Mispredict recovery
            PC_next = ADDR'(correct_branch_target.addr);
        end else if (send_to_ib) begin
            if (first_branch_idx == 3'd4) begin
                // No branch: sequential
                PC_next = PC_aligned + 16;
            end else if (first_is_jal) begin
                PC_next = jal_target;
            end else if (first_is_jalr) begin
                PC_next = bp_response.target;
            end else if (first_is_cond) begin
                if (first_branch_taken) begin
                    PC_next = branch_target;
                end else if (second_branch_idx != 3'd4) begin
                    // Not taken, but there's a second branch we couldn't handle
                    PC_next = PC_aligned + (ADDR'(second_branch_idx) << 2);
                end else begin
                    PC_next = PC_aligned + 16;
                end
            end
        end
    end

    // =========================================================================
    // Cache Read Requests
    // =========================================================================
    
    always_comb begin
        read_addrs[0].valid = 1'b1;
        read_addrs[0].addr  = I_ADDR'(PC_aligned);
        read_addrs[1].valid = 1'b1;
        read_addrs[1].addr  = I_ADDR'(PC_aligned + 8);
    end

    // =========================================================================
    // Fetch Packet Output
    // =========================================================================
    
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            fetch_packet[i].pc    = PC_aligned + (ADDR'(i) << 2);
            fetch_packet[i].inst  = insts[i];
            fetch_packet[i].valid = send_to_ib ? valid_bits[i] : 1'b0;
            
            // Default branch metadata
            fetch_packet[i].is_branch       = 1'b0;
            fetch_packet[i].bp_pred_taken   = 1'b0;
            fetch_packet[i].bp_pred_target  = '0;
            fetch_packet[i].bp_ghr_snapshot = '0;
        end
        
        // Set branch metadata for first branch
        if (send_to_ib && first_branch_idx != 3'd4 && valid_bits[first_branch_idx]) begin
            fetch_packet[first_branch_idx].is_branch     = 1'b1;
            fetch_packet[first_branch_idx].bp_pred_taken = first_branch_taken;
            
            if (first_is_jalr) begin
                fetch_packet[first_branch_idx].bp_pred_target  = bp_response.target;
                fetch_packet[first_branch_idx].bp_ghr_snapshot = bp_response.ghr_snapshot;
            end else if (first_is_cond) begin
                fetch_packet[first_branch_idx].bp_pred_target  = first_branch_taken ? branch_target : (first_branch_pc + 4);
                fetch_packet[first_branch_idx].bp_ghr_snapshot = bp_response.ghr_snapshot;
            end else begin  // JAL
                fetch_packet[first_branch_idx].bp_pred_target  = jal_target;
                fetch_packet[first_branch_idx].bp_ghr_snapshot = '0;
            end
        end
    end

    // =========================================================================
    // PC Register
    // =========================================================================
    
    always_ff @(posedge clock) begin
        if (reset) begin
            PC <= '0;
        end else begin
            PC <= PC_next;
        end
    end
    
endmodule
