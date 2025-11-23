`include "sys_defs.svh"
`include "ISA.svh"
// Fetch stage fetches a bundle of instructions (4 instructions) from iCache.
// Stall If any of cache_data is invalid | IB is full | mispredict
//    when done stalling, Check if there is a branch in any of the 4
//        if has branch send BP packet
//        BP responds with prediction, we send that packet to IB
//    Update PC, send new read_addrs

module stage_fetch (
    input  logic                     clock,
    input  logic                     reset,

    // Icache_subsystem IOs (2 lines, 2 words/line -> 4 instructions)
    output I_ADDR_PACKET  [1:0]      read_addrs,
    input  CACHE_DATA     [1:0]      cache_data,

    // Branch Predictor IOs
    output  BP_PREDICT_REQUEST       bp_request,
    input   BP_PREDICT_RESPONSE      bp_response,

    // Instruction buffer IOs
    input  logic                     ib_full,        // less than 4 free spaces
    output logic                     ib_bundle_valid, // NEW: bundle valid -> IB
    output FETCH_PACKET    [3:0]     fetch_packet,   // 4 instructions per cycle



    // Pipeline control

    // Resolved branch information from retire
  //  input  logic                     branch_taken_out,
    input  ADDR                      branch_target_out,

    output ADDR        [3:0]      dbg_fetch_pc           ,
    output logic [3:0][31:0]      dbg_fetch_inst         ,
    output logic      [3:0]       dbg_fetch_valid        ,
    output logic      [3:0]       dbg_fetch_is_branch    ,
    output logic      [3:0]       dbg_fetch_bp_pred_taken,
    output ADDR       [3:0]       dbg_fetch_bp_pred_target,
    output logic [3:0][`BP_GH-1:0]dbg_fetch_bp_ghr_snapshot
);

    // -----------------------------
    // Internal state
    // -----------------------------
    ADDR PC, PC_next;

    // Four instructions in the bundle
    INST inst [3:0];

    // Valid bits per lane (derived from cache_data.valid)
    logic [3:0] valid_bit_array;
    logic [3:0] conditional_branch_bit_array;
    logic       found_branch;
    int         first_branch_idx;

    logic       icache_ready;
    logic       fetch_stall;
    ADDR        branch_pc;

    // -----------------------------
    // Helper: conditional branch
    // -----------------------------
    function automatic logic is_conditional_branch (logic [31:0] instr);
        return instr[6:0] == `RV32_BRANCH;
    endfunction

    assign inst[0] = cache_data[0].data.word_level[0];
    assign inst[1] = cache_data[0].data.word_level[1];
    assign inst[2] = cache_data[1].data.word_level[0];
    assign inst[3] = cache_data[1].data.word_level[1];

    always_comb begin
        valid_bit_array[0] = cache_data[0].valid;
        valid_bit_array[1] = cache_data[0].valid;
        valid_bit_array[2] = cache_data[1].valid;
        valid_bit_array[3] = cache_data[1].valid;
    end

    assign icache_ready = &valid_bit_array;
    assign fetch_stall  = !icache_ready || ib_full || mispredict;

    // This is the signal that tells the IB:
    // "there is a real 4-wide bundle on fetch_packet this cycle"
    assign ib_bundle_valid = icache_ready && !ib_full && !mispredict;

    // -----------------------------
    // iCache read address logic
    // -----------------------------
    always_comb begin
        // Addresses for two 64b lines
        read_addrs[0].addr  = PC;
        read_addrs[1].addr  = PC + 32'h8;

        // Default: don't issue
        read_addrs[0].valid = 1'b0;
        read_addrs[1].valid = 1'b0;

        if (!ib_full) begin
            read_addrs[0].valid = 1'b1;
            read_addrs[1].valid = 1'b1;
        end
    end

    always_comb begin
        found_branch     = 1'b0;
        first_branch_idx = 0;

        // Determine which lanes are conditional branches
        for (int i = 0; i < 4; i++) begin
            conditional_branch_bit_array[i] =
                valid_bit_array[i] && is_conditional_branch(inst[i]);
        end

        // Scan in program order and pick the earliest one
        for (int i = 0; i < 4; i++) begin
            if (!found_branch && conditional_branch_bit_array[i]) begin
                found_branch     = 1'b1;
                first_branch_idx = i;
            end
        end
    end

    assign branch_pc = PC + (ADDR'(first_branch_idx) << 2);

    always_comb begin
        bp_request.valid = 1'b0;
        bp_request.pc    = '0;

        if (!fetch_stall && found_branch) begin
            bp_request.valid = 1'b1;
            bp_request.pc    = branch_pc;
        end
    end

    always_comb begin
        // Default: fill PCs / insts, clear branch metadata
        for (int i = 0; i < 4; i++) begin
            fetch_packet[i].pc              = PC + (ADDR'(i) << 2);
            fetch_packet[i].inst            = inst[i];
            fetch_packet[i].valid           = 1'b1;
            fetch_packet[i].is_branch       = 1'b0;
            fetch_packet[i].bp_pred_taken   = 1'b0;
            fetch_packet[i].bp_pred_target  = '0;
            fetch_packet[i].bp_ghr_snapshot = '0;
        end

        if (ib_bundle_valid && found_branch) begin
            fetch_packet[first_branch_idx].is_branch       = 1'b1;
            fetch_packet[first_branch_idx].bp_pred_taken   = bp_response.taken;
            fetch_packet[first_branch_idx].bp_pred_target  = bp_response.target;
            fetch_packet[first_branch_idx].bp_ghr_snapshot = bp_response.ghr_snapshot;
        end

        // -----------------------------
        // Debug signals for fetch
        // -----------------------------
        for (int i = 0; i < 4; i++) begin
            dbg_fetch_pc[i]              = fetch_packet[i].pc;
            dbg_fetch_inst[i]            = fetch_packet[i].inst;
            dbg_fetch_valid[i]           = fetch_packet[i].valid;
            dbg_fetch_is_branch[i]       = fetch_packet[i].is_branch;
            dbg_fetch_bp_pred_taken[i]   = fetch_packet[i].bp_pred_taken;
            dbg_fetch_bp_pred_target[i]  = fetch_packet[i].bp_pred_target;
            dbg_fetch_bp_ghr_snapshot[i] = fetch_packet[i].bp_ghr_snapshot;
        end
    end

    // PC increment constants for clarity
    localparam logic [31:0] BUNDLE_SIZE_BYTES = 32'h10;  // 4 instructions * 4 bytes each

    always_comb begin
        PC_next = PC;  // Default: stay at current PC (when stalled)

        if (mispredict) begin
            // Handle mispredicted branch recovery
            PC_next = branch_target_out;

        end else if (!fetch_stall) begin
            if (found_branch && bp_response.taken) begin
                // Branch predictor says taken - follow prediction
                PC_next = bp_response.target;
            end else begin
                // No branch or branch predicted not taken - advance by bundle size
                PC_next = PC + BUNDLE_SIZE_BYTES;
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
