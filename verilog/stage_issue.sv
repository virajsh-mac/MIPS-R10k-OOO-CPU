`include "sys_defs.svh"

module stage_issue (
    input clock,
    input reset,
    input logic mispredict,

    input RS_BANKS  rs_banks,  // Structured RS entries
    input FU_GRANTS fu_grants, // FU availability grants (structured) - from cdb arbiter

    output ISSUE_CLEAR   issue_clear,    // Clear signals (structured)
    output ISSUE_ENTRIES issue_entries,  // To EX stage (structured)
    output FU_REQUESTS   cdb_requests,   // CDB requests for single-cycle FUs

    // Debug outputs
    output logic [`RS_ALU_SZ-1:0] rs_alu_ready_dbg,
    output ISSUE_ENTRIES issue_entries_dbg,
    output logic [`RS_ALU_SZ-1:0] rs_alu_requests_dbg,
    output logic [`RS_MULT_SZ-1:0] rs_mult_requests_dbg,
    output logic [`RS_BRANCH_SZ-1:0] rs_branch_requests_dbg,
    output logic [`RS_MEM_SZ-1:0] rs_mem_requests_dbg,
    output logic [`NUM_FU_ALU-1:0] alu_clear_signals_dbg,  // TEMP: ALU clear signals from CDB
    output logic [`RS_ALU_SZ-1:0][`NUM_FU_ALU-1:0] grants_alu_dbg
);

    // Helper: Check if RS entry is ready
    function automatic logic is_ready(RS_ENTRY entry);
        return entry.valid && entry.src1_ready && entry.src2_ready;
    endfunction

    // Internal state (structured)
    ISSUE_ENTRIES issue_register, issue_register_next;

    // Ready signals for each RS bank
    logic [`RS_ALU_SZ-1:0]    rs_ready_alu;
    logic [`RS_MULT_SZ-1:0]   rs_ready_mult;
    logic [`RS_BRANCH_SZ-1:0] rs_ready_branch;
    logic [`RS_MEM_SZ-1:0]    rs_ready_mem;

    // Grant outputs from allocators
    logic [`RS_ALU_SZ-1:0][`NUM_FU_ALU-1:0] grants_alu;
    logic [`RS_MULT_SZ-1:0][`NUM_FU_MULT-1:0] grants_mult;
    logic [`RS_BRANCH_SZ-1:0][`NUM_FU_BRANCH-1:0] grants_branch;
    logic [`RS_MEM_SZ-1:0][`NUM_FU_MEM-1:0] grants_mem;

    // Compute ready signals for each bank
    always_comb begin
        for (int i = 0; i < `RS_ALU_SZ; i++) begin
            rs_ready_alu[i] = is_ready(rs_banks.alu[i]);
        end
        for (int i = 0; i < `RS_MULT_SZ; i++) begin
            rs_ready_mult[i] = is_ready(rs_banks.mult[i]);
        end
        for (int i = 0; i < `RS_BRANCH_SZ; i++) begin
            rs_ready_branch[i] = is_ready(rs_banks.branch[i]);
        end
        for (int i = 0; i < `RS_MEM_SZ; i++) begin
            rs_ready_mem[i] = is_ready(rs_banks.mem[i]);
        end
    end

    // Allocators for each FU category
    allocator #(
        .NUM_RESOURCES(`NUM_FU_ALU),
        .NUM_REQUESTS (`RS_ALU_SZ)
    ) alu_allocator (
        .reset(reset | mispredict),
        .clock(clock),
        .req  (rs_ready_alu),
        .clear(fu_grants.alu),
        .mispredict(1'b0),
        .initial_mask('1),
        .restore_mask('1),
        .grant(grants_alu)
    );

    allocator #(
        .NUM_RESOURCES(`NUM_FU_MULT),
        .NUM_REQUESTS (`RS_MULT_SZ)
    ) mult_allocator (
        .reset(reset | mispredict),
        .clock(clock),
        .req  (rs_ready_mult),
        .clear(fu_grants.mult),
        .mispredict(1'b0),
        .initial_mask('1),
        .restore_mask('1),
        .grant(grants_mult)
    );

    allocator #(
        .NUM_RESOURCES(`NUM_FU_BRANCH),
        .NUM_REQUESTS (`RS_BRANCH_SZ)
    ) branch_allocator (
        .reset(reset | mispredict),
        .clock(clock),
        .req  (rs_ready_branch),
        .clear(fu_grants.branch),
        .mispredict(1'b0),
        .initial_mask('1),
        .restore_mask('1),
        .grant(grants_branch)
    );

    allocator #(
        .NUM_RESOURCES(`NUM_FU_MEM),
        .NUM_REQUESTS (`RS_MEM_SZ)
    ) mem_allocator (
        .reset(reset | mispredict),
        .clock(clock),
        .req  (rs_ready_mem),
        .clear(fu_grants.mem),
        .mispredict(1'b0),
        .initial_mask('1),
        .restore_mask('1),
        .grant(grants_mem)
    );

    // Generate clear signals (grant to index conversion)
    // Note: clear indices contain local RS bank indices since each RS module
    // maintains its own entry array. The RS modules handle their own indexing.
    always_comb begin
        // drive issue clear
        issue_clear = '0;

        // ALU - use local ALU RS indices
        for (int rs = 0; rs < `RS_ALU_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_ALU; fu++) begin
                if (grants_alu[rs][fu]) begin
                    issue_clear.valid_alu[fu] = 1'b1;
                    issue_clear.idxs_alu[fu]  = RS_IDX'(rs);
                end
            end
        end

        // MULT - use local MULT RS indices
        for (int rs = 0; rs < `RS_MULT_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_MULT; fu++) begin
                if (grants_mult[rs][fu]) begin
                    issue_clear.valid_mult[fu] = 1'b1;
                    issue_clear.idxs_mult[fu]  = RS_IDX'(rs);
                end
            end
        end

        // BRANCH - use local BRANCH RS indices
        for (int rs = 0; rs < `RS_BRANCH_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_BRANCH; fu++) begin
                if (grants_branch[rs][fu]) begin
                    issue_clear.valid_branch[fu] = 1'b1;
                    issue_clear.idxs_branch[fu]  = RS_IDX'(rs);
                end
            end
        end

        // MEM - use local MEM RS indices
        for (int rs = 0; rs < `RS_MEM_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_MEM; fu++) begin
                if (grants_mem[rs][fu]) begin
                    issue_clear.valid_mem[fu] = 1'b1;
                    issue_clear.idxs_mem[fu]  = RS_IDX'(rs);
                end
            end
        end
    end

    // Update issue register from granted RS entries
    always_comb begin
        // drive issue_register_next
        issue_register_next = issue_register;

        // Clear completed FUs from issue register
        for (int fu = 0; fu < `NUM_FU_ALU; fu++) begin
            if (fu_grants.alu[fu]) begin
                issue_register_next.alu[fu].valid = 1'b0;
            end
        end
        for (int fu = 0; fu < `NUM_FU_MULT; fu++) begin
            if (fu_grants.mult[fu]) begin
                issue_register_next.mult[fu].valid = 1'b0;
            end
        end
        for (int fu = 0; fu < `NUM_FU_BRANCH; fu++) begin
            if (fu_grants.branch[fu]) begin
                issue_register_next.branch[fu].valid = 1'b0;
            end
        end
        for (int fu = 0; fu < `NUM_FU_MEM; fu++) begin
            if (fu_grants.mem[fu]) begin
                issue_register_next.mem[fu].valid = 1'b0;
            end
        end

        // ALU - use structured ALU bank
        for (int rs = 0; rs < `RS_ALU_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_ALU; fu++) begin
                if (grants_alu[rs][fu]) begin
                    issue_register_next.alu[fu] = rs_banks.alu[rs];
                end
            end
        end

        // MULT - use structured MULT bank
        for (int rs = 0; rs < `RS_MULT_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_MULT; fu++) begin
                if (grants_mult[rs][fu]) begin
                    issue_register_next.mult[fu] = rs_banks.mult[rs];
                end
            end
        end

        // BRANCH - use structured BRANCH bank
        for (int rs = 0; rs < `RS_BRANCH_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_BRANCH; fu++) begin
                if (grants_branch[rs][fu]) begin
                    issue_register_next.branch[fu] = rs_banks.branch[rs];
                end
            end
        end

        // MEM - use structured MEM bank
        for (int rs = 0; rs < `RS_MEM_SZ; rs++) begin
            for (int fu = 0; fu < `NUM_FU_MEM; fu++) begin
                if (grants_mem[rs][fu]) begin
                    issue_register_next.mem[fu] = rs_banks.mem[rs];
                end
            end
        end
    end

    // Generate CDB requests for single-cycle FUs
    // Request for each FU that has a valid allocated instruction (use next state for same-cycle requests)
    always_comb begin
        cdb_requests = '0;

        // ALU requests: one-hot array where each bit indicates FU has valid instruction
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            cdb_requests.alu[i] = issue_register_next.alu[i].valid;
        end

        // BRANCH requests: one-hot array where each bit indicates FU has valid instruction
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            cdb_requests.branch[i] = issue_register_next.branch[i].valid;
        end

        // MEM requests: MEM operations request CDB access from execute stage when results are ready
        // (not at issue time due to variable cache miss latency)
        cdb_requests.mem = '0;

        // MULT requests come from execute stage (pipelined), not from issue register
        cdb_requests.mult = '0;
    end

    // Output assignment
    assign issue_entries = issue_register;

    // Debug assignments
    assign rs_alu_ready_dbg = rs_ready_alu;
    assign issue_entries_dbg = issue_register;
    assign rs_alu_requests_dbg = rs_ready_alu;
    assign rs_mult_requests_dbg = rs_ready_mult;
    assign rs_branch_requests_dbg = rs_ready_branch;
    assign rs_mem_requests_dbg = rs_ready_mem;
    assign alu_clear_signals_dbg = fu_grants.alu;  // TEMP: ALU clear signals from CDB
    assign grants_alu_dbg = grants_alu;

    always_ff @(posedge clock) begin
        if (reset | mispredict) begin
            issue_register <= '0;
        end else begin
            issue_register <= issue_register_next;
        end
    end

endmodule
