`include "sys_defs.svh"

module cdb (
    input logic clock,
    input logic reset,

    // Arbiter inputs (structured)
    input FU_REQUESTS requests,

    // Arbiter outputs indicating which requests are going on the CDB
    // (the next cycle i.e. what are you allowed to issue to the issue register)
    output FU_GRANTS grants,

    // Grant bus output to EX for metadata selection to complete
    output logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus_out,

    // CDB inputs (structured)
    input CDB_FU_OUTPUTS fu_outputs,

    // CDB output indicating which tags should be awoken a cycle early
    output CDB_EARLY_TAG_ENTRY [`N-1:0] early_tags,

    // CDB register outputs broadcasting to Physical Register File, EX stage (data forwarding), and Map Table
    output CDB_ENTRY [`N-1:0] cdb_output,

    // Debug outputs
    output FU_REQUESTS requests_dbg,
    output CDB_FU_OUTPUTS fu_outputs_dbg,
    output logic [`NUM_FU_TOTAL-1:0] grants_flat_dbg,
    output logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus_dbg,
    output CDB_EARLY_TAG_ENTRY [`N-1:0] early_tags_dbg
);

    logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus, gnt_bus_next;
    logic [`NUM_FU_TOTAL-1:0] grants_flat, grants_flat_next;

    // Flatten fu_outputs in PRIORITY ORDER: BRANCH (highest), ALU, MEM, MULT (lowest)
    // This ordering MUST match the psel_gen request order for correct arbitration
    CDB_ENTRY [`NUM_FU_TOTAL-1:0] fu_outputs_flat;
    assign fu_outputs_flat = {fu_outputs.mult, fu_outputs.mem, fu_outputs.alu, fu_outputs.branch};

    psel_gen #(
        .WIDTH(`NUM_FU_TOTAL),  // 6
        .REQS (`N)              // 3
    ) cdb_arbiter (
        // CRITICAL: Priority order is BRANCH (highest), ALU, MEM, MULT (lowest)
        // This concatenation order determines arbitration priority
        .req({requests.mult, requests.mem, requests.alu, requests.branch}),
        .gnt(grants_flat_next),
        .gnt_bus(gnt_bus_next)
    );

    CDB_ENTRY [`N-1:0] cdb, cdb_next;
    always_comb begin
        cdb_next = '0;
        for (int i = 0; i < `N; i++) begin
            for (int j = 0; j < `NUM_FU_TOTAL; j++) begin
                cdb_next[i] |= gnt_bus[i][j] ? fu_outputs_flat[j] : '0;
            end
        end
    end

    always_comb begin
        for (int k = 0; k < `N; k++) begin
            early_tags[k].valid = cdb_next[k].valid;
            early_tags[k].tag   = cdb_next[k].tag;
        end
    end

    // Unflatten grants back to structured format, maintaining same order
    // grants_flat order: [mult, mem, alu, branch] (same as request concatenation)
    assign {grants.mult, grants.mem, grants.alu, grants.branch} = grants_flat;
    assign cdb_output = cdb;
    //assign gnt_bus_next = gnt_bus;

    always_ff @(posedge clock) begin
        if (reset) begin
            gnt_bus <= '0;
            grants_flat <= '0;
            cdb <= '0;
        end else begin
            gnt_bus <= gnt_bus_next;
            grants_flat <= grants_flat_next;
            cdb <= cdb_next;
        end
    end

    // Connect gnt_bus to output
    assign gnt_bus_out = gnt_bus;

    // Debug assignments
    assign requests_dbg = requests;
    assign fu_outputs_dbg = fu_outputs;
    assign grants_flat_dbg = grants_flat;
    assign gnt_bus_dbg = gnt_bus;
    assign early_tags_dbg = early_tags;

endmodule
