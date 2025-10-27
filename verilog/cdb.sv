`include "sys_defs.svh"

module cdb (
    input  logic                  clock,
    input  logic                  reset,

    // Arbiter inputs
    input logic [(`NUM_FU_BRANCH-1:0] branch_requests, // 1 highest priority
    input logic [(`NUM_FU_ALU-1:0] alu_requests, // 3
    input logic [(`NUM_FU_MEM-1:0] mem_requests, // 4 lowest priority
    input logic [(`NUM_FU_MULT-1:0] mult_requests, // 2

    // Arbiter outputs indicating which requests are going on the CDB
    // (the next cycle i.e. what are you allowed to issue to the issue register)
    input logic [(`NUM_FU_BRANCH-1:0] branch_grants, // 1 highest priority
    input logic [(`NUM_FU_ALU-1:0] alu_grants, // 3
    input logic [(`NUM_FU_MEM-1:0] mem_grants, // 4 lowest priority
    input logic [(`NUM_FU_MULT-1:0] mult_grants, // 2

    // CDB inputs
    input CDB_ENTRY [`NUM_FU_TOTAL-1:0] fu_outputs, // ordered as above

    // CDB output indicating which tags should be awoken a cycle early
    output CDB_EARLY_TAG_ENTRY [`N-1:0] early_tags,

    // CDB register outputs broadcasting to Physical Register File, EX stage (data forwarding), and Map Table
    output CDB_ENTRY [`N-1:0] cdb_output;
);

logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus, gnt_bus_next;
logic [`NUM_FU_TOTAL-1:0] grants;
psel_gen #(
    .WIDTH(`NUM_FU_TOTAL),   // 6
    .REQS(`N)                // 2
) cdb_arbiter (
    // Priority on the request bus alternates between highest and lowest
    .req('{branch_requests, alu_requests, mem_requests, mult_requests}),
    .gnt(grants),
    .gnt_bus(gnt_bus_next)
);

CDB_ENTRY [`N-1:0] cdb, cdb_next;
always_comb begin
    cdb_next = '0;
    for (int i = 0; i < `N; i++) begin
        for (int j = 0; j < `NUM_FU_TOTAL; j++) begin
            cdb_next[i] |= gnt_bus[i][j] ? fu_outputs[j] : '0;
        end
    end
end

always_comb begin
    for (int k = 0; k < `N; k++) begin
        early_tags[k].valid = cdb_next[k].valid;
        early_tags[k].tag = cdb_next[k].tag;
    end
end

assign '{branch_grants, alu_grants, mem_grants, mult_grants} = grants;
assign cdb_output = cdb;

always_ff @(posedge clock) begin
    if (reset) begin
        gnt_bus <='0;
        cdb <= '0;
    end else begin
        gnt_bus <= gnt_bus_next;
        cdb <=cdb_next;
    end
end

endmodule
