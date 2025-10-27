`include "sys_defs.svh"

module cdb (
    input  logic                  clock,
    input  logic                  reset,

    // CDB arbiter inputs
    input CDB_ENTRY [`NUM_FU_TOTAL-1:0] fu_outputs,

    // Broadcasting to Physical Register File, EX stage (data forwarding), and Map Table
    output CDB_ENTRY [`N-1:0] cdb_output;
);

logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus, gnt_bus_next;
logic [`NUM_FU_TOTAL-1:0] req_bus;
always_comb begin
    for (int i = 0; i < `NUM_FU_TOTAL; i++) begin
        req_bus[i] = fu_outputs[i].valid;
    end
end

psel_gen #(
    .WIDTH(`NUM_FU_TOTAL),
    .REQS(`N)
) cdb_arbiter (
    .req(req_bus),
    .gnt_bus(gnt_bus_next),
);

CDB_ENTRY [`N-1:0] cdb, cdb_next;
always_comb begin
    cdb_next = '0;
    for (int i = 0; i < `N; i++) begin
        for (int j = 0; j < `NUM_FU_TOTAL; j++) begin
            cdb_next |= {$bits(CDB_ENTRY){gnt_bus_next[i][j]}} & fu_outputs[j];
        end
    end
end

always_ff @(posedge clock) begin
    if (reset) begin
        gnt_bus <='0;
        cdb <= '0;
    end else begin
        gnt_bus <= gnt_bus_next;
        cdb <=cdb_next;
    end
end

assign cdb_output = cdb;

endmodule
