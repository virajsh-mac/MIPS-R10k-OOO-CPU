`include "sys_defs.svh"

// Memory module: compute store addresses and data for store instructions
// TODO currently only works for stores add load instruction functionality
// Purely combinational
module mem_fu (
    input  DATA rs1,         // Base register for address
    input  DATA rs2,         // Data to store
    input  DATA imm,         // Immediate offset from instruction

    output DATA addr,        // Effective store address
    output DATA data         // Value to store
);

    always_comb begin
        addr = rs1 + imm;   // Compute effective address
        data = rs2;         // Pass store data through
    end

endmodule  // mem