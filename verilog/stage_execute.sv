`include "sys_defs.svh"

module stage_execute (
    input clock,
    input reset,

    input logic mispredict,  // Flush pipelines on mispredict

    // Inputs from issue stage
    input logic [`N-1:0] issue_valid,
    input RS_ENTRY [`N-1:0] issued_entries,

    // Input from CDB
    input DATA cdb_forwarding,

    // To PRF for operand reads (up to 2 per issued instruction)
    output logic [2*`N-1:0] prf_read_en,
    output PHYS_TAG [2*`N-1:0] prf_read_tag,
    input DATA [2*`N-1:0] prf_read_data, // physical register file read data

    // // Interface to D-cache for memory operations (IGNORE FOR NOW)
    // output MEM_COMMAND proc2Dcache_command [`NUM_FU_MEM-1:0],
    // output ADDR proc2Dcache_addr [`NUM_FU_MEM-1:0],
    // output DATA proc2Dcache_data [`NUM_FU_MEM-1:0],  // For stores
    // output MEM_SIZE proc2Dcache_size [`NUM_FU_MEM-1:0],
    // input logic [`NUM_FU_MEM-1:0] Dcache_valid,
    // input DATA [`NUM_FU_MEM-1:0] Dcache_data,  // For loads

    // Outputs to issue stage: FU availability
    output logic [`NUM_FU_ALU-1:0] alu_avail,
    output logic [`NUM_FU_MULT-1:0] mult_avail,
    output logic [`NUM_FU_BRANCH-1:0] branch_avail,
    output logic [`NUM_FU_ADDR-1:0] addr_avail,
    output logic [`NUM_FU_MEM-1:0] mem_avail,

    // Outputs to execute/complete pipeline register (for complete stage)
    output logic [`N-1:0] ex_comp_valid,
    output ROB_IDX [`N-1:0] ex_comp_rob_idxs,
    output PHYS_TAG [`N-1:0] ex_comp_dest_tags, // maybe we can delete?
    output DATA [`N-1:0] ex_comp_values, // maybe we can delete?
    output logic [`N-1:0] mispredict,
    output logic [`N-1:0] ex_comp_branch_valid, // maybe we can delete?
    output logic [`N-1:0] ex_comp_branch_taken, , // maybe we can delete?
    output ADDR [`N-1:0] ex_comp_branch_targets, // maybe we can delete?

    // For early tag broadcast (advanced feature)
    output CDB_PACKET early_cdb_broadcast  // Tags broadcast early for wakeup
);


endmodule
