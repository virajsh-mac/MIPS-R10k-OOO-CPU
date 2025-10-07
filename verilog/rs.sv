/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rs.sv                                               //
//                                                                     //
//  Description :  Reservation Station module; holds up to RS_SZ       //
//                 instructions waiting for operands to become ready.  //
//                 Supports allocation of new entries from dispatch,   //
//                 wakeup via CDB broadcasts from complete, clearing   //
//                 of issued entries from issue, and flushing of       //
//                 speculative entries on branch mispredictions.       //
//                 Entries are allocated to the lowest available       //
//                 indices to approximate age ordering. Issue selects  //
//                 ready entries preferring lower indices.            //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// RS entry structure (extended for full control signals)
typedef struct packed {
    logic valid;               // Entry occupied
    ALU_OPA_SELECT opa_select; // From decode (where is OPA coming from)
    ALU_OPB_SELECT opb_select; // From decode (where is OPB coming from)
    OP_TYPE op_type;           // Which unit are we routing to in EX and what suboperation
    PHYS_TAG src1_tag;         // Physical source 1 tag
    logic src1_ready;          // Source 1 ready
    DATA src1_value;           // Source 1 value if immediate
    PHYS_TAG src2_tag;         // Physical source 2 tag
    logic src2_ready;          // Source 2 ready
    DATA src2_value;           // Source 2 value if immediate
    PHYS_TAG dest_tag;         // Physical destination tag
    ROB_IDX rob_idx;           // Associated ROB index (for flush and potential age selection)
    ADDR PC;                   // PC for branch/debug (MIGHT merge with SRC but only if we can resolve mispredicts othersive)
    // Added for branches (prediction info from fetch via dispatch)
    logic pred_taken;
    ADDR pred_target;
} RS_ENTRY;

// CDB packet (from complete, for wakeup)
typedef struct packed {
    logic [`CDB_SZ-1:0] valid;  // Valid broadcasts this cycle
    PHYS_TAG [`CDB_SZ-1:0] tags;  // Physical dest tags
} CDB_PACKET;

module rs (
    input              clock,           // system clock
    input              reset,           // system reset

    // From dispatch: allocation signals
    input logic [`N-1:0] alloc_valid,   // Valid allocations this cycle
    input RS_ENTRY [`N-1:0] alloc_entries,  // New entries to allocate

    // From complete: CDB broadcasts for operand wakeup
    input CDB_PACKET   cdb_broadcast,

    // From issue: clear signals for issued entries
    input logic [`N-1:0] clear_valid,   // Valid clears this cycle
    input RS_IDX [`N-1:0] clear_idxs,   // RS indices to clear

    // From execute: mispredict flush signal
    input logic        mispredict,      // Mispredict detected (flush speculative)
    input ROB_IDX      mispred_rob_idx, // ROB index of mispredicted branch

    // Outputs to issue/dispatch
    output RS_ENTRY [`RS_SZ-1:0] entries,  // Full RS entries for issue selection
    output logic [$clog2(`RS_SZ+1)-1:0] free_count  // Number of free entries (for dispatch stall)
);

    // Internal storage: array of RS entries
    RS_ENTRY [`RS_SZ-1:0] rs_array, rs_array_next;

    // Combinational logic for free count (number of free RS entries)

    // Sequential logic for updates: wakeup, clear, alloc, flush

        // Step 1: Wakeup operands via CDB (associative tag match)
        // Step 2: Clear issued entries
        // Step 3: Allocate new entries into lowest free indices
        // Step 4: Flush speculative entries on mispredict

    // Clocked update
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i = 0; i < `RS_SZ; i++) begin
                rs_array[i].valid <= 1'b0;
            end
        end else begin
            rs_array <= rs_array_next;
        end
    end

endmodule // rs