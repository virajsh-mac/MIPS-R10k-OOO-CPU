/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_complete.sv                                   //
//                                                                     //
//  Description :  Complete stage of the pipeline; broadcasts up to    //
//                 3 results on CDBs to wake up dependents in the RS,  //
//                 sets ready bits in the map table for physical dest  //
//                 registers, marks ROB entries as done with stored    //
//                 values. Handles all completing instructions from    //
//                 Execute (including multi-cycle ops like mul, which  //
//                 are output from their pipelines in Execute).        //
//                 Demonstrates out-of-order completion by allowing    //
//                 instructions to finish in any order but ensuring    //
//                 in-order commit via ROB. No decode here (handled   //
//                 in Fetch).                                         //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// Packet from Execute to Complete (results for CDB broadcast and ROB update)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid results this cycle
    DATA [`N-1:0] results;     // Computed values (ALU/mult/load result)
    PHYS_TAG [`N-1:0] dest_tags;  // Physical dest tags for broadcast
    ROB_IDX [`N-1:0] rob_idxs; // ROB indices for complete marking
} EX_COMP_PACKET;

// ROB update packet (to mark done and store value in ROB entries)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid updates this cycle
    ROB_IDX [`N-1:0] idx;      // ROB indices to update
    DATA [`N-1:0] values;      // Values to store in ROB
} ROB_UPDATE_PACKET;

// No direct packet to Retire (Retire checks ROB head independently)
// If needed for custom retire signaling, could add COMP_RET_PACKET
// typedef struct packed {
//     logic [`N-1:0] valid;      // Completed this cycle (for debug/monitoring)
//     ROB_IDX [`N-1:0] rob_idxs; // Completed ROB indices
// } COMP_RET_PACKET;

module stage_complete (
    input              clock,           // system clock
    input              reset,           // system reset

    // From Execute: completed results bundle
    input EX_COMP_PACKET execute_packet,

    // From Execute: mispredict signal (suppress broadcasts/updates if flush in progress)
    input logic        mispredict,

    // To RS/Map Table/Issue: CDB broadcasts for wakeup and ready bit updates
    output CDB_PACKET  cdb_broadcast,

    // To ROB: updates for done bits and stored values
    output ROB_UPDATE_PACKET rob_update

    // No output to Retire (Retire polls ROB head)
    // output COMP_RET_PACKET complete_packet  // Optional for debug
);

    // Internal signals
    logic [`CDB_SZ-1:0] broadcast_en;  // Enable broadcast per CDB (only if dest_tag valid)
    integer cdb_idx;  // Index for assigning to CDBs

    // Main complete logic (combinational; parallel broadcast, sequential per-inst processing)
    always_comb begin
        // Defaults
        cdb_broadcast = '0;
        rob_update = '0;
        // complete_packet = '0;  // Optional
        broadcast_en = '0;
        cdb_idx = 0;

        if (!mispredict) begin  // Suppress on mispredict to avoid corrupting state during flush
            for (int i = 0; i < `N; i++) begin
                if (execute_packet.valid[i]) begin
                    // Always update ROB for all completing insts (even no-dest like stores/branches)
                    rob_update.valid[i] = 1'b1;
                    rob_update.idx[i] = execute_packet.rob_idxs[i];
                    rob_update.values[i] = execute_packet.results[i];

                    // Optional: Signal to retire for monitoring
                    // complete_packet.valid[i] = 1'b1;
                    // complete_packet.rob_idxs[i] = execute_packet.rob_idxs[i];

                    // Broadcast on CDB only if valid dest (phys tag != 0, assuming 0 invalid)
                    if (execute_packet.dest_tags[i] != '0) begin
                        broadcast_en[cdb_idx] = 1'b1;
                        cdb_broadcast.valid[cdb_idx] = 1'b1;
                        cdb_broadcast.tags[cdb_idx] = execute_packet.dest_tags[i];
                        cdb_broadcast.values[cdb_idx] = execute_packet.results[i];
                        cdb_idx++;
                    end
                end
            end
        end
    end

    // Note: Wakeup logic is in RS (comparators per entry: if src_tag == cdb_tag && cdb_valid, set ready/value)
    // Map table update: for each phys reg, if tag == cdb_tag && cdb_valid, set ready bit
    // (Map table module would subscribe to cdb_broadcast)

    // No clocked logic needed (combinational stage; ROB/RS/map updates combo or FF in their modules)

endmodule // stage_complete