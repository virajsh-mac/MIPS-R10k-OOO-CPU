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
//                 ready entries preferring lower indices.             //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// RS entry structure is now defined in sys_defs.svh

module rs (
    input clock,  // system clock
    input reset,  // system reset

    // From dispatch: allocation signals
    input logic [`N-1:0] alloc_valid,  // Valid allocations this cycle
    input RS_ENTRY [`N-1:0] alloc_entries,  // New entries to allocate

    // From complete: CDB broadcasts for operand wakeup
    input CDB_PACKET cdb_broadcast,

    // From issue: clear signals for issued entries
    input logic  [`N-1:0] clear_valid,  // Valid clears this cycle
    input RS_IDX [`N-1:0] clear_idxs,   // RS indices to clear

    // From execute: mispredict flush signal
    input logic   mispredict,      // Mispredict detected (flush speculative)
    input ROB_IDX mispred_rob_idx, // ROB index of mispredicted branch TODO: maybe delete?

    // Outputs to issue/dispatch
    output RS_ENTRY [`RS_SZ-1:0] entries,  // Full RS entries for issue selection
    output logic [$clog2(`RS_SZ+1)-1:0] free_count  // Number of free entries (for dispatch stall)
);

    // Internal storage: array of RS entries
    RS_ENTRY [`RS_SZ-1:0] rs_array, rs_array_next;

    // Priority selector module that take turns dispatches to highest and lowest index entries in RS
    // input:
    //
    // output:
    // available_entries
    logic [`N-1:0][`RS_SZ-1:0] available_entries;
    psel_gen #(
        .WIDTH(RS_SZ),
        .REQS(`N),
    ) priority_selector (
        .req(alloc_valid),
        .gnt(), // will need for back propogation
        .gnt_bus(available_entries),
    );

    // Combinational logic for updates: wakeup, clear, alloc, flush
    always_comb begin
        rs_array_next = rs_array;

        // Step 1: Allocate new entries from dispatch
        //
        // For each availiable entry that is valid and has a allocated index
        // assign the value to that index in the rs
        for (int i = 0; i < `N; i++) begin
            for (int j = 0; j < `RS; j++) begin
                if (available_entries[i][j]) begin
                    rs_array_next[j] = alloc_entries[i];
                end
            end
        end

        // Step 2: Wakeup operands via CDB (associative tag match)
        for (int i = 0; i < `RS_SZ; i++) begin
            if (rs_array[i].valid) begin
                for (int j = 0; j < `CDB_SZ; j++) begin
                    // Note that we theoretically dont care wether the existing
                    // bit is zero because we will only broadcast one physical
                    // register over the CDB that could ever correspond to a RS entry
                    // over its lifetime.
                    if (rs_array[i].src1_tag == cdb_braodcast.tags[j]) begin
                        rs_array_next[i].src1_ready = 1'b1;
                    end

                    if rs_array[i].src2_tag == cdb_braodcast.tags[j]) begin
                        rs_array_next[i].src2_ready = 1'b1;
                    end
                end
            end
        end

        // Step 3: Clear issued entries
        for (int i = 0; i < `N; i++) begin
            if (clear_valid[k]) begin
                rs_array_next[clear_idxs[k]].valid = 1'b0;
            end
        end

        // Step 4: Flush speculative entries on mispredict
        // TODO will be determined by our exception handling strategy
    end

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

endmodule  // rs
