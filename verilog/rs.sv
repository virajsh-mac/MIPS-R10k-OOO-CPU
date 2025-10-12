/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rs.sv                                               //
//                                                                     //
//  Description :  Reservation Station module; holds up to RS_SZ       //
//                 instructions waiting for operands to become ready.  //
//                 Supports allocation of new entries from dispatch,   //
//                 wakeup via CDB broadcasts from complete, clearing   //
//                 of issued entries from issue.                       //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"


module rs (
    input clock,  // system clock
    input reset,  // system reset

    // From dispatch: allocation signals
    input logic [`N-1:0] alloc_valid,       // Valid allocations this cycle
    input RS_ENTRY [`N-1:0] alloc_entries,  // N way allocating entries

    // From complete: CDB broadcasts for operand wakeup
    input CDB_PACKET cdb_broadcast,

    // From issue: clear signals for issued entries
    input logic  [`N-1:0] clear_valid,  // Valid clears this cycle
    input RS_IDX [`N-1:0] clear_idxs,   // RS indices to clear

    // From execute: mispredict flush signal
    input logic   mispredict,      // Mispredict detected (flush speculative)
    // input ROB_IDX mispred_rob_idx, ROB index of mispredicted branch used for EARLY BRANCH RESOLUTION

    // Outputs to issue/dispatch
    output RS_ENTRY [`RS_SZ-1:0] entries,  // Full RS entries for issue selection
    output logic [$clog2(`RS_SZ+1)-1:0] free_count  // Number of free entries (for dispatch stall)
);

    // Internal storage: array of RS entries
    RS_ENTRY [`RS_SZ-1:0] rs_array, rs_array_next;

    // Priority selector module that take turns dispatches to highest and lowest index entries in RS
    logic [`N-1:0][`RS_SZ-1:0] available_entries;
    logic [`RS_SZ-1:0] free_mask;
    psel_gen #(
        .WIDTH(`RS_SZ),
        .REQS(`N)
    ) priority_selector (
        .req(free_mask),
        .gnt(), // will need for back propagation
        .gnt_bus(available_entries),
        .empty()
    );

    // Combinational logic for updates: wakeup, clear, alloc, flush
    always_comb begin
        rs_array_next = rs_array;
        free_mask = `RS_SZ'b0;

        // Compute free_mask
        for (int i = 0; i < `RS_SZ; i++) begin
            free_mask[i] = !rs_array[i].valid;  // 1 for free slots
        end

        // Clear issued entries
        for (int i = 0; i < `N; i++) begin
            if (clear_valid[i]) begin
                rs_array_next[clear_idxs[i]].valid = 1'b0;
                free_mask[clear_idxs[i]] = 1'b1;
            end
        end

        // Allocate new entries from dispatch
        //
        // For each available entry that is valid and has a allocated index
        // assign the value to that index in the rs
        for (int i = 0; i < `N; i++) begin
            if (alloc_valid[i]) begin
                for (int j = 0; j < `RS_SZ; j++) begin
                    if (available_entries[i][j]) begin
                        rs_array_next[j] = alloc_entries[i];
                    end
                end
            end
        end

        // Wakeup operands via CDB (associative tag match)
        for (int i = 0; i < `RS_SZ; i++) begin
            if (rs_array[i].valid) begin
                for (int j = 0; j < `CDB_SZ; j++) begin
                    // Note that we theoretically don't care whether the existing
                    // bit is zero because we will only broadcast one physical
                    // register over the CDB that could ever correspond to a RS entry
                    // over its lifetime.
                    if (rs_array[i].src1_tag == cdb_broadcast.tags[j]) begin
                        rs_array_next[i].src1_ready = 1'b1;
                    end

                    if (rs_array[i].src2_tag == cdb_broadcast.tags[j]) begin
                        rs_array_next[i].src2_ready = 1'b1;
                    end
                end
            end
        end
    end

    logic [$clog2(`RS_SZ+1)-1:0] effective_free, next_effective_free;
    assign next_effective_free = effective_free + $countones(clear_valid) - $countones(alloc_valid);

    assign free_count = (effective_free > `N) ? `N : effective_free;  // Capped at N=3

    // Assign output entries
    assign entries = rs_array;

    // Clocked update
    always_ff @(posedge clock) begin
        if (reset | mispredict) begin
            effective_free <=`RS_SZ;

            for (int i = 0; i < `RS_SZ; i++) begin
                rs_array[i].valid <= 1'b0;
            end
        end else begin
            effective_free <=next_effective_free;
            rs_array <= rs_array_next;
        end
    end

endmodule  // rs
