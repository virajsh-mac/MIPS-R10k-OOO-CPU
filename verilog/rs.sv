/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  rs.sv                                                //
//                                                                     //
//  Description :  Reservation Station module; holds up to RS_SZ       //
//                 instructions waiting for operands to become ready.  //
//                 Supports allocation of new entries from dispatch,   //
//                 wakeup via CDB broadcasts from complete, clearing   //
//                 of issued entries from issue.                       //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"


module rs #(
    parameter ALLOC_WIDTH = `N,             // Number of allocation entries per cycle
    parameter RS_SIZE     = `RS_SZ,         // Reservation station size
    parameter CLEAR_WIDTH = `NUM_FU_TOTAL,  // Number of clear ports (functional units)
    parameter CDB_WIDTH   = `CDB_SZ         // Number of CDB broadcast ports
) (
    input clock,  // system clock
    input reset,  // system reset

    // From dispatch: allocation signals
    input logic    [ALLOC_WIDTH-1:0] alloc_valid,   // Valid allocations this cycle
    input RS_ENTRY [ALLOC_WIDTH-1:0] alloc_entries, // N way allocating entries

    // From complete: CDB broadcasts for operand wakeup
    input CDB_EARLY_TAG_ENTRY [CDB_WIDTH-1:0] early_tag_broadcast,

    // From issue: clear signals for issued entries
    input logic  [CLEAR_WIDTH-1:0] clear_valid,  // Valid clears this cycle
    input RS_IDX [CLEAR_WIDTH-1:0] clear_idxs,   // RS indices to clear

    // From execute: mispredict flush signal
    input logic mispredict,  // Mispredict detected (flush speculative)
    // input ROB_IDX mispred_rob_idx, ROB index of mispredicted branch used for EARLY BRANCH RESOLUTION

    // Outputs to issue/dispatch
    output RS_ENTRY [RS_SIZE-1:0] entries,  // Full RS entries for issue selection
    output logic [ALLOC_WIDTH-1:0][RS_SIZE-1:0] granted_entries  // Granted RS slots for each allocation request
);

    // Internal storage: array of RS entries
    RS_ENTRY [RS_SIZE-1:0] rs_array, rs_array_next;

    // Generic allocator for RS entries - handles allocation in parallel
    logic [RS_SIZE-1:0] clear_mask;

    // Convert clear signals to bit vector for allocator
    always_comb begin
        clear_mask = '0;
        for (int i = 0; i < CLEAR_WIDTH; i++) begin
            if (clear_valid[i]) begin
                clear_mask[clear_idxs[i]] = 1'b1;
            end
        end
    end

    allocator #(
        .NUM_RESOURCES(RS_SIZE),
        .NUM_REQUESTS (ALLOC_WIDTH)
    ) rs_allocator (
        .reset(reset | mispredict),
        .clock(clock),
        .req  (alloc_valid),
        .clear(clear_mask),
        .grant(granted_entries)
    );

    // Allocating and Freeing RS entries
    always_comb begin
        rs_array_next = rs_array;

        // Clear valid bits for entries that are being issued
        // Note: Clearing happens here for the RS array state, while the allocator
        // tracks resource availability separately via clear_mask
        for (int i = 0; i < CLEAR_WIDTH; i++) begin
            if (clear_valid[i]) begin
                rs_array_next[clear_idxs[i]].valid = 1'b0;
            end
        end

        // Allocate new RS entries from dispatch
        // The allocator module has already determined which slots are available,
        // so we just write to the granted slots
        for (int i = 0; i < ALLOC_WIDTH; i++) begin
            for (int j = 0; j < RS_SIZE; j++) begin
                if (granted_entries[i][j]) begin
                    rs_array_next[j] = alloc_entries[i];
                end
            end
        end

        // Wakeup operands via CDB (associative tag match)
        for (int i = 0; i < RS_SIZE; i++) begin
            for (int j = 0; j < CDB_WIDTH; j++) begin
                if (rs_array_next[i].valid &&
                    early_tag_broadcast[j].valid &&
                    rs_array_next[i].src1_tag == early_tag_broadcast[j].tag) begin
                    rs_array_next[i].src1_ready = 1'b1;
                end

                if (rs_array_next[i].valid &&
                    early_tag_broadcast[j].valid &&
                    rs_array_next[i].src2_tag == early_tag_broadcast[j].tag) begin
                    rs_array_next[i].src2_ready = 1'b1;
                end
            end
        end

    end

    // RS Output: expose entries array for issue selection
    assign entries = rs_array;

    always_ff @(posedge clock) begin
        if (reset | mispredict) begin
            rs_array <= '0;
        end else begin
            rs_array <= rs_array_next;
        end
    end

endmodule  // rs
