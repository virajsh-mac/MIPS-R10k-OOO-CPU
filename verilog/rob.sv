/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Reorder Buffer module; manages up to ROB_SZ         //
//                 in-flight instructions for in-order commit,         //
//                 branch misprediction recovery, and exception        //
//                 handling. Acts as a circular buffer with head       //
//                 (oldest) and tail (next alloc) pointers.            //
//                 Allocates entries from Dispatch, updates from       //
//                 Complete (complete bit, value, branch info),        //
//                 provides head entries to Retire, advances head      //
//                 on retire, and truncates on mispredict flush.       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "sys_defs.svh"

// Parameters and typedefs are now centrally defined in sys_defs.svh

// ROB entry structure is now defined in sys_defs.svh

// ROB update packet (extended for branch info)
typedef struct packed {
    logic [`N-1:0] valid;      // Valid updates this cycle
    ROB_IDX [`N-1:0] idx;      // ROB indices to update
    DATA [`N-1:0] values;      // Values to store (if applicable)
    logic [`N-1:0] branch_taken;  // Resolved taken/not taken (if branch)
    ADDR [`N-1:0] branch_targets; // Resolved branch targets (if branch)
} ROB_UPDATE_PACKET;

module rob (
    input              clock,           // system clock
    input              reset,           // system reset

    // Allocation from Dispatch
    input logic [`N-1:0] alloc_valid,   // Valid allocations this cycle
    input ROB_ENTRY [`N-1:0] alloc_entries,  // New entries (partial fields set by Dispatch)
    output ROB_IDX [`N-1:0] alloc_idxs,  // Assigned ROB indices for new entries
    output logic [$clog2(`ROB_SZ+1)-1:0] free_slots,  // Number of free slots (for stall check)

    // Updates from Complete
    input ROB_UPDATE_PACKET update,     // Updates for complete bit, value, and branch info

    // Retire interface
    output ROB_ENTRY [`N-1:0] head_entries,  // Up to N consecutive head entries for Retire
    output logic [`N-1:0] head_valids,   // Valid bits for each head entry
    input logic [`ROB_IDX_BITS:0] retire_count,  // Number of instructions to retire (0 to N)

    // Flush on mispredict (from Execute or Retire)
    input logic mispredict,             // Flush signal
    input ROB_IDX mispred_idx           // ROB index of mispredicted branch (truncate after this)
);

    // Internal storage: circular buffer of entries
    ROB_ENTRY [`ROB_SZ-1:0] rob_array;

    // Head (oldest) and tail (next allocation) pointers
    ROB_IDX head, tail;

    // Combinational: compute free slots
    always_comb begin
        free_slots = `ROB_SZ - ((tail - head) % `ROB_SZ);
    end

    // Combinational: assign allocation indices starting from tail
    always_comb begin
        ROB_IDX current_idx = tail;
        for (int i = 0; i < `N; i++) begin
            alloc_idxs[i] = current_idx;
            if (alloc_valid[i]) begin
                current_idx = (current_idx + 1) % `ROB_SZ;
            end
        end
    end

    // Combinational: output head entries and valids
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            ROB_IDX idx = (head + i) % `ROB_SZ;
            head_entries[i] = rob_array[idx];
            // Valid if within committed range and entry is valid
            head_valids[i] = ((tail - head) % `ROB_SZ > i) && rob_array[idx].valid;
        end
    end

    // Next state logic (combinational)
    ROB_ENTRY [`ROB_SZ-1:0] rob_next;
    ROB_IDX head_next, tail_next;
    always_comb begin
        rob_next = rob_array;
        head_next = head;
        tail_next = tail;

        // Priority: handle mispredict flush
        if (mispredict) begin
            tail_next = (mispred_idx + 1) % `ROB_SZ;
            // No need to invalidate entries explicitly; overwriting on future alloc suffices
        end else begin
            // Allocation: write new entries at alloc_idxs
            for (int i = 0; i < `N; i++) begin
                if (alloc_valid[i]) begin
                    ROB_IDX idx = alloc_idxs[i];
                    rob_next[idx] = alloc_entries[i];
                    rob_next[idx].valid = 1'b1;
                    rob_next[idx].complete = 1'b0;
                    rob_next[idx].exception = NO_ERROR;
                end
            end
            // Advance tail by number allocated
            int alloc_cnt = 0;
            for (int i = 0; i < `N; i++) alloc_cnt += alloc_valid[i];
            tail_next = (tail + alloc_cnt) % `ROB_SZ;

            // Updates from Complete: set complete, value, and branch info
            for (int i = 0; i < `N; i++) begin
                if (update.valid[i]) begin
                    ROB_IDX idx = update.idx[i];
                    rob_next[idx].complete = 1'b1;
                    rob_next[idx].value = update.values[i];
                    if (rob_next[idx].branch) begin
                        rob_next[idx].branch_taken = update.branch_taken[i];
                        rob_next[idx].branch_target = update.branch_targets[i];
                    end
                end
            end

            // Retire: advance head, invalidate retired entries (optional)
            head_next = (head + retire_count) % `ROB_SZ;
            for (int i = 0; i < retire_count; i++) begin
                if (i < `N) begin  // Guard against over-retire
                    ROB_IDX idx = (head + i) % `ROB_SZ;
                    rob_next[idx].valid = 1'b0;
                end
            end
        end
    end

    // Clocked update
    always_ff @(posedge clock) begin
        if (reset) begin
            head <= 0;
            tail <= 0;
            for (int i = 0; i < `ROB_SZ; i++) begin
                rob_array[i].valid <= 1'b0;
            end
        end else begin
            head <= head_next;
            tail <= tail_next;
            rob_array <= rob_next;
        end
    end

endmodule // rob