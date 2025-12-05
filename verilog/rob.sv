/////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                             //
//  Modulename :  rob.sv                                                                       //
//                                                                                             //
//  Description :  Reorder Buffer module;                                                      //
//                TODO: Dispatch interface requires valid instructions to be contiguous from   //
//                index 0 (packed left, no gaps). E.g., for 3 valid out of `N`=5: [1 1 1 0 0]  //
//                is valid, but [1 0 1 0 1] is invalid. This simplifies ROB insertion logic.   //
//                                                                                             //   
//                1. No retire on current instructions completed this cycle for clock period   //
//                2. Write to Physical Register File in complete stage                         //
//                3. Retire a longest in-order prefix (0..N) of {valid && complete}            //
//                   entries at the head.                                                      //
//                4. free_count_next, head_idx_next, tail_idx_next are computed                //
//                   from retired/dispatch counts                                              //
/////////////////////////////////////////////////////////////////////////////////////////////////
`include "sys_defs.svh"

module rob (
    input logic clock,
    input logic reset,  // reset on mispredict

    // Dispatch
    input  ROB_ENTRY [               `N-1:0] rob_entry_packet,
    output logic     [$clog2(`ROB_SZ+1)-1:0] free_slots,
    output ROB_IDX   [               `N-1:0] alloc_idxs,        // Allocation indices

    // Complete
    input ROB_UPDATE_PACKET rob_update_packet,

    // Retire (controlled by stage_retire)
    input logic [$clog2(`N+1)-1:0] retire_count_in,  // How many to actually retire (from stage_retire)
    output ROB_ENTRY [`N-1:0] head_entries,  // Could be retired
    output ROB_IDX   [`N-1:0] head_idxs,     // Head entry indices
    output logic     [`N-1:0] head_valids    // Head entry valid flags
);
    ROB_ENTRY [`ROB_SZ-1:0] rob_entries, rob_entries_next;
    logic [$clog2(`ROB_SZ+1)-1:0] free_count, free_count_next;
    logic [`ROB_IDX_BITS-1:0] head_idx, head_idx_next, tail_idx, tail_idx_next;
    logic [`N-1:0] entry_packet_valid_bits;

    // For calculating free count
  //  logic retire;
  //  logic [`N-1:0] next_N_complete_bits;
    logic [$clog2(`N+1)-1:0] num_retired, num_dispatched;
    // Prefix-retire bookkeeping
    logic [$clog2(`N+1)-1:0] retire_count;

    always_comb begin
        free_count_next = free_count;
        rob_entries_next = rob_entries;
        retire_count = retire_count_in;

        // 1. Retire: Invalidate the retired prefix at the head FIRST
        //    (Must happen before dispatch so dispatch can reuse freed slots)
        for (int i = 0; i < `N; i++) begin
            if (i < retire_count) begin
                rob_entries_next[(head_idx + i) % `ROB_SZ].valid = 1'b0;
            end
        end

        // 2. Dispatch: Write new entries (overwrites any just-invalidated slots)
        for (int i = 0; i < `N; i++) begin
            if (rob_entry_packet[i].valid) begin
                rob_entries_next[(tail_idx + i) % `ROB_SZ] = rob_entry_packet[i];
            end
            entry_packet_valid_bits[i] = rob_entry_packet[i].valid;
        end

        // 3. Complete: Update ROB entries
        for (int i = 0; i < `N; i++) begin
            if (rob_update_packet.valid[i]) begin
                rob_entries_next[rob_update_packet.idx[i]].complete = 1'b1;
                rob_entries_next[rob_update_packet.idx[i]].branch_taken = rob_update_packet.branch_taken[i];
                rob_entries_next[rob_update_packet.idx[i]].branch_target = rob_update_packet.branch_targets[i];
            end
        end

        // 4. Update counters and pointers
        num_retired = retire_count;
        num_dispatched = $countones(entry_packet_valid_bits);
        free_count_next = free_count + num_retired - num_dispatched;
        head_idx_next = (head_idx + retire_count) % `ROB_SZ;
        tail_idx_next = (tail_idx + num_dispatched) % `ROB_SZ;
    end

    // Generate allocation indices (tail + i)
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            alloc_idxs[i] = ROB_IDX'((tail_idx + i) % `ROB_SZ);
        end
    end

    // Expose head window: 0 = oldest, N-1 = youngest
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            head_entries[i] = rob_entries[(head_idx + i) % `ROB_SZ];
            head_idxs[i]    = ROB_IDX'((head_idx + i) % `ROB_SZ);
            head_valids[i]  = rob_entries[(head_idx + i) % `ROB_SZ].valid;
        end
    end

    assign free_slots = free_count;

    always_ff @(posedge clock) begin
        if (reset) begin
            rob_entries <= '0;
            free_count <= `ROB_SZ;
            head_idx <= '0;
            tail_idx <= '0;
        end else begin
            rob_entries <= rob_entries_next;
            head_idx <= head_idx_next;
            tail_idx <= tail_idx_next;
            free_count <= free_count_next;
        end
    end

`ifdef DEBUG
    always_ff @(posedge clock) begin
        if (!reset) begin
            $display("========================================");
            $display("=== ROB STATE (Cycle %0t) ===", $time);
            $display("========================================");
            
            // Pointers and counters
            $display("--- Pointers ---");
            $display("  Head: %0d, Tail: %0d, Free Slots: %0d", head_idx, tail_idx, free_count);
            $display("  Retire Count: %0d, Dispatched: %0d", retire_count, $countones(entry_packet_valid_bits));
            
            // ROB entries
            $display("--- ROB Entries ---");
            for (int i = 0; i < `ROB_SZ; i++) begin
                automatic ROB_ENTRY entry = rob_entries[i];
                if (entry.valid) begin
                    if (entry.branch) begin
                        $display("  [%2d] PC=%h Inst=%h rd=x%0d->P%0d Complete=%0d Branch(taken=%0d tgt=%h) %s%s%s",
                                 i, entry.PC, entry.inst, entry.arch_rd, entry.phys_rd, entry.complete,
                                 entry.branch_taken, entry.branch_target,
                                 (i == head_idx) ? "<-HEAD" : "",
                                 (i == tail_idx) ? "<-TAIL" : "",
                                 entry.halt ? " HALT" : "");
                    end else begin
                        $display("  [%2d] PC=%h Inst=%h rd=x%0d->P%0d Complete=%0d Val=%h %s%s%s",
                                 i, entry.PC, entry.inst, entry.arch_rd, entry.phys_rd, entry.complete,
                                 entry.value,
                                 (i == head_idx) ? "<-HEAD" : "",
                                 (i == tail_idx) ? "<-TAIL" : "",
                                 entry.halt ? " HALT" : "");
                    end
                end else begin
                    $display("  [%2d] Valid=0 %s%s", i,
                             (i == head_idx) ? "<-HEAD" : "",
                             (i == tail_idx) ? "<-TAIL" : "");
                end
            end
            
            // Dispatch inputs
            $display("--- Dispatch Inputs ---");
            for (int i = 0; i < `N; i++) begin
                if (rob_entry_packet[i].valid) begin
                    $display("  Dispatch[%0d]: PC=%h Inst=%h rd=x%0d->P%0d -> Alloc Idx=%0d",
                             i, rob_entry_packet[i].PC, rob_entry_packet[i].inst,
                             rob_entry_packet[i].arch_rd, rob_entry_packet[i].phys_rd, alloc_idxs[i]);
                end
            end
            
            // Complete updates
            $display("--- Complete Updates ---");
            for (int i = 0; i < `N; i++) begin
                if (rob_update_packet.valid[i]) begin
                    $display("  Complete[%0d]: ROB_Idx=%0d Branch(taken=%0d tgt=%h)",
                             i, rob_update_packet.idx[i],
                             rob_update_packet.branch_taken[i], rob_update_packet.branch_targets[i]);
                end
            end
            
            // Retire outputs
            $display("--- Retire ---");
            for (int i = 0; i < `N; i++) begin
                if (head_valids[i] && (i < retire_count)) begin
                    $display("  Retiring[%0d]: ROB_Idx=%0d PC=%h rd=x%0d->P%0d",
                             i, head_idxs[i], head_entries[i].PC,
                             head_entries[i].arch_rd, head_entries[i].phys_rd);
                end
            end
            
            $display("");
        end
    end
`endif

endmodule