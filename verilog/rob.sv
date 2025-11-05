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

    // Retire
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
        retire_count = '0;

        for (int i = 0; i < `N; i++) begin
            // Dispatch, assume incoming valid instructions to be contiguous from index 0
            if (rob_entry_packet[i].valid) begin
                rob_entries_next[(tail_idx+i)%`ROB_SZ] = rob_entry_packet[i];
            end

            // Complete ROB entries update
            if (rob_update_packet.valid[i]) begin
                rob_entries_next[rob_update_packet.idx[i]].complete = 1'b1;
                rob_entries_next[rob_update_packet.idx[i]].mispredict = rob_update_packet.mispredicts[i];
                rob_entries_next[rob_update_packet.idx[i]].branch_taken = rob_update_packet.branch_taken[i];
                rob_entries_next[rob_update_packet.idx[i]].branch_target = rob_update_packet.branch_targets[i];
            end

            // For determining whether to retire
            // head_entries[i] = rob_entries[(head_idx+i)%`ROB_SZ];
            if ((i == retire_count) && rob_entries[(head_idx + i) % `ROB_SZ].valid && rob_entries[(head_idx + i) % `ROB_SZ].complete) begin
                retire_count = retire_count + 1;
            end

            // Free Count calculation
            entry_packet_valid_bits[i] = rob_entry_packet[i].valid;
        end

        //num_retired = retire ? `N : 0;
        num_retired = retire_count;
        // Invalidate only the retired prefix at the head
        for (int i = 0; i < retire_count; i++) begin
            rob_entries_next[(head_idx + i) % `ROB_SZ].valid = 1'b0;
        end
        num_dispatched = $countones(entry_packet_valid_bits);
        free_count_next = free_count + num_retired - num_dispatched;

        // Head and tail pointers
        head_idx_next = (head_idx + retire_count) % `ROB_SZ;
        //    head_idx_next = retire ? ((head_idx + `N) % `ROB_SZ) : head_idx;
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


    // Expose head window: N-1 = oldest, 0 = youngest (matches stage_retire)
    // always_comb begin
    //     for (int i = 0; i < `N; i++) begin
    //         head_entries[`N-1 - i] = rob_entries[(head_idx + i) % `ROB_SZ];
    //         head_idxs[`N-1 - i]    = ROB_IDX'((head_idx + i) % `ROB_SZ);
    //         head_valids[`N-1 - i]  = rob_entries[(head_idx + i) % `ROB_SZ].valid;
    //     end
    // end

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

`ifndef SYNTH
always_ff @(posedge clock) begin
  if (!reset) begin
    $display("%0t | head=%0d tail=%0d retire_count=%0d dispatched=%0d",
             $time, head_idx, tail_idx, retire_count, $countones(entry_packet_valid_bits));
    for (int i = 0; i < `N; i++) begin
      $display("  H[%0d] idx=%0d valid=%0b complete=%0b",
               i,
               ((head_idx + i) % `ROB_SZ),
               rob_entries[((head_idx + i) % `ROB_SZ)].valid,
               rob_entries[((head_idx + i) % `ROB_SZ)].complete);
    end
  end
end
`endif

endmodule