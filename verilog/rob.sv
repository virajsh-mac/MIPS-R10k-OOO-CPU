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
//                3. Only retire when all `N after head pointers are marked as complete        //
//                4. free_count_next, head_idx_next, tail_idx_next are calculated based on     //
//                the number of retired and dispatched instructions                            //
/////////////////////////////////////////////////////////////////////////////////////////////////
`include "sys_defs.svh"

module rob (
    input logic clock,
    input logic reset,  // reset on mispredict

    // Dispatch
    input  ROB_ENTRY [               `N-1:0] rob_entry_packet,
    output logic     [$clog2(`ROB_SZ+1)-1:0] free_slots,

    // Complete
    input ROB_UPDATE_PACKET rob_update_packet,

    // Retire
    output ROB_ENTRY [`N-1:0] head_entries  // Could be retired
);
    ROB_ENTRY [`ROB_SZ-1:0] rob_entries, rob_entries_next;
    logic [$clog2(`ROB_SZ+1)-1:0] free_count, free_count_next;
    logic [`ROB_IDX_BITS-1:0] head_idx, head_idx_next, tail_idx, tail_idx_next;
    logic [`N-1:0] entry_packet_valid_bits;

    // For calculating free count
    logic retire;
    logic [`N-1:0] next_N_complete_bits;
    logic [$clog2(`N+1)-1:0] num_retired, num_dispatched;

    assign retire = &next_N_complete_bits;

    always_comb begin
        free_count_next = free_count;
        rob_entries_next = rob_entries;
        next_N_complete_bits = '0;

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
            head_entries[i] = rob_entries[(head_idx+i)%`ROB_SZ];
            if (head_entries[i].complete) begin
                next_N_complete_bits[i] = 1'b1;
            end

            if (retire) begin
                rob_entries_next[(head_idx+i)%`ROB_SZ].valid = 1'b0;
            end

            // Free Count calculation
            entry_packet_valid_bits[i] = rob_entry_packet[i].valid;
        end

        num_retired = retire ? `N : 0;
        num_dispatched = $countones(entry_packet_valid_bits);
        free_count_next = free_count + num_retired - num_dispatched;

        // Head and tail pointers
        head_idx_next = retire ? ((head_idx + `N) % `ROB_SZ) : head_idx;
        tail_idx_next = (tail_idx + num_dispatched) % `ROB_SZ;
    end

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

    assign free_slots = free_count;

endmodule
