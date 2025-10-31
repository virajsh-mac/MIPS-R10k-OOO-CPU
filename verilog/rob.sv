/////////////////////////////////////////////////////////////////////////
//                                                                     //
//  Modulename :  rob.sv                                               //
//                                                                     //
//  Description :  Reorder Buffer module; TODO. `
//                   instruction buffer,
//                  write to Physical Register File in complete
//                  Only retire when all `N after head pointers are retired
  //                  complete//                              //
/////////////////////////////////////////////////////////////////////////
`include "sys_defs.svh"

module rob (
    input logic                     clock,
    input logic                     reset, // reset on mispredict

    // Dispatch
    input  ROB_ENTRY  [`N-1:0]      rob_entry_packet,
    output logic [$clog2(`ROB_SZ+1)-1:0] free_slots
    //output logic [`ROB_IDX_BITS-1:0] free_slots,

    // Complete
    input  ROB_UPDATE_PACKET        rob_update_packet,

    // Retire
    output ROB_ENTRY [`N-1:0]       head_entries, // Could be retired
);
    ROB_ENTRY [`ROB_SZ-1:0] rob_entries, rob_entries_next;
    logic [`ROB_IDX_BITS-1:0] free_count, free_count_next;
    logic [`ROB_IDX_BITS-1:0] head_idx, head_idx_next, tail_idx, tail_idx_next;
    logic [`N-1:0] entry_packet_valid_bits;
    logic [$clog2(`ROB_SZ+1)-1:0] inflight;
    logic full, full_next;

    // For calculating free count
    logic retire;
    logic [`N-1:0] retire_valid_bits;
    logic [$clog2(`N+1)-1:0] increment, decrement;

    assign retire = &retire_valid_bits;

    always_comb begin
        free_count_next = free_count;

        // Dispatch
        for (int i = 0; i < `N; i++) begin
            if (rob_entry_packet[i].valid) begin
                rob_entries[tail_idx + i] = rob_entry_packet[i];
            end
        end

        // Complete ROB entries update
        for (int rob_row = 0; rob_row < `N; rob_row++) begin
            for (int i = 0; i < `N; i++) begin
                if (rob_row == rob_update_packet[i] && rob_update_packet[i].valid) begin
                    rob_entries[rob_row].value = rob_update_packet[i].value;
                    rob_entries[rob_row].branch_taken = rob_update_packet[i].branch_taken;
                    rob_entries[rob_row].branch_target = rob_update_packet[i].branch_target;
                end
            end
        end

        // Output for retire stage
        for (int i = 0; i < `N; i++) begin
            head_entries[i] = rob_entries[(head_idx + i) % (`ROB_SZ - 1)];
            if (head_entries[i].complete) begin // Retire valid bits for free count calculation
                retire_valid_bits[i] = 1'b1;
            end
        end

        // Free Count calculation
        for (int i = 0; i < `N; i++) begin
            entry_packet_valid_bits[i] = rob_entry_packet[i].valid;
        end

        increment = retire ? `N : 0;
        decrement = $countones(entry_packet_valid_bits);
        free_count_next = free_count + increment - decrement;
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            rob_entries <= '0;
            free_count <= '0;
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
