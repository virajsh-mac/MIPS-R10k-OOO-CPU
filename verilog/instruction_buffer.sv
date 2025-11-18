`include "sys_defs.svh"

module instr_buffer #(
    parameter DEPTH = 32,
    parameter PUSH_WIDTH = 4,  // Number of push ports
    parameter POP_WIDTH = 3    // Number of pop ports
) (
    input  logic                 clock,
    input  logic                 reset,

    // Retire on branch mispredict
    input  logic                 flush,

    output logic [$clog2(PUSH_WIDTH+1)-1:0]    available_slots, // number of available slots
    output logic                               full,
    output logic                               empty,

    // Fetch stage - supports up to PUSH_WIDTH pushes per cycle
    input logic [$clog2(PUSH_WIDTH+1)-1:0]     num_pushes,      // number of valid entries to push
    input FETCH_PACKET [PUSH_WIDTH-1:0]        new_ib_entry,    // PUSH_WIDTH entries from fetch

    // Decode and Dispatch IO - supports up to POP_WIDTH pops per cycle
    // Dispatch inspection window - shows next POP_WIDTH instructions available for decode/dispatch
    input logic [$clog2(POP_WIDTH+1)-1:0]      num_pops,        // number of entries to pop
    output FETCH_PACKET [POP_WIDTH-1:0]        dispatch_window, // Window of next available instructions
    output logic [$clog2(POP_WIDTH+1)-1:0]     window_valid_count // Number of valid instructions in window
);

    // Helper functions for cleaner code
    function automatic logic [$clog2(DEPTH)-1:0] wrap_ptr(input logic [$clog2(DEPTH)-1:0] ptr, input logic [$clog2(DEPTH)-1:0] offset);
        return (ptr + offset) % DEPTH;
    endfunction

    function automatic logic [$clog2(PUSH_WIDTH+1)-1:0] calc_free_slots(input logic [$clog2(PUSH_WIDTH+1)-1:0] current_free, input logic [$clog2(POP_WIDTH+1)-1:0] pops, input logic [$clog2(PUSH_WIDTH+1)-1:0] pushes);
        // Optimized for timing: Simple arithmetic, no underflow check needed since we validate can_push/can_pop
        automatic logic [$clog2(PUSH_WIDTH+1):0] temp = {1'b0, current_free} + pops - pushes;  // Extra bit prevents underflow
        return (temp > PUSH_WIDTH) ? PUSH_WIDTH[$clog2(PUSH_WIDTH+1)-1:0] : temp[$clog2(PUSH_WIDTH+1)-1:0];
    endfunction

    // FIFO storage
    FETCH_PACKET [DEPTH-1:0] ib_entries, ib_entries_next;

    // FIFO state - individual signals to avoid concatenation
    logic [$clog2(DEPTH)-1:0] head_ptr, head_ptr_next;
    logic [$clog2(DEPTH)-1:0] tail_ptr, tail_ptr_next;
    logic [$clog2(DEPTH+1)-1:0] count, count_next;
    logic [$clog2(PUSH_WIDTH+1)-1:0] free_slots, free_slots_next;

    // Full and empty logic
    assign full = (free_slots == 0);
    assign empty = (count == 0);
    assign available_slots = free_slots;

    // Next state logic
    always_comb begin
        ib_entries_next = ib_entries;
        head_ptr_next = head_ptr;
        tail_ptr_next = tail_ptr;
        count_next = count;
        free_slots_next = free_slots;

        if (flush) begin
            // Flush: reset all pointers and counters
            head_ptr_next = '0;
            tail_ptr_next = '0;
            count_next = '0;
            free_slots_next = (DEPTH > PUSH_WIDTH) ? PUSH_WIDTH[$clog2(PUSH_WIDTH+1)-1:0] : DEPTH[$clog2(PUSH_WIDTH+1)-1:0];
        end else begin
            // Handle operations in priority order: pops first, then pushes
            automatic logic can_pop = (num_pops > 0 && count >= num_pops);
            automatic logic can_push = (num_pushes > 0 && (count - (can_pop ? num_pops : 0) + num_pushes) <= DEPTH);

            if (can_pop) begin
                head_ptr_next = wrap_ptr(head_ptr, num_pops);
                count_next = count - num_pops;
            end

            if (can_push) begin
                // Push entries to FIFO
                foreach (new_ib_entry[i]) begin
                    if (i < num_pushes) begin
                        ib_entries_next[wrap_ptr(tail_ptr, i)] = new_ib_entry[i];
                    end
                end
                tail_ptr_next = wrap_ptr(tail_ptr, num_pushes);
                count_next = count_next + num_pushes;
            end

            // Update free slots using helper function
            free_slots_next = calc_free_slots(free_slots, can_pop ? num_pops : 0, can_push ? num_pushes : 0);
        end
    end

    // Sequential logic
    always_ff @(posedge clock) begin
        if (reset) begin
            ib_entries <= '0;
            head_ptr <= '0;
            tail_ptr <= '0;
            count <= '0;
            free_slots <= (DEPTH > PUSH_WIDTH) ? PUSH_WIDTH[$clog2(PUSH_WIDTH+1)-1:0] : DEPTH[$clog2(PUSH_WIDTH+1)-1:0];
        end else begin
            ib_entries <= ib_entries_next;
            head_ptr <= head_ptr_next;
            tail_ptr <= tail_ptr_next;
            count <= count_next;
            free_slots <= free_slots_next;
        end
    end

    // Dispatch inspection window - continuously show next POP_WIDTH instructions available for decode/dispatch
    always_comb begin
        dispatch_window = '{POP_WIDTH{'0}};  // Initialize all to invalid
        window_valid_count = 0;

        foreach (dispatch_window[i]) begin
            if (count > i) begin
                dispatch_window[i] = ib_entries[wrap_ptr(head_ptr, i)];
                window_valid_count = window_valid_count + 1;
            end
        end
    end
endmodule
