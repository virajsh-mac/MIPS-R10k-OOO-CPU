`include "sys_defs.svh"

module instr_buffer (
    input  logic                 clock,
    input  logic                 reset,

    // Retire on branch mispredict
    input  logic                 flush,

    output logic [`IB_IDX_BITS:0]              available_slots, // total number of available slots

    // Fetch stage - supports up to `IB_PUSH_WIDTH pushes per cycle
    input logic [$clog2(`IB_PUSH_WIDTH+1)-1:0]     num_pushes,      // number of valid entries to push
    input FETCH_PACKET [`IB_PUSH_WIDTH-1:0]        new_ib_entries,    // `IB_PUSH_WIDTH entries from fetch

    // Decode and Dispatch IO - supports up to `N pops per cycle
    // Dispatch inspection window - shows next `N instructions available for decode/dispatch
    input logic [$clog2(`N+1)-1:0]      num_pops,        // number of entries to pop
    output FETCH_PACKET [`N-1:0]        dispatch_window, // Window of next available instructions
    output logic [$clog2(`N+1)-1:0]     window_valid_count // Number of valid instructions in window
);

    // Helper function for pointer wrapping
    function automatic logic [`IB_IDX_BITS-1:0] wrap_ptr(input logic [`IB_IDX_BITS-1:0] ptr, input logic [`IB_IDX_BITS-1:0] offset);
        return (ptr + offset) % `IB_SZ;
    endfunction

    // FIFO storage
    FETCH_PACKET [`IB_SZ-1:0] ib_entries, ib_entries_next;

    // FIFO state
    logic [`IB_IDX_BITS-1:0] head_ptr, head_ptr_next;
    logic [`IB_IDX_BITS-1:0] tail_ptr, tail_ptr_next;
    logic [`IB_IDX_BITS:0] count, count_next;

    // Available slots calculation
    assign available_slots = `IB_SZ - count;

    // Next state logic
    always_comb begin
        ib_entries_next = ib_entries;
        head_ptr_next = head_ptr;
        tail_ptr_next = tail_ptr;
        count_next = count;

        if (flush) begin
            // Flush: reset all pointers and counters
            head_ptr_next = '0;
            tail_ptr_next = '0;
            count_next = '0;
        end else begin
            // Handle pops first
            if (num_pops > 0 && count >= num_pops) begin
                head_ptr_next = wrap_ptr(head_ptr, num_pops);
                count_next = count - num_pops;
            end

            // Handle pushes
            if (num_pushes > 0 && (count_next + num_pushes) <= `IB_SZ) begin
                // Push entries to FIFO
                foreach (new_ib_entries[i]) begin
                    if (i < num_pushes) begin
                        ib_entries_next[wrap_ptr(tail_ptr, i)] = new_ib_entries[i];
                    end
                end
                tail_ptr_next = wrap_ptr(tail_ptr, num_pushes);
                count_next = count_next + num_pushes;
            end
        end
    end

    // Sequential logic
    always_ff @(posedge clock) begin
        if (reset || flush) begin
            ib_entries <= '0;
            head_ptr <= '0;
            tail_ptr <= '0;
            count <= '0;
        end else begin
            ib_entries <= ib_entries_next;
            head_ptr <= head_ptr_next;
            tail_ptr <= tail_ptr_next;
            count <= count_next;
        end
    end

    // Dispatch inspection window - continuously show next `N instructions available for decode/dispatch
    always_comb begin
        dispatch_window = '{`N{'0}};  // Initialize all to invalid
        window_valid_count = 0;

        foreach (dispatch_window[i]) begin
            if (count > i) begin
                dispatch_window[i] = ib_entries[wrap_ptr(head_ptr, i)];
                window_valid_count = window_valid_count + 1;
            end
        end
    end

endmodule
