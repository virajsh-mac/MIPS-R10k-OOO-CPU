`include "sys_defs.svh"
`include "ISA.svh"

module instr_buffer (
    input  logic                 clock,
    input  logic                 reset,

    // Retire on branch mispredict
    input  logic                 flush,

    output logic [`IB_IDX_BITS:0]              available_slots, // total number of available slots

    // Fetch stage - supports up to `IB_PUSH_WIDTH pushes per cycle
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

    // Helper: get instruction type name
    function automatic string get_inst_type(INST instr);
        case (instr.r.opcode)
            `RV32_LOAD:     return "LOAD";
            `RV32_STORE:    return "STORE";
            `RV32_BRANCH:   return "BRANCH";
            `RV32_JALR_OP:  return "JALR";
            `RV32_JAL_OP:   return "JAL";
            `RV32_OP_IMM:   return "OP_IMM";
            `RV32_OP:       return "OP";
            `RV32_SYSTEM:   return "SYSTEM";
            `RV32_AUIPC_OP: return "AUIPC";
            `RV32_LUI_OP:   return "LUI";
            `RV32_FENCE:    return "FENCE";
            `RV32_AMO:      return "AMO";
            default:        return "UNKNOWN";
        endcase
    endfunction

    // FIFO storage
    FETCH_PACKET [`IB_SZ-1:0] ib_entries, ib_entries_next;

    // FIFO state
    logic [`IB_IDX_BITS-1:0] head_ptr, head_ptr_next;
    logic [`IB_IDX_BITS-1:0] tail_ptr, tail_ptr_next;
    logic [`IB_IDX_BITS:0] count, count_next;
    
    // Helper variable for counting actual pushes
    logic [$clog2(`IB_PUSH_WIDTH+1)-1:0] actual_pushes;

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

            // Handle pushes - push all valid entries
            // stage_fetch already checks ib_free_slots, so we can push all valid entries
            actual_pushes = 0;
            for (int i = 0; i < `IB_PUSH_WIDTH; i++) begin
                if (new_ib_entries[i].valid) begin
                    ib_entries_next[wrap_ptr(tail_ptr, actual_pushes)] = new_ib_entries[i];
                    actual_pushes = actual_pushes + 1;
                end
            end
            if (actual_pushes > 0) begin
                tail_ptr_next = wrap_ptr(tail_ptr, actual_pushes);
                count_next = count_next + actual_pushes;
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

    // DEBUG: Track previous values to avoid duplicate prints
    logic [`IB_IDX_BITS:0] prev_count;
    logic [$clog2(`IB_PUSH_WIDTH+1)-1:0] prev_valid_count;
    logic [$clog2(`N+1)-1:0] prev_num_pops;
    FETCH_PACKET [`IB_SZ-1:0] prev_ib_entries;

    // DEBUG: Print IB state (condensed format)
    always_ff @(posedge clock) begin
        if (!reset && !flush) begin
            // Print when entries are pushed (condensed, single line)
            // Count valid entries and check if any were pushed
            integer valid_count;
            integer i;
            valid_count = 0;
            for (i = 0; i < `IB_PUSH_WIDTH; i++) begin
                if (new_ib_entries[i].valid) valid_count = valid_count + 1;
            end
            
            if (valid_count > 0 && valid_count != prev_valid_count) begin
                $write("[IB] PUSH %0d: ", valid_count);
                for (i = 0; i < `IB_PUSH_WIDTH; i++) begin
                    if (new_ib_entries[i].valid) begin
                        $write("PC=%0d(%s) ", new_ib_entries[i].pc, get_inst_type(new_ib_entries[i].inst));
                    end
                end
                $display("");
            end
            
            prev_valid_count <= valid_count;

            // Print when entries are popped (condensed, single line)
            if (num_pops > 0 && num_pops != prev_num_pops) begin
                $write("[IB] POP %0d: ", num_pops);
                for (int i = 0; i < num_pops; i++) begin
                    if (i < window_valid_count && dispatch_window[i].valid) begin
                        $write("PC=%0d(%s) ", dispatch_window[i].pc, get_inst_type(dispatch_window[i].inst));
                    end
                end
                $display("");
            end

            // Update previous values
            prev_count <= count;
            prev_num_pops <= num_pops;
            prev_ib_entries <= ib_entries;
        end else begin
            prev_count <= '0;
            prev_valid_count <= '0;
            prev_num_pops <= '0;
            prev_ib_entries <= '0;
        end
    end

endmodule

