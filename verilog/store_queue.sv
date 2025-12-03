// -----------------------------------------------------------------------------
// Store Queue FIFO (simple, registered I/O)
// - Depth comes from `LSQ_SZ` (set in sys_defs.svh)
// - Payload type: store_queue_entry_t (defined in sys_defs.svh)
// - Enqueue from DISPATCH/ISSUE; Dequeue at RETIRE
// - All acks and outputs are registered (no combinational assigns)
// - Allows same-cycle enqueue+dequeue
// -----------------------------------------------------------------------------

`include "sys_defs.svh"

module store_queue (
    input logic clock,
    input logic reset,

    // ============================================================
    // Dispatch I/O
    // ============================================================
    input  STOREQ_ENTRY [               `N-1:0] sq_dispatch_packet,  // must be contiguous valid
    output logic        [$clog2(`LSQ_SZ+1)-1:0] free_slots,          // number of free SQ slots
    output STOREQ_IDX   [               `N-1:0] sq_alloc_idxs,       // allocation indices

    // ============================================================
    // Execute I/O
    // ============================================================
    // from MEM FUs (updates store instructions data and address fields in store queue)
    input EXECUTE_STOREQ_ENTRY [`NUM_FU_MEM-1:0] mem_storeq_entries,

    // ============================================================
    // Retire I/O
    // ============================================================
    input logic                     mispredict,
    input logic [$clog2(`N+1)-1:0]  free_count,  // number of entries to free (from rob)

    output logic [$clog2(`LSQ_SZ)-1:0] complete_ptr
);

    // ============================================================
    // Storage
    // ============================================================
    STOREQ_ENTRY [`LSQ_SZ-1:0] sq_entries, sq_entries_next;

    logic [$clog2(`LSQ_SZ+1)-1:0] free_slots_reg, free_slots_reg_next;
    logic [$clog2(`LSQ_SZ)-1:0] head_idx, head_idx_next;
    logic [$clog2(`LSQ_SZ)-1:0] tail_idx, tail_idx_next;
    logic [$clog2(`N+1)-1:0] num_dispatched;
    logic [`LSQ_SZ-1:0] completed, completed_next;
    
    always_comb begin
        sq_entries_next      = sq_entries;
        free_slots_reg_next  = free_slots_reg;
        head_idx_next        = head_idx;
        tail_idx_next        = tail_idx;
        completed_next       = completed;

        // ============================
        // 1. Retire logic
        // ============================
        // Clear up to free_count entries starting at head
        for (int i = 0; i < free_count; i++) begin
            sq_entries_next[(head_idx + i) % `LSQ_SZ].valid = 1'b0;
            completed_next[(head_idx + i) % `LSQ_SZ]        = 1'b0; 
        end


        // Advance head pointer
        head_idx_next = (head_idx + free_count) % `LSQ_SZ;

        // ============================
        // 2. Dispatch logic (enqueue)
        // ============================
        num_dispatched = 0;
        for (int i = 0; i < `N; i++) begin
            if (sq_dispatch_packet[i].valid) begin
                sq_entries_next[(tail_idx + num_dispatched) % `LSQ_SZ] = sq_dispatch_packet[i];
                completed_next[(tail_idx + num_dispatched) % `LSQ_SZ]  = 1'b0;
                num_dispatched++;
            end
        end

        // ============================
        // 3. Update entries with executed store address/data
        // ============================

        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            if (mem_storeq_entries.valid[i]) begin
                sq_entries_next[mem_storeq_entries.store_queue_idx[i]].address = mem_storeq_entries.addr[i];
                sq_entries_next[mem_storeq_entries.store_queue_idx[i]].data    = mem_storeq_entries.data[i];
                completed_next[mem_storeq_entries.store_queue_idx[i]]          = 1'b1; 
            end
        end

        // Advance tail pointer by number of enqueues
        tail_idx_next = (tail_idx + num_dispatched) % `LSQ_SZ;

        // ============================
        // 4. Update free slot counter
        // ============================
        free_slots_reg_next = free_slots_reg + free_count - num_dispatched;
    end


    // ============================================================
    // Allocation indices (dispatch)
    // ============================================================
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            sq_alloc_idxs[i] = STOREQ_IDX'((tail_idx + i) % `LSQ_SZ);
        end
    end

    assign free_slots = free_slots_reg;

        assign free_slots = free_slots_reg;

    // Walk forward from head_idx while entries are {valid && completed}.
    // complete_ptr = first index that is NOT (valid && completed).
    always_comb begin
        logic [$clog2(`LSQ_SZ)-1:0] ptr;
        ptr = head_idx;

        // at most LSQ_SZ steps to avoid infinite loop
        for (int step = 0; step < `LSQ_SZ; step++) begin
            if (sq_entries[ptr].valid && completed[ptr]) begin
                ptr = (ptr + 1) % `LSQ_SZ;
            end else begin
                break;
            end
        end

        complete_ptr = ptr;
    end


    // ============================================================
    // Sequential
    // ============================================================
    always_ff @(posedge clock) begin
        if (reset || mispredict) begin
            sq_entries <= '0;
            head_idx   <= '0;
            tail_idx   <= '0;
            completed  <= '0;
            free_slots_reg <= `LSQ_SZ;
        end else begin
            sq_entries <= sq_entries_next;
            head_idx   <= head_idx_next;
            tail_idx   <= tail_idx_next;
            completed <= completed_next;
            free_slots_reg <= free_slots_reg_next;
        end
    end

endmodule