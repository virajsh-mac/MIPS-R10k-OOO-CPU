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

    // Outputs
    output logic [$clog2(`LSQ_SZ)-1:0] complete_ptr,  // First index that is NOT (valid && executed)
    output logic                       unexecuted_store,  // Has any store that is valid but not yet executed
    
    // To D-Cache
    output logic                    dcache_store_valid,
    output ADDR                     dcache_store_addr,
    output DATA                     dcache_store_data
);

    // ============================================================
    // Storage
    // ============================================================
    STOREQ_ENTRY [`LSQ_SZ-1:0] sq_entries, sq_entries_next;

    logic [$clog2(`LSQ_SZ+1)-1:0] free_slots_reg, free_slots_reg_next;
    logic [$clog2(`LSQ_SZ)-1:0] head_idx, head_idx_next;
    logic [$clog2(`LSQ_SZ)-1:0] tail_idx, tail_idx_next;
    logic [$clog2(`N+1)-1:0] num_dispatched;
    logic [`LSQ_SZ-1:0] executed, executed_next; // change this to executed
    
    always_comb begin
        sq_entries_next      = sq_entries;
        free_slots_reg_next  = free_slots_reg;
        head_idx_next        = head_idx;
        tail_idx_next        = tail_idx;
        executed_next       = executed;

        // D-Cache Outputs (Default)
        dcache_store_valid = 1'b0;
        dcache_store_addr  = '0;
        dcache_store_data  = '0;

        // ============================
        // 1. Retire logic
        // ============================
        // Output the head store to the dcache (only if executed)
        // Retire should only commit stores that have been marked complete in the ROB,
        // which requires the store to have executed and computed its address/data
        if (sq_entries[head_idx].valid && executed[head_idx]) begin
            dcache_store_valid = 1'b1;
            dcache_store_addr  = sq_entries[head_idx].address;
            dcache_store_data  = sq_entries[head_idx].data;
        end

        for (int i = 0; i < free_count; i++) begin
            sq_entries_next[(head_idx + i) % `LSQ_SZ].valid = 1'b0;
            executed_next[(head_idx + i) % `LSQ_SZ]        = 1'b0; 
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
                executed_next[(tail_idx + num_dispatched) % `LSQ_SZ]  = 1'b0;
                num_dispatched++;
            end
        end

        // ============================
        // 3. Update entries with executed store address/data
        // ============================

        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            if (mem_storeq_entries[i].valid) begin
                sq_entries_next[mem_storeq_entries[i].store_queue_idx].address = mem_storeq_entries[i].addr;
                sq_entries_next[mem_storeq_entries[i].store_queue_idx].data    = mem_storeq_entries[i].data;
                executed_next[mem_storeq_entries[i].store_queue_idx]          = 1'b1; 
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

    // Walk forward from head_idx while entries are {valid && executed}.
    // complete_ptr = first index that is NOT (valid && executed).
    always_comb begin
        logic [$clog2(`LSQ_SZ)-1:0] ptr;
        ptr = head_idx;

        // at most LSQ_SZ steps to avoid infinite loop
        for (int step = 0; step < `LSQ_SZ; step++) begin
            if (sq_entries[ptr].valid && executed[ptr]) begin
                ptr = (ptr + 1) % `LSQ_SZ;
            end else begin
                break;
            end
        end

        complete_ptr = ptr;
    end

    // ------------------------------------------------------------
    // Has-pending-store flag
    // 1 if there is any store that is valid but not yet executed
    // ------------------------------------------------------------
    always_comb begin
        unexecuted_store = 1'b0;
        for (int i = 0; i < `LSQ_SZ; i++) begin
            if (sq_entries[i].valid && !executed[i]) begin
                unexecuted_store = 1'b1;
            end
        end
    end



    // ============================================================
    // Sequential
    // ============================================================
    always_ff @(posedge clock) begin
        if (reset || mispredict) begin
            sq_entries <= '0;
            head_idx   <= '0;
            tail_idx   <= '0;
            executed  <= '0;
            free_slots_reg <= `LSQ_SZ;
        end else begin
            sq_entries <= sq_entries_next;
            head_idx   <= head_idx_next;
            tail_idx   <= tail_idx_next;
            executed <= executed_next;
            free_slots_reg <= free_slots_reg_next;
        end
    end

endmodule