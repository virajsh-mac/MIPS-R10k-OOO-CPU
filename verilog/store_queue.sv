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
    // from execute (updates store instructions data and address fields in store queue)
    input EXECUTE_STOREQ_PACKET execute_storeq_packet,

    // ============================================================
    // Retire I/O
    // ============================================================
    input logic                     mispredict,
    input logic [$clog2(`N+1)-1:0]  free_count // number of entries to free (from rob)
);

    // ============================================================
    // Storage
    // ============================================================
    STOREQ_ENTRY [`LSQ_SZ-1:0] sq_entries, sq_entries_next;

    logic [$clog2(`LSQ_SZ+1)-1:0] free_slots_reg, free_slots_reg_next;
    logic [$clog2(`LSQ_SZ)-1:0] head_idx, head_idx_next;
    logic [$clog2(`LSQ_SZ)-1:0] tail_idx, tail_idx_next;
    logic [$clog2(`N+1)-1:0] num_dispatched;
    
    always_comb begin
        sq_entries_next      = sq_entries;
        free_slots_reg_next  = free_slots_reg;
        head_idx_next        = head_idx;
        tail_idx_next        = tail_idx;

        // ============================
        // 1. Retire logic
        // ============================
        // Clear up to free_count entries starting at head
        for (int i = 0; i < free_count; i++) begin
            sq_entries_next[(head_idx + i) % `LSQ_SZ].valid = 1'b0;
        end

        // Advance head pointer
        head_idx_next = (head_idx + free_count) % `LSQ_SZ;

        // ============================
        // 2. Dispatch logic (enqueue)
        // ============================
        num_dispatched = 0;

        for (int i = 0; i < `N; i++) begin
            if (sq_dispatch_packet[i].valid) begin
                sq_entries_next[(tail_idx + num_dispatched) % `LSQ_SZ] 
                    = sq_dispatch_packet[i];
                num_dispatched++;
            end
        end

        // ============================
        // 3. Update entries with executed store address/data
        // ============================
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            if (execute_storeq_packet.valid[i]) begin
                sq_entries_next[execute_storeq_packet.store_queue_idx[i]].address = execute_storeq_packet.addr[i];
                sq_entries_next[execute_storeq_packet.store_queue_idx[i]].data = execute_storeq_packet.data[i];
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

    // ============================================================
    // Sequential
    // ============================================================
    always_ff @(posedge clock) begin
        if (reset || mispredict) begin
            sq_entries <= '0;
            head_idx   <= '0;
            tail_idx   <= '0;
            free_slots_reg <= `LSQ_SZ;
        end else begin
            sq_entries <= sq_entries_next;
            head_idx   <= head_idx_next;
            tail_idx   <= tail_idx_next;
            free_slots_reg <= free_slots_reg_next;
        end
    end

endmodule