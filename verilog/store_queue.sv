// -----------------------------------------------------------------------------
// Store Queue FIFO with Store-to-Load Forwarding
// - Depth: `LSQ_SZ` (set in sys_defs.svh)
// - Enqueue from DISPATCH; Dequeue at RETIRE
// - Provides forwarding interface for loads to get data from older stores
// -----------------------------------------------------------------------------

`include "sys_defs.svh"

module store_queue (
    input logic clock,
    input logic reset,

    // ============================================================
    // Dispatch I/O
    // ============================================================
    input  STOREQ_ENTRY [               `N-1:0] sq_dispatch_packet,
    output logic        [$clog2(`LSQ_SZ+1)-1:0] free_slots,
    output STOREQ_IDX   [               `N-1:0] sq_alloc_idxs,
    output STOREQ_IDX                           sq_tail_idx,  // Current tail (for load forwarding)

    // ============================================================
    // Execute I/O
    // ============================================================
    // From MEM FUs: updates store instructions data and address fields
    input EXECUTE_STOREQ_ENTRY [`NUM_FU_MEM-1:0] mem_storeq_entries,

    // ============================================================
    // Load Forwarding Interface (from MEM FUs)
    // ============================================================
    // Load lookup requests - each MEM FU can request a forwarding check
    input logic       [`NUM_FU_MEM-1:0] load_lookup_valid,      // Is this a load lookup request?
    input ADDR        [`NUM_FU_MEM-1:0] load_lookup_addr,       // Load's effective address
    input STOREQ_IDX  [`NUM_FU_MEM-1:0] load_lookup_sq_tail,    // SQ tail when load was dispatched (stores < this are older)
    
    // Forwarding results back to MEM FUs
    output logic      [`NUM_FU_MEM-1:0] forward_valid,          // Forward data is valid (use instead of cache)
    output DATA       [`NUM_FU_MEM-1:0] forward_data,           // Forwarded data from store queue
    output logic      [`NUM_FU_MEM-1:0] forward_stall,          // Matching store exists but hasn't executed yet

    // ============================================================
    // Retire I/O
    // ============================================================
    input logic                     mispredict,
    input logic [$clog2(`N+1)-1:0]  free_count,

    // Outputs
    // output logic [$clog2(`LSQ_SZ)-1:0] complete_ptr,
    output logic                       unexecuted_store,
    
    // To D-Cache
    output logic                    dcache_store_valid,
    output ADDR                     dcache_store_addr,
    output DATA                     dcache_store_data,

    // To Dbg
    output logic [$clog2(`LSQ_SZ)-1:0] head_idx_dbg,
    output logic [$clog2(`LSQ_SZ)-1:0] tail_idx_dbg
);

    // ============================================================
    // Internal Storage
    // ============================================================
    STOREQ_ENTRY [`LSQ_SZ-1:0] sq_entries, sq_entries_next;
    logic [`LSQ_SZ-1:0] executed, executed_next;
    
    logic [$clog2(`LSQ_SZ+1)-1:0] free_slots_reg, free_slots_reg_next;
    logic [$clog2(`LSQ_SZ)-1:0] head_idx, head_idx_next;
    logic [$clog2(`LSQ_SZ)-1:0] tail_idx, tail_idx_next;
    
    // Count of stores dispatched this cycle
    logic [$clog2(`N+1)-1:0] num_dispatched;

    // ============================================================
    // Helper function: Check if index is in range [head, tail) with wrap-around
    // Returns true if 'idx' is between head (inclusive) and tail (exclusive)
    // ============================================================
    function automatic logic idx_in_range(
        input logic [$clog2(`LSQ_SZ)-1:0] idx,
        input logic [$clog2(`LSQ_SZ)-1:0] head,
        input logic [$clog2(`LSQ_SZ)-1:0] tail
    );
        if (head <= tail) begin
            // No wrap: idx must be >= head and < tail
            return (idx >= head) && (idx < tail);
        end else begin
            // Wrapped: idx must be >= head OR < tail
            return (idx >= head) || (idx < tail);
        end
    endfunction

    // ============================================================
    // Main Combinational Logic
    // ============================================================
    always_comb begin
        // Default: maintain current state
        sq_entries_next     = sq_entries;
        executed_next       = executed;
        free_slots_reg_next = free_slots_reg;
        head_idx_next       = head_idx;
        tail_idx_next       = tail_idx;

        // D-Cache outputs (default)
        dcache_store_valid = 1'b0;
        dcache_store_addr  = '0;
        dcache_store_data  = '0;

        // ============================
        // 1. Retire: Output head store to D-Cache and free entries
        // ============================
        if (sq_entries[head_idx].valid && executed[head_idx]) begin
            dcache_store_valid = 1'b1;
            dcache_store_addr  = sq_entries[head_idx].address;
            dcache_store_data  = sq_entries[head_idx].data;
        end

        // Free retired entries
        for (int i = 0; i < `N; i++) begin
            if (i < free_count) begin
                sq_entries_next[(head_idx + i) % `LSQ_SZ].valid = 1'b0;
                executed_next[(head_idx + i) % `LSQ_SZ] = 1'b0;
            end
        end
        head_idx_next = (head_idx + free_count) % `LSQ_SZ;

        // ============================
        // 2. Dispatch: Enqueue new stores
        // ============================
        num_dispatched = '0;
        for (int i = 0; i < `N; i++) begin
            if (sq_dispatch_packet[i].valid) begin
                sq_entries_next[(tail_idx + num_dispatched) % `LSQ_SZ] = sq_dispatch_packet[i];
                executed_next[(tail_idx + num_dispatched) % `LSQ_SZ] = 1'b0;
                num_dispatched++;
            end
        end
        tail_idx_next = (tail_idx + num_dispatched) % `LSQ_SZ;

        // ============================
        // 3. Execute: Update store entries with computed address/data
        // ============================
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            if (mem_storeq_entries[i].valid) begin
                sq_entries_next[mem_storeq_entries[i].store_queue_idx].address = mem_storeq_entries[i].addr;
                sq_entries_next[mem_storeq_entries[i].store_queue_idx].data    = mem_storeq_entries[i].data;
                executed_next[mem_storeq_entries[i].store_queue_idx] = 1'b1;
            end
        end

        // ============================
        // 4. Update free slot counter
        // ============================
        free_slots_reg_next = free_slots_reg + free_count - num_dispatched;
    end

    // ============================================================
    // Allocation Indices Output (for dispatch stage)
    // ============================================================
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            sq_alloc_idxs[i] = STOREQ_IDX'((tail_idx + i) % `LSQ_SZ);
        end
    end

    assign free_slots = free_slots_reg;
    assign sq_tail_idx = tail_idx;

    // dbg sig
    assign head_idx_dbg = head_idx;
    assign tail_idx_dbg = tail_idx;

    // ============================================================
    // Complete Pointer: First index that is NOT (valid && executed)
    // Used to track how many stores at head are ready to retire
    // ============================================================
    always_comb begin
        logic [$clog2(`LSQ_SZ)-1:0] ptr;
        ptr = head_idx;
        
        for (int step = 0; step < `LSQ_SZ; step++) begin
            if (sq_entries[ptr].valid && executed[ptr]) begin
                ptr = (ptr + 1) % `LSQ_SZ;
            end else begin
                break;
            end
        end
        // complete_ptr = ptr;
    end

    // ============================================================
    // Unexecuted Store Flag: Any valid store not yet executed
    // ============================================================
    always_comb begin
        unexecuted_store = 1'b0;
        for (int i = 0; i < `LSQ_SZ; i++) begin
            if (sq_entries[i].valid && !executed[i]) begin
                unexecuted_store = 1'b1;
            end
        end
    end

    // ============================================================
    // Store-to-Load Forwarding Logic
    // Since dispatch blocks loads until all stores execute, we only need
    // to search for address matches - no stall checking needed.
    // ============================================================
    always_comb begin
        // Default outputs
        forward_valid = '0;
        forward_data  = '0;
        forward_stall = '0;  // Always 0 - dispatch ensures no unexecuted older stores

        // Process each MEM FU's load lookup request
        for (int fu = 0; fu < `NUM_FU_MEM; fu++) begin
            if (load_lookup_valid[fu]) begin
                logic [$clog2(`LSQ_SZ)-1:0] search_idx;
                STOREQ_IDX load_tail;
                
                load_tail = load_lookup_sq_tail[fu];
                
                // Search for address match among older stores
                // Walk backwards from youngest to oldest to find youngest match
                for (int step = 0; step < `LSQ_SZ; step++) begin
                    search_idx = (load_tail + `LSQ_SZ - 1 - step) % `LSQ_SZ;
                    
                    // Stop if we've passed head (no more older stores)
                    if (!idx_in_range(search_idx, head_idx, load_tail)) begin
                        break;
                    end
                    
                    // Check for address match
                    if (sq_entries[search_idx].valid) begin
                        // Compare addresses (word-aligned - compare upper bits)
                        if (sq_entries[search_idx].address[31:2] == load_lookup_addr[fu][31:2]) begin
                            // Found youngest matching store - forward its data
                            forward_valid[fu] = 1'b1;
                            forward_data[fu]  = sq_entries[search_idx].data;
                            break;
                        end
                    end
                end
                // If no match found, forward_valid stays 0 - load goes to cache
            end
        end
    end

    // ============================================================
    // Sequential Logic
    // ============================================================
    always_ff @(posedge clock) begin
        if (reset || mispredict) begin
            sq_entries     <= '0;
            executed       <= '0;
            head_idx       <= '0;
            tail_idx       <= '0;
            free_slots_reg <= `LSQ_SZ;
        end else begin
            sq_entries     <= sq_entries_next;
            executed       <= executed_next;
            head_idx       <= head_idx_next;
            tail_idx       <= tail_idx_next;
            free_slots_reg <= free_slots_reg_next;
        end
    end

    // ============================================================
    // Store Queue Debug Display
    // ============================================================
`ifdef DEBUG
    always_ff @(posedge clock) begin
        if (!reset) begin
            $display("========================================");
            $display("=== STORE QUEUE STATE (Cycle %0t) ===", $time);
            $display("========================================");
            
            // Pointers and counters
            $display("--- Pointers ---");
            $display("  Head: %0d, Tail: %0d, Free Slots: %0d", head_idx, tail_idx, free_slots_reg);
            $display("  Unexecuted Store: %0d", unexecuted_store);
            
            // All entries
            $display("--- Store Queue Entries ---");
            for (int i = 0; i < `LSQ_SZ; i++) begin
                if (sq_entries[i].valid) begin
                    $display("  [%2d] Valid=1 Executed=%0d Addr=%h Data=%h %s%s", 
                             i, executed[i], 
                             sq_entries[i].address, sq_entries[i].data,
                             (i == head_idx) ? "<-HEAD" : "",
                             (i == tail_idx) ? "<-TAIL" : "");
                end else begin
                    $display("  [%2d] Valid=0 %s%s", i,
                             (i == head_idx) ? "<-HEAD" : "",
                             (i == tail_idx) ? "<-TAIL" : "");
                end
            end
            
            // Dispatch inputs
            $display("--- Dispatch Inputs ---");
            for (int i = 0; i < `N; i++) begin
                if (sq_dispatch_packet[i].valid) begin
                    $display("  Dispatch[%0d]: Valid=1 Addr=%h Data=%h -> Alloc Idx=%0d", 
                             i, sq_dispatch_packet[i].address, sq_dispatch_packet[i].data, sq_alloc_idxs[i]);
                end
            end
            
            // Execute inputs (from MEM FUs)
            $display("--- Execute Updates (from MEM FUs) ---");
            for (int i = 0; i < `NUM_FU_MEM; i++) begin
                if (mem_storeq_entries[i].valid) begin
                    $display("  MEM_FU[%0d]: Valid=1 SQ_Idx=%0d Addr=%h Data=%h", 
                             i, mem_storeq_entries[i].store_queue_idx,
                             mem_storeq_entries[i].addr, mem_storeq_entries[i].data);
                end
            end
            
            // Load forwarding lookups
            $display("--- Load Forwarding Lookups ---");
            for (int i = 0; i < `NUM_FU_MEM; i++) begin
                if (load_lookup_valid[i]) begin
                    $display("  Lookup[%0d]: Addr=%h SQ_Tail=%0d -> Forward_Valid=%0d Forward_Stall=%0d Forward_Data=%h", 
                             i, load_lookup_addr[i], load_lookup_sq_tail[i],
                             forward_valid[i], forward_stall[i], forward_data[i]);
                end
            end
            
            // Retire outputs
            $display("--- Retire / D-Cache Output ---");
            if (dcache_store_valid) begin
                $display("  D-Cache Store: Valid=1 Addr=%h Data=%h", dcache_store_addr, dcache_store_data);
            end else begin
                $display("  D-Cache Store: Valid=0");
            end
            $display("  Free Count (from retire): %0d", free_count);
            
            $display("");
        end
    end
`endif

endmodule
