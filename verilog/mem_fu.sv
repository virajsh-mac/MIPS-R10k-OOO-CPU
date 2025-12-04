`include "sys_defs.svh"

// Memory Functional Unit: compute addresses and handle load/store operations
// Supports store-to-load forwarding via store queue interface
module mem_fu (
    input clock,
    input reset,
    input valid,
    input MEM_FUNC func,
    input DATA rs1,
    input DATA rs2,
    input DATA imm,
    input STOREQ_IDX store_queue_idx,  // For stores: SQ slot; For loads: SQ tail (to find older stores)
    input PHYS_TAG dest_tag,
    input CACHE_DATA cache_hit_data,

    // Store-to-load forwarding from store queue
    input logic forward_valid,   // Store queue found matching store with data
    input DATA  forward_data,    // Forwarded data from store queue
    input logic forward_stall,   // UNUSED - kept for interface compatibility (always 0)

    // Outputs
    output DATA addr,
    output DATA data,
    output EXECUTE_STOREQ_ENTRY store_queue_entry,
    output CDB_ENTRY cdb_result,
    output logic cdb_request,
    output logic is_load_request,
    output logic is_store_op,
    output D_ADDR dcache_addr,
    
    // Store queue forwarding lookup outputs
    output logic lookup_valid,      // Request forwarding lookup from store queue
    output ADDR  lookup_addr,       // Address to look up
    output STOREQ_IDX lookup_sq_tail // SQ tail to determine which stores are older
);

    // =========================================================================
    // State for handling cache misses
    // =========================================================================

    typedef struct packed {
        logic valid;
        PHYS_TAG dest_tag;
        ADDR full_addr;     // Full 32-bit address for forwarding lookup
        STOREQ_IDX sq_tail; // SQ tail for forwarding lookup on retry
    } PENDING_LOAD;

    PENDING_LOAD pending_load, pending_load_next;

    // =========================================================================
    // Combinational Logic
    // =========================================================================

    // Helper signals for operation type detection
    logic is_load, is_store;
    logic pending_load_hit;
    D_ADDR current_dcache_addr;
    ADDR computed_addr;

    always_comb begin
        // Compute effective address
        computed_addr = rs1 + imm;
        addr = computed_addr;
        data = rs2;

        // Extract dcache address from computed address
        // D_ADDR: zeros[15:0] + tag[31:3] + block_offset[2:0]
        // For 8-byte cache lines: tag = addr[31:3], block_offset = addr[2:0]
        current_dcache_addr = '{
            zeros: 16'b0,
            tag: computed_addr[31:3],
            block_offset: computed_addr[2:0]
        };

        // Determine operation type
        is_load = (func == LOAD_BYTE   || func == LOAD_HALF   || func == LOAD_WORD   ||
                   func == LOAD_DOUBLE || func == LOAD_BYTE_U || func == LOAD_HALF_U);

        is_store = (func == STORE_BYTE || func == STORE_HALF ||
                    func == STORE_WORD || func == STORE_DOUBLE);

        // Check if pending load now gets data (from cache or forwarding)
        pending_load_hit = pending_load.valid && (cache_hit_data.valid || forward_valid);
    end

    // Store queue entry generation (for store operations)
    always_comb begin
        if (valid && is_store) begin
            store_queue_entry = '{
                valid: 1'b1,
                addr: computed_addr,
                data: rs2,
                store_queue_idx: store_queue_idx
            };
            is_store_op = 1'b1;
        end else begin
            store_queue_entry = '0;
            is_store_op = 1'b0;
        end
    end

    // Store queue forwarding lookup request
    always_comb begin
        lookup_valid   = 1'b0;
        lookup_addr    = '0;
        lookup_sq_tail = '0;

        if (valid && is_load) begin
            // New load - request forwarding lookup
            lookup_valid   = 1'b1;
            lookup_addr    = computed_addr;
            lookup_sq_tail = store_queue_idx;  // SQ tail when load was dispatched
        end else if (pending_load.valid) begin
            // Pending load - continue lookup (in case it was stalled before)
            lookup_valid   = 1'b1;
            lookup_addr    = pending_load.full_addr;
            lookup_sq_tail = pending_load.sq_tail;
        end
    end

    // Extract correct word from cache line based on address
    DATA loaded_data;
    logic word_select;
    always_comb begin
        // Determine which word to extract: use pending load's address if completing a pending load
        if (pending_load_hit && pending_load.valid) begin
            word_select = pending_load.full_addr[2];
        end else begin
            word_select = computed_addr[2];
        end
        
        // Priority: forwarded data > cache data
        if (forward_valid) begin
            // Use forwarded data from store queue
            loaded_data = forward_data;
        end else if (cache_hit_data.valid) begin
            // Extract word from 64-bit cache line based on word offset
            if (word_select) begin
                loaded_data = cache_hit_data.data.word_level[1];  // Upper word
            end else begin
                loaded_data = cache_hit_data.data.word_level[0];  // Lower word
            end
        end else begin
            loaded_data = '0;
        end
        
        // TODO: Handle byte/halfword loads with proper sign extension
    end

    // CDB result and request generation
    always_comb begin
        cdb_result  = '0;
        cdb_request = 1'b0;

        if (pending_load_hit) begin
            // Pending load completes (either from cache hit or forwarding)
            cdb_result = '{
                valid: 1'b1,
                tag: pending_load.dest_tag,
                data: loaded_data
            };
            cdb_request = 1'b1;
        end else if (valid && is_load && (cache_hit_data.valid || forward_valid)) begin
            // New load completes immediately (cache hit or store forwarding)
            cdb_result = '{
                valid: 1'b1,
                tag: dest_tag,
                data: loaded_data
            };
            cdb_request = 1'b1;
        end else if (valid && is_store) begin
            // Store completes immediately (address/data sent to store queue)
            cdb_result  = '0;
            cdb_request = 1'b1;
        end
    end

    // Dcache request generation
    // Only request from cache if forwarding didn't provide data
    always_comb begin
        is_load_request = 1'b0;
        dcache_addr     = '0;

        if (valid && is_load && !forward_valid) begin
            // New load request - no forwarding available, go to dcache
            is_load_request = 1'b1;
            dcache_addr     = current_dcache_addr;
        end else if (pending_load.valid && !forward_valid) begin
            // Continue requesting for pending load (cache miss, no forwarding)
            is_load_request = 1'b1;
            dcache_addr     = '{
                zeros: 16'b0,
                tag: pending_load.full_addr[31:3],
                block_offset: pending_load.full_addr[2:0]
            };
        end
    end

    // Pending load state management
    always_comb begin
        pending_load_next = pending_load;  // Default: keep state

        if (pending_load_hit) begin
            // Clear pending load when it completes
            pending_load_next.valid = 1'b0;
        end else if (valid && is_load && !cache_hit_data.valid && !forward_valid && !pending_load.valid) begin
            // New load misses both cache and store queue - make it pending
            pending_load_next = '{
                valid: 1'b1,
                dest_tag: dest_tag,
                full_addr: computed_addr,
                sq_tail: store_queue_idx
            };
        end
    end

    // Sequential update of pending load state
    always_ff @(posedge clock) begin
        if (reset) begin
            pending_load <= '0;
        end else begin
            pending_load <= pending_load_next;
        end
    end
endmodule  // mem_fu
