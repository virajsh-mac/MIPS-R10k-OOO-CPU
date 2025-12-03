`include "sys_defs.svh"

// Memory Functional Unit: compute addresses and handle load/store operations
// Stateful to handle cache miss latency - loads wait for cache hits before completing
module mem_fu (
    input clock,
    reset,
    valid,
    input MEM_FUNC func,
    input DATA rs1,
    rs2,
    imm,
    input STOREQ_IDX store_queue_idx,
    input PHYS_TAG dest_tag,
    input CACHE_DATA cache_hit_data,

    output DATA addr,
    data,
    output EXECUTE_STOREQ_ENTRY store_queue_entry,
    output CDB_ENTRY cdb_result,
    output logic cdb_request,
    is_load_request,
    is_store_op,
    output D_ADDR dcache_addr
);

    // =========================================================================
    // State for handling cache misses
    // =========================================================================

    typedef struct packed {
        logic valid;
        PHYS_TAG dest_tag;
        D_ADDR addr;
        logic word_offset;  // bit [2] of full address for word selection
    } PENDING_LOAD;

    PENDING_LOAD pending_load, pending_load_next;

    // =========================================================================
    // Combinational Logic
    // =========================================================================

    // Helper signals for operation type detection
    logic is_load, is_store;
    logic pending_load_hit;
    D_ADDR current_dcache_addr;

    always_comb begin
        // Compute effective address (always valid)
        addr = rs1 + imm;
        data = rs2;

        // Extract dcache address from computed address
        current_dcache_addr = '{
            zeros: 16'b0,
            tag: addr[31:12],
            block_offset: addr[4:3]
        };

        // Determine operation type
        is_load = (func == LOAD_BYTE   || func == LOAD_HALF   || func == LOAD_WORD   ||
                   func == LOAD_DOUBLE || func == LOAD_BYTE_U || func == LOAD_HALF_U);

        is_store = (func == STORE_BYTE || func == STORE_HALF ||
                    func == STORE_WORD || func == STORE_DOUBLE);

        // Check if pending load now hits cache
        pending_load_hit = pending_load.valid && cache_hit_data.valid;
    end

    // Store queue entry generation
    always_comb begin
        if (valid && is_store) begin
            store_queue_entry = '{
                valid: 1'b1,
                addr: addr,
                data: data,
                store_queue_idx: store_queue_idx
            };
            is_store_op = 1'b1;
        end else begin
            store_queue_entry = '0;
            is_store_op = 1'b0;
        end
    end

    // Extract correct word from cache line based on address
    DATA loaded_data;
    logic word_select;
    always_comb begin
        // Determine which word to extract: use pending load offset if that's what hit
        // otherwise use current instruction's address
        if (pending_load_hit) begin
            word_select = pending_load.word_offset;
        end else begin
            word_select = addr[2];
        end
        
        // Extract word from 64-bit cache line based on word offset
        if (word_select) begin
            loaded_data = cache_hit_data.data.word_level[1];  // Upper word
        end else begin
            loaded_data = cache_hit_data.data.word_level[0];  // Lower word
        end
        
        // TODO: Handle byte/halfword loads with proper sign extension
        // For now, assumes all loads are word-sized
    end

    // CDB result and request generation
    always_comb begin
        if (pending_load_hit) begin
            // Pending load completes when cache hit occurs
            cdb_result = '{
                valid: 1'b1,
                tag: pending_load.dest_tag,
                data: loaded_data
            };
            cdb_request = 1'b1;
        end else if (valid && is_load && cache_hit_data.valid) begin
            // New load hits cache immediately (no store queue forwarding yet)
            cdb_result = '{
                valid: 1'b1,
                tag: dest_tag,
                data: loaded_data
            };
            cdb_request = 1'b1;
        end else if (valid && is_store) begin
            // Store completes immediately (address/data sent to store queue)
            cdb_result = '0;
            cdb_request = 1'b1;
        end else begin
            // No completion this cycle
            cdb_result = '0;
            cdb_request = 1'b0;
        end
    end

    // Dcache request generation
    // NOTE: Load-Store Forwarding is NOT yet implemented
    // Loads always go to D-cache; they do not check store queue for newer values
    always_comb begin
        if (valid && is_load) begin
            // New load request - goes directly to dcache
            is_load_request = 1'b1;
            dcache_addr = current_dcache_addr;
        end else if (pending_load.valid) begin
            // Continue requesting for pending load (cache miss case)
            is_load_request = 1'b1;
            dcache_addr = pending_load.addr;
        end else begin
            is_load_request = 1'b0;
            dcache_addr = '0;
        end
    end

    // Pending load state management
    always_comb begin
        pending_load_next = pending_load;  // Default: keep state

        if (pending_load_hit) begin
            // Clear pending load when it completes
            pending_load_next.valid = 1'b0;
        end else if (valid && is_load && !cache_hit_data.valid && !pending_load.valid) begin
            // New load misses cache - make it pending
            pending_load_next = '{
                valid: 1'b1,
                dest_tag: dest_tag,
                addr: '{
                    zeros: 16'b0,
                    tag: addr[31:12],
                    block_offset: addr[4:3]
                },
                word_offset: addr[2]
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