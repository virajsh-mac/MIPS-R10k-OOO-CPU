`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module dcache_subsystem (
    input clock,
    input reset,

    // Memory operations read
    input  D_ADDR_PACKET [1:0]  read_addrs,  // read_addr[0] is older operations
    output CACHE_DATA [1:0]     cache_outs,

    // Mem.sv IOs - Read requests
    input MEM_TAG               current_req_tag,
    input MEM_BLOCK             mem_data,
    input MEM_TAG               mem_data_tag,

    // Arbitor IOs - Read requests
    output D_ADDR_PACKET        mem_req_addr,
    input  logic                mem_req_accepted,
    
    // Arbitor IOs - Write requests (dirty writebacks)
    output D_ADDR_PACKET        mem_write_addr,
    output MEM_BLOCK            mem_write_data,
    output logic                mem_write_valid,

    // Processor Store Interface (from Store Queue)
    input logic                 proc_store_valid,
    input ADDR                  proc_store_addr,
    input DATA                  proc_store_data,
    output logic                proc_store_response,  // 1 = Store Complete, 0 = Stall/Retry
    // debug to expose DCache to testbench
    output D_CACHE_LINE [`DCACHE_LINES-1:0]      cache_lines_debug
);

    // Internal wires
    D_ADDR_PACKET dcache_write_addr, oldest_miss_addr, dcache_write_addr_refill;
    logic dcache_full;
    D_MSHR_PACKET new_mshr_entry;
    D_CACHE_LINE evicted_line;
    logic evicted_valid;
    CACHE_DATA [1:0] dcache_outs;
    logic mshr_addr_found;  // MSHR already has this address

    // Store logic signals
    D_ADDR_PACKET store_req_addr;
    logic     store_hit_dcache;

    // D-cache write control signals
    D_ADDR_PACKET dcache_write_addr_refill_local;
    logic         dcache_store_en_local;
    MEM_BLOCK     dcache_store_data_local;
    logic [7:0]   dcache_store_byte_en;  // Byte enable mask for sub-word stores

    dcache dcache_inst (
        .clock        (clock),
        .reset        (reset),
        // Fetch Stage read
        .read_addrs   (read_addrs),
        .cache_outs   (dcache_outs),
        // Snoop for store hits
        .snooping_addr(store_req_addr),
        .addr_found   (store_hit_dcache),
        .full         (dcache_full),
        // Dcache write mem_data (refill)
        .write_addr   (dcache_write_addr),
        .write_data   (mem_data),
        // Dcache Store Update
        .store_en     (dcache_store_en_local),
        .store_addr   (store_req_addr),
        .store_data   (dcache_store_data_local),
        .store_byte_en(dcache_store_byte_en),
        // Eviction interface (for dirty writeback to memory)
        .evicted_line (evicted_line),
        .evicted_valid(evicted_valid),
        // debug to expose DCache to testbench
        .cache_lines_debug(cache_lines_debug)
    );

    // Direct output from dcache (no victim cache)
    assign cache_outs = dcache_outs;

    d_mshr d_mshr_inst (
        .clock          (clock),
        .reset          (reset),
        // Snoop for duplicate requests (loads & stores)
        .snooping_addr  (oldest_miss_addr.addr),
        .addr_found     (mshr_addr_found),
        // When mem_req_accepted
        .new_entry      (new_mshr_entry),
        // Mem data back
        .mem_data_tag   (mem_data_tag),
        .mem_data_d_addr(dcache_write_addr_refill)
    );

    assign dcache_write_addr_refill_local = dcache_write_addr_refill;

    // D-cache write mux: Refill takes priority
    always_comb begin
        dcache_write_addr = '0;
        if (dcache_write_addr_refill_local.valid) begin
            dcache_write_addr = dcache_write_addr_refill_local;
        end 
    end

    // Store Request Processing
    // Convert processor store address to cache address format and generate byte enables
    // NOTE: Address breakdown for 8-byte cache lines:
    //   tag = addr[31:3] (29 bits - uniquely identifies 8-byte aligned block)
    //   block_offset = addr[2:0] (3 bits - byte offset within 8-byte line)
    //   word_offset = addr[2] (1 bit - which word: 0=lower, 1=upper)
    always_comb begin
        store_req_addr = '0;
        dcache_store_data_local = '0;
        dcache_store_byte_en = '0;
        
        if (proc_store_valid) begin
            store_req_addr.valid = 1'b1;
            store_req_addr.addr.tag = proc_store_addr[31:3];  // Full tag for 8-byte lines
            store_req_addr.addr.block_offset = proc_store_addr[2:0];  // Byte offset within line
            store_req_addr.addr.zeros = '0;
            
            // For now, assume WORD stores (4 bytes)
            // Place data in correct position within 64-bit line based on word offset (bit 2)
            // TODO: Support BYTE and HALF stores with appropriate byte enables
            if (proc_store_addr[2]) begin
                // Upper word (bytes 4-7)
                dcache_store_data_local.word_level[1] = proc_store_data;
                dcache_store_byte_en = 8'b1111_0000;  // Enable upper 4 bytes
            end else begin
                // Lower word (bytes 0-3)
                dcache_store_data_local.word_level[0] = proc_store_data;
                dcache_store_byte_en = 8'b0000_1111;  // Enable lower 4 bytes
            end
        end
    end

    // Store completion logic
    // A store can only complete if the line is already in the cache (hit)
    // If it misses, we request the line and the store will retry next cycle
    always_comb begin
        dcache_store_en_local = 1'b0;
        proc_store_response = 1'b0;

        // Only process stores if NO refill is active (Refill has priority)
        if (proc_store_valid && !dcache_write_addr_refill_local.valid) begin
            if (store_hit_dcache) begin
                // Hit: write to cache and signal completion
                dcache_store_en_local = 1'b1;
                proc_store_response = 1'b1;
            end
            // Miss: response stays 0 (stall), line will be fetched via MSHR
        end
    end

    // Oldest miss address logic - prioritize store misses over load misses
    always_comb begin
        oldest_miss_addr = '0;
        
        // If store misses the cache, request the line
        if (proc_store_valid && !store_hit_dcache) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr.tag = proc_store_addr[31:3];  // Full tag for 8-byte lines
            oldest_miss_addr.addr.block_offset = '0;  // Request full line
            oldest_miss_addr.addr.zeros = '0;
        end
        // Otherwise check for load misses
        else if (read_addrs[0].valid && !dcache_outs[0].valid) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[0].addr;
        end else if (read_addrs[1].valid && !dcache_outs[1].valid) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[1].addr;
        end
    end

    // Memory write logic - send dirty evictions to memory
    // Since we removed victim cache, evictions go directly to memory
    always_comb begin
        mem_write_valid = evicted_valid && evicted_line.dirty;
        mem_write_addr = '0;
        mem_write_data = '0;
        
        if (mem_write_valid) begin
            mem_write_addr.valid = 1'b1;
            // Reconstruct D_ADDR from stored tag (matches mem_fu.sv format)
            mem_write_addr.addr = '{zeros: 16'b0,
                                   tag: evicted_line.tag,
                                   block_offset: '0};
            mem_write_data = evicted_line.data;
        end
    end

    // Memory read request logic - handle cache misses
    always_comb begin
        mem_req_addr = '0;
        
        // Send read requests for cache misses
        // ARBITRATION: Prioritize dirty writeback over read request
        // DUPLICATE CHECK: Only send if not already in MSHR
        if (oldest_miss_addr.valid && !mem_write_valid && !mshr_addr_found) begin
            mem_req_addr = oldest_miss_addr;
        end
    end

    // MSHR entry logic - add when request is accepted
    always_comb begin
        new_mshr_entry = '0;
        if (mem_req_accepted && current_req_tag != 0) begin
            new_mshr_entry = '{valid: 1'b1,
                              mem_tag: current_req_tag,
                              d_addr: mem_req_addr.addr};
        end
    end

    // D-Cache Subsystem Display - Mem FU Interface
    always_ff @(posedge clock) begin
        if (!reset) begin
            $display("========================================");
            $display("=== D-CACHE SUBSYSTEM (Cycle %0t) ===", $time);
            $display("========================================");
            
            // Mem FU Requests
            $display("--- Mem FU Requests ---");
            for (int i = 0; i < 2; i++) begin
                if (read_addrs[i].valid) begin
                    $display("  Read[%0d]: Valid=1 Addr.tag=%h Addr.block_offset=%0d", 
                             i, read_addrs[i].addr.tag, read_addrs[i].addr.block_offset);
                end else begin
                    $display("  Read[%0d]: Valid=0", i);
                end
            end
            
            // Our Responses
            $display("--- Cache Responses ---");
            for (int i = 0; i < 2; i++) begin
                if (cache_outs[i].valid) begin
                    $display("  Out[%0d]: Valid=1 Data=%h", i, cache_outs[i].data.dbbl_level);
                end else begin
                    $display("  Out[%0d]: Valid=0 (miss)", i);
                end
            end
            
            // Memory Requests We're Making
            $display("--- Memory Requests ---");
            if (mem_req_addr.valid) begin
                $display("  Read Request: Valid=1 Addr.tag=%h Addr.block_offset=%0d Accepted=%0d", 
                         mem_req_addr.addr.tag, mem_req_addr.addr.block_offset, mem_req_accepted);
            end else begin
                $display("  Read Request: Valid=0");
            end
            
            // Memory Writebacks
            if (mem_write_valid) begin
                $display("  Writeback: Valid=1 Addr.tag=%h Data=%h", 
                         mem_write_addr.addr.tag, mem_write_data.dbbl_level);
            end else begin
                $display("  Writeback: Valid=0");
            end
            
            // Memory Data Coming Back
            if (mem_data_tag != 0) begin
                $display("  Memory Data Returned: Tag=%0d Data=%h", 
                         mem_data_tag, mem_data.dbbl_level);
            end
            
            // Store Interface
            $display("--- Store Interface ---");
            if (proc_store_valid) begin
                $display("  Store Request: Valid=1 Addr=%h Data=%h Hit=%0d Response=%0d", 
                         proc_store_addr, proc_store_data, store_hit_dcache, proc_store_response);
            end else begin
                $display("  Store Request: Valid=0");
            end
            
            // MSHR State
            $display("--- MSHR State ---");
            $display("  Oldest Miss Addr: Valid=%0d Tag=%h", 
                     oldest_miss_addr.valid, oldest_miss_addr.addr.tag);
            $display("  MSHR Addr Found (duplicate): %0d", mshr_addr_found);
            $display("  Cache Full: %0d", dcache_full);
            
            // Refill State
            if (dcache_write_addr_refill_local.valid) begin
                $display("  Refill: Valid=1 Addr.tag=%h", dcache_write_addr_refill_local.addr.tag);
            end
            
            // Eviction State
            if (evicted_valid) begin
                $display("  Eviction: Valid=1 Dirty=%0d Tag=%h", 
                         evicted_line.dirty, evicted_line.tag);
            end
            
            $display("");
        end
    end

endmodule

// D-Cache MSHR (Miss Status Handling Register)
// Tracks outstanding memory requests
module d_mshr #(
    parameter MSHR_WIDTH = `NUM_MEM_TAGS + `N
) (
    input clock,
    input reset,

    // Duplicate request detection
    input  D_ADDR snooping_addr,
    output logic  addr_found,

    // When mem_req_accepted
    input D_MSHR_PACKET new_entry,

    // Mem data back
    input  MEM_TAG       mem_data_tag,
    output D_ADDR_PACKET mem_data_d_addr
);

    localparam D_CACHE_INDEX_BITS = $clog2(MSHR_WIDTH);
    D_MSHR_PACKET [MSHR_WIDTH-1:0] mshr_entries, next_mshr_entries;
    logic [D_CACHE_INDEX_BITS-1:0] head, next_head, tail, next_tail;

    // Snooping logic - check if address is already in MSHR
    always_comb begin
        addr_found = 1'b0;
        for (int i = 0; i < MSHR_WIDTH; i++) begin
            if (mshr_entries[i].valid && (mshr_entries[i].d_addr.tag == snooping_addr.tag)) begin
                addr_found = 1'b1;
            end
        end
    end

    // MSHR logic
    logic pop_condition, push_condition;
    logic pop_cond_has_data, pop_cond_head_valid, pop_cond_tag_match;
    
    always_comb begin
        next_head = head;
        next_tail = tail;
        mem_data_d_addr = '0;
        next_mshr_entries = mshr_entries;

        // Data returned from Memory, Pop MSHR Entry
        pop_cond_has_data = (mem_data_tag != '0);
        pop_cond_head_valid = mshr_entries[head].valid;
        pop_cond_tag_match = (mem_data_tag == mshr_entries[head].mem_tag);
        pop_condition = pop_cond_has_data && pop_cond_head_valid && pop_cond_tag_match;
        
        if (pop_condition) begin
            next_head = D_CACHE_INDEX_BITS'((head + 1'b1) % MSHR_WIDTH);
            next_mshr_entries[head].valid = '0;
            mem_data_d_addr.valid = 1'b1;
            mem_data_d_addr.addr = mshr_entries[head].d_addr;
        end

        // New memory request, push new MSHR Entry
        if (new_entry.valid) begin
            next_mshr_entries[tail] = new_entry;
            next_tail = D_CACHE_INDEX_BITS'((tail + 1'b1) % MSHR_WIDTH);
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            head <= 1'b0;
            tail <= 1'b0;
            mshr_entries <= 1'b0;
        end else begin
            head <= next_head;
            tail <= next_tail;
            mshr_entries <= next_mshr_entries;
        end
    end

    // MSHR Debug Display
    always_ff @(posedge clock) begin
        if (!reset) begin
            $display("--- D-Cache MSHR State ---");
            $display("  Head: %0d, Tail: %0d", head, tail);
            $display("  Entries:");
            for (int i = 0; i < MSHR_WIDTH; i++) begin
                if (mshr_entries[i].valid) begin
                    $display("    [%0d] Valid=1, MemTag=%0d, DAddr.tag=%h", 
                             i, mshr_entries[i].mem_tag, mshr_entries[i].d_addr.tag);
                end
            end
            if (mem_data_tag != 0) begin
                $display("  Memory Data Returned: Tag=%0d", mem_data_tag);
            end
            if (new_entry.valid) begin
                $display("  New Entry Pushed: MemTag=%0d, DAddr.tag=%h", 
                         new_entry.mem_tag, new_entry.d_addr.tag);
            end
            $display("");
        end
    end

endmodule

// D-Cache module - fully associative cache with byte-enable store support
module dcache #(
    parameter MEM_DEPTH = `DCACHE_LINES,
    parameter D_CACHE_INDEX_BITS = $clog2(MEM_DEPTH),
    parameter MEM_WIDTH = 1 + 1 + `DTAG_BITS + `MEM_BLOCK_BITS  // valid + dirty + tag + data
) (
    input clock,
    input reset,

    // Memory operations read
    input D_ADDR_PACKET [1:0] read_addrs,
    output CACHE_DATA [1:0] cache_outs,

    // Store hit snooping
    input  D_ADDR_PACKET snooping_addr,
    output logic         addr_found,
    output logic         full,

    // Dcache write (refill from memory)
    input D_ADDR_PACKET write_addr,
    input MEM_BLOCK     write_data,
    
    // Store update interface (byte-granular)
    input logic         store_en,
    input D_ADDR_PACKET store_addr,
    input MEM_BLOCK     store_data,
    input logic [7:0]   store_byte_en,  // Byte enable mask
    
    // Eviction interface
    output D_CACHE_LINE evicted_line,
    output logic        evicted_valid,
    // debug to expose DCache to testbench
    output D_CACHE_LINE [MEM_DEPTH-1:0]      cache_lines_debug
);

    CACHE_DATA [1:0]                  cache_outs_temp;
    D_CACHE_LINE [MEM_DEPTH-1:0]      cache_lines;
    D_CACHE_LINE                      cache_line_write;
    logic [MEM_DEPTH-1:0]             cache_write_enable_mask;
    logic [MEM_DEPTH-1:0]             cache_write_no_evict_one_hot;
    logic [D_CACHE_INDEX_BITS-1:0]    cache_write_evict_index;
    logic [D_CACHE_INDEX_BITS-1:0]    lfsr_out;
    logic [MEM_DEPTH-1:0]             valid_bits;

    // Hit logic for stores
    logic [D_CACHE_INDEX_BITS-1:0]    hit_index;
    logic                             hit_valid;

    assign cache_lines_debug = cache_lines;
    memDP #(
        .WIDTH(MEM_WIDTH),
        .DEPTH(1'b1)
    ) cache_line[MEM_DEPTH-1:0] (
        .clock(clock),
        .reset(reset),
        .re(1'b1),
        .raddr(1'b0),
        .rdata(cache_lines),
        .we(cache_write_enable_mask),
        .waddr(1'b0),
        .wdata(cache_line_write)
    );

    // Write selection - find free slot
    psel_gen #(
        .WIDTH(MEM_DEPTH),
        .REQS(1'b1)
    ) psel_gen_inst (
        .req(~valid_bits),
        .gnt(cache_write_no_evict_one_hot)
    );

    // LFSR for random eviction
    LFSR #(
        .WIDTH(D_CACHE_INDEX_BITS)
    ) LFSR_inst (
        .clk(clock),
        .rst(reset),
        .op(lfsr_out)
    );
    
    assign cache_write_evict_index = D_CACHE_INDEX_BITS'(lfsr_out % MEM_DEPTH);

    // Hit detection for store snooping
    always_comb begin
        addr_found = 1'b0;
        hit_index = '0;
        hit_valid = 1'b0;
        for (int i = 0; i < MEM_DEPTH; i++) begin
            if (snooping_addr.valid && cache_lines[i].valid && 
                (snooping_addr.addr.tag == cache_lines[i].tag)) begin
                addr_found = 1'b1;
                hit_index = D_CACHE_INDEX_BITS'(i);
                hit_valid = 1'b1;
            end
        end
    end

    // Full detection
    always_comb begin
        for (int i = 0; i < MEM_DEPTH; i++) begin
            valid_bits[i] = cache_lines[i].valid;
        end
        full = &valid_bits;
    end

    // Cache write logic with byte-enable support for stores
    MEM_BLOCK merged_data;
    
    always_comb begin
        cache_write_enable_mask = '0;
        cache_line_write = '0;
        evicted_line = '0;
        evicted_valid = 1'b0;
        merged_data = '0;

        // Priority 1: Refill (allocating new line from memory)
        if (write_addr.valid) begin
            cache_line_write = '{valid: 1'b1,
                                dirty: 1'b0,  // Data from memory is clean
                                tag: write_addr.addr.tag,
                                data: write_data};
            
            // Try to find a free slot first
            if (|cache_write_no_evict_one_hot) begin
                cache_write_enable_mask = cache_write_no_evict_one_hot;
            end else begin
                // No free slot, evict using LFSR-selected index
                cache_write_enable_mask[cache_write_evict_index] = 1'b1;
                evicted_line = cache_lines[cache_write_evict_index];
                evicted_valid = cache_lines[cache_write_evict_index].valid;
            end
        end
        // Priority 2: Store update (hit only - merge with existing data)
        else if (store_en && hit_valid) begin
            cache_write_enable_mask[hit_index] = 1'b1;
            
            // Byte-granular merge: only update bytes enabled by store_byte_en
            merged_data = cache_lines[hit_index].data;
            for (int b = 0; b < 8; b++) begin
                if (store_byte_en[b]) begin
                    merged_data.byte_level[b] = store_data.byte_level[b];
                end
            end

            cache_line_write = '{valid: 1'b1,
                                dirty: cache_lines[hit_index].dirty || (merged_data != cache_lines[hit_index].data),
                                tag: cache_lines[hit_index].tag,
                                data: merged_data};
        end
    end

    // Cache read logic
    always_comb begin
        cache_outs_temp = '0;
        for (int j = 0; j < 2; j++) begin
            for (int i = 0; i < MEM_DEPTH; i++) begin
                if (read_addrs[j].valid && cache_lines[i].valid && 
                    (read_addrs[j].addr.tag == cache_lines[i].tag)) begin
                    cache_outs_temp[j].data = cache_lines[i].data;
                    cache_outs_temp[j].valid = 1'b1;
                end
            end
        end
    end
    assign cache_outs = cache_outs_temp;

    // D-Cache State Display
    // Format matches final memory output: @@@ mem[%5d] = %h : %0d
    always_ff @(posedge clock) begin
        if (!reset) begin
            logic [31:0] byte_addr;
            $display("========================================");
            $display("=== D-CACHE STATE (Cycle %0t) ===", $time);
            $display("========================================");
            $display("Cache Lines (Total: %0d):", MEM_DEPTH);
            for (int i = 0; i < MEM_DEPTH; i++) begin
                if (cache_lines[i].valid) begin
                    // Convert tag to byte address: tag is addr[31:3], so byte_addr = tag << 3
                    byte_addr = {cache_lines[i].tag, 3'b0};
                    if (cache_lines[i].dirty) begin
                        $display("  [%2d] @@@ mem[%5d] = %016h : %0d (DIRTY)", 
                                 i, byte_addr, cache_lines[i].data.dbbl_level,
                                 cache_lines[i].data.dbbl_level);
                    end else begin
                        $display("  [%2d] @@@ mem[%5d] = %016h : %0d", 
                                 i, byte_addr, cache_lines[i].data.dbbl_level,
                                 cache_lines[i].data.dbbl_level);
                    end
                end else begin
                    $display("  [%2d] (empty)", i);
                end
            end
            $display("Cache Full: %0d", full);
            $display("");
        end
    end

endmodule