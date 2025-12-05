`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module icache_subsystem_optimized (
    input clock,
    input reset,

    // Fetch
    input  I_ADDR_PACKET [1:0]  read_addrs,  // read_addr[0] is older instructions
    output CACHE_DATA [1:0]     cache_outs,

    // Mem.sv IOs
    input MEM_TAG               current_req_tag,
    input MEM_BLOCK             mem_data,
    input MEM_TAG               mem_data_tag,

    // Arbitor IOs
    output I_ADDR_PACKET        mem_req_addr,
    input  logic                mem_req_accepted
);

    // Internal wires
    I_ADDR_PACKET prefetcher_snooping_addr, icache_write_addr, oldest_miss_addr;
    logic icache_full, snooping_found_icache, snooping_found_mshr;
    MSHR_PACKET new_mshr_entry;

    icache icache_inst (
        .clock        (clock),
        .reset        (reset),
        // Fetch Stage read
        .read_addrs   (read_addrs),
        .cache_outs   (cache_outs),
        // Prefetch snooping
        .snooping_addr(prefetcher_snooping_addr),
        .addr_found   (snooping_found_icache),
        .full         (icache_full),
        // Icache write mem_data, when mem_data_tag matches head of MSHR
        .write_addr   (icache_write_addr),
        .write_data   (mem_data)
    );

    i_prefetcher i_prefetcher_inst (
        .clock                   (clock),
        .reset                   (reset),
        .icache_miss_addr        (oldest_miss_addr),
        .icache_full             (icache_full),
        .mem_req_accepted        (mem_req_accepted),
        .prefetcher_snooping_addr(prefetcher_snooping_addr)
    );

    i_mshr i_mshr_inst (
        .clock          (clock),
        .reset          (reset),
        // Prefetch snooping
        .snooping_addr  (prefetcher_snooping_addr.addr),
        .addr_found     (snooping_found_mshr),
        // When mem_req_accepted
        .new_entry      (new_mshr_entry),
        // Mem data back
        .mem_data_tag   (mem_data_tag),
        .mem_data_i_addr(icache_write_addr)
    );

    // Oldest miss address logic
    always_comb begin
        oldest_miss_addr = '0;
        if (read_addrs[0].valid && !cache_outs[0].valid) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[0].addr;
        end else if (read_addrs[1].valid && !cache_outs[1].valid) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[1].addr;
        end
    end

    // Mem request address logic
    always_comb begin
        mem_req_addr = '0;
        if (~snooping_found_icache & ~snooping_found_mshr) begin
            mem_req_addr = {prefetcher_snooping_addr[32:3], 3'b0};
        end
    end

    // MSHR entry logic - add immediately when request is accepted and tag is valid
    always_comb begin
        new_mshr_entry = '0;
        if (mem_req_accepted && (current_req_tag != 0)) begin
            new_mshr_entry.valid   = '1;
            new_mshr_entry.mem_tag = current_req_tag;
            new_mshr_entry.i_tag   = mem_req_addr.addr.tag;
        end
    end

endmodule

// this should never be full, so no logic for handling full FIFO head tail edge case
module i_mshr #(
    parameter MSHR_WIDTH = `NUM_MEM_TAGS + `N
) (
    input clock,
    input reset,

    // Prefetch snooping
    input  I_ADDR snooping_addr,  // to decide whether to send mem request
    output logic  addr_found,

    // When mem_req_accepted
    input MSHR_PACKET new_entry,

    // Mem data back
    input  MEM_TAG       mem_data_tag,
    output I_ADDR_PACKET mem_data_i_addr  // to write to icache
);

    // MSHR Internals
    localparam I_CACHE_INDEX_BITS = $clog2(MSHR_WIDTH);
    MSHR_PACKET [MSHR_WIDTH-1:0] mshr_entries, next_mshr_entries;
    logic [I_CACHE_INDEX_BITS-1:0] head, next_head, tail, next_tail;

    // Snooping logic
    logic [MSHR_WIDTH-1:0] snooping_one_hot;
    for (genvar i = 0; i < MSHR_WIDTH; i++) begin
        assign snooping_one_hot[i] = mshr_entries[i].valid && (mshr_entries[i].i_tag == snooping_addr.tag);
    end
    assign addr_found = |snooping_one_hot;

    // MSHR logic
    logic pop_condition, push_condition;
    logic pop_cond_has_data, pop_cond_head_valid, pop_cond_tag_match;
    
    always_comb begin
        next_head = head;
        next_tail = tail;
        mem_data_i_addr = '0;
        next_mshr_entries = mshr_entries;

        // Data returned from Memory, Pop MSHR Entry
        pop_cond_has_data = (mem_data_tag != '0);
        pop_cond_head_valid = mshr_entries[head].valid;
        pop_cond_tag_match = (mem_data_tag == mshr_entries[head].mem_tag);
        pop_condition = pop_cond_has_data && pop_cond_head_valid && pop_cond_tag_match;
        
        if (pop_condition) begin
            next_head = I_CACHE_INDEX_BITS'((head + 1'b1) % MSHR_WIDTH);
            next_mshr_entries[head].valid = '0;
            mem_data_i_addr.valid = 1'b1;
            mem_data_i_addr.addr = '{zeros: 16'b0, 
                                     tag: mshr_entries[head].i_tag, 
                                     block_offset: 3'b0};
        end

        // New memory request, push new MSHR Entry
        push_condition = new_entry.valid;
        if (push_condition) begin
            next_mshr_entries[tail] = new_entry;
            next_tail = I_CACHE_INDEX_BITS'((tail + 1'b1) % MSHR_WIDTH);
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

endmodule

module i_prefetcher #(
    parameter PREFETCH_DEPTH = 6
) (
    input clock,
    input reset,

    input I_ADDR_PACKET icache_miss_addr,
    input logic         icache_full,

    input  logic         mem_req_accepted,
    output I_ADDR_PACKET prefetcher_snooping_addr
);
    I_ADDR_PACKET last_icache_miss_mem_req, next_last_icache_miss_mem_req;
    I_ADDR addr_incrementor, next_addr_incrementor;
    logic [$clog2(PREFETCH_DEPTH + 1):0] prefetch_count, next_prefetch_count;

    always_comb begin
        prefetcher_snooping_addr = '0;
        next_addr_incrementor = addr_incrementor;
        next_last_icache_miss_mem_req = last_icache_miss_mem_req;
        next_prefetch_count = prefetch_count;  // Preserve current count by default

        // New or first icache miss yet to successfully request
        if (icache_miss_addr.valid & (icache_miss_addr.addr != last_icache_miss_mem_req.addr | ~last_icache_miss_mem_req.valid)) begin
            prefetcher_snooping_addr.valid = '1;
            prefetcher_snooping_addr.addr  = icache_miss_addr.addr;
            next_prefetch_count = '0;  // Reset counter on new miss
            if (mem_req_accepted) begin
                next_last_icache_miss_mem_req.valid = '1;
                next_last_icache_miss_mem_req.addr = icache_miss_addr.addr;
                next_addr_incrementor = icache_miss_addr.addr;
            end 
        // On every miss, prefetch until icache_full, or up to until PREFETCH_DEPTH
        end else if (last_icache_miss_mem_req.valid && (~icache_full || prefetch_count < PREFETCH_DEPTH) ) begin
            prefetcher_snooping_addr.valid = '1;
            prefetcher_snooping_addr.addr  = addr_incrementor + 'h8;
            if (mem_req_accepted) begin
                next_addr_incrementor = addr_incrementor + 'h8;
                next_prefetch_count = prefetch_count + 1;
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            last_icache_miss_mem_req <= '0;
            addr_incrementor <= '0;
            prefetch_count <= '0;
        end else begin
            addr_incrementor <= next_addr_incrementor;
            last_icache_miss_mem_req <= next_last_icache_miss_mem_req;
            prefetch_count <= next_prefetch_count;
        end
    end

endmodule

module icache #(
    parameter MEM_DEPTH = `ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE,
    parameter I_CACHE_INDEX_BITS = $clog2(MEM_DEPTH),
    parameter MEM_WIDTH = 1 + `ITAG_BITS + `MEM_BLOCK_BITS  // valid + tag + data
) (
    input clock,
    input reset,

    // Fetch Stage read
    input I_ADDR_PACKET [1:0] read_addrs,
    output CACHE_DATA [1:0] cache_outs,

    // Prefetch snooping
    input  I_ADDR_PACKET snooping_addr,  // to decide whether to send mem request
    output logic         addr_found,
    output logic         full,

    // Icache write mem_data, when mem_data_tag matches head of MSHR
    input I_ADDR_PACKET write_addr,
    input MEM_BLOCK     write_data
);

    CACHE_DATA [1:0]                      cache_outs_temp;
    I_CACHE_LINE [MEM_DEPTH-1:0]          cache_lines;
    I_CACHE_LINE                          cache_line_write;
    logic [MEM_DEPTH-1:0]                 cache_write_enable_mask;
    logic [MEM_DEPTH-1:0]                 cache_write_no_evict_one_hot;
    logic [I_CACHE_INDEX_BITS-1:0]        cache_write_evict_index;
    logic [MEM_DEPTH-1:0]                 valid_bits;
    
    // LRU signals
    logic [I_CACHE_INDEX_BITS-1:0]        lru_index;
    logic [I_CACHE_INDEX_BITS-1:0]        cpu_read_hit_index [1:0];
    logic [1:0]                           cpu_read_hit_valid;
    logic [I_CACHE_INDEX_BITS-1:0]        prefetch_write_index;
    logic                                 prefetch_write_valid;
    logic [I_CACHE_INDEX_BITS-1:0]        free_slot_index;  // Index from one_hot_to_index conversion

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

    // Write selection no eviction
    psel_gen #(
        .WIDTH(MEM_DEPTH),
        .REQS(1'b1)
    ) psel_gen_inst (
        .req(~valid_bits),
        .gnt(cache_write_no_evict_one_hot)
    );

    // Convert one-hot free slot selection to index
    one_hot_to_index #(
        .INPUT_WIDTH(MEM_DEPTH)
    ) one_hot_to_index_inst (
        .one_hot(cache_write_no_evict_one_hot),
        .index(free_slot_index)
    );

    // Pseudo Tree LRU for replacement policy
    // Pass both read ports separately for parallel processing
    // Read 1 = older instruction (index 0), Read 2 = newer instruction (index 1)
    pseudo_tree_lru #(
        .CACHE_SIZE(MEM_DEPTH),
        .INDEX_BITS(I_CACHE_INDEX_BITS)
    ) lru_inst (
        .clock(clock),
        .reset(reset),
        .read1_index(cpu_read_hit_index[0]),      // Older instruction (Read 1)
        .read1_valid(cpu_read_hit_valid[0]),
        .read2_index(cpu_read_hit_index[1]),      // Newer instruction (Read 2) - highest priority
        .read2_valid(cpu_read_hit_valid[1]),
        .write_index(prefetch_write_index),       // Prefetch write - lowest priority
        .write_valid(prefetch_write_valid),
        .lru_index(lru_index)
    );
    
    // Use LRU index for eviction
    assign cache_write_evict_index = lru_index;


    // Cache write logic and LRU update index determination
    always_comb begin
        // Default values
        prefetch_write_index = '0;
        prefetch_write_valid = 1'b0;
        cache_write_enable_mask = '0;
        cache_line_write = '{valid: write_addr.valid,
                            tag: write_addr.addr.tag,
                            data: write_data};
        
        if (write_addr.valid) begin
            if (|cache_write_no_evict_one_hot) begin
                // Use free slot: one-hot mask for write enable, index for LRU update
                cache_write_enable_mask = cache_write_no_evict_one_hot;
                prefetch_write_index = free_slot_index;
                prefetch_write_valid = 1'b1;
            end else begin
                // No free slot, evict using LRU-selected index
                cache_write_enable_mask[cache_write_evict_index] = 1'b1;
                prefetch_write_index = cache_write_evict_index;
                prefetch_write_valid = 1'b1;
            end
        end
    end

    // Prefetch snooping logic
    always_comb begin
        addr_found = 1'b0;
        for (int i = 0; i < MEM_DEPTH; i++) begin
            if (snooping_addr.valid && cache_lines[i].valid && 
                (snooping_addr.addr.tag == cache_lines[i].tag)) begin
                addr_found = 1'b1;
            end
        end
    end

    // Full detection
    always_comb begin
        // Extract valid bits
        for (int i = 0; i < MEM_DEPTH; i++) begin
            valid_bits[i] = cache_lines[i].valid;
        end
        full = &valid_bits;
    end

    // Cache read logic with hit index tracking for LRU
    always_comb begin
        cache_outs_temp = '0;
        cpu_read_hit_index[0] = '0;
        cpu_read_hit_index[1] = '0;
        cpu_read_hit_valid[0] = 1'b0;
        cpu_read_hit_valid[1] = 1'b0;
        
        for (int j = 0; j < 2; j++) begin
            for (int i = 0; i < MEM_DEPTH; i++) begin
                if (read_addrs[j].valid && cache_lines[i].valid && 
                    (read_addrs[j].addr.tag == cache_lines[i].tag)) begin
                    cache_outs_temp[j].data = cache_lines[i].data;
                    cache_outs_temp[j].valid = 1'b1;
                    // Track which cache line index was hit for LRU update
                    cpu_read_hit_index[j] = I_CACHE_INDEX_BITS'(i);
                    cpu_read_hit_valid[j] = 1'b1;
                end
            end
        end
    end
    assign cache_outs = cache_outs_temp;

endmodule

// Pseudo Tree LRU module for cache replacement policy
// Uses a binary tree structure with N-1 bits to track LRU state
// Each bit indicates which subtree is LRU (0 = left, 1 = right)
// Optimized for parallel path calculation with priority resolution
// Handles 3 simultaneous access ports: Read 1, Read 2, Write
// Priority: Read 2 (highest) > Read 1 > Write (lowest)
module pseudo_tree_lru #(
    parameter CACHE_SIZE = `ICACHE_LINES,
    parameter INDEX_BITS = $clog2(CACHE_SIZE)
) (
    input clock,
    input reset,

    // Read 1 access update interface (Instruction N - older)
    input logic [INDEX_BITS-1:0] read1_index,   // Cache line index accessed by Read 1
    input logic                  read1_valid,   // Whether Read 1 access occurred

    // Read 2 access update interface (Instruction N+1 - newer, highest priority)
    input logic [INDEX_BITS-1:0] read2_index,    // Cache line index accessed by Read 2
    input logic                  read2_valid,    // Whether Read 2 access occurred

    // Prefetch Write access update interface (lowest priority)
    input logic [INDEX_BITS-1:0] write_index,    // Cache line index written by prefetcher
    input logic                  write_valid,    // Whether prefetch write occurred

    // LRU output
    output logic [INDEX_BITS-1:0] lru_index     // Index of least recently used cache line
);

    // Tree structure: for N cache lines, we need N-1 internal nodes
    // Each node stores 1 bit indicating which subtree is LRU (0=left, 1=right)
    localparam TREE_NODES = CACHE_SIZE - 1;
    localparam TREE_NODE_BITS = $clog2(TREE_NODES + CACHE_SIZE + 1);  // Bits needed for tree node indices
    
    logic [TREE_NODES-1:0] lru_tree, next_lru_tree;
    
    // Parallel path calculation signals
    logic [TREE_NODES-1:0] r1_mask, r1_val;  // Read 1 update mask and values
    logic [TREE_NODES-1:0] r2_mask, r2_val;  // Read 2 update mask and values
    logic [TREE_NODES-1:0] w_mask,  w_val;   // Write update mask and values
    
    // LRU traversal variable
    logic [TREE_NODE_BITS-1:0] node_idx;

    // Helper function to calculate update path masks and values
    // This is purely combinational logic, executed in parallel
    function automatic void get_update_path(
        input logic [INDEX_BITS-1:0] idx,
        input logic valid,
        output logic [TREE_NODES-1:0] mask,
        output logic [TREE_NODES-1:0] val
    );
        logic [TREE_NODE_BITS-1:0] node_idx;
        logic [TREE_NODE_BITS-1:0] parent_idx;
        
        mask = '0;
        val  = '0;
        
        if (valid) begin
            node_idx = TREE_NODES + idx;
            
            // Traverse from leaf to root, calculating which nodes need updates
            for (int i = 0; i < INDEX_BITS; i++) begin
                if (node_idx > 0) begin
                    parent_idx = (node_idx - 1) >> 1;
                    if (parent_idx < TREE_NODES) begin
                        mask[parent_idx] = 1'b1;  // Mark this node as affected
                        
                        // Set value: Right child (even) accessed -> set parent to 0 (left subtree becomes LRU)
                        //            Left child (odd) accessed -> set parent to 1 (right subtree becomes LRU)
                        if ((node_idx & 1) == 0) begin
                            // Right child (even): set parent to 0 (left subtree becomes LRU, right is MRU)
                            val[parent_idx] = 1'b0;
                        end else begin
                            // Left child (odd): set parent to 1 (right subtree becomes LRU, left is MRU)
                            val[parent_idx] = 1'b1;
                        end
                        
                        node_idx = parent_idx;
                    end
                end
            end
        end
    endfunction

    // -----------------------------------------------------------
    // 1. Parallel Path Calculation
    // -----------------------------------------------------------
    // Calculate update paths for all 3 ports simultaneously
    always_comb begin
        get_update_path(read1_index, read1_valid, r1_mask, r1_val);
        get_update_path(read2_index, read2_valid, r2_mask, r2_val);
        get_update_path(write_index, write_valid, w_mask, w_val);
    end

    // -----------------------------------------------------------
    // 2. Priority Resolution (The "Merge")
    // -----------------------------------------------------------
    // Priority: Read 2 > Read 1 > Write > Keep Current
    always_comb begin
        for (int i = 0; i < TREE_NODES; i++) begin
            if (r2_mask[i]) begin
                // Read 2 has highest priority
                next_lru_tree[i] = r2_val[i];
            end else if (r1_mask[i]) begin
                // Read 1 has second priority
                next_lru_tree[i] = r1_val[i];
            end else if (w_mask[i]) begin
                // Write has third priority
                next_lru_tree[i] = w_val[i];
            end else begin
                // Keep current value
                next_lru_tree[i] = lru_tree[i];
            end
        end
    end

    // -----------------------------------------------------------
    // 3. LRU Index Output Logic
    // -----------------------------------------------------------
    // Find LRU index by traversing the tree from root to leaf
    always_comb begin
        lru_index = '0;
        node_idx = '0;
        
        // Traverse tree from root (node 0) to leaf following LRU bits
        for (int level = 0; level < INDEX_BITS; level++) begin
            if (node_idx < TREE_NODES) begin
                if (lru_tree[node_idx]) begin
                    // Right subtree is LRU, go right
                    node_idx = (node_idx << 1) + 2;  // Right child: 2*node + 2
                end else begin
                    // Left subtree is LRU, go left
                    node_idx = (node_idx << 1) + 1;  // Left child: 2*node + 1
                end
            end
        end
        
        // Convert final tree node index to cache line index
        // Cache index = leaf_node_index - TREE_NODES
        lru_index = INDEX_BITS'(node_idx - TREE_NODES);
    end

    // -----------------------------------------------------------
    // 4. Sequential State Update
    // -----------------------------------------------------------
    always_ff @(posedge clock) begin
        if (reset) begin
            lru_tree <= '0;  // Initialize: all bits 0 means left subtrees are LRU
        end else begin
            lru_tree <= next_lru_tree;
        end
    end

endmodule
