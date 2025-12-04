`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module icache_subsystem (
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

module i_prefetcher (
    input clock,
    input reset,

    input I_ADDR_PACKET icache_miss_addr,
    input logic         icache_full,

    input  logic         mem_req_accepted,
    output I_ADDR_PACKET prefetcher_snooping_addr
);
    I_ADDR_PACKET last_icache_miss_mem_req, next_last_icache_miss_mem_req;
    I_ADDR addr_incrementor, next_addr_incrementor;

    always_comb begin
        prefetcher_snooping_addr = '0;
        next_addr_incrementor = addr_incrementor;
        next_last_icache_miss_mem_req = last_icache_miss_mem_req;

        // New or first icache miss yet to successfully request
        if (icache_miss_addr.valid & (icache_miss_addr.addr != last_icache_miss_mem_req.addr | ~last_icache_miss_mem_req.valid)) begin
            // Send mem snooping request
            prefetcher_snooping_addr.valid = '1;
            prefetcher_snooping_addr.addr  = icache_miss_addr.addr;
            if (mem_req_accepted) begin
                next_last_icache_miss_mem_req.valid = '1;
                next_last_icache_miss_mem_req.addr = icache_miss_addr.addr;
                next_addr_incrementor = icache_miss_addr.addr;
            end 
        end else if (~icache_full & last_icache_miss_mem_req.valid) begin
            // Send lookahead snooping request
            prefetcher_snooping_addr.valid = '1;
            prefetcher_snooping_addr.addr  = addr_incrementor + 'h8;
            if (mem_req_accepted) begin
                next_addr_incrementor = addr_incrementor + 'h8;
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            last_icache_miss_mem_req <= '0;
            addr_incrementor <= '0;
        end else begin
            addr_incrementor <= next_addr_incrementor;
            last_icache_miss_mem_req <= next_last_icache_miss_mem_req;
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

    CACHE_DATA [1:0]                  cache_outs_temp;
    I_CACHE_LINE [MEM_DEPTH-1:0]          cache_lines;
    I_CACHE_LINE                          cache_line_write;
    logic [MEM_DEPTH-1:0]                 cache_write_enable_mask;
    logic [MEM_DEPTH-1:0]                 cache_write_no_evict_one_hot;
    logic [I_CACHE_INDEX_BITS-1:0]        cache_write_evict_index;
    logic [I_CACHE_INDEX_BITS-1:0]        lfsr_out;
    logic [MEM_DEPTH-1:0]                 valid_bits;

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

    // Write selection random eviction
    LFSR #(
        .WIDTH(I_CACHE_INDEX_BITS)
    ) LFSR_inst (
        .clk(clock),
        .rst(reset),
        .op(lfsr_out)
    );
    
    // Extend LFSR output to full index width (modulo handles out-of-range)
    assign cache_write_evict_index = I_CACHE_INDEX_BITS'(lfsr_out % MEM_DEPTH);


    // Cache write logic
    always_comb begin
        cache_write_enable_mask = '0;
        cache_line_write = '{valid: write_addr.valid,
                            tag: write_addr.addr.tag,
                            data: write_data};
        
        if (write_addr.valid) begin
            // Try to find an invalid (free) slot first
            if (|cache_write_no_evict_one_hot) begin
                cache_write_enable_mask = cache_write_no_evict_one_hot;
            end else begin
                // No free slot, evict using LFSR-selected index
                cache_write_enable_mask[cache_write_evict_index] = 1'b1;
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

endmodule
