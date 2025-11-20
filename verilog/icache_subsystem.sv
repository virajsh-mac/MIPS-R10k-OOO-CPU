`include "verilog/sys_defs.svh"

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
    input  logic                mem_req_accepted,

    // Debug outputs
    output I_ADDR_PACKET [1:0]  read_addrs_dbg,
    output CACHE_DATA [1:0]     cache_outs_dbg,
    output logic [1:0]          icache_hits_dbg,
    output logic [1:0]          icache_misses_dbg,
    output logic                icache_full_dbg,
    output I_ADDR_PACKET        prefetch_addr_dbg,
    output I_ADDR_PACKET        oldest_miss_addr_dbg,
    output logic                mshr_addr_found_dbg,
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] mshr_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] mshr_tail_dbg,
    output MSHR_PACKET [`NUM_MEM_TAGS-1:0]   mshr_entries_dbg,
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] mshr_next_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] mshr_next_tail_dbg,
    output logic                             mshr_pop_condition_dbg,
    output logic                             mshr_push_condition_dbg,
    output logic                             mshr_pop_cond_has_data_dbg,
    output logic                             mshr_pop_cond_head_valid_dbg,
    output logic                             mshr_pop_cond_tag_match_dbg,
    output logic                mem_write_icache_dbg,
    output I_ADDR_PACKET        mem_write_addr_dbg,
    output MEM_BLOCK            mem_data_dbg,
    output MEM_TAG              mem_data_tag_dbg,
    output I_ADDR_PACKET        icache_write_addr_dbg,
    output MEM_BLOCK            icache_write_data_dbg,
    output I_CACHE_LINE         icache_line_write_dbg,
    output logic [(`ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE)-1:0] icache_write_enable_mask_dbg
);

    // Internal wires
    I_ADDR_PACKET prefetcher_snooping_addr, icache_write_addr, oldest_miss_addr;
    logic icache_full, snooping_found_icache, snooping_found_mshr;
    MSHR_PACKET new_mshr_entry;
    
    // ICache debug wires
    I_CACHE_LINE icache_line_write;
    logic [(`ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE)-1:0] icache_write_enable_mask;

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
        .write_data   (mem_data),
        // Debug outputs
        .hits_dbg     (icache_hits_dbg),
        .misses_dbg   (icache_misses_dbg),
        .write_addr_dbg(icache_write_addr_dbg),
        .write_data_dbg(icache_write_data_dbg),
        .cache_line_write_dbg(icache_line_write),
        .cache_write_enable_mask_dbg(icache_write_enable_mask)
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
        .mem_data_i_addr(icache_write_addr),
        // Debug outputs
        .head_dbg       (mshr_head_dbg),
        .tail_dbg       (mshr_tail_dbg),
        .entries_dbg    (mshr_entries_dbg),
        .next_head_dbg  (mshr_next_head_dbg),
        .next_tail_dbg  (mshr_next_tail_dbg),
        .pop_condition_dbg(mshr_pop_condition_dbg),
        .push_condition_dbg(mshr_push_condition_dbg),
        .pop_cond_has_data_dbg(mshr_pop_cond_has_data_dbg),
        .pop_cond_head_valid_dbg(mshr_pop_cond_head_valid_dbg),
        .pop_cond_tag_match_dbg(mshr_pop_cond_tag_match_dbg)
    );

    // Oldest miss address logic
    always_comb begin
        oldest_miss_addr = '0;
        if (read_addrs[0].valid && (cache_outs[0].valid == 1'b0)) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[0].addr;
        end else if (read_addrs[1].valid && (cache_outs[1].valid == 1'b0)) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[1].addr;
        end
    end

    // Mem request address logic
    always_comb begin
        mem_req_addr = '0;  // Default assignment to prevent latch
        if (~snooping_found_icache & ~snooping_found_mshr) begin
            mem_req_addr = prefetcher_snooping_addr;
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

    // Debug signal assignments
    assign read_addrs_dbg = read_addrs;
    assign cache_outs_dbg = cache_outs;
    assign icache_full_dbg = icache_full;
    assign prefetch_addr_dbg = prefetcher_snooping_addr;
    assign oldest_miss_addr_dbg = oldest_miss_addr;
    assign mshr_addr_found_dbg = snooping_found_mshr;
    assign mem_write_icache_dbg = icache_write_addr.valid;
    assign mem_write_addr_dbg = icache_write_addr;
    assign mem_data_dbg = mem_data;
    assign mem_data_tag_dbg = mem_data_tag;
    assign icache_line_write_dbg = icache_line_write;
    assign icache_write_enable_mask_dbg = icache_write_enable_mask;

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
    output I_ADDR_PACKET mem_data_i_addr,  // to write to icache

    // Debug outputs
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] tail_dbg,
    output MSHR_PACKET [`NUM_MEM_TAGS-1:0]   entries_dbg,
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] next_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS)-1:0] next_tail_dbg,
    output logic                             pop_condition_dbg,
    output logic                             push_condition_dbg,
    output logic                             pop_cond_has_data_dbg,
    output logic                             pop_cond_head_valid_dbg,
    output logic                             pop_cond_tag_match_dbg
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

    // Debug signal assignments
    assign head_dbg = head;
    assign tail_dbg = tail;
    assign entries_dbg = mshr_entries;
    assign next_head_dbg = next_head;
    assign next_tail_dbg = next_tail;
    assign pop_condition_dbg = pop_condition;
    assign push_condition_dbg = push_condition;
    assign pop_cond_has_data_dbg = pop_cond_has_data;
    assign pop_cond_head_valid_dbg = pop_cond_head_valid;
    assign pop_cond_tag_match_dbg = pop_cond_tag_match;

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
        end 
        // TODO: WARNING PREFETCHER DISABLED BECAUSE ITS OVERRIDING THE OLDEST MISS 
        // right now the prefetcher is the only thing sending out vaild memory requests
        // else if (~icache_full & last_icache_miss_mem_req.valid) begin
        //     // Send lookahead snooping request
        //     prefetcher_snooping_addr.valid = '1;
        //     prefetcher_snooping_addr.addr  = addr_incrementor + 'h4;
        //     if (mem_req_accepted) begin
        //         next_addr_incrementor = addr_incrementor + 'h4;
        //     end
        // end
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

module icache (
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
    input MEM_BLOCK     write_data,

    // Debug outputs
    output logic [1:0]   hits_dbg,
    output logic [1:0]   misses_dbg,
    output I_ADDR_PACKET write_addr_dbg,
    output MEM_BLOCK     write_data_dbg,
    output I_CACHE_LINE  cache_line_write_dbg,
    output logic [(`ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE)-1:0] cache_write_enable_mask_dbg
);

    localparam MEM_DEPTH = `ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE;
    localparam I_CACHE_INDEX_BITS = $clog2(MEM_DEPTH);
    localparam MEM_WIDTH = 1 + `ITAG_BITS + `MEM_BLOCK_BITS;

    wor CACHE_DATA [1:0]                  cache_outs_temp;
    I_CACHE_LINE [MEM_DEPTH-1:0]          cache_lines;
    I_CACHE_LINE                          cache_line_write;
    logic [MEM_DEPTH-1:0]                 cache_write_enable_mask;
    logic [MEM_DEPTH-1:0]                 cache_write_no_evict_one_hot;
    logic [I_CACHE_INDEX_BITS-1:0]        cache_write_evict_index;
    logic [MEM_DEPTH-1:0]                 cache_write_evict_one_hot;

    logic [1:0][MEM_DEPTH-1:0]            cache_reads_one_hot;
    logic [1:0][I_CACHE_INDEX_BITS-1:0]   cache_reads_index;

    logic [MEM_DEPTH-1:0]                 snooping_one_hot;
    logic [MEM_DEPTH-1:0]                 valid_bits;

    memDP #(
        .WIDTH(MEM_WIDTH),
        .DEPTH(1'b1),
        .READ_PORTS(1'b1),
        .BYPASS_EN(1'b0)
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
        .NUM_BITS (I_CACHE_INDEX_BITS)
    ) LFSR_inst (
        .clock(clock),
        .reset(reset),
        .seed_data(I_CACHE_INDEX_BITS'(`LFSR_SEED)),
        .data_out(cache_write_evict_index)
    );

    // Cache write logic
    for (genvar k = 0; k < MEM_DEPTH; k++) begin
        assign cache_write_evict_one_hot[k] = (cache_write_evict_index == k);
    end
    
    assign cache_write_enable_mask = write_addr.valid ? 
                                    (|cache_write_no_evict_one_hot ? cache_write_no_evict_one_hot : cache_write_evict_one_hot) : 
                                    '0;

    assign cache_line_write = '{valid: write_addr.valid,
                                tag: write_addr.addr.tag,
                                data: write_data};

    // Prefetch snooping logic
    for (genvar i = 0; i < MEM_DEPTH; i++) begin
        assign snooping_one_hot[i] = (snooping_addr.addr.tag == cache_lines[i].tag) & 
                                      snooping_addr.valid & 
                                      cache_lines[i].valid;
    end
    assign addr_found = |snooping_one_hot;

    for (genvar i = 0; i < MEM_DEPTH; i++) begin
        assign valid_bits[i] = cache_lines[i].valid;
    end
    assign full = &valid_bits;

    // Cache read logic
    for (genvar j = 0; j <= 1; j++) begin
        for (genvar i = 0; i < MEM_DEPTH; i++) begin
            assign cache_reads_one_hot[j][i] = (read_addrs[j].addr.tag == cache_lines[i].tag) &
                                                read_addrs[j].valid &
                                                cache_lines[i].valid;

            assign cache_outs_temp[j].data = cache_lines[i].data & {`MEM_BLOCK_BITS{cache_reads_one_hot[j][i]}};
        end
        // Assign valid bit based on whether any cache line matched (hit)
        assign cache_outs_temp[j].valid = |cache_reads_one_hot[j];
    end
    assign cache_outs = cache_outs_temp;

    // Debug signal assignments
    assign hits_dbg[0] = read_addrs[0].valid & (|cache_reads_one_hot[0]);
    assign hits_dbg[1] = read_addrs[1].valid & (|cache_reads_one_hot[1]);
    assign misses_dbg[0] = read_addrs[0].valid & ~(|cache_reads_one_hot[0]);
    assign misses_dbg[1] = read_addrs[1].valid & ~(|cache_reads_one_hot[1]);
    assign write_addr_dbg = write_addr;
    assign write_data_dbg = write_data;
    assign cache_line_write_dbg = cache_line_write;
    assign cache_write_enable_mask_dbg = cache_write_enable_mask;

endmodule
