`include "verilog/sys_defs.svh"

module icache_subsystem (
    input clock,
    input reset,

    // Fetch
    input  ADDR       [1:0] read_addrs,  // read_addr[0] is older instructions
    output CACHE_DATA [1:0] cache_outs,

    // Mem.sv IOs
    input MEM_TAG   current_req_tag,
    input MEM_BLOCK mem_data,
    input MEM_TAG   mem_data_tag,

    // Arbitor IOs
    output I_ADDR_PACKET mem_req_addr,
    input  logic         mem_req_accepted
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
        .mem_data     (mem_data)
    );

    prefetcher prefetcher_inst (
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
        .snooping_addr  (prefetcher_snooping_addr),
        .addr_found     (snooping_found_mshr),
        // When mem_req_accepted
        .new_entry      (new_mshr_entry),
        // Mem data back
        .mem_data_tag   (mem_data_tag),
        .mem_data_i_addr(icache_write_addr)
    );

    // Oldest miss address logic
    always_comb begin
        oldest_miss_addr.valid = '0;
        if (read_addrs[0].valid & ~cache_outs[0].valid) begin
            oldest_miss_addr.valid = '1;
            oldest_miss_addr.addr  = read_addrs[0].addr;
        end else if (read_addrs[1].valid & ~cache_outs[1].valid) begin
            oldest_miss_addr.valid = '1;
            oldest_miss_addr.addr  = read_addrs[1].addr;
        end
    end

    // Mem request address logic
    always_comb begin
        mem_req_addr.valid = '0;
        if (~snooping_found_icache & ~snooping_found_mshr) begin
            mem_req_addr = prefetcher_snooping_addr;
        end
    end

    // New MSHR entry logic
    always_comb begin
        new_mshr_entry.valid = '0;
        if (mem_req_accepted) begin
            new_mshr_entry.valid   = '1;
            new_mshr_entry.mem_tag = current_req_tag;
            new_mshr_entry.i_tag   = mem_req_addr.addr.tag;
        end
    end

endmodule


module i_mshr #(
    parameter MSHR_WIDTH = `NUM_MEM_TAGS
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

    // MSHR Internal logic
    localparam I_INDEX_BITS = $clog2(`NUM_MEM_TAGS);
    MSHR_PACKET [`NUM_MEM_TAGS-1:0] mshr_entries, next_mshr_entries;
    logic [I_INDEX_BITS-1:0] head, next_head, tail, next_tail;

    // Snooping logic
    logic [`NUM_MEM_TAGS-1:0] snooping_one_hot;
    for (genvar i = 0; i < `NUM_MEM_TAGS; i++) begin
        assign snooping_one_hot[i] = mshr_entries[i].i_tag == snooping_addr.tag;
    end
    assign addr_found = |snooping_one_hot;

    // MSHR logic
    always_comb begin
        next_head = head;
        next_tail = tail;

        // Data returned from Memory, Pop MSHR Entry
        if (mem_data_tag != '0 & mshr_entries[head].valid & mem_data_tag == mshr_entries[head].mem_tag) begin
            next_head = (head_pointer + '1) % `NUM_MEM_TAGS;
            next_mshr_entries[head_pointer].valid = '0;
            mem_data_i_addr.valid = '1;
            mem_data_i_addr.addr = {16'b0, mshr_entries[head].mem_tag, 3'b0};  // TODO: iffy about syntax
        end

        // New memory request, push new MSHR Entry
        if (new_entry.valid) begin
            next_mshr_entries[tail_pointer] = new_entry;
            next_tail_pointer = (tail_pointer + '1) % `NUM_MEM_TAGS;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            head <= '0;
            tail <= '0;
            mshr_entries <= '0;
        end else begin
            head <= next_head;
            tail <= next_tail;
            mshr_entries <= next_mshr_entries;
        end
    end
endmodule

module prefetcher (
    input clock,
    input reset,

    input I_ADDR_PACKET icache_miss_addr,
    input logic         icache_full,

    input  logic         mem_req_accepted,
    output I_ADDR_PACKET prefetcher_snooping_addr
);
    I_ADDR_PACKET last_icache_miss_mem_req, next_last_icache_miss_mem_req;
    ADDR addr_incrementor, next_addr_incrementor;

    always_comb begin
        mem_req = '0;
        next_addr_incrementor = addr_incrementor;
        next_last_icache_miss_mem_req = last_icache_miss_mem_req;

        // New or first icache miss yet to successfully request
        if (icache_miss_addr.valid & (icache_miss_addr.addr != next_last_icache_miss_mem_req | ~last_requested_icache_miss.valid)) begin
            // Send mem snooping request
            prefetcher_snooping_addr.valid = '1;
            prefetcher_snooping_addr.addr  = icache_miss_addr.addr;
            if (mem_req_accepted) begin
                next_last_icache_miss_mem_req.vaid = '1;
                next_last_icache_miss_mem_req.addr = icache_miss_addr.addr;
                addr_incrementor_next = icache_miss_addr.addr;
            end
        end else if (~icache_full & last_requested_icache_miss.valid) begin
            // Send lookahead snooping request
            prefetcher_snooping_addr.valid = '1;
            prefetcher_snooping_addr.addr  = addr_incrementor + 'h4;
            if (mem_req_accepted) begin
                next_addr_incrementor = addr_incrementor + 'h4;
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

module icache (
    input clock,
    input reset,

    // Fetch Stage read
    input I_ADDR_PACKET [1:0] read_addrs,
    output wor CACHE_DATA [1:0] cache_outs,

    // Prefetch snooping
    input  I_ADDR_PACKET snooping_addr,  // to decide whether to send mem request
    output logic         addr_found,
    output logic         full,

    // Icache write mem_data, when mem_data_tag matches head of MSHR
    input I_ADDR_PACKET write_addr,
    input MEM_BLOCK     write_data
);

    localparam MEM_DEPTH = `ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE;
    localparam I_INDEX_BITS = $clog2(MEM_DEPTH);

    // Internal logics
    logic [MEM_DEPTH-1:0]                 valids, valids_next;
    logic [MEM_DEPTH-1:0][`ITAG_BITS-1:0] tags, tags_next;

    MEM_BLOCK [MEM_DEPTH-1:0]             cache_lines;
    logic [MEM_DEPTH-1:0]                 cache_write_enable_mask;
    logic [MEM_DEPTH-1:0]                 cache_write_no_evict_one_hot;
    logic [I_INDEX_BITS-1:0]              cache_write_evict_write_index;
    logic [MEM_DEPTH-1:0]                 cache_write_evict_one_hot;

    logic [1:0][MEM_DEPTH-1:0]            cache_reads_one_hot;
    logic [1:0][I_INDEX_BITS-1:0]         cache_reads_index;

    logic [MEM_WIDTH-1:0]                 snooping_one_hot;


    memDP #(
        .WIDTH(`MEM_BLOCK_BITS),
        .DEPTH(1'b1),
        .READ_PORTS(1'b1),
        .BYPASS_EN(1'b1)
    ) cache_line[MEM_DEPTH-1:0] (
        .clock(clock),
        .reset(reset),
        .re(1'b1),
        .raddr(1'b0),
        .rdata(cache_lines),
        .we(cache_write_enable_mask),
        .waddr(1'b0),
        .wdata(write_data)
    );

    psel_gen #(
        .WIDTH(MEM_DEPTH),
        .REQS(1'b1)
    ) psel_gen_inst (
        .req(~valids),
        .gn(cache_write_no_evict_one_hot)
    );

    LFSR #(
        .NUM_BITS (MEM_DEPTH)
    ) LFSR_inst (
        .clock(clock),
        .reset(reset),
        .seed_data(`LFSR_SEED),
        .data_out(cache_write_evict_write_index)
    );

    index_to_onehot #(
        .OUTPUT_WIDTH    (MEM_DEPTH)
    ) evict_index_to_onehot_inst (
        .idx             (cache_write_evict_write_index),
        .one_hot         (cache_write_evict_one_hot)
    );

    one_hot_to_index #(
        .INPUT_WIDTH   (MEM_DEPTH)
    ) one_hot_to_index_inst[1:0] (
        .one_hot        (cache_reads_one_hot),
        .index          (cache_reads_index)
    );


    // prefetch snooping logic
    for (genvar i = 0; i < MEM_DEPTH; i++) begin
        assign snooping_one_hot[i] = snooping_addr.addr.tag == tags[i] & snooping_addr.valid & valids[i];
    end
    assign addr_found = |snooping_one_hot;

    // cache read logic
    for (genvar j = 0; j <= 1; j++) begin
        for (genvar i = 0; i < MEM_DEPTH; i++) begin
            assign cache_reads_one_hot[j] = read_addrs[j].addr.tag & read_addrs[j].addr.valid & valids[i];
            assign cache_outs[j].cache_line = cache_lines[i] & {(`MEM_BLOCK_BITS){cache_reads_one_hot[j][i]}}
        end
        assign cache_outs[j].valid = 
    end

    // cache write logic
    assign cache_write_enable_mask = |cache_write_no_evict_one_hot ? cache_write_no_evict_one_hot : cache_write_evict_one_hot;

    // valids and tags update when write to icache
    always_comb begin
        valids_next = valids;
        tags_next = tags;
        for (int i = 0; i < MEM_DEPTH; i++) begin
            if (cache_write_enable_mask[i] & write_addr.valid) begin
                valids_next[i] = 1'b1;
                tags_next[i] = write_addr.tag;
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            valids <= '0;
            tags <= '0;
        end else begin
            valids <= valids_next;
            tags <= tags_next;
        end
    end

endmodule

module index_to_onehot #(
    parameter OUTPUT_WIDTH = 1
) (
    input  logic [$clog2(OUTPUT_WIDTH)-1:0] idx,
    output logic [OUTPUT_WIDTH-1:0] one_hot
);

    integer i;
    always_comb begin
        one_hot = '0;
        for (i = 0; i < OUTPUT_WIDTH; i = i + 1) begin
            if (idx == i[$clog2(OUTPUT_WIDTH)-1:0])
                one_hot[i] = 1'b1;
        end
    end

endmodule

module one_hot_to_index #(
    parameter int INPUT_WIDTH = 1
) (
    input logic [INPUT_WIDTH-1:0] one_hot,
    output wor [((INPUT_WIDTH <= 1) ? 1 : $clog2(INPUT_WIDTH))-1:0] index
);

    localparam INDEX_WIDTH = (INPUT_WIDTH <= 1) ? 1 : $clog2(INPUT_WIDTH);

    assign index = '0;
    for (genvar i = 0; i < INPUT_WIDTH; i++) begin : gen_index_terms
        assign index = {INDEX_WIDTH{one_hot[i]}} & i;
    end

endmodule
