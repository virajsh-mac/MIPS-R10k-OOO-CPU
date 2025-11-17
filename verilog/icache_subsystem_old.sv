`include "verilog/sys_defs.svh"

module icache_subsystem (
    input clock,
    input reset,

    // Fetch
    input  ADDR       [1:0]   read_addr,                 // assume read_addr[0] is older instructions
    output CACHE_DATA [1:0]   cache_out,

    // Memory
    input  MEM_TAG            current_req_tag,           // Tag of current mem request (0 = rejected)
    input  MEM_BLOCK          return_data,               // Mem requested data coming back
    input  MEM_TAG            return_data_tag,           // Tag for returned data (0 = no data)

    output ADDR_PACKET        mem_req,
    input  logic              mem_req_accepted,          // Mem reading request was successful from arbitor
);

    logic icache_full;
    ADDR_PACKET oldest_icache_miss;

    icache icache_inst (
        .clock                    (clock),
        .reset                    (reset),

        // Fetch Stage IOs
        .read_addr                (read_addr),
        .cache_out                (cache_out),
    
        // Prefetcher IOs
        .snooping_addr           (prefetcher_mem_req.addr),
        .addr_found               (found_in_icache),
        .full                     (icache_full),

        // Write to icache
        .write_addr               (current_mem_data_addr),
        .write_data                 (icache_wirte_in)
    );

    MSHR_PACKET new_mshr_entry;
    assign new_mshr_entry.valid = 

    ADDR_PACKET current_mem_data_addr;
    CACHE_DATA icache_wirte_in;
    assign icache_wirte_in.valid = current_mem_data_addr.valid;
    assign icache_wirte_in.cache_line = return_data;

    i_mshr i_mshr_inst (
        .clock           (clock),
        .reset           (reset),

        // Prefetch snooping
        .snooping_addr  (prefetcher_mem_req.addr),
        .addr_found      (found_in_mshr),
        
        // New accepted mem request
        .new_entry       (new_mshr_entry),

        // Mem IOs
        .data_back_tag   (return_data_tag),
        .data_back_addr  (current_mem_data_addr)
    );

    assign oldest_icache_miss.valid = cache_out[0].valid | cache_out[1].valid;
    assign oldest_icache_miss.addr = ~cache_out[0].valid ? cache_out[0].addr : cache_out[1].addr;

    ADDR_PACKET prefetcher_mem_req;
    logic found_in_icache, found_in_mshr;

    prefetcher prefetcher_inst (
        .clock                (clock),
        .reset                (reset),

        .icache_miss          (oldest_icache_miss),
        .icache_full          (icache_full),

        .mem_req_accepted     (mem_req_accepted),
        .mem_req              (prefetcher_mem_req)
    );

    assign mem_req = (~found_in_icache && ~found_in_mshr) ? prefetcher_mem_req : '0;

endmodule

// Instruction Miss Status History Table
module i_mshr #(
    parameter MSHR_WIDTH = `NUM_MEM_TAGS
) (
    input             clock,
    input             reset,

    // Prefetch snooping
    input  I_ADDR     snooping_addr,
    output logic      addr_found,

    // New accepted mem request
    input MSHR_PACKET new_entry,

    // Mem request came back from memory
    input MEM_TAG      data_back_tag,
    output ADDR_PACKET data_back_addr,
);
    localparam I_INDEX_BITS = $clog2(`NUM_MEM_TAGS);

    MSHR_PACKET [`NUM_MEM_TAGS-1:0] mshr_entries;
    logic       [I_INDEX_BITS-1:0]  head_pointer, next_head_pointer;
    logic       [I_INDEX_BITS-1:0]  tail_pointer, next_tail_pointer;

    // Pre-fetch snoop logic
    logic [`NUM_MEM_TAGS-1:0] addr_search_one_hot;
    for (genvar i = 0; i < `NUM_MEM_TAGS; i++) begin
        assign addr_search_one_hot[i] = snooping_addr.tag == mshr_entries[i].addr.tag;
    end
    assign addr_found = |addr_search_one_hot;

    always_comb begin
        next_head_pointer = head_pointer;
        next_tail_pointer = tail_pointer;
        data_back_addr = '0;

        // Pop FIFO
        if (data_back_tag != '0 && 
            data_back_tag == mshr_entries[head_pointer].mem_tag &&
            mshr_entries[head_pointer].valid) begin
            next_head_pointer = (head_pointer + '1) % `NUM_MEM_TAGS;
            mshr_entries[head_pointer].valid = '0;
            data_back_addr.valid = '1;
            data_back_addr.addr = mshr_entries[head_pointer].addr;
        end

        // Push FIFO
        if (new_entry.valid) begin
            mshr_entries[tail_pointer] = new_entry;
            next_tail_pointer = (tail_pointer + '1) % `NUM_MEM_TAGS;
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            head_pointer <= '0;
            tail_pointer <= '0;
        end else begin
            head_pointer <= next_head_pointer;
            tail_pointer <= next_tail_pointer;
        end
    end

endmodule

module prefetcher (
    input clock,
    input reset,

    input ADDR_PACKET   icache_miss,
    input logic         icache_full,

    input logic         mem_req_accepted,
    output ADDR_PACKET  mem_req
);

    ADDR_PACKET last_requested_icache_miss, next_last_requested_icache_miss;
    ADDR addr_incrementor, next_addr_incrementor;

    always_comb begin
        addr_incrementor_next = addr_incrementor;
        mem_req.valid = '0;

        if (icache_miss.valid && 
           (icache_miss.addr != last_requested_icache_miss.addr || ~last_requested_icache_miss.valid)) begin
            mem_req.valid = '1;
            mem_req.addr = icache_miss.addr;
            if (mem_req_accepted) begin
                next_last_requested_icache_miss.valid = '1;
                next_last_requested_icache_miss.addr = icache_miss.addr;
                addr_incrementor_next = icache_miss.addr;
            end
        end else if (~icache_full) begin
            mem_req.valid = '1;
            mem_req.addr = addr_incrementor + 'h4;
            if (mem_req_accepted) begin
                next_addr_incrementor = addr_incrementor + 'h4;
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            last_requested_icache_miss.valid <= '0;
            last_requested_icache_miss.addr <= '0;
            addr_incrementor <= '0;
        end else begin
            last_requested_icache_miss <= next_last_requested_icache_miss;
            addr_incrementor <= next_addr_incrementor;
        end
    end
endmodule

module icache (
    input clock,
    input reset,

    // Fetch read
    input  I_ADDR       [1:0] read_addr,
    output CACHE_DATA   [1:0] cache_out,

    // Prefetcher read
    input  I_ADDR             snooping_addr,
    output logic              addr_found,
    output logic              full

    // Write to icache
    input  I_ADDR             write_addr,
    input  CACHE_DATA         write_data,

);
    localparam MEM_WIDTH = `ICACHE_LINES + `PREFETCH_STREAM_BUFFER_SIZE;
    localparam I_INDEX_BITS = $clog2(MEM_WIDTH);

    logic [MEM_WIDTH-1:0]                 valids, valids_next;
    logic [MEM_WIDTH-1:0][`ITAG_BITS-1:0] tags, tags_next;
    MEM_BLOCK [MEM_WIDTH-1:0]             cache_lines;

    memDP #(
        .WIDTH        ($BITS(MEM_BLOCK)),
        .DEPTH        (1'b1),
        .READ_PORTS   (1),
        .BYPASS_EN    (0)
    ) cache_bank [MEM_WIDTH-1:0] (
        .clock        (clock),
        .reset        (reset),
        .re           (1'b1),
        .raddr        (1'b0),
        .rdata        (cache_lines),
        .we           (cache_write_enable_mask),
        .waddr        (1'b0),
        .wdata        (write_data.cache_line)
    );

    logic [1:0][MEM_WIDTH-1:0]            read_addr_one_hot;
    logic [1:0][$clog2(MEM_WIDTH)-1:0]    read_addr_index;

    one_hot_to_index #(
        .INPUT_WIDTH   (MEM_WIDTH)
    ) one_hot_to_index_inst[1:0] (
        .one_hot        (read_addr_one_hot),
        .index          (read_addr_index)
    );

    wor MEM_BLOCK [1:0]                    cache_lines_out; // gonna just be output

    // Pre-fetch snoop logic
    logic [MEM_WIDTH-1:0] addr_search_one_hot;
    for (genvar i = 0; i < MEM_WIDTH; i++) begin
        assign addr_search_one_hot[i] = snooping_addr.tag == tags[i];
    end
    assign addr_found = |addr_search_one_hot;

    // Fetch Read logic
    for (genvar i = 0; i < MEM_WIDTH; i++) begin : read_cache // Anding and Or-reducing
        assign read_addr_one_hot[0][i] = read_addr[0].tag == tags[i] && valids[i];  // Find read index by matching tag for cache read port 0
        assign read_addr_one_hot[1][i] = read_addr[1].tag == tags[i] && valids[i];

        assign cache_lines_out[0] = cache_lines[i] & {$bits(MEM_BLOCK){read_addr_one_hot[0][i]}};
        assign cache_lines_out[1] = cache_lines[i] & {$bits(MEM_BLOCK){read_addr_one_hot[1][i]}};
    end

    assign cache_out[0].cache_line = write_data.valid && write_addr.tag == read_addr[0].tag ? // forwarding
                        write_data.cache_line : cache_lines_out[0];
    assign cache_out[1].cache_line = write_data.valid && write_addr.tag == read_addr[1].tag ? // forwarding
                        write_data.cache_line : cache_lines_out[1];

    assign cache_out[0].valid = (valids[read_addr_index[0]] & |read_addr_one_hot[0]) ||
                                (write_data.valid && write_addr.tag == read_addr[0].tag);     // forwarding
    assign cache_out[1].valid = (valids[read_addr_index[1]] & |read_addr_one_hot[1]) ||
                                (write_data.valid && write_addr.tag == read_addr[1].tag);     // forwarding

    logic [MEM_WIDTH-1:0]  cache_write_one_hot;

    // Write logic
    psel_gen #(
        .WIDTH (MEM_WIDTH),
        .REQS  (1)
    ) psel_bank (
        .req   (~valids),
        .gn    (cache_write_one_hot)
    );

    logic [I_INDEX_BITS-1:0] evict_index;
    logic [MEM_WIDTH-1:0] cache_write_enable_mask;

    LFSR #(
        .NUM_BITS        (I_INDEX_BITS)
    ) LFSR0 (
        .clock           (clock),
        .reset           (reset),
        .seed_data       (`LFSR_SEED),
        .data_out        (evict_index)
    );

    logic [MEM_WIDTH-1:0] evict_index_one_hot;

    index_to_onehot #(
        .OUTPUT_WIDTH    (MEM_WIDTH)
    ) evict_index_to_onehot_inst (
        .idx             (evict_index),
        .one_hot         (evict_index_one_hot)
    );

    assign cache_write_enable_mask = write_data.valid ? 
        ((|cache_write_one_hot) ? cache_write_one_hot : evict_index_one_hot) : 
        '0;

    assign full = &valids;
    // Valid and tags logic
    always_comb begin
        valids_next = valids;
        tags_next = tags;
        for (int i = 0; i < MEM_WIDTH; i++) begin
            if (cache_write_enable_mask[i] && write_data.valid) begin
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

module LFSR #(
    parameter NUM_BITS
) (
    input clock,
    input reset,

    input  [NUM_BITS-1:0] seed_data,
    output [NUM_BITS-1:0] data_out
);

    logic [NUM_BITS-1:0] LFSR;
    logic                r_XNOR;

    always @(posedge clock) begin
        if (reset) LFSR <= seed_data;
        else LFSR <= {LFSR[NUM_BITS-1:1], r_XNOR};
    end

    always @(*) begin
        case (NUM_BITS)
            3: begin
                r_XNOR = LFSR[3] ^~ LFSR[2];
            end
            4: begin
                r_XNOR = LFSR[4] ^~ LFSR[3];
            end
            5: begin
                r_XNOR = LFSR[5] ^~ LFSR[3];
            end
            6: begin
                r_XNOR = LFSR[6] ^~ LFSR[5];
            end
            7: begin
                r_XNOR = LFSR[7] ^~ LFSR[6];
            end
            8: begin
                r_XNOR = LFSR[8] ^~ LFSR[6] ^~ LFSR[5] ^~ LFSR[4];
            end
            9: begin
                r_XNOR = LFSR[9] ^~ LFSR[5];
            end
            10: begin
                r_XNOR = LFSR[10] ^~ LFSR[7];
            end
            11: begin
                r_XNOR = LFSR[11] ^~ LFSR[9];
            end
            12: begin
                r_XNOR = LFSR[12] ^~ LFSR[6] ^~ LFSR[4] ^~ LFSR[1];
            end
            13: begin
                r_XNOR = LFSR[13] ^~ LFSR[4] ^~ LFSR[3] ^~ LFSR[1];
            end
            14: begin
                r_XNOR = LFSR[14] ^~ LFSR[5] ^~ LFSR[3] ^~ LFSR[1];
            end
            15: begin
                r_XNOR = LFSR[15] ^~ LFSR[14];
            end
            16: begin
                r_XNOR = LFSR[16] ^~ LFSR[15] ^~ LFSR[13] ^~ LFSR[4];
            end
            17: begin
                r_XNOR = LFSR[17] ^~ LFSR[14];
            end
            18: begin
                r_XNOR = LFSR[18] ^~ LFSR[11];
            end
            19: begin
                r_XNOR = LFSR[19] ^~ LFSR[6] ^~ LFSR[2] ^~ LFSR[1];
            end
            20: begin
                r_XNOR = LFSR[20] ^~ LFSR[17];
            end
            21: begin
                r_XNOR = LFSR[21] ^~ LFSR[19];
            end
            22: begin
                r_XNOR = LFSR[22] ^~ LFSR[21];
            end
            23: begin
                r_XNOR = LFSR[23] ^~ LFSR[18];
            end
            24: begin
                r_XNOR = LFSR[24] ^~ LFSR[23] ^~ LFSR[22] ^~ LFSR[17];
            end
            25: begin
                r_XNOR = LFSR[25] ^~ LFSR[22];
            end
            26: begin
                r_XNOR = LFSR[26] ^~ LFSR[6] ^~ LFSR[2] ^~ LFSR[1];
            end
            27: begin
                r_XNOR = LFSR[27] ^~ LFSR[5] ^~ LFSR[2] ^~ LFSR[1];
            end
            28: begin
                r_XNOR = LFSR[28] ^~ LFSR[25];
            end
            29: begin
                r_XNOR = LFSR[29] ^~ LFSR[27];
            end
            30: begin
                r_XNOR = LFSR[30] ^~ LFSR[6] ^~ LFSR[4] ^~ LFSR[1];
            end
            31: begin
                r_XNOR = LFSR[31] ^~ LFSR[28];
            end
            32: begin
                r_XNOR = LFSR[32] ^~ LFSR[22] ^~ LFSR[2] ^~ LFSR[1];
            end

        endcase
    end

    assign data_out = LFSR;

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
