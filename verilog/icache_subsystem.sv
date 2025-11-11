`include "verilog/sys_defs.svh"

module icache_subsystem (
    input clock,
    input reset,

    // Memory
    input  MEM_TAG     Imem2proc_transaction_tag,  // Tag of current mem request (0 = rejected)
    input  MEM_BLOCK   Imem2proc_data,             // Mem requested data coming back
    input  MEM_TAG     Imem2proc_data_tag,         // Tag for returned data (0 = no data)
    input  logic       mem_request_success,        // Mem reading request was successful
    output logic       mem_req_valid,
    output ADDR        mem_req_addr,
    output MEM_COMMAND mem_req_command,

    // Fetch
    input  ADDR       [1:0] read_addr,
    output CACHE_DATA [1:0] cache_out
);


endmodule


// ============================================================================
// Prefetcher Module
// ============================================================================
// Sequential prefetcher that predicts next N cache lines based on access patterns
// Monitors available MSHR slots and cache fullness to avoid resource conflicts
module prefetcher #(
    parameter PREFETCH_WIDTH = `PREFETCH_WIDTH
) (
    input clock,
    input reset,

    // Icache misses from fetch stage
    input ICACHE_MISS_PACKET icache_miss,
    input logic              icache_full,  // High when Icache is full


    // MSHR status
    input logic [$clog2(`NUM_MEM_TAGS):0] mshr_free_slots,  // Number of free slots in MSHR

    // MSHR requests output (misses + prefetches)
    output MSHR_REQUEST_PACKET mshr_req
);

    // on the first cycle fetch the entire size of the MSHR
    // after that if there is a miss
    // and there isnt already a open request for that address in the MSHR
    // invalidate everything in the MSHR that doesnt have a open request
    // insert the misses on top of the entries in the MSHR that have open requests
    // prefetch the prefetch width on top of the miss
    // Size the mshr to be superscalar width above the maximum number of requests to memory
    // never prefetch above the maximum number of requests to memory

endmodule


// ============================================================================
// ICache (2-way banked, fully associative per bank)
// ============================================================================
module icache (
    input clock,
    input reset,

    // Fetch read
    input  I_ADDR       [1:0] read_addr,
    output CACHE_DATA   [1:0] cache_out,

    // Write to icache
    input  I_ADDR             write_addr,
    input  CACHE_DATA         write_in,

    output logic        [1:0] hit,
    output logic              full
);
    localparam MEM_WIDTH = `ICACHE_LINES + `PREFETCH_SIZE;
    localparam I_INDEX_BITS = $clog2(MEM_WIDTH);

    logic [MEM_WIDTH-1:0]                 valids, valids_next;
    logic [MEM_WIDTH-1:0][`ITAG_BITS-1:0] tags, tags_next;
    MEM_BLOCK [MEM_WIDTH-1:0]             cache_lines;

    memDP #(
        .WIDTH   ($BITS(MEM_BLOCK)),
        .DEPTH   (1'b1),
        .READ_PORTS (1),
        .BYPASS_EN  (0)
    ) cache_bank[MEM_WIDTH-1:0] (
        .clock(clock),
        .reset(reset),
        .re   (1'b1),
        .raddr(1'b0),
        .rdata(cache_lines),
        .we   (cache_write_enable_mask),
        .waddr(1'b0),
        .wdata(write_in.cache_line)
    );

    logic [1:0][MEM_WIDTH-1:0]            read_addr_one_hot;
    logic [1:0][$clog2(MEM_WIDTH)-1:0]    read_addr_index;

    one_hot_to_index #(
        .OUTPUT_WIDTH (MEM_WIDTH)
    ) one_hot_to_index_inst[1:0] (
        .one_hot(read_addr_one_hot),
        .index(read_addr_index)
    );

    wor MEM_BLOCK [1:0]                       cache_lines_out;

    // Fetch Read logic
    for (genvar i = 0; i < MEM_WIDTH; i++) begin : read_cache // Anding and Or-reducing
        assign read_addr_one_hot[0][i] = read_addr[0].tag == tags[i] && valids[i];  // Find read index by matching tag for cache read port 0
        assign read_addr_one_hot[1][i] = read_addr[1].tag == tags[i] && valids[i];

        assign cache_lines_out[0] = cache_lines[i] & {$bits(MEM_BLOCK){read_addr_one_hot[0][i]}};
        assign cache_lines_out[1] = cache_lines[i] & {$bits(MEM_BLOCK){read_addr_one_hot[1][i]}};
    end

    assign cache_out[0].cache_line = write_addr.valid && write_addr.tag == read_addr[0].tag ? // forwarding
                        write_in.cache_line : cache_lines_out[0];
    assign cache_out[1].cache_line = write_addr.valid && write_addr.tag == read_addr[1].tag ? // forwarding
                        write_in.cache_line : cache_lines_out[1];

    assign cache_out[0].valid = (valids[read_addr_index[0]] & |read_addr_one_hot[0]) ||
                                (write_addr.valid && write_addr.tag == read_addr[0].tag);     // forwarding
    assign cache_out[1].valid = (valids[read_addr_index[1]] & |read_addr_one_hot[1]) ||
                                (write_addr.valid && write_addr.tag == read_addr[1].tag);     // forwarding

    logic [MEM_WIDTH-1:0]  cache_write_one_hot;

    // Write logic
    psel_gen #(
        .WIDTH(MEM_WIDTH),
        .REQS(1)
    ) psel_bank (
        .req(~valids),
        .gn(cache_write_one_hot)
    );

    logic [I_INDEX_BITS-1:0] evict_index;
    logic [MEM_WIDTH-1:0] cache_write_enable_mask;

    LFSR #(
        .NUM_BITS(I_INDEX_BITS)
    ) LFSR0 (
        .clock(clock),
        .reset(reset),
        .seed_data(`LFSR_SEED),
        .data_out(evict_index)
    );

    logic [MEM_WIDTH-1:0] evict_index_one_hot;

    index_to_onehot #(
        .INPUT_WIDTH(I_INDEX_BITS)
    ) evict_index_to_onehot_inst (
        .idx(evict_index),
        .one_hot(evict_index_one_hot)
    );

    assign cache_write_enable_mask = (|cache_write_one_hot) ? cache_write_one_hot : evict_index_one_hot;

    assign hit = ;

    // Valid and tags logic
    always_comb begin
        valids_next = valids;
        tags_next = tags;
        for (int i = 0; i < MEM_WIDTH; i++) begin
            if (cache_write_enable_mask[i] && write_in.valid) begin
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

// Basically a lookup table very efficient
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
