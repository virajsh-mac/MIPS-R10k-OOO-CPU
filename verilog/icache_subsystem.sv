`include "verilog/sys_defs.svh"

module icache_subsystem (
    input clock,
    input reset,

    // Memory
    input MEM_TAG       Imem2proc_transaction_tag,  // Tag of current mem request (0 = rejected)
    input MEM_BLOCK     Imem2proc_data,             // Mem requested data coming back
    input MEM_TAG       Imem2proc_data_tag,         // Tag for returned data (0 = no data)
    input logic         mem_request_success,        // Mem reading request was successful
    output logic        mem_req_valid,
    output ADDR         mem_req_addr,
    output MEM_COMMAND  mem_req_command,

    // Fetch
    input  ADDR       [1:0] read_addr,
    output CACHE_DATA [1:0] cache_out
);


endmodule


// ============================================================================
// ICache (2-way banked, fully associative per bank)
// ============================================================================
// Two memDP modules for odd/even banking to support 2 simultaneous reads
// Each bank is fully associative (16 lines per bank = 32 total lines)
// Uses LFSR for pseudo-random eviction policy within each bank
// Prioritize victim cache eviction read over fetch read, might change later
// when read hit in victim cache/prefetcher, it empties that line in victim cache and reinstate to icache
// if icache was full, it evict a random one to victim cache, that's practically a swap.
// what if I just don't promote when icache is already full?
// but when I write to full icache again, it may overwrite that just used victim cache line
// and being in victim cache is more line to be overwritten than being in a icache bank
// because icache bank has 16 lines, victim cache only has 4 and is sharing with dcache
// therefore, we should do the swap
// there could be promotion write and reinstatement write
// each write request means it's a hit this cycle, which means only one of them can be valid
// so we don't have to decide priority between them.
// I still need to decide the prioirty between MSHR write and victim/prefetcher write
// those could happen in the same cycle
// if delay MSHR, there needs to be a structure to save it
// if delay victim/prefetcher, i also need a buffer to save it
// sicne a buffer is required, I might as well prioritize fetch stage read over eviction read
module icache (
    input clock,
    input reset,

    // Fetch read
    input  I_ADDR       [1:0] read_addr,
    output CACHE_DATA   [1:0] cache_out,

    // Write to icache
    input  I_ADDR             write_addr,
    input  CACHE_DATA         write_in,

    // Victim cache
    output ADDR               evict_addr,
    output CACHE_DATA         evict_out,

    output logic        [1:0] hit
);
    logic [`ICACHE_LINES-1:0]                 valids, valids_next;
    logic [`ICACHE_LINES-1:0][`ITAG_BITS-1:0] tags, tags_next;
    MEM_BLOCK [`ICACHE_LINES-1:0]             cache_lines;

    memDP #(
        .WIDTH   ($BITS(MEM_BLOCK)),
        .DEPTH   (1'b1),
        .READ_PORTS (1),
        .BYPASS_EN  (0)
    ) cache_bank[`ICACHE_LINES-1:0] (
        .clock(clock),
        .reset(reset),
        .re   (1'b1),
        .raddr(1'b0),
        .rdata(cache_lines),
        .we   (cache_write_enable_mask),
        .waddr(1'b0),
        .wdata(write_in.cache_line)
    );

    logic [1:0][`ICACHE_LINES-1:0]            read_addr_one_hot;
    logic [1:0][$clog2(`ICACHE_LINES)-1:0]    read_addr_index;

    one_hot_to_index #(
        .OUTPUT_WIDTH (`ICACHE_LINES)
    ) one_hot_to_index_inst[1:0] (
        .one_hot(read_addr_one_hot),
        .index(read_addr_index)
    );

    wor MEM_BLOCK [1:0]                       cache_lines_out;

    // Fetch Read logic
    for (genvar i = 0; i < `ICACHE_LINES; i++) begin : read_cache // Anding and Or-reducing
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

    logic [`ICACHE_LINES-1:0]  cache_write_one_hot;

    // Write logic
    psel_gen #(
        .WIDTH(`ICACHE_LINES),
        .REQS(1)
    ) psel_bank (
        .req(~valids),
        .gn(cache_write_one_hot)
    );

    logic [`I_INDEX_BITS-1:0] evict_index;
    logic [`ICACHE_LINES-1:0] cache_write_enable_mask;

    LFSR #(
        .NUM_BITS(`I_INDEX_BITS)
    ) LFSR0 (
        .clock(clock),
        .reset(reset),
        .seed_data(`LFSR_SEED),
        .data_out(evict_index)
    );

    assign cache_write_enable_mask = (|cache_write_one_hot) ? cache_write_one_hot : evict_index;

    assign evict_out[0].cache_line = cache_lines[evict_index];
    assign evict_out[1].valid = ~|cache_write_one_hot;

    // Valid and tags logic
    always_comb begin
        valids_next = valids;
        tags_next = tags;
        for (int i = 0; i < `ICACHE_LINES; i++) begin
            if (cache_write_enable_mask[i]) begin
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
    input  logic [INPUT_WIDTH-1:0] one_hot,
    output wor   [((INPUT_WIDTH <= 1) ? 1 : $clog2(INPUT_WIDTH))-1:0] index
);

    localparam INDEX_WIDTH = (INPUT_WIDTH <= 1) ? 1 : $clog2(INPUT_WIDTH);

    assign index = '0;
    for (genvar i = 0; i < INPUT_WIDTH; i++) begin : gen_index_terms
        assign index = {INDEX_WIDTH{one_hot[i]}} & i;
    end

endmodule

module LFSR #(parameter NUM_BITS) (
   input clock,
   input reset,

   input [NUM_BITS-1:0] seed_data,
   output [NUM_BITS-1:0] data_out
);

    logic [NUM_BITS-1:0] LFSR;
    logic                r_XNOR;

    always @(posedge clock) begin
        if (reset)
           LFSR <= seed_data;
        else
           LFSR <= {LFSR[NUM_BITS-1:1], r_XNOR};
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
