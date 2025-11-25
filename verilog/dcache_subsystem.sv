`include "verilog/sys_defs.svh"

module dcache_subsystem (
    input clock,
    input reset,

    // Execute (load requests)
    input  D_ADDR_PACKET [1:0]  read_addrs,  // read_addr[0] is older requests
    output D_CACHE_DATA [1:0]     cache_outs,

    // Mem.sv IOs
    input MEM_TAG               current_req_tag,
    input MEM_BLOCK             mem_data,
    input MEM_TAG               mem_data_tag,

    // Arbitor IOs
    output D_ADDR_PACKET        mem_req_addr,
    input  logic                mem_req_accepted,

    // Debug outputs
    output D_ADDR_PACKET [1:0]  read_addrs_dbg,
    output D_CACHE_DATA [1:0]     cache_outs_dbg,
    output logic [1:0]          dcache_hits_dbg,
    output logic [1:0]          dcache_misses_dbg,
    output logic                dcache_full_dbg,
    output D_ADDR_PACKET        oldest_miss_addr_dbg,
    output logic                mshr_addr_found_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_tail_dbg,
    output MSHR_PACKET [`NUM_MEM_TAGS + `N-1:0]   mshr_entries_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_next_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] mshr_next_tail_dbg,
    output logic                             mshr_pop_condition_dbg,
    output logic                             mshr_push_condition_dbg,
    output logic                             mshr_pop_cond_has_data_dbg,
    output logic                             mshr_pop_cond_head_valid_dbg,
    output logic                             mshr_pop_cond_tag_match_dbg,
    output logic                mem_write_dcache_dbg,
    output D_ADDR_PACKET        mem_write_addr_dbg,
    output MEM_BLOCK            mem_data_dbg,
    output MEM_TAG              mem_data_tag_dbg,
    output D_ADDR_PACKET        dcache_write_addr_dbg,
    output MEM_BLOCK            dcache_write_data_dbg,
    output D_CACHE_LINE         dcache_line_write_dbg,
    output logic [(`DCACHE_LINES)-1:0] dcache_write_enable_mask_dbg,
    // Logic block debug outputs
    output D_ADDR_PACKET        mem_req_addr_dbg,
    output MSHR_PACKET          new_mshr_entry_dbg
);

    // Internal wires
    D_ADDR_PACKET dcache_write_addr, oldest_miss_addr;
    logic dcache_full, snooping_found_mshr;
    MSHR_PACKET new_mshr_entry;
    
    // DCache debug wires
    D_CACHE_LINE dcache_line_write;
    logic [(`DCACHE_LINES)-1:0] dcache_write_enable_mask;

    dcache dcache_inst (
        .clock        (clock),
        .reset        (reset),
        // Execute Stage read (loads)
        .read_addrs   (read_addrs),
        .cache_outs   (cache_outs),
        .full         (dcache_full),
        // Dcache write mem_data, when mem_data_tag matches head of MSHR
        .write_addr   (dcache_write_addr),
        .write_data   (mem_data),
        // Debug outputs
        .hits_dbg     (dcache_hits_dbg),
        .misses_dbg   (dcache_misses_dbg),
        .write_addr_dbg(dcache_write_addr_dbg),
        .write_data_dbg(dcache_write_data_dbg),
        .cache_line_write_dbg(dcache_line_write),
        .cache_write_enable_mask_dbg(dcache_write_enable_mask)
    );

    d_mshr d_mshr_inst (
        .clock          (clock),
        .reset          (reset),
        // When mem_req_accepted
        .new_entry      (new_mshr_entry),
        // Mem data back
        .mem_data_tag   (mem_data_tag),
        .mem_data_d_addr(dcache_write_addr),
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

    // Mem request address logic - directly request on miss
    always_comb begin
        mem_req_addr = '0;
        if (oldest_miss_addr.valid) begin
            mem_req_addr.valid = 1'b1;
            mem_req_addr.addr  = oldest_miss_addr.addr;
        end
    end

    // MSHR entry logic - add immediately when request is accepted and tag is valid
    always_comb begin
        new_mshr_entry = '0;
        if (mem_req_accepted && (current_req_tag != 0)) begin
            new_mshr_entry.valid   = '1;
            new_mshr_entry.mem_tag = current_req_tag;
            new_mshr_entry.i_tag   = mem_req_addr.addr.tag;  // Note: may need to adjust for d_tag if different
        end
    end

    // Debug signal assignments
    assign read_addrs_dbg = read_addrs;
    assign cache_outs_dbg = cache_outs;
    assign dcache_full_dbg = dcache_full;
    assign oldest_miss_addr_dbg = oldest_miss_addr;
    assign mshr_addr_found_dbg = snooping_found_mshr;  // May remove if no snooping
    assign mem_write_dcache_dbg = dcache_write_addr.valid;
    assign mem_write_addr_dbg = dcache_write_addr;
    assign mem_data_dbg = mem_data;
    assign mem_data_tag_dbg = mem_data_tag;
    assign dcache_line_write_dbg = dcache_line_write;
    assign dcache_write_enable_mask_dbg = dcache_write_enable_mask;
    // Logic block debug signal assignments
    assign mem_req_addr_dbg = mem_req_addr;
    assign new_mshr_entry_dbg = new_mshr_entry;

endmodule

// this should never be full, so no logic for handling full FIFO head tail edge case
module d_mshr #(
    parameter MSHR_WIDTH = `NUM_MEM_TAGS + `N
) (
    input clock,
    input reset,

    // When mem_req_accepted
    input MSHR_PACKET new_entry,

    // Mem data back
    input  MEM_TAG       mem_data_tag,
    output D_ADDR_PACKET mem_data_d_addr,  // to write to dcache

    // Debug outputs
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] tail_dbg,
    output MSHR_PACKET [`NUM_MEM_TAGS + `N-1:0]   entries_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] next_head_dbg,
    output logic [$clog2(`NUM_MEM_TAGS + `N)-1:0] next_tail_dbg,
    output logic                             pop_condition_dbg,
    output logic                             push_condition_dbg,
    output logic                             pop_cond_has_data_dbg,
    output logic                             pop_cond_head_valid_dbg,
    output logic                             pop_cond_tag_match_dbg
);

    // MSHR Internals
    localparam D_CACHE_INDEX_BITS = $clog2(MSHR_WIDTH);
    MSHR_PACKET [MSHR_WIDTH-1:0] mshr_entries, next_mshr_entries;
    logic [D_CACHE_INDEX_BITS-1:0] head, next_head, tail, next_tail;

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
            mem_data_d_addr.addr = '{zeros: 16'b0, 
                                     tag: mshr_entries[head].i_tag,  // Note: adjust if needed
                                     block_offset: 3'b0};
        end

        // New memory request, push new MSHR Entry
        push_condition = new_entry.valid;
        if (push_condition) begin
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

module dcache (
    input clock,
    input reset,

    // Execute Stage read (loads)
    input D_ADDR_PACKET [1:0] read_addrs,
    output D_CACHE_DATA [1:0] cache_outs,

    input logic         full,

    // Dcache write mem_data, when mem_data_tag matches head of MSHR
    input D_ADDR_PACKET write_addr,
    input MEM_BLOCK     write_data,

    // Debug outputs
    output logic [1:0]   hits_dbg,
    output logic [1:0]   misses_dbg,
    output D_ADDR_PACKET write_addr_dbg,
    output MEM_BLOCK     write_data_dbg,
    output D_CACHE_LINE  cache_line_write_dbg,
    output logic [(`DCACHE_LINES)-1:0] cache_write_enable_mask_dbg
);

    localparam MEM_DEPTH = `DCACHE_LINES;
    localparam D_CACHE_INDEX_BITS = $clog2(MEM_DEPTH);
    localparam MEM_WIDTH = 1 + `DTAG_BITS + `MEM_BLOCK_BITS;  // Assume `DTAG_BITS, adjust if same as `ITAG_BITS

    wor D_CACHE_DATA [1:0]                  cache_outs_temp;
    D_CACHE_LINE [MEM_DEPTH-1:0]            cache_lines;
    D_CACHE_LINE                            cache_line_write;
    logic [MEM_DEPTH-1:0]                   cache_write_enable_mask;
    logic [MEM_DEPTH-1:0]                   cache_write_no_evict_one_hot;
    logic [D_CACHE_INDEX_BITS-1:0]          cache_write_evict_index;
    logic [MEM_DEPTH-1:0]                   cache_write_evict_one_hot;

    logic [1:0][MEM_DEPTH-1:0]              cache_reads_one_hot;
    logic [1:0][D_CACHE_INDEX_BITS-1:0]     cache_reads_index;

    logic [MEM_DEPTH-1:0]                   valid_bits;

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
    LFSR LFSR_inst (
        .clk(clock),
        .rst(reset),
        .op(cache_write_evict_index)
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
