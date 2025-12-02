`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module dcache_subsystem (
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
    I_ADDR_PACKET dcache_write_addr, oldest_miss_addr;
    logic dcache_full;
    MSHR_PACKET new_mshr_entry;

    dcache dcache_inst (
        .clock        (clock),
        .reset        (reset),
        // Fetch Stage read
        .read_addrs   (read_addrs),
        .cache_outs   (cache_outs),
        // Prefetch snooping - removed for dcache
        .snooping_addr('0),
        .addr_found   (),
        .full         (dcache_full),
        // Dcache write mem_data, when mem_data_tag matches head of MSHR
        .write_addr   (dcache_write_addr),
        .write_data   (mem_data)
    );


    d_mshr d_mshr_inst (
        .clock          (clock),
        .reset          (reset),
        // Prefetch snooping - removed for dcache
        .snooping_addr  ('0),
        .addr_found     (),
        // When mem_req_accepted
        .new_entry      (new_mshr_entry),
        // Mem data back
        .mem_data_tag   (mem_data_tag),
        .mem_data_d_addr(dcache_write_addr)
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

    // Mem request address logic - TODO: implement based on load/store misses for dcache
    always_comb begin
        mem_req_addr = '0;
        // TODO: Add logic for dcache memory requests based on load/store misses
    end

    // MSHR entry logic - add immediately when request is accepted and tag is valid
    always_comb begin
        new_mshr_entry = '0;
        if (mem_req_accepted && current_req_tag != 0) begin
            new_mshr_entry = '{valid: 1'b1,
                              mem_tag: current_req_tag,
                              d_tag: mem_req_addr.addr.tag};
        end
    end

endmodule

// this should never be full, so no logic for handling full FIFO head tail edge case
module d_mshr #(
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
    output I_ADDR_PACKET mem_data_d_addr  // to write to dcache
);

    // MSHR Internals
    localparam D_CACHE_INDEX_BITS = $clog2(MSHR_WIDTH);
    MSHR_PACKET [MSHR_WIDTH-1:0] mshr_entries, next_mshr_entries;
    logic [D_CACHE_INDEX_BITS-1:0] head, next_head, tail, next_tail;

    // Snooping logic - check if address is already in MSHR
    always_comb begin
        addr_found = 1'b0;
        for (int i = 0; i < MSHR_WIDTH; i++) begin
            if (mshr_entries[i].valid && (mshr_entries[i].d_tag == snooping_addr.tag)) begin
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
            mem_data_d_addr.addr = '{zeros: 16'b0,
                                     tag: mshr_entries[head].d_tag,
                                     block_offset: 3'b0};
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

endmodule

module dcache #(
    parameter MEM_DEPTH = `DCACHE_LINES,
    parameter D_CACHE_INDEX_BITS = $clog2(MEM_DEPTH),
    parameter MEM_WIDTH = 1 + 1 + `DTAG_BITS + `MEM_BLOCK_BITS  // valid + dirty + tag + data
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

    // Dcache write mem_data, when mem_data_tag matches head of MSHR
    input I_ADDR_PACKET write_addr,
    input MEM_BLOCK     write_data
);

    CACHE_DATA [1:0]                  cache_outs_temp;
    D_CACHE_LINE [MEM_DEPTH-1:0]          cache_lines;
    D_CACHE_LINE                          cache_line_write;
    logic [MEM_DEPTH-1:0]                 cache_write_enable_mask;
    logic [MEM_DEPTH-1:0]                 cache_write_no_evict_one_hot;
    logic [D_CACHE_INDEX_BITS-1:0]        cache_write_evict_index;
    logic [D_CACHE_INDEX_BITS-1:0]        lfsr_out;
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
        .WIDTH(D_CACHE_INDEX_BITS)
    ) LFSR_inst (
        .clk(clock),
        .rst(reset),
        .op(lfsr_out)
    );
    
    // Modulo to ensure index is within bounds
    assign cache_write_evict_index = D_CACHE_INDEX_BITS'(lfsr_out % MEM_DEPTH);


    // Cache write logic
    always_comb begin
        cache_write_enable_mask = '0;
        cache_line_write = '{valid: write_addr.valid,
                            dirty: 1'b0,  // Data from memory is clean
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
