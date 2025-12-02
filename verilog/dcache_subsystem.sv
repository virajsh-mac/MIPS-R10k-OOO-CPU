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
    output I_ADDR_PACKET        mem_req_addr,
    input  logic                mem_req_accepted,
    
    // Arbitor IOs - Write requests (dirty writebacks)
    output I_ADDR_PACKET        mem_write_addr,
    output MEM_BLOCK            mem_write_data,
    output logic                mem_write_valid
);

    // Internal wires
    D_ADDR_PACKET dcache_write_addr, oldest_miss_addr;
    logic dcache_full;
    D_MSHR_PACKET new_mshr_entry;
    D_CACHE_LINE evicted_line, writeback_line;
    logic evicted_valid, writeback_valid;
    CACHE_DATA [1:0] dcache_outs, victim_outs, combined_outs;

    dcache dcache_inst (
        .clock        (clock),
        .reset        (reset),
        // Fetch Stage read
        .read_addrs   (read_addrs),
        .cache_outs   (dcache_outs),
        // Prefetch snooping - removed for dcache
        .snooping_addr('0),
        .addr_found   (),
        .full         (dcache_full),
        // Dcache write mem_data, when mem_data_tag matches head of MSHR
        .write_addr   (dcache_write_addr),
        .write_data   (mem_data),
        // Victim cache interface
        .evicted_line (evicted_line),
        .evicted_valid(evicted_valid)
    );

    victim_cache victim_cache_inst (
        .clock          (clock),
        .reset          (reset),
        // Evicted line from dcache
        .evicted_line   (evicted_line),
        .evicted_valid  (evicted_valid),
        // Read interface
        .read_addrs     (read_addrs),
        .victim_outs    (victim_outs),
        // Writeback interface
        .writeback_line (writeback_line),
        .writeback_valid(writeback_valid)
    );

    // Combine dcache and victim cache outputs
    // Priority: dcache hits take precedence over victim cache hits
    always_comb begin
        for (int i = 0; i < 2; i++) begin
            if (dcache_outs[i].valid) begin
                combined_outs[i] = dcache_outs[i];
            end else begin
                combined_outs[i] = victim_outs[i];
            end
        end
    end
    assign cache_outs = combined_outs;


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

    // Oldest miss address logic - check combined outputs (dcache + victim cache)
    always_comb begin
        oldest_miss_addr = '0;
        if (read_addrs[0].valid && !combined_outs[0].valid) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[0].addr;
        end else if (read_addrs[1].valid && !combined_outs[1].valid) begin
            oldest_miss_addr.valid = 1'b1;
            oldest_miss_addr.addr  = read_addrs[1].addr;
        end
    end

    // Mem write logic - send dirty writebacks from victim cache
    always_comb begin
        mem_write_valid = writeback_valid && writeback_line.dirty;
        mem_write_addr = '0;
        mem_write_data = '0;
        
        if (mem_write_valid) begin
            mem_write_addr.valid = 1'b1;
            mem_write_addr.addr = '{zeros: 16'b0,
                                   tag: writeback_line.tag,
                                   index: writeback_line.index,
                                   bank: writeback_line.bank,
                                   block_offset: 3'b0};
            mem_write_data = writeback_line.data;
        end
    end

    // Mem read request address logic - handle cache misses
    always_comb begin
        mem_req_addr = '0;
        
        // Send read requests for cache misses (load/store)
        if (oldest_miss_addr.valid) begin
            mem_req_addr = oldest_miss_addr;
        end
    end

    // MSHR entry logic - add immediately when request is accepted and tag is valid
    always_comb begin
        new_mshr_entry = '0;
        if (mem_req_accepted && current_req_tag != 0) begin
            new_mshr_entry = '{valid: 1'b1,
                              mem_tag: current_req_tag,
                              d_addr: mem_req_addr.addr};
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
    input D_MSHR_PACKET new_entry,

    // Mem data back
    input  MEM_TAG       mem_data_tag,
    output D_ADDR_PACKET mem_data_d_addr  // to write to dcache
);

    // MSHR Internals
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

endmodule

module dcache #(
    parameter MEM_DEPTH = `DCACHE_LINES,
    parameter D_CACHE_INDEX_BITS = $clog2(MEM_DEPTH),
    parameter MEM_WIDTH = 1 + 1 + `DTAG_BITS + `DSET_INDEX_BITS + 1 + `MEM_BLOCK_BITS  // valid + dirty + tag + index + bank + data
) (
    input clock,
    input reset,

    // Memory operations read
    input D_ADDR_PACKET [1:0] read_addrs,
    output CACHE_DATA [1:0] cache_outs,

    // Prefetch snooping
    input  D_ADDR_PACKET snooping_addr,  // to decide whether to send mem request
    output logic         addr_found,
    output logic         full,

    // Dcache write mem_data, when mem_data_tag matches head of MSHR
    input D_ADDR_PACKET write_addr,
    input MEM_BLOCK     write_data,
    
    // Victim cache interface - output evicted lines
    output D_CACHE_LINE evicted_line,
    output logic        evicted_valid
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
                            index: write_addr.addr.index,
                            bank: write_addr.addr.bank,
                            data: write_data};
        evicted_line = '0;
        evicted_valid = 1'b0;
        
        if (write_addr.valid) begin
            // Try to find an invalid (free) slot first
            if (|cache_write_no_evict_one_hot) begin
                cache_write_enable_mask = cache_write_no_evict_one_hot;
                // No eviction when writing to a free slot
            end else begin
                // No free slot, evict using LFSR-selected index
                cache_write_enable_mask[cache_write_evict_index] = 1'b1;
                // Output the evicted line to victim cache
                evicted_line = cache_lines[cache_write_evict_index];
                evicted_valid = cache_lines[cache_write_evict_index].valid;
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

// Victim cache module - FIFO for evicted dcache lines
module victim_cache #(
    parameter VICTIM_DEPTH = `DCACHE_VICTIM_SZ,
    parameter VICTIM_INDEX_BITS = $clog2(VICTIM_DEPTH),
    parameter MEM_WIDTH = 1 + 1 + `DTAG_BITS + `DSET_INDEX_BITS + 1 + `MEM_BLOCK_BITS  // valid + dirty + tag + index + bank + data
) (
    input clock,
    input reset,

    // Evicted line from dcache
    input D_CACHE_LINE evicted_line,
    input logic        evicted_valid,

    // Read interface for checking hits
    input  D_ADDR_PACKET [1:0] read_addrs,
    output CACHE_DATA [1:0]    victim_outs,

    // Writeback interface - evicted dirty line from victim cache
    output D_CACHE_LINE writeback_line,
    output logic        writeback_valid
);

    D_CACHE_LINE [VICTIM_DEPTH-1:0] victim_lines;
    D_CACHE_LINE                    victim_line_write;
    logic [VICTIM_DEPTH-1:0]        victim_write_enable_mask;
    logic [VICTIM_INDEX_BITS-1:0]   head, next_head, tail, next_tail;
    logic                           full, empty;

    // memDP instances for each victim cache entry
    memDP #(
        .WIDTH(MEM_WIDTH),
        .DEPTH(1'b1)
    ) victim_line[VICTIM_DEPTH-1:0] (
        .clock(clock),
        .reset(reset),
        .re(1'b1),
        .raddr(1'b0),
        .rdata(victim_lines),
        .we(victim_write_enable_mask),
        .waddr(1'b0),
        .wdata(victim_line_write)
    );

    // FIFO status
    always_comb begin
        full = (tail + 1'b1) % VICTIM_DEPTH == head && victim_lines[head].valid;
        empty = (head == tail) && !victim_lines[head].valid;
    end

    // FIFO write logic
    always_comb begin
        next_head = head;
        next_tail = tail;
        victim_write_enable_mask = '0;
        victim_line_write = '0;
        writeback_line = '0;
        writeback_valid = 1'b0;

        if (evicted_valid) begin
            // Write evicted line to tail
            victim_write_enable_mask[tail] = 1'b1;
            victim_line_write = evicted_line;
            next_tail = VICTIM_INDEX_BITS'((tail + 1'b1) % VICTIM_DEPTH);

            // If full, evict from head
            if (full) begin
                writeback_line = victim_lines[head];
                writeback_valid = victim_lines[head].valid && victim_lines[head].dirty;
                next_head = VICTIM_INDEX_BITS'((head + 1'b1) % VICTIM_DEPTH);
            end
        end
    end

    // Sequential logic
    always_ff @(posedge clock) begin
        if (reset) begin
            head <= '0;
            tail <= '0;
        end else begin
            head <= next_head;
            tail <= next_tail;
        end
    end

    // Read logic - check for hits
    CACHE_DATA [1:0] victim_outs_temp;
    always_comb begin
        victim_outs_temp = '0;
        for (int j = 0; j < 2; j++) begin
            for (int i = 0; i < VICTIM_DEPTH; i++) begin
                if (read_addrs[j].valid && victim_lines[i].valid &&
                    (read_addrs[j].addr.tag == victim_lines[i].tag)) begin
                    victim_outs_temp[j].data = victim_lines[i].data;
                    victim_outs_temp[j].valid = 1'b1;
                end
            end
        end
    end
    assign victim_outs = victim_outs_temp;

endmodule
