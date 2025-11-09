/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  icache.sv                                           //
//                                                                     //
//  Description :  The instruction cache module that reroutes memory   //
//                 accesses to decrease misses.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

typedef struct packed {
    logic [12-`ICACHE_LINE_BITS:0] tags;
    logic                          valid;
} ICACHE_TAG;

/**
 * A quick overview of the cache and memory:
 *
 * We've increased the memory latency from 1 cycle to 100ns. which will be
 * multiple cycles for any reasonable processor. Thus, memory can have multiple
 * transactions pending and coordinates them via memory tags (different meaning
 * than cache tags) which represent a transaction it's working on. Memory tags
 * are 4 bits long since 15 mem accesses can be live at one time, and only one
 * access happens per cycle.
 *
 * On a request, memory responds with the tag it will use for that transaction.
 * Then, ceiling(100ns/clock period) cycles later, it will return the data with
 * the corresponding tag. The 0 tag is a sentinel value and unused. It would be
 * very difficult to push your clock period past 100ns/15=6.66ns, so 15 tags is
 * sufficient.
 *
 * This cache coordinates those memory tags to speed up fetching reused data.
 *
 * Note that this cache is blocking, and will wait on one memory request before
 * sending another (unless the input address changes, in which case it abandons
 * that request). Implementing a non-blocking cache can count towards simple
 * feature points, but will require careful management of memory tags.
 */

module icache_blocking (
    input clock,
    input reset,

    // From memory
    input MEM_TAG   Imem2proc_transaction_tag, // Should be zero unless there is a response
    input MEM_BLOCK Imem2proc_data,
    input MEM_TAG   Imem2proc_data_tag,

    // From fetch stage
    input ADDR proc2Icache_addr,

    // To memory
    output MEM_COMMAND proc2Imem_command,
    output ADDR        proc2Imem_addr,

    // To fetch stage
    output MEM_BLOCK Icache_data_out, // Data is mem[proc2Icache_addr]
    output logic     Icache_valid_out // When valid is high
);

    // Note: cache tags, not memory tags
    logic [12-`$clog2(`ICACHE_LINES):0] current_tag,   last_tag;
    logic [`$clog2(`ICACHE_LINES) -1:0] current_index, last_index;
    logic                          got_mem_data;
    logic                          icache_we;


    // ---- Cache data ---- //

    ICACHE_TAG [`ICACHE_LINES-1:0] icache_tags;

    memDP #(
        .WIDTH     ($bits(MEM_BLOCK)),
        .DEPTH     (`ICACHE_LINES),
        .READ_PORTS(1),
        .BYPASS_EN (0))
    icache_mem (
        .clock(clock),
        .reset(reset),
        .re   (1'b1),
        .raddr(current_index),
        .rdata(Icache_data_out),
        .we   (icache_we),
        .waddr(current_index),
        .wdata(Imem2proc_data)
    );
    

    // ---- Addresses and final outputs ---- //

    assign {current_tag, current_index} = proc2Icache_addr[15:3];

    assign Icache_valid_out =  icache_tags[current_index].valid &&
                              (icache_tags[current_index].tags == current_tag);

    // ---- Main cache logic ---- //

    MEM_TAG current_mem_tag; // The current memory tag we might be waiting on
    logic miss_outstanding; // Whether a miss has received its response tag to wait on

    logic changed_addr;
    logic update_mem_tag;
    logic unanswered_miss;

    assign got_mem_data = (current_mem_tag == Imem2proc_data_tag) && (current_mem_tag != 0);
    // ignore incoming write if address is changing, otherwise we will write the new tag
    assign icache_we    = got_mem_data && !changed_addr;

    assign changed_addr = (current_index != last_index) || (current_tag != last_tag);

    // Set mem tag to zero if we changed_addr, and keep resetting while there is
    // a miss_outstanding. Then set to zero when we got_mem_data.
    // (this relies on Imem2proc_transaction_tag being zero when there is no request)
    assign update_mem_tag = changed_addr || miss_outstanding || got_mem_data;

    // If we have a new miss or still waiting for the response tag, we might
    // need to wait for the response tag because dcache has priority over icache
    assign unanswered_miss = changed_addr ? !Icache_valid_out :
                                        miss_outstanding && (Imem2proc_transaction_tag == 0);

    // Keep sending memory requests until we receive a response tag or change addresses
    assign proc2Imem_command = (miss_outstanding && !changed_addr) ? MEM_LOAD : MEM_NONE;
    assign proc2Imem_addr    = {proc2Icache_addr[31:3],3'b0};

    // ---- Cache state registers ---- //

    always_ff @(posedge clock) begin
        if (reset) begin
            last_index       <= -1; // These are -1 to get ball rolling when
            last_tag         <= -1; // reset goes low because addr "changes"
            current_mem_tag  <= '0;
            miss_outstanding <= '0;
            icache_tags      <= '0; // Set all cache tags and valid bits to 0
        end else begin
            last_index       <= current_index;
            last_tag         <= current_tag;
            miss_outstanding <= unanswered_miss;
            if (update_mem_tag) begin
                current_mem_tag <= Imem2proc_transaction_tag;
            end
            if (icache_we) begin // If data came from memory, meaning tag matches
                icache_tags[current_index].tags  <= current_tag;
                icache_tags[current_index].valid <= 1'b1;
            end
        end
    end

endmodule // icache
