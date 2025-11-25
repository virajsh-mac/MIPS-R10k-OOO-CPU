`include "verilog/sys_defs.svh"

module victim #(
    parameter VICTIM_SIZE = 4
) (
    input clock,
    input reset,

    // Lookup for loads (similar to dcache reads)
    input  D_ADDR_PACKET [1:0] lookup_addrs,
    output D_CACHE_DATA  [1:0] victim_outs,
    output logic [1:0]          victim_hits,

    // Insert evicted line from main dcache
    input  D_CACHE_LINE         insert_line,
    input  D_ADDR               insert_addr,
    output logic                inserted,

    // Debug outputs
    output logic [1:0]          victim_hits_dbg
);

    localparam VICTIM_INDEX_BITS = $clog2(VICTIM_SIZE);
    D_CACHE_LINE [VICTIM_SIZE-1:0] victim_entries;
    logic [VICTIM_SIZE-1:0] valid_bits;
    logic [VICTIM_INDEX_BITS-1:0] evict_index;
    logic victim_full;

    // Random eviction index generator
    LFSR #(.WIDTH(VICTIM_INDEX_BITS)) lfsr_inst (
        .clk(clock),
        .rst(reset),
        .op(evict_index)
    );

    // Victim entries storage (use memDP or simple reg array for small size)
    // For simplicity, use reg array
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i = 0; i < VICTIM_SIZE; i++) begin
                victim_entries[i] <= '0;
            end
        end else if (insert_line.valid) begin
            // Insert logic: find empty or evict random if full
            if (~victim_full) begin
                // Find first empty slot
                for (int i = 0; i < VICTIM_SIZE; i++) begin
                    if (~victim_entries[i].valid) begin
                        victim_entries[i] <= insert_line;
                        victim_entries[i].tag <= insert_addr.tag;
                        inserted <= 1'b1;
                    end
                end
                inserted <= 1'b0;
            end else begin
                // Evict random
                victim_entries[evict_index] <= insert_line;
                victim_entries[evict_index].tag <= insert_addr.tag;
                inserted <= 1'b1;
            end
        end
    end

    // Lookup logic (combinational)
    always_comb begin
        victim_outs[0] = '0;
        victim_outs[1] = '0;
        victim_hits[0] = 1'b0;
        victim_hits[1] = 1'b0;

        for (int j = 0; j <= 1; j++) begin
            for (int i = 0; i < VICTIM_SIZE; i++) begin
                logic hit;
                hit = (lookup_addrs[j].valid && victim_entries[i].valid && 
                       (lookup_addrs[j].addr.tag == victim_entries[i].tag));
                if (hit) begin
                    victim_outs[j].valid = 1'b1;
                    victim_outs[j].data = victim_entries[i].data;
                    victim_hits[j] = 1'b1;
                end
            end
        end
    end

    // Full check
    assign victim_full = &valid_bits;

    // Valid bits
    for (genvar i = 0; i < VICTIM_SIZE; i++) begin
        assign valid_bits[i] = victim_entries[i].valid;
    end

    // Insert success
    assign inserted = insert_line.valid && ( ~victim_full || 1'b1 );  // Always insert, evicting if full

    // Debug
    assign victim_hits_dbg = victim_hits;

endmodule
