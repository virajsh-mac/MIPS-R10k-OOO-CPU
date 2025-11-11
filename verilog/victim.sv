module victim_cache (
    // lookup suite
    input logic lookup_en,
    input ADDR lookup_addr,

    // Insert suite
    input logic insert_en,
    input ADDR insert_addr,
    input cache_data insert_data,

    // swap suite
    input logic swap_en,
    input ADDR swap_addr,
    input cache_data swap_data,


    // output
    output logic vc_full,

    output cache_data reinstate_data,
    out ADDR reinstate_addr

    output logic vc_hit,
    // TODO: if hit, needs to output cache line back to icache
)




    // TODO finish victim cache
    memDP #(
        .WIDTH   ($BITS(MEM_BLOCK)),
        .DEPTH   (4),      // victim cache only allowed 4 mem_blocks
        .READ_PORTS(1),
        .BYPASS_EN (0)
    ) cache_bank[1:0] (
        .clock(clock),
        .reset(reset),
        .re   ('1),
        .raddr(),
        .rdata(),
        .we(),
        .waddr(),
        .wdata(write_data.cache_line)
    );

endmodule
