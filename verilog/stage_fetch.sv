 `include "sys_defs.svh"
`include "ISA.svh"

module stage_fetch (
    input logic                       clock,
    input logic                       reset,

    // icache_subsystem
    output I_ADDR_PACKET [1:0]        read_addrs,
    input CACHE_DATA     [1:0]        cache_data,

    // branch predictor
    output BP_PREDICT_REQUEST         bp_request,
    input  BP_PREDICT_RESPONSE        bp_response,
    output logic                      bp_response_used,

    // retire when mispredict
    input I_ADDR_PACKET               correct_branch_target,

    // instruction buffer
    input logic          [`IB_IDX_BITs-1:0]   ib_free_slots,
    output FETCH_PACKET  [3:0]                fetch_packet,
);

    ADDR PC, PC_next;
    INST [3:0] inst;

    // Logic for fetching this PC to IB and send out


    // Logic for next PC calculation


    // PC update
    always_ff @(posedge clock) begin
        if (reset) begin
            PC <= '0;
        end else begin
            PC <= PC_next;
        end
    end 

endmodule