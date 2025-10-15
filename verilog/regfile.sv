`include "sys_defs.svh"

module regfile (
    input logic clock,
    input logic reset_n,

    input  PHYS_TAG [2*`N-1:0] read_idx,
    output DATA     [2*`N-1:0] read_out,

    input logic    [`N-1:0] write_en,
    input PHYS_TAG [`N-1:0] write_idx,
    input DATA     [`N-1:0] write_data
);

    DATA mem[`PHYS_REG_SZ_R10K-1:0];

    // Combinational reads with same-cycle bypass
    genvar i;
    generate
        for (i = 0; i < 2 * `N; i++) begin : GEN_READS
            always_comb begin
                DATA r;

                // zero-reg or array
                if (read_idx[i] == PHYS_TAG'(0)) r = '0;
                else r = mem[read_idx[i]];

                // same-cycle bypass logic
                for (int w = 0; w < `N; w++) begin
                    if (write_en[w] &&
                        (write_idx[w] != PHYS_TAG'(0)) &&
                        (write_idx[w] == read_idx[i])) begin
                        r = write_data[w];
                    end
                end

                read_out[i] = r;
            end
        end
    endgenerate

    // Posedge writes and reset logic
    always_ff @(posedge clock or negedge reset_n) begin
        if (!reset_n) begin
            for (int k = 0; k < `PHYS_REG_SZ_R10K; k++) mem[k] <= '0;
        end else begin
            for (int w = 0; w < `N; w++) begin
                if (write_en[w] && (write_idx[w] != PHYS_TAG'(0))) begin
                    mem[write_idx[w]] <= write_data[w];
                end
            end

            mem[PHYS_TAG'(0)] <= '0;  // keep zero hard-wired
        end
    end

endmodule
