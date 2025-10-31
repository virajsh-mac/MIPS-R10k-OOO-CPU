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

    localparam int READ_PORTS = 2 * `N;
    localparam int WRITE_PORTS = `N;

    DATA mem[`PHYS_REG_SZ_R10K-1:0];

    always_comb begin
        for (int rp = 0; rp < READ_PORTS; rp++) begin
            DATA val;

            if (read_idx[rp] == PHYS_TAG'(0)) val = '0;
            else val = mem[read_idx[rp]];

            for (int wp = 0; wp < WRITE_PORTS; wp++) begin
                if (write_en[wp] && (write_idx[wp] != PHYS_TAG'(0)) && (write_idx[wp] == read_idx[rp])) begin
                    val = write_data[wp];
                end
            end

            read_out[rp] = val;
        end
    end


    always_ff @(posedge clock or negedge reset_n) begin
        if (!reset_n) begin
            for (int i = 0; i < `PHYS_REG_SZ_R10K; i++) mem[i] <= '0;
        end else begin
            // commit up to N writes; ignore writes to phys-0
            for (int wp = 0; wp < WRITE_PORTS; wp++) begin  // variable wp here means "read port"
                if (write_en[wp] && (write_idx[wp] != PHYS_TAG'(0))) mem[write_idx[wp]] <= write_data[wp];
            end
            // keep phys-0 hardwired to zero
            mem[PHYS_TAG'(0)] <= '0;
        end
    end

endmodule
