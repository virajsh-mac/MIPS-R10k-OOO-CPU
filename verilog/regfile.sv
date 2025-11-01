`include "sys_defs.svh"

module reg_file (
    input logic clock,
    input logic reset,

    input PHYS_TAG [2*`N-1:0] read_tags,
    output DATA    [2*`N-1:0] read_outputs,

    input logic    [`N-1:0] write_en, // At most `N inst completes
    input PHYS_TAG [`N-1:0] write_tags,
    input DATA     [`N-1:0] write_data
);

    DATA [`PHYS_REG_SZ_R10K-1:0] register_file_entries, register_file_entries_next; // synthesis inference: packed array -> flip flops,  unpacked array -> RAM

    always_comb begin
        register_file_entries_next = register_file_entries;

        for (int i = 0; i < 2 * `N; i++) begin
            if (read_tags[i] == '0) begin // 0 register read
                read_outputs[i] = '0;
            end else if (read_tags[i] == write_tags[i]) begin // write forwarding
                read_outputs[i] = write_data[i];
            end else begin
                read_outputs[i] = register_file_entries[read_tags[i]]; // normal read
            end
        end

        for (int i = 0; i < `N; i++) begin
            if (write_en[i]) begin
                register_file_entries_next[write_tags[i]] = write_data[i];
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            register_file_entries <= '0;
        end else begin
            register_file_entries <= register_file_entries_next;
        end
    end

endmodule
