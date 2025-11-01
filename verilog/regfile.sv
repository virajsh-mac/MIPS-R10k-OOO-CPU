`include "sys_defs.svh"

module regfile #(
    parameter int NUM_READ_PORTS  = `NUM_FU_TOTAL,
    parameter int NUM_WRITE_PORTS = `CDB_SZ
) (
    input logic clock,
    input logic reset,

    input PHYS_TAG [NUM_READ_PORTS-1:0] read_tags,
    output DATA    [NUM_READ_PORTS-1:0] read_outputs,

    input logic    [NUM_WRITE_PORTS-1:0] write_en,    // At most `N inst completes
    input PHYS_TAG [NUM_WRITE_PORTS-1:0] write_tags,
    input DATA     [NUM_WRITE_PORTS-1:0] write_data
);

    DATA [`PHYS_REG_SZ_R10K-1:0]
        register_file_entries,
        register_file_entries_next;  // synthesis inference: packed array -> flip flops,  unpacked array -> RAM

    DATA  [NUM_READ_PORTS-1:0] forwarding_data;  // forwarding tags for all read ports
    logic [NUM_READ_PORTS-1:0] forwarding;  // forwarding check for all read ports

    always_comb begin
        forwarding = '0;
        for (int i = 0; i < NUM_READ_PORTS; i++) begin
            // Parallel forwarding logic
            for (int write_port = 0; write_port < NUM_WRITE_PORTS; write_port++) begin
                if (read_tags[i] == write_tags[write_port] && write_en[write_port]) begin
                    forwarding[i] = 1'b1;
                    forwarding_data[i] = write_data[write_port];
                end
            end

            // Read outputs
            if (read_tags[i] == '0) begin  // 0 register read
                read_outputs[i] = '0;
            end else if (forwarding[i]) begin  // write forwarding
                read_outputs[i] = forwarding_data[i];
            end else begin
                read_outputs[i] = register_file_entries[read_tags[i]];  // normal read
            end
        end

        // Write
        register_file_entries_next = register_file_entries;
        for (int i = 0; i < NUM_WRITE_PORTS; i++) begin
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
