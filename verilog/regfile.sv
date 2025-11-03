`include "sys_defs.svh"

module regfile (
    input logic clock,
    input logic reset,

    // Structured read interface (organized by FU type)
    input  PRF_READ_TAGS read_tags,
    output PRF_READ_DATA read_data,

    // Write interface from CDB
    input CDB_ENTRY [`CDB_SZ-1:0] cdb_writes,

    // debug output TODO remove when no longer needed (especially for synthesis)
    output DATA [`PHYS_REG_SZ_R10K-1:0] regfile_entries

);

    DATA [`PHYS_REG_SZ_R10K-1:0]
        register_file_entries,
        register_file_entries_next;  // synthesis inference: packed array -> flip flops,  unpacked array -> RAM

    // Forwarding logic for each FU type
    always_comb begin
        // ALU reads
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            read_data.alu[i] = read_register_with_forwarding(read_tags.alu[i]);
        end

        // MULT reads
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            read_data.mult[i] = read_register_with_forwarding(read_tags.mult[i]);
        end

        // BRANCH reads
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            read_data.branch[i] = read_register_with_forwarding(read_tags.branch[i]);
        end

        // MEM reads
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            read_data.mem[i] = read_register_with_forwarding(read_tags.mem[i]);
        end

        // Write from CDB
        register_file_entries_next = register_file_entries;
        for (int i = 0; i < `CDB_SZ; i++) begin
            if (cdb_writes[i].valid) begin
                register_file_entries_next[cdb_writes[i].tag] = cdb_writes[i].data;
            end
        end
    end

    // Helper function for reading with forwarding
    function DATA read_register_with_forwarding(PHYS_TAG tag);
        // Check for forwarding from CDB writes
        for (int write_port = 0; write_port < `CDB_SZ; write_port++) begin
            if (cdb_writes[write_port].valid && tag == cdb_writes[write_port].tag) begin
                return cdb_writes[write_port].data;
            end
        end

        // Normal read
        if (tag == '0) begin  // 0 register read
            return '0;
        end else begin
            return register_file_entries[tag];
        end
    endfunction

    always_ff @(posedge clock) begin
        if (reset) begin
            register_file_entries <= '0;
        end else begin
            register_file_entries <= register_file_entries_next;
        end
    end

    // remove as part of TODO above (when no longer needed)
    assign regfile_entries = register_file_entries;


endmodule
