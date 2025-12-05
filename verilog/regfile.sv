`include "sys_defs.svh"

module regfile (
    input logic clock,
    input logic reset,

    // Structured read interface (organized by FU type)
    input  PRF_READ_TAGS [1:0] read_tags,
    output PRF_READ_DATA [1:0] read_data,

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
        // ----------------
        // SRC1
        // ----------------

        // ALU reads
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            read_data[0].alu[i] = read_register_with_forwarding(read_tags[0].alu[i]);
        end

        // MULT reads
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            read_data[0].mult[i] = read_register_with_forwarding(read_tags[0].mult[i]);
        end

        // BRANCH reads
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            read_data[0].branch[i] = read_register_with_forwarding(read_tags[0].branch[i]);
        end

        // MEM reads
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            read_data[0].mem[i] = read_register_with_forwarding(read_tags[0].mem[i]);
        end

        // ----------------
        // SRC2
        // ----------------

        // ALU reads
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            read_data[1].alu[i] = read_register_with_forwarding(read_tags[1].alu[i]);
        end

        // MULT reads
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            read_data[1].mult[i] = read_register_with_forwarding(read_tags[1].mult[i]);
        end

        // BRANCH reads
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            read_data[1].branch[i] = read_register_with_forwarding(read_tags[1].branch[i]);
        end

        // MEM reads
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            read_data[1].mem[i] = read_register_with_forwarding(read_tags[1].mem[i]);
        end


        // Write from CDB
        register_file_entries_next = register_file_entries;
        for (int i = 0; i < `CDB_SZ; i++) begin
            if (cdb_writes[i].valid && cdb_writes[i].tag != '0) begin
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

    // ============================================================
    // Register File Debug Display
    // ============================================================
`ifdef DEBUG
    always_ff @(posedge clock) begin
        logic found_nonzero;

        if (!reset) begin
            $display("========================================");
            $display("=== REGISTER FILE STATE (Cycle %0t) ===", $time);
            $display("========================================");
            
            // Print non-zero register file entries
            $display("--- Non-Zero Register File Entries ---");
            found_nonzero = 1'b0;
            for (int i = 0; i < `PHYS_REG_SZ_R10K; i++) begin
                if (register_file_entries[i] != '0) begin
                    $display("  PRF[%0d] = %h", i, register_file_entries[i]);
                    found_nonzero = 1'b1;
                end
            end
            if (!found_nonzero) begin
                $display("  (All entries are zero)");
            end
            
            // Print CDB writes
            $display("--- CDB Writes (this cycle) ---");
            for (int i = 0; i < `CDB_SZ; i++) begin
                if (cdb_writes[i].valid && cdb_writes[i].tag != '0) begin
                    $display("  CDB[%0d]: Tag=%0d Data=%h", i, cdb_writes[i].tag, cdb_writes[i].data);
                end
            end
            
            $display("");
        end
    end
`endif

endmodule
