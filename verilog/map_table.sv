`include "sys_defs.svh"

module map_table (
    input clock,
    input reset,

    // From dispatch: new register mappings (structured)
    input MAP_TABLE_WRITE_REQUEST [`N-1:0] write_reqs,

    // From dispatch: read requests (structured)
    input MAP_TABLE_READ_REQUEST read_req,

    // To dispatch: read responses (structured)
    output MAP_TABLE_READ_RESPONSE read_resp,

    // From CDB: broadcasts that update ready bits
    input CDB_ENTRY [`N-1:0] cdb_broadcasts,

    // Mispredict recovery: output entire table and accept table overwrite
    output MAP_ENTRY [`ARCH_REG_SZ-1:0] table_snapshot,   // For copying to arch table
    input  MAP_ENTRY [`ARCH_REG_SZ-1:0] table_restore,    // For restoring from arch table
    input  logic                        table_restore_en  // Enable table restore on mispredict
);

    // Internal map table state (ARCH_REG_SZ architectural registers)
    MAP_ENTRY [`ARCH_REG_SZ-1:0] map_table_reg, map_table_next;

    // Helper function: Check if a physical register tag matches any CDB broadcast
    function automatic logic tag_ready(PHYS_TAG tag);
        for (int i = 0; i < `N; i++) begin
            if (cdb_broadcasts[i].valid && cdb_broadcasts[i].tag == tag) begin
                return 1'b1;
            end
        end
        return 1'b0;
    endfunction

    // Combinational logic: Compute next state
    always_comb begin
        map_table_next = map_table_reg;

        // First, update ready bits based on CDB broadcasts
        for (int arch = 0; arch < `ARCH_REG_SZ; arch++) begin
            if (tag_ready(map_table_reg[arch].phys_reg)) begin
                map_table_next[arch].ready = 1'b1;
            end
        end

        // Then, apply new mappings from dispatch write ports (overrides CDB updates if same register)
        for (int i = 0; i < `N; i++) begin
            if (write_reqs[i].valid) begin
                map_table_next[write_reqs[i].addr].phys_reg = write_reqs[i].phys_reg;
                // New mappings start as not ready (unless CDB broadcast happens this cycle)
                map_table_next[write_reqs[i].addr].ready = tag_ready(write_reqs[i].phys_reg);
            end
        end
    end

    // Output assignment: drive read ports from internal table
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            read_resp.rs1_entries[i]  = map_table_reg[read_req.rs1_addrs[i]];
            read_resp.rs2_entries[i]  = map_table_reg[read_req.rs2_addrs[i]];
            read_resp.told_entries[i] = map_table_reg[read_req.told_addrs[i]];
        end
    end

    // Output entire table for mispredict recovery
    assign table_snapshot = map_table_reg;

    // Sequential logic: Update state on clock edge
    always_ff @(posedge clock) begin
        if (reset) begin
            // Initialize map table: x0 is always mapped to physical register 0 and ready
            map_table_reg <= '0;

            for (int i = 0; i < `ARCH_REG_SZ; i++) begin
                map_table_reg[i].phys_reg <= PHYS_TAG'(i);  // Identity mapping initially
                map_table_reg[i].ready    <= 1'b1;  // All initially ready
            end
        end else if (table_restore_en) begin
            // Mispredict recovery: restore entire table from architected state
            map_table_reg <= table_restore;
        end else begin
            map_table_reg <= map_table_next;
        end
    end

endmodule

module arch_map_table #(
    parameter int NUM_WRITE_PORTS = `N
) (
    input clock,
    input reset,

    // From retire: update architected register mappings (parameterized write ports)
    input logic [NUM_WRITE_PORTS-1:0] write_enables,
    input REG_IDX [NUM_WRITE_PORTS-1:0] write_addrs,
    input PHYS_TAG [NUM_WRITE_PORTS-1:0] write_phys_regs,

    // Read ports for selective access (always ready)
    input  REG_IDX   [(`N)-1:0] read_addrs,
    output MAP_ENTRY [(`N)-1:0] read_entries,

    // Mispredict recovery: output entire table and accept table overwrite
    output MAP_ENTRY [`ARCH_REG_SZ-1:0] table_snapshot,   // For copying from map_table
    input  MAP_ENTRY [`ARCH_REG_SZ-1:0] table_restore,    // For restoring from map_table
    input  logic                        table_restore_en  // Enable table restore on mispredict
);

    // Internal architected map table state (ARCH_REG_SZ architectural registers)
    MAP_ENTRY [`ARCH_REG_SZ-1:0] arch_map_table_reg, arch_map_table_next;

    // Combinational logic: Compute next state
    always_comb begin
        arch_map_table_next = arch_map_table_reg;

        // Apply new mappings from retire write ports
        for (int i = 0; i < NUM_WRITE_PORTS; i++) begin
            if (write_enables[i]) begin
                arch_map_table_next[write_addrs[i]].phys_reg = write_phys_regs[i];
                arch_map_table_next[write_addrs[i]].ready    = 1'b1;  // Architected mappings are always ready
            end
        end
    end

    // Output assignment: drive read ports from internal table
    always_comb begin
        for (int i = 0; i < `N; i++) begin
            read_entries[i] = arch_map_table_reg[read_addrs[i]];
        end
    end

    // Output entire table for mispredict recovery
    assign table_snapshot = arch_map_table_reg;

    // Sequential logic: Update state on clock edge
    always_ff @(posedge clock) begin
        if (reset) begin
            // Initialize architected map table: identity mapping
            arch_map_table_reg <= '0;

            for (int i = 0; i < `ARCH_REG_SZ; i++) begin
                arch_map_table_reg[i].phys_reg <= PHYS_TAG'(i);  // Identity mapping initially
                arch_map_table_reg[i].ready    <= 1'b1;  // Architected state always ready
            end
        end else if (table_restore_en) begin
            // Mispredict recovery: restore entire table from speculative map_table
            arch_map_table_reg <= table_restore;
        end else begin
            arch_map_table_reg <= arch_map_table_next;
        end
    end

endmodule
