`include "sys_defs.svh"

module stage_execute (
    input clock,
    input reset,

    input logic mispredict,  // Flush pipelines on mispredict

    // Inputs from issue stage (structured)
    input ISSUE_ENTRIES issue_entries,

    // Input from CDB for early tag broadcast (data forwarding)
    input CDB_ENTRY [`N-1:0] cdb_data,

    // To PRF for operand reads (structured)
    output PRF_READ_EN   prf_read_en_src1,
    output PRF_READ_EN   prf_read_en_src2,
    output PRF_READ_TAGS prf_read_tag_src1,
    output PRF_READ_TAGS prf_read_tag_src2,
    input  PRF_READ_DATA prf_read_data_src1,
    input  PRF_READ_DATA prf_read_data_src2,

    // // Interface to D-cache for memory operations (IGNORE FOR NOW)
    // output MEM_COMMAND proc2Dcache_command [`NUM_FU_MEM-1:0],
    // output ADDR proc2Dcache_addr [`NUM_FU_MEM-1:0],
    // output DATA proc2Dcache_data [`NUM_FU_MEM-1:0],  // For stores
    // output MEM_SIZE proc2Dcache_size [`NUM_FU_MEM-1:0],
    // input logic [`NUM_FU_MEM-1:0] Dcache_valid,
    // input DATA [`NUM_FU_MEM-1:0] Dcache_data,  // For loads

    output logic [`NUM_FU_MULT-1:0] mult_request,
    output CDB_FU_OUTPUTS fu_outputs,

    // To complete stage
    output logic [`N-1:0] ex_valid,
    output EX_COMPLETE_ENTRY ex_comp[`N-1:0],

    // From CDB for grant selection
    input logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus
);

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // ALU signals
    DATA [`NUM_FU_ALU-1:0] alu_opas, alu_opbs, alu_results;
    ALU_FUNC [`NUM_FU_ALU-1:0] alu_funcs;

    // MULT signals
    DATA [`NUM_FU_MULT-1:0] mult_rs1, mult_rs2, mult_result;
    MULT_FUNC [`NUM_FU_MULT-1:0] mult_func;
    logic [`NUM_FU_MULT-1:0] mult_start, mult_done;

    // FU metadata for completion
    EX_COMPLETE_ENTRY [`NUM_FU_ALU-1:0] alu_meta;
    EX_COMPLETE_ENTRY [`NUM_FU_MULT-1:0] mult_meta;
    EX_COMPLETE_ENTRY [`NUM_FU_MULT-1:0] mult_meta_in;
    EX_COMPLETE_ENTRY [`NUM_FU_MULT-1:0] mult_meta_out;
    EX_COMPLETE_ENTRY [`NUM_FU_BRANCH-1:0] branch_meta;
    EX_COMPLETE_ENTRY [`NUM_FU_MEM-1:0] mem_meta;
    EX_COMPLETE_ENTRY [`NUM_FU_TOTAL-1:0] flat_meta;

    // BRANCH signals
    DATA [`NUM_FU_BRANCH-1:0] branch_rs1, branch_rs2;
    logic [2:0][`NUM_FU_BRANCH-1:0] branch_funcs;  // Array of 3-bit func values
    logic [`NUM_FU_BRANCH-1:0] branch_take;
    ADDR [`NUM_FU_BRANCH-1:0] branch_target;

    // MEM signals (placeholder for future implementation)
    // DATA [`NUM_FU_MEM-1:0] mem_addr, mem_data;

    // Operand resolution: choose between PRF data, CDB forwarding, or RS stored value
    PRF_READ_DATA resolved_src1, resolved_src2;

    // =========================================================================
    // PRF Read Request Generation (Structured)
    // =========================================================================

    // Read from PRF when operands are needed (structured by FU type)
    always_comb begin
        // Initialize
        prf_read_en_src1  = '0;
        prf_read_en_src2  = '0;
        prf_read_tag_src1 = '0;
        prf_read_tag_src2 = '0;

        // ALU reads
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            if (issue_entries.alu[i].valid) begin
                prf_read_en_src1.alu[i] = 1'b1;  // SRC1 always comes from register
                prf_read_en_src2.alu[i]  = (issue_entries.alu[i].opb_select == OPB_IS_RS2);  // SRC2 only from PRF if it's a register
                prf_read_tag_src1.alu[i] = issue_entries.alu[i].src1_tag;
                prf_read_tag_src2.alu[i] = issue_entries.alu[i].src2_tag;
            end
        end

        // MULT reads
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            if (issue_entries.mult[i].valid) begin
                prf_read_en_src1.mult[i]  = 1'b1;  // SRC1 always comes from register
                prf_read_en_src2.mult[i]  = 1'b1;  // SRC2 always comes from register for MULT
                prf_read_tag_src1.mult[i] = issue_entries.mult[i].src1_tag;
                prf_read_tag_src2.mult[i] = issue_entries.mult[i].src2_tag;
            end
        end

        // BRANCH reads
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            if (issue_entries.branch[i].valid) begin
                prf_read_en_src1.branch[i]  = 1'b1;  // SRC1 always comes from register
                prf_read_en_src2.branch[i]  = 1'b1;  // SRC2 always comes from register for BRANCH
                prf_read_tag_src1.branch[i] = issue_entries.branch[i].src1_tag;
                prf_read_tag_src2.branch[i] = issue_entries.branch[i].src2_tag;
            end
        end

        // MEM reads (placeholder)
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            if (issue_entries.mem[i].valid) begin
                prf_read_en_src1.mem[i]  = 1'b1;  // SRC1 always comes from register
                prf_read_en_src2.mem[i]  = 1'b1;  // SRC2 always comes from register for MEM (placeholder)
                prf_read_tag_src1.mem[i] = issue_entries.mem[i].src1_tag;
                prf_read_tag_src2.mem[i] = issue_entries.mem[i].src2_tag;
            end
        end
    end

    // =========================================================================
    // Operand Resolution with CDB Forwarding (Structured)
    // =========================================================================

    always_comb begin
        // Default: use PRF read data
        resolved_src1 = prf_read_data_src1;
        resolved_src2 = prf_read_data_src2;

        // Check for CDB forwarding (data just computed this cycle)
        // ALU forwarding
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            for (int cdb_idx = 0; cdb_idx < `N; cdb_idx++) begin
                if (cdb_data[cdb_idx].valid) begin
                    if (prf_read_tag_src1.alu[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src1.alu[i] = cdb_data[cdb_idx].data;
                    end
                    if (prf_read_tag_src2.alu[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src2.alu[i] = cdb_data[cdb_idx].data;
                    end
                end
            end
        end

        // MULT forwarding
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            for (int cdb_idx = 0; cdb_idx < `N; cdb_idx++) begin
                if (cdb_data[cdb_idx].valid) begin
                    if (prf_read_tag_src1.mult[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src1.mult[i] = cdb_data[cdb_idx].data;
                    end
                    if (prf_read_tag_src2.mult[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src2.mult[i] = cdb_data[cdb_idx].data;
                    end
                end
            end
        end

        // BRANCH forwarding
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            for (int cdb_idx = 0; cdb_idx < `N; cdb_idx++) begin
                if (cdb_data[cdb_idx].valid) begin
                    if (prf_read_tag_src1.branch[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src1.branch[i] = cdb_data[cdb_idx].data;
                    end
                    if (prf_read_tag_src2.branch[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src2.branch[i] = cdb_data[cdb_idx].data;
                    end
                end
            end
        end

        // MEM forwarding
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            for (int cdb_idx = 0; cdb_idx < `N; cdb_idx++) begin
                if (cdb_data[cdb_idx].valid) begin
                    if (prf_read_tag_src1.mem[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src1.mem[i] = cdb_data[cdb_idx].data;
                    end
                    if (prf_read_tag_src2.mem[i] == cdb_data[cdb_idx].tag) begin
                        resolved_src2.mem[i] = cdb_data[cdb_idx].data;
                    end
                end
            end
        end
    end

    // =========================================================================
    // FU Metadata Computation
    // =========================================================================

    always_comb begin
        // ALU metadata (full entry)
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            if (issue_entries.alu[i].valid) begin
                alu_meta[i].rob_idx = issue_entries.alu[i].rob_idx;
                alu_meta[i].branch_valid = 0;
                alu_meta[i].mispredict = 0;
                alu_meta[i].branch_taken = 0;
                alu_meta[i].branch_target = 0;
                alu_meta[i].dest_pr = issue_entries.alu[i].dest_tag;
                alu_meta[i].result = alu_results[i];
            end else begin
                alu_meta[i] = '0;
            end
        end

        // MULT metadata in (full entry to FU)
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            if (issue_entries.mult[i].valid) begin
                mult_meta_in[i].rob_idx = issue_entries.mult[i].rob_idx;
                mult_meta_in[i].branch_valid = 0;
                mult_meta_in[i].mispredict = 0;
                mult_meta_in[i].branch_taken = 0;
                mult_meta_in[i].branch_target = 0;
                mult_meta_in[i].dest_pr = issue_entries.mult[i].dest_tag;
                mult_meta_in[i].result = 0;  // Set on completion in FU
            end else begin
                mult_meta_in[i] = '0;
            end
        end

        // MULT metadata out (from FU - override result when done)
        mult_meta = mult_meta_out;
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            if (mult_done[i]) begin
                mult_meta[i].result = mult_result[i];
            end
        end

        // BRANCH metadata (full entry)
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            if (issue_entries.branch[i].valid) begin
                branch_meta[i].rob_idx = issue_entries.branch[i].rob_idx;
                branch_meta[i].branch_valid = 1;
                branch_meta[i].branch_taken = branch_take[i];
                branch_meta[i].branch_target = branch_target[i];
                branch_meta[i].mispredict = (branch_take[i] != issue_entries.branch[i].pred_taken);
                branch_meta[i].dest_pr = issue_entries.branch[i].dest_tag;
                branch_meta[i].result = issue_entries.branch[i].PC + 4;
            end else begin
                branch_meta[i] = '0;
            end
        end

        // MEM metadata (full entry, placeholder)
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            if (issue_entries.mem[i].valid) begin
                mem_meta[i].rob_idx = issue_entries.mem[i].rob_idx;
                mem_meta[i].branch_valid = 0;
                mem_meta[i].mispredict = 0;
                mem_meta[i].branch_taken = 0;
                mem_meta[i].branch_target = 0;
                mem_meta[i].dest_pr = issue_entries.mem[i].dest_tag;
                mem_meta[i].result = 0;  // Placeholder for mem ops
            end else begin
                mem_meta[i] = '0;
            end
        end

        // Flatten metadata (explicit elements to avoid replication issues)
        flat_meta = '{mult_meta[0], mem_meta[0], alu_meta[0], alu_meta[1], alu_meta[2], branch_meta[0]};
    end

    // =========================================================================
    // ALU Functional Units (Combinational)
    // =========================================================================

    always_comb begin
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            // Select operand A
            case (issue_entries.alu[i].opa_select)
                OPA_IS_RS1:  alu_opas[i] = resolved_src1.alu[i];
                OPA_IS_NPC:  alu_opas[i] = issue_entries.alu[i].PC + 4;
                OPA_IS_PC:   alu_opas[i] = issue_entries.alu[i].PC;
                OPA_IS_ZERO: alu_opas[i] = 0;
                default:     alu_opas[i] = 32'hdeadface;
            endcase

            // Select operand B
            case (issue_entries.alu[i].opb_select)
                OPB_IS_RS2:   alu_opbs[i] = resolved_src2.alu[i];
                OPB_IS_I_IMM: alu_opbs[i] = issue_entries.alu[i].src2_immediate;
                OPB_IS_S_IMM: alu_opbs[i] = issue_entries.alu[i].src2_immediate;
                OPB_IS_B_IMM: alu_opbs[i] = issue_entries.alu[i].src2_immediate;
                OPB_IS_U_IMM: alu_opbs[i] = issue_entries.alu[i].src2_immediate;
                OPB_IS_J_IMM: alu_opbs[i] = issue_entries.alu[i].src2_immediate;
                default:      alu_opbs[i] = 32'hfacefeed;
            endcase

            // Extract ALU function
            alu_funcs[i] = issue_entries.alu[i].op_type.func;
        end
    end

    // Instantiate ALU modules
    alu alu_inst[`NUM_FU_ALU-1:0] (
        .opa(alu_opas),
        .opb(alu_opbs),
        .alu_func(alu_funcs),
        .result(alu_results)
    );

    // ALU outputs to CDB
    always_comb begin
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            fu_outputs.alu[i].valid = issue_entries.alu[i].valid;
            fu_outputs.alu[i].tag   = issue_entries.alu[i].dest_tag;
            fu_outputs.alu[i].data  = alu_results[i];
        end
    end

    // =========================================================================
    // MULT Functional Units (Pipelined)
    // =========================================================================

    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            mult_rs1[i]   = resolved_src1.mult[i];
            mult_rs2[i]   = resolved_src2.mult[i];
            mult_func[i]  = issue_entries.mult[i].op_type.func;
            mult_start[i] = issue_entries.mult[i].valid;
        end
    end

    // Instantiate MULT modules

    mult mult_inst[`NUM_FU_MULT-1:0] (
        .clock(clock),
        .reset(reset | mispredict),
        .start(mult_start),
        .rs1(mult_rs1),
        .rs2(mult_rs2),
        .func(mult_func),
        .meta_in(mult_meta_in),
        .result(mult_result),
        .request(mult_request),
        .done(mult_done),
        .meta_out(mult_meta_out)
    );

    // MULT outputs to CDB (only when done)
    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            fu_outputs.mult[i].valid = mult_done[i];
            fu_outputs.mult[i].tag   = issue_entries.mult[i].dest_tag;
            fu_outputs.mult[i].data  = mult_result[i];
            // mult_request[i] is driven by mult module structurally - no procedural assignment needed
        end
    end

    // =========================================================================
    // BRANCH Functional Units (Combinational)
    // =========================================================================

    always_comb begin
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            branch_rs1[i]   = resolved_src1.branch[i];
            branch_rs2[i]   = resolved_src2.branch[i];
            branch_funcs[i] = issue_entries.branch[i].op_type.func[2:0];
        end
    end

    // Instantiate BRANCH modules
    conditional_branch branch_inst[`NUM_FU_BRANCH-1:0] (
        .rs1 (branch_rs1),
        .rs2 (branch_rs2),
        .func(branch_funcs),  // Connect func array directly
        .take(branch_take)
    );

    // Compute branch targets: PC + offset (offset is pre-stored in src2_immediate)
    always_comb begin
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            branch_target[i] = issue_entries.branch[i].PC + issue_entries.branch[i].src2_immediate;
        end
    end

    // BRANCH outputs to CDB
    always_comb begin
        fu_outputs.branch[0].valid = issue_entries.branch[0].valid;
        fu_outputs.branch[0].tag   = issue_entries.branch[0].dest_tag;
        // For branches, data could be PC+4 for branch-and-link or 0 for conditional branches
        fu_outputs.branch[0].data  = issue_entries.branch[0].PC + 4;  // TODO: check if needed
    end

    // =========================================================================
    // MEM Functional Units (Placeholder)
    // =========================================================================

    // TODO: Implement memory functional units when mem.sv is completed
    always_comb begin
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            fu_outputs.mem[i].valid = 1'b0;
            fu_outputs.mem[i].tag   = '0;
            fu_outputs.mem[i].data  = '0;
        end
    end

    // =========================================================================
    // EX/COMP Register Fill via CDB Grants
    // =========================================================================

    always_comb begin
        ex_valid = '0;
        ex_comp = '{
            default: '{
                rob_idx: 0,
                branch_valid: 0,
                mispredict: 0,
                branch_taken: 0,
                branch_target: 0,
                dest_pr: 0,
                result: 0
            }
        };
        for (int i = 0; i < `N; i++) begin
            for (int j = 0; j < `NUM_FU_TOTAL; j++) begin
                if (gnt_bus[i][j]) begin
                    ex_valid[i] = 1'b1;
                    ex_comp[i]  = flat_meta[j];
                end
            end
        end
    end

endmodule
