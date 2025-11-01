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
    output EX_COMPLETE_PACKET ex_comp,

    // From CDB for grant selection
    input logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus
);

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // FU result storage (SoA)
    FU_RESULTS fu_results;

    // ALU signals
    DATA [`NUM_FU_ALU-1:0] alu_opas, alu_opbs;
    ALU_FUNC [`NUM_FU_ALU-1:0] alu_funcs;

    // MULT signals
    DATA [`NUM_FU_MULT-1:0] mult_rs1, mult_rs2;
    MULT_FUNC [`NUM_FU_MULT-1:0] mult_func;
    logic [`NUM_FU_MULT-1:0] mult_start, mult_done;

    // FU metadata computed on-the-fly in EX/COMP register fill (SoA approach)

    // Dummy metadata for mult module (not used in SoA approach)
    EX_COMPLETE_ENTRY mult_meta_dummy_in [`NUM_FU_MULT-1:0];
    EX_COMPLETE_ENTRY mult_meta_dummy_out[`NUM_FU_MULT-1:0];

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
        .result(fu_results.alu)
    );

    // ALU outputs to CDB
    always_comb begin
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            fu_outputs.alu[i].valid = issue_entries.alu[i].valid;
            fu_outputs.alu[i].tag   = issue_entries.alu[i].dest_tag;
            fu_outputs.alu[i].data  = fu_results.alu[i];
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
        .meta_in(mult_meta_dummy_in),
        .result(fu_results.mult),
        .request(mult_request),
        .done(mult_done),
        .meta_out(mult_meta_dummy_out)
    );

    // MULT outputs to CDB (only when done)
    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            fu_outputs.mult[i].valid = mult_done[i];
            fu_outputs.mult[i].tag   = issue_entries.mult[i].dest_tag;
            fu_outputs.mult[i].data  = fu_results.mult[i];
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
            fu_results.mem[i] = '0;  // Initialize MEM results to 0
            fu_outputs.mem[i].valid = 1'b0;
            fu_outputs.mem[i].tag = '0;
            fu_outputs.mem[i].data = '0;
        end
    end

    // Add constants for FU index ranges (to avoid manual offset calculations)
    localparam int MULT_START = 0;
    localparam int MEM_START = MULT_START + `NUM_FU_MULT;
    localparam int ALU_START = MEM_START + `NUM_FU_MEM;
    localparam int BRANCH_START = ALU_START + `NUM_FU_ALU;

    // Separate grant signals for each FU type (extracted from global gnt_bus)
    logic [`N-1:0][`NUM_FU_MULT-1:0]   gnt_mult;
    logic [`N-1:0][`NUM_FU_MEM-1:0]    gnt_mem;
    logic [`N-1:0][`NUM_FU_ALU-1:0]    gnt_alu;
    logic [`N-1:0][`NUM_FU_BRANCH-1:0] gnt_branch;

    always_comb begin
        // Extract MULT grants
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_MULT; k++) begin
                gnt_mult[i][k] = gnt_bus[i][MULT_START+k];
            end
        end

        // Extract MEM grants
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_MEM; k++) begin
                gnt_mem[i][k] = gnt_bus[i][MEM_START+k];
            end
        end

        // Extract ALU grants
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_ALU; k++) begin
                gnt_alu[i][k] = gnt_bus[i][ALU_START+k];
            end
        end

        // Extract BRANCH grants
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_BRANCH; k++) begin
                gnt_branch[i][k] = gnt_bus[i][BRANCH_START+k];
            end
        end
    end

    // =========================================================================
    // EX/COMP Register Fill via CDB Grants (SoA approach with per-FU grants)
    // =========================================================================

    always_comb begin
        ex_valid = '0;
        ex_comp = '0;

        // Initialize defaults (non-branch)
        ex_comp.branch_valid = '0;
        ex_comp.mispredict = '0;
        ex_comp.branch_taken = '0;
        ex_comp.branch_target = '0;

        // MULT grants
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_MULT; k++) begin
                if (gnt_mult[i][k]) begin
                    ex_valid[i] = 1'b1;
                    ex_comp.rob_idx[i] = mult_done[k] ? issue_entries.mult[k].rob_idx : '0;
                    ex_comp.dest_pr[i] = issue_entries.mult[k].dest_tag;
                    ex_comp.result[i] = fu_results.mult[k];
                end
            end
        end

        // MEM grants (placeholder)
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_MEM; k++) begin
                if (gnt_mem[i][k]) begin
                    ex_valid[i] = 1'b1;
                    ex_comp.rob_idx[i] = issue_entries.mem[k].rob_idx;
                    ex_comp.dest_pr[i] = issue_entries.mem[k].dest_tag;
                    ex_comp.result[i] = fu_results.mem[k];  // Will be 0 for now
                end
            end
        end

        // ALU grants
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_ALU; k++) begin
                if (gnt_alu[i][k]) begin
                    ex_valid[i] = 1'b1;
                    ex_comp.rob_idx[i] = issue_entries.alu[k].rob_idx;
                    ex_comp.dest_pr[i] = issue_entries.alu[k].dest_tag;
                    ex_comp.result[i] = fu_results.alu[k];
                end
            end
        end

        // BRANCH grants
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_BRANCH; k++) begin
                if (gnt_branch[i][k]) begin
                    ex_valid[i] = 1'b1;
                    ex_comp.rob_idx[i] = issue_entries.branch[k].rob_idx;
                    ex_comp.branch_valid[i] = 1;
                    ex_comp.branch_taken[i] = branch_take[k];
                    ex_comp.branch_target[i] = branch_target[k];
                    ex_comp.mispredict[i] = (branch_take[k] != issue_entries.branch[k].pred_taken);
                    ex_comp.dest_pr[i] = issue_entries.branch[k].dest_tag;
                    ex_comp.result[i] = issue_entries.branch[k].PC + 4;
                end
            end
        end
    end

endmodule
