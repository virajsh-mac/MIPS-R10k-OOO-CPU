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

    output logic [`NUM_FU_MULT-1:0] mult_request,
    output CDB_FU_OUTPUTS fu_outputs,

    // To complete stage
    output logic [`N-1:0] ex_valid,
    output EX_COMPLETE_PACKET ex_comp,

    // Late CDB requests from MEM FUs (for cache miss handling)
    output logic [`NUM_FU_MEM-1:0] mem_cdb_requests_out,

    // to store queue
    output EXECUTE_STOREQ_ENTRY [`NUM_FU_MEM-1:0] mem_storeq_entries,

    // to/from dcache
    output D_ADDR_PACKET [1:0] dcache_read_addrs,
    input  CACHE_DATA    [1:0] dcache_read_data,

    // Store queue forwarding interface
    output logic      [`NUM_FU_MEM-1:0] sq_lookup_valid,
    output ADDR       [`NUM_FU_MEM-1:0] sq_lookup_addr,
    output STOREQ_IDX [`NUM_FU_MEM-1:0] sq_lookup_sq_tail,
    input  logic      [`NUM_FU_MEM-1:0] sq_forward_valid,
    input  DATA       [`NUM_FU_MEM-1:0] sq_forward_data,
    input  logic      [`NUM_FU_MEM-1:0] sq_forward_stall,

    // From CDB for grant selection
    input logic [`N-1:0][`NUM_FU_TOTAL-1:0] gnt_bus
);

    // =========================================================================
    // FU Flat Indexing Constants
    // =========================================================================
    localparam int BRANCH_START = 0;
    localparam int ALU_START    = BRANCH_START + `NUM_FU_BRANCH;
    localparam int MEM_START    = ALU_START + `NUM_FU_ALU;
    localparam int MULT_START   = MEM_START + `NUM_FU_MEM;

    // =========================================================================
    // Unified Grant Computation
    // =========================================================================
    // Single flat array: for each FU, OR together grants from all CDB slots
    logic [`NUM_FU_TOTAL-1:0] fu_grant_any;

    always_comb begin
        for (int k = 0; k < `NUM_FU_TOTAL; k++) begin
            fu_grant_any[k] = 1'b0;
            for (int i = 0; i < `N; i++) begin
                fu_grant_any[k] |= gnt_bus[i][k];
            end
        end
    end

    // Extract grants for multi-cycle FUs (used to clear their holding state)
    logic [`NUM_FU_MULT-1:0] mult_grant;
    logic [`NUM_FU_MEM-1:0]  mem_grant;

    assign mult_grant = fu_grant_any[MULT_START +: `NUM_FU_MULT];
    assign mem_grant  = fu_grant_any[MEM_START +: `NUM_FU_MEM];

    // =========================================================================
    // Internal Signals
    // =========================================================================
    FU_RESULTS fu_results;

    // ALU signals
    DATA [`NUM_FU_ALU-1:0] alu_opas, alu_opbs;
    ALU_FUNC [`NUM_FU_ALU-1:0] alu_funcs;

    // MULT signals
    DATA [`NUM_FU_MULT-1:0] mult_rs1, mult_rs2;
    MULT_FUNC [`NUM_FU_MULT-1:0] mult_func;
    logic [`NUM_FU_MULT-1:0] mult_start, mult_done;

    // BRANCH signals
    DATA [`NUM_FU_BRANCH-1:0] branch_rs1, branch_rs2;
    BRANCH_FUNC [`NUM_FU_BRANCH-1:0] branch_funcs;
    ADDR [`NUM_FU_BRANCH-1:0] branch_pcs;
    DATA [`NUM_FU_BRANCH-1:0] branch_offsets;
    logic [`NUM_FU_BRANCH-1:0] branch_take;
    ADDR [`NUM_FU_BRANCH-1:0] branch_targets;

    // MEM signals
    DATA [`NUM_FU_MEM-1:0] mem_rs1, mem_rs2, mem_src2_imm;
    MEM_FUNC [`NUM_FU_MEM-1:0] mem_funcs;
    DATA [`NUM_FU_MEM-1:0] mem_data;
    ADDR [`NUM_FU_MEM-1:0] mem_addr;

    // Metadata for mult pipeline
    EX_COMPLETE_ENTRY mult_meta_in  [`NUM_FU_MULT-1:0];
    EX_COMPLETE_ENTRY mult_meta_out [`NUM_FU_MULT-1:0];

    // Mult start pulse generation state
    logic [`NUM_FU_MULT-1:0] prev_mult_valid;
    ROB_IDX [`NUM_FU_MULT-1:0] prev_mult_rob_idx;

    // Operand resolution
    PRF_READ_DATA resolved_src1, resolved_src2;

    // Load counter for dcache port allocation
    int load_count;

    // =========================================================================
    // PRF Read Request Generation
    // =========================================================================
    always_comb begin
        prf_read_en_src1  = '0;
        prf_read_en_src2  = '0;
        prf_read_tag_src1 = '0;
        prf_read_tag_src2 = '0;

        // ALU
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            if (issue_entries.alu[i].valid) begin
                prf_read_en_src1.alu[i]  = 1'b1;
                prf_read_en_src2.alu[i]  = (issue_entries.alu[i].opb_select == OPB_IS_RS2);
                prf_read_tag_src1.alu[i] = issue_entries.alu[i].src1_tag;
                prf_read_tag_src2.alu[i] = issue_entries.alu[i].src2_tag;
            end
        end

        // MULT
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            if (issue_entries.mult[i].valid) begin
                prf_read_en_src1.mult[i]  = 1'b1;
                prf_read_en_src2.mult[i]  = 1'b1;
                prf_read_tag_src1.mult[i] = issue_entries.mult[i].src1_tag;
                prf_read_tag_src2.mult[i] = issue_entries.mult[i].src2_tag;
            end
        end

        // BRANCH
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            if (issue_entries.branch[i].valid) begin
                prf_read_en_src1.branch[i]  = 1'b1;
                prf_read_en_src2.branch[i]  = 1'b1;
                prf_read_tag_src1.branch[i] = issue_entries.branch[i].src1_tag;
                prf_read_tag_src2.branch[i] = issue_entries.branch[i].src2_tag;
            end
        end

        // MEM
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            if (issue_entries.mem[i].valid) begin
                prf_read_en_src1.mem[i]  = 1'b1;
                prf_read_en_src2.mem[i]  = 1'b1;
                prf_read_tag_src1.mem[i] = issue_entries.mem[i].src1_tag;
                prf_read_tag_src2.mem[i] = issue_entries.mem[i].src2_tag;
            end
        end
    end

    // =========================================================================
    // Operand Resolution with CDB Forwarding
    // =========================================================================
    always_comb begin
        resolved_src1 = prf_read_data_src1;
        resolved_src2 = prf_read_data_src2;

        // ALU forwarding
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            for (int c = 0; c < `N; c++) begin
                if (cdb_data[c].valid && prf_read_tag_src1.alu[i] == cdb_data[c].tag)
                    resolved_src1.alu[i] = cdb_data[c].data;
                if (cdb_data[c].valid && prf_read_tag_src2.alu[i] == cdb_data[c].tag)
                    resolved_src2.alu[i] = cdb_data[c].data;
            end
        end

        // MULT forwarding
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            for (int c = 0; c < `N; c++) begin
                if (cdb_data[c].valid && prf_read_tag_src1.mult[i] == cdb_data[c].tag)
                    resolved_src1.mult[i] = cdb_data[c].data;
                if (cdb_data[c].valid && prf_read_tag_src2.mult[i] == cdb_data[c].tag)
                    resolved_src2.mult[i] = cdb_data[c].data;
            end
        end

        // BRANCH forwarding
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            for (int c = 0; c < `N; c++) begin
                if (cdb_data[c].valid && prf_read_tag_src1.branch[i] == cdb_data[c].tag)
                    resolved_src1.branch[i] = cdb_data[c].data;
                if (cdb_data[c].valid && prf_read_tag_src2.branch[i] == cdb_data[c].tag)
                    resolved_src2.branch[i] = cdb_data[c].data;
            end
        end

        // MEM forwarding
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            for (int c = 0; c < `N; c++) begin
                if (cdb_data[c].valid && prf_read_tag_src1.mem[i] == cdb_data[c].tag)
                    resolved_src1.mem[i] = cdb_data[c].data;
                if (cdb_data[c].valid && prf_read_tag_src2.mem[i] == cdb_data[c].tag)
                    resolved_src2.mem[i] = cdb_data[c].data;
            end
        end
    end

    // =========================================================================
    // ALU Functional Units (Combinational)
    // =========================================================================
    always_comb begin
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            case (issue_entries.alu[i].opa_select)
                OPA_IS_RS1:  alu_opas[i] = resolved_src1.alu[i];
                OPA_IS_NPC:  alu_opas[i] = issue_entries.alu[i].PC + 4;
                OPA_IS_PC:   alu_opas[i] = issue_entries.alu[i].PC;
                OPA_IS_ZERO: alu_opas[i] = 0;
                default:     alu_opas[i] = 32'hdeadface;
            endcase

            case (issue_entries.alu[i].opb_select)
                OPB_IS_RS2:   alu_opbs[i] = resolved_src2.alu[i];
                OPB_IS_I_IMM,
                OPB_IS_S_IMM,
                OPB_IS_B_IMM,
                OPB_IS_U_IMM,
                OPB_IS_J_IMM: alu_opbs[i] = issue_entries.alu[i].src2_immediate;
                default:      alu_opbs[i] = 32'hfacefeed;
            endcase

            alu_funcs[i] = issue_entries.alu[i].op_type.func;
        end
    end

    alu alu_inst[`NUM_FU_ALU-1:0] (
        .opa(alu_opas),
        .opb(alu_opbs),
        .alu_func(alu_funcs),
        .result(fu_results.alu)
    );

    // =========================================================================
    // MULT Functional Units (Pipelined)
    // =========================================================================

    // Start pulse: new instruction when valid rises or ROB index changes
    always_ff @(posedge clock) begin
        if (reset | mispredict) begin
            prev_mult_valid   <= '0;
            prev_mult_rob_idx <= '0;
        end else begin
            for (int i = 0; i < `NUM_FU_MULT; i++) begin
                prev_mult_valid[i]   <= issue_entries.mult[i].valid;
                prev_mult_rob_idx[i] <= issue_entries.mult[i].rob_idx;
            end
        end
    end

    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            mult_start[i] = issue_entries.mult[i].valid &&
                           (!prev_mult_valid[i] ||
                            issue_entries.mult[i].rob_idx != prev_mult_rob_idx[i]);

            mult_rs1[i]  = resolved_src1.mult[i];
            mult_rs2[i]  = resolved_src2.mult[i];
            mult_func[i] = issue_entries.mult[i].op_type.func;

            mult_meta_in[i] = '{
                rob_idx:       issue_entries.mult[i].valid ? issue_entries.mult[i].rob_idx : '0,
                branch_valid:  1'b0,
                branch_taken:  1'b0,
                branch_target: '0,
                dest_pr:       issue_entries.mult[i].valid ? issue_entries.mult[i].dest_tag : '0,
                result:        '0
            };
        end
    end

    mult mult_inst[`NUM_FU_MULT-1:0] (
        .clock(clock),
        .reset(reset | mispredict),
        .start(mult_start),
        .grant(mult_grant),
        .rs1(mult_rs1),
        .rs2(mult_rs2),
        .func(mult_func),
        .meta_in(mult_meta_in),
        .result(fu_results.mult),
        .request(mult_request),
        .done(mult_done),
        .meta_out(mult_meta_out)
    );

    // =========================================================================
    // BRANCH Functional Units (Combinational)
    // =========================================================================
    always_comb begin
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            branch_rs1[i]     = resolved_src1.branch[i];
            branch_rs2[i]     = resolved_src2.branch[i];
            branch_funcs[i]   = issue_entries.branch[i].op_type.func;
            branch_pcs[i]     = issue_entries.branch[i].PC;
            branch_offsets[i] = issue_entries.branch[i].src2_immediate;
        end
    end

    branch branch_inst[`NUM_FU_BRANCH-1:0] (
        .rs1(branch_rs1),
        .rs2(branch_rs2),
        .func(branch_funcs),
        .pc(branch_pcs),
        .offset(branch_offsets),
        .take(branch_take),
        .target(branch_targets)
    );

    // =========================================================================
    // MEM Functional Units
    // =========================================================================
    EXECUTE_STOREQ_ENTRY [`NUM_FU_MEM-1:0] mem_store_queue_entries;
    CDB_ENTRY [`NUM_FU_MEM-1:0] mem_cdb_results;
    logic [`NUM_FU_MEM-1:0] mem_cdb_requests;
    logic [`NUM_FU_MEM-1:0] mem_is_load_request;
    logic [`NUM_FU_MEM-1:0] mem_is_store;
    D_ADDR [`NUM_FU_MEM-1:0] mem_dcache_addrs;
    logic [`NUM_FU_MEM-1:0] mem_lookup_valid;
    ADDR [`NUM_FU_MEM-1:0] mem_lookup_addr;
    STOREQ_IDX [`NUM_FU_MEM-1:0] mem_lookup_sq_tail;
    logic [1:0] mem_fu_to_dcache_slot [`NUM_FU_MEM-1:0];

    always_comb begin
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            mem_rs1[i]      = resolved_src1.mem[i];
            mem_rs2[i]      = resolved_src2.mem[i];
            mem_src2_imm[i] = issue_entries.mem[i].src2_immediate;
            mem_funcs[i]    = issue_entries.mem[i].op_type.func;
        end
    end

    generate
        for (genvar i = 0; i < `NUM_FU_MEM; i++) begin : mem_fu_gen
            mem_fu mem_inst (
                .clock(clock),
                .reset(reset | mispredict),
                .valid(issue_entries.mem[i].valid),
                .func(mem_funcs[i]),
                .rs1(mem_rs1[i]),
                .rs2(mem_rs2[i]),
                .imm(mem_src2_imm[i]),
                .store_queue_idx(issue_entries.mem[i].store_queue_idx),
                .dest_tag(issue_entries.mem[i].dest_tag),
                .cache_hit_data(dcache_read_data[mem_fu_to_dcache_slot[i]]),
                .forward_valid(sq_forward_valid[i]),
                .forward_data(sq_forward_data[i]),
                .forward_stall(sq_forward_stall[i]),
                .grant(mem_grant[i]),
                .addr(mem_addr[i]),
                .data(mem_data[i]),
                .store_queue_entry(mem_store_queue_entries[i]),
                .cdb_result(mem_cdb_results[i]),
                .cdb_request(mem_cdb_requests[i]),
                .is_load_request(mem_is_load_request[i]),
                .is_store_op(mem_is_store[i]),
                .dcache_addr(mem_dcache_addrs[i]),
                .lookup_valid(mem_lookup_valid[i]),
                .lookup_addr(mem_lookup_addr[i]),
                .lookup_sq_tail(mem_lookup_sq_tail[i])
            );
        end
    endgenerate

    assign sq_lookup_valid   = mem_lookup_valid;
    assign sq_lookup_addr    = mem_lookup_addr;
    assign sq_lookup_sq_tail = mem_lookup_sq_tail;
    assign mem_storeq_entries = mem_store_queue_entries;
    assign mem_cdb_requests_out = mem_cdb_requests;

    // Dcache port allocation: first 2 load requests get ports
    always_comb begin
        dcache_read_addrs = '0;
        mem_fu_to_dcache_slot = '{default: '0};
        load_count = 0;
        for (int i = 0; i < `NUM_FU_MEM && load_count < 2; i++) begin
            if (mem_is_load_request[i]) begin
                dcache_read_addrs[load_count].valid = 1'b1;
                dcache_read_addrs[load_count].addr  = mem_dcache_addrs[i];
                mem_fu_to_dcache_slot[i] = load_count;
                load_count++;
            end
        end
    end

    // =========================================================================
    // CDB Output Generation
    // =========================================================================
    always_comb begin
        // ALU: valid when issued and has destination
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            fu_outputs.alu[i].valid = issue_entries.alu[i].valid &&
                                      (issue_entries.alu[i].dest_tag != '0);
            fu_outputs.alu[i].tag   = issue_entries.alu[i].dest_tag;
            fu_outputs.alu[i].data  = fu_results.alu[i];
        end

        // MULT: valid when done and has destination
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            fu_outputs.mult[i].valid = mult_done[i] && (mult_meta_out[i].dest_pr != '0);
            fu_outputs.mult[i].tag   = mult_meta_out[i].dest_pr;
            fu_outputs.mult[i].data  = fu_results.mult[i];
        end

        // BRANCH: only JAL/JALR produce results
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            if (issue_entries.branch[i].valid &&
                (issue_entries.branch[i].op_type.func == JAL ||
                 issue_entries.branch[i].op_type.func == JALR) &&
                (issue_entries.branch[i].dest_tag != '0)) begin
                fu_outputs.branch[i].valid = 1'b1;
                fu_outputs.branch[i].tag   = issue_entries.branch[i].dest_tag;
                fu_outputs.branch[i].data  = issue_entries.branch[i].PC + 4;
            end else begin
                fu_outputs.branch[i] = '0;
            end
        end

        // MEM: from mem_fu results
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            fu_outputs.mem[i] = mem_cdb_results[i];
        end

        fu_results.branch = '0;
        fu_results.mem = '0;
    end

    // =========================================================================
    // Completion Candidates (Unified structure for ex_comp generation)
    // =========================================================================
    EX_COMPLETE_ENTRY [`NUM_FU_TOTAL-1:0] candidates;
    logic [`NUM_FU_TOTAL-1:0] candidate_valid;

    always_comb begin
        candidates = '0;
        candidate_valid = '0;

        // BRANCH candidates
        for (int k = 0; k < `NUM_FU_BRANCH; k++) begin
            candidates[BRANCH_START+k] = '{
                rob_idx:       issue_entries.branch[k].rob_idx,
                branch_valid:  1'b1,
                branch_taken:  branch_take[k],
                branch_target: branch_targets[k],
                dest_pr:       issue_entries.branch[k].dest_tag,
                result:        issue_entries.branch[k].PC + 4
            };
            candidate_valid[BRANCH_START+k] = issue_entries.branch[k].valid;
        end

        // ALU candidates
        for (int k = 0; k < `NUM_FU_ALU; k++) begin
            candidates[ALU_START+k] = '{
                rob_idx:       issue_entries.alu[k].rob_idx,
                branch_valid:  1'b0,
                branch_taken:  1'b0,
                branch_target: '0,
                dest_pr:       issue_entries.alu[k].dest_tag,
                result:        fu_results.alu[k]
            };
            candidate_valid[ALU_START+k] = issue_entries.alu[k].valid;
        end

        // MEM candidates
        for (int k = 0; k < `NUM_FU_MEM; k++) begin
            candidates[MEM_START+k] = '{
                rob_idx:       issue_entries.mem[k].rob_idx,
                branch_valid:  1'b0,
                branch_taken:  1'b0,
                branch_target: '0,
                dest_pr:       issue_entries.mem[k].dest_tag,
                result:        mem_cdb_results[k].data
            };
            candidate_valid[MEM_START+k] = mem_cdb_results[k].valid;
        end

        // MULT candidates (use pipelined metadata)
        for (int k = 0; k < `NUM_FU_MULT; k++) begin
            candidates[MULT_START+k] = '{
                rob_idx:       mult_meta_out[k].rob_idx,
                branch_valid:  mult_meta_out[k].branch_valid,
                branch_taken:  mult_meta_out[k].branch_taken,
                branch_target: mult_meta_out[k].branch_target,
                dest_pr:       mult_meta_out[k].dest_pr,
                result:        fu_results.mult[k]
            };
            candidate_valid[MULT_START+k] = mult_done[k];
        end
    end

    // =========================================================================
    // Unified EX/COMP Register Fill
    // =========================================================================
    always_comb begin
        ex_valid = '0;
        ex_comp = '0;

        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_TOTAL; k++) begin
                if (gnt_bus[i][k] && candidate_valid[k]) begin
                    ex_valid[i]              = 1'b1;
                    ex_comp.rob_idx[i]       = candidates[k].rob_idx;
                    ex_comp.branch_valid[i]  = candidates[k].branch_valid;
                    ex_comp.branch_taken[i]  = candidates[k].branch_taken;
                    ex_comp.branch_target[i] = candidates[k].branch_target;
                    ex_comp.dest_pr[i]       = candidates[k].dest_pr;
                    ex_comp.result[i]        = candidates[k].result;
                end
            end
        end
    end

endmodule
