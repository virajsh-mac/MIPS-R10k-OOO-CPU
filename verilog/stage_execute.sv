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

    // Load counter for dcache address selection
    int load_count;

    // Metadata for mult module (passed through pipeline)
    EX_COMPLETE_ENTRY mult_meta_in[`NUM_FU_MULT-1:0];
    EX_COMPLETE_ENTRY mult_meta_out[`NUM_FU_MULT-1:0];

    // Pulse generation for mult start (mult expects start pulse, not held high)
    logic [`NUM_FU_MULT-1:0] prev_mult_valid;

    // BRANCH signals
    DATA [`NUM_FU_BRANCH-1:0] branch_rs1, branch_rs2;
    BRANCH_FUNC [`NUM_FU_BRANCH-1:0] branch_funcs;  // Array of 3-bit func values
    ADDR [`NUM_FU_BRANCH-1:0] branch_pcs;           // PC values for branch FUs
    DATA [`NUM_FU_BRANCH-1:0] branch_offsets;       // Branch offsets for branch FUs
    logic [`NUM_FU_BRANCH-1:0] branch_take;
    ADDR [`NUM_FU_BRANCH-1:0] branch_targets;

    // MEM signals (placeholder for future implementation)
    DATA [`NUM_FU_MEM-1:0] mem_rs1, mem_rs2, mem_src2_imm;
    MEM_FUNC [`NUM_FU_MEM-1:0] mem_funcs;
    DATA [`NUM_FU_MEM-1:0] mem_data;
    ADDR [`NUM_FU_MEM-1:0] mem_addr;


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
    // Mult start pulse generation
    // =========================================================================

    always_ff @(posedge clock) begin
        if (reset | mispredict) begin
            prev_mult_valid <= '0;
        end else begin
            for (int i = 0; i < `NUM_FU_MULT; i++) begin
                prev_mult_valid[i] <= issue_entries.mult[i].valid;
            end
        end
    end

    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            mult_start[i] = issue_entries.mult[i].valid & ~prev_mult_valid[i];
        end
    end

    // =========================================================================
    // Mult metadata input setup
    // =========================================================================

    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            if (issue_entries.mult[i].valid) begin
                mult_meta_in[i].rob_idx = issue_entries.mult[i].rob_idx;
                mult_meta_in[i].branch_valid = 0;
                mult_meta_in[i].mispredict = 0;
                mult_meta_in[i].branch_taken = 0;
                mult_meta_in[i].branch_target = 0;
                mult_meta_in[i].dest_pr = issue_entries.mult[i].dest_tag;
                mult_meta_in[i].result = 0;  // Will be set in mult module
            end else begin
                mult_meta_in[i] = '0;
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
            fu_outputs.alu[i].valid = issue_entries.alu[i].valid && (issue_entries.alu[i].dest_tag != '0);
            fu_outputs.alu[i].tag   = issue_entries.alu[i].dest_tag;
            fu_outputs.alu[i].data  = fu_results.alu[i];
        end
    end

    // =========================================================================
    // MULT Functional Units (Pipelined)
    // =========================================================================

    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            mult_rs1[i]  = resolved_src1.mult[i];
            mult_rs2[i]  = resolved_src2.mult[i];
            mult_func[i] = issue_entries.mult[i].op_type.func;
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
        .result(fu_results.mult),
        .request(mult_request),
        .done(mult_done),
        .meta_out(mult_meta_out)
    );

    // MULT outputs to CDB (only when done)
    always_comb begin
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            fu_outputs.mult[i].valid = mult_done[i] && (issue_entries.mult[i].dest_tag != '0);
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
            branch_rs1[i]     = resolved_src1.branch[i];
            branch_rs2[i]     = resolved_src2.branch[i];
            branch_funcs[i]   = issue_entries.branch[i].op_type.func;
            branch_pcs[i]     = issue_entries.branch[i].PC;
            branch_offsets[i] = issue_entries.branch[i].src2_immediate;
        end
    end

    // Instantiate BRANCH modules
    branch branch_inst[`NUM_FU_BRANCH-1:0] (
        .rs1 (branch_rs1),
        .rs2 (branch_rs2),
        .func(branch_funcs),  // Connect func array directly
        .pc  (branch_pcs),    // Current PC for each branch
        .offset(branch_offsets), // Branch offset for each branch
        .take(branch_take),
        .target(branch_targets) // Target address computed by branch FU
    );

    // Branch targets are now computed inside the branch functional units

    // BRANCH outputs to CDB
    // Only JAL and JALR produce data results; conditional branches don't
    always_comb begin
        if (issue_entries.branch[0].valid &&
            (issue_entries.branch[0].op_type.func == JAL ||
             issue_entries.branch[0].op_type.func == JALR) &&
            (issue_entries.branch[0].dest_tag != '0)) begin
            fu_outputs.branch[0].valid = 1'b1;
            fu_outputs.branch[0].tag   = issue_entries.branch[0].dest_tag;
            fu_outputs.branch[0].data  = issue_entries.branch[0].PC + 4;  // Return address for JAL/JALR
        end else begin
            fu_outputs.branch[0].valid = 1'b0;
            fu_outputs.branch[0].tag   = '0;
            fu_outputs.branch[0].data  = '0;
        end
    end

    // =========================================================================
    // MEM Functional Units
    // =========================================================================
    // Store operations handled by MEM FUs with store queue interaction

    always_comb begin
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            mem_rs1[i] = resolved_src1.mem[i];
            mem_rs2[i] = resolved_src2.mem[i];
            mem_src2_imm[i] = issue_entries.mem[i].src2_immediate;
            mem_funcs[i] = issue_entries.mem[i].op_type.func;
        end
    end

    // Outputs from each MEM FU
    EXECUTE_STOREQ_ENTRY [`NUM_FU_MEM-1:0] mem_store_queue_entries;
    CDB_ENTRY [`NUM_FU_MEM-1:0] mem_cdb_results;
    logic [`NUM_FU_MEM-1:0] mem_cdb_requests;  // CDB requests from MEM FUs
    logic [`NUM_FU_MEM-1:0] mem_is_load_request;
    logic [`NUM_FU_MEM-1:0] mem_is_store;
    D_ADDR [`NUM_FU_MEM-1:0] mem_dcache_addrs;
    
    // Store queue forwarding lookup signals from MEM FUs
    logic      [`NUM_FU_MEM-1:0] mem_lookup_valid;
    ADDR       [`NUM_FU_MEM-1:0] mem_lookup_addr;
    STOREQ_IDX [`NUM_FU_MEM-1:0] mem_lookup_sq_tail;
    
    // Maps MEM FU index to dcache slot (0 or 1) for load data routing
    logic [1:0] mem_fu_to_dcache_slot [`NUM_FU_MEM-1:0];

    // Generate MEM FUs
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
                
                // Store queue forwarding inputs
                .forward_valid(sq_forward_valid[i]),
                .forward_data(sq_forward_data[i]),
                .forward_stall(sq_forward_stall[i]),
                
                // Outputs
                .addr(mem_addr[i]),
                .data(mem_data[i]),
                .store_queue_entry(mem_store_queue_entries[i]),
                .cdb_result(mem_cdb_results[i]),
                .cdb_request(mem_cdb_requests[i]),
                .is_load_request(mem_is_load_request[i]),
                .is_store_op(mem_is_store[i]),
                .dcache_addr(mem_dcache_addrs[i]),
                
                // Store queue forwarding lookup outputs
                .lookup_valid(mem_lookup_valid[i]),
                .lookup_addr(mem_lookup_addr[i]),
                .lookup_sq_tail(mem_lookup_sq_tail[i])
            );
        end
    endgenerate
    
    // Route forwarding lookup signals to store queue
    assign sq_lookup_valid   = mem_lookup_valid;
    assign sq_lookup_addr    = mem_lookup_addr;
    assign sq_lookup_sq_tail = mem_lookup_sq_tail;

    // Output MEM FU store queue entries directly
    assign mem_storeq_entries = mem_store_queue_entries;

    // Late CDB requests from MEM FUs
    assign mem_cdb_requests_out = mem_cdb_requests;

    // Send load addresses to dcache (first 2 MEM FUs that request loads)
    always_comb begin
        dcache_read_addrs = '0;
        mem_fu_to_dcache_slot = '{default: '0};

        // Collect load requests from MEM FUs and send first 2 to dcache
        load_count = 0;
        for (int i = 0; i < `NUM_FU_MEM && load_count < 2; i++) begin
            if (mem_is_load_request[i]) begin
                dcache_read_addrs[load_count].valid = 1'b1;
                dcache_read_addrs[load_count].addr = mem_dcache_addrs[i];

                mem_fu_to_dcache_slot[i] = load_count;  // Record which dcache slot this FU uses
                load_count++;
            end
        end
    end

    // Set CDB outputs from MEM FU results
    always_comb begin
        for (int i = 0; i < `NUM_FU_MEM; i++) begin
            fu_results.mem[i] = '0;  // Initialize MEM results to 0
            // Use CDB results from MEM FUs (for cache hit loads)
            fu_outputs.mem[i] = mem_cdb_results[i];
        end
    end

    // Add constants for FU index ranges (to avoid manual offset calculations)
    // localparam int MULT_START = 0;
    // localparam int MEM_START = MULT_START + `NUM_FU_MULT;
    // localparam int ALU_START = MEM_START + `NUM_FU_MEM;
    // localparam int BRANCH_START = ALU_START + `NUM_FU_ALU;
    localparam int BRANCH_START = 0;
    localparam int ALU_START = BRANCH_START + `NUM_FU_BRANCH;
    localparam int MEM_START = ALU_START + `NUM_FU_ALU;
    localparam int MULT_START = MEM_START + `NUM_FU_MEM;


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
                if (gnt_mult[i][k] && mult_done[k]) begin
                    ex_valid[i] = 1'b1;
                    ex_comp.rob_idx[i] = mult_meta_out[k].rob_idx;
                    ex_comp.branch_valid[i] = mult_meta_out[k].branch_valid;
                    ex_comp.mispredict[i] = mult_meta_out[k].mispredict;
                    ex_comp.branch_taken[i] = mult_meta_out[k].branch_taken;
                    ex_comp.branch_target[i] = mult_meta_out[k].branch_target;
                    ex_comp.dest_pr[i] = mult_meta_out[k].dest_pr;
                    ex_comp.result[i] = mult_meta_out[k].result;
                end
            end
        end

        // MEM grants (loads/stores that complete)
        for (int i = 0; i < `N; i++) begin
            for (int k = 0; k < `NUM_FU_MEM; k++) begin
                if (gnt_mem[i][k]) begin
                    ex_comp.rob_idx[i] = issue_entries.mem[k].rob_idx;
                    ex_comp.dest_pr[i] = issue_entries.mem[k].dest_tag;

                    // MEM operations complete when they get CDB grants (they only request when ready)
                    ex_valid[i] = 1'b1;
                    ex_comp.result[i] = mem_cdb_results[k].valid ? mem_cdb_results[k].data : '0;
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
                    ex_comp.branch_target[i] = branch_targets[k];
                    ex_comp.mispredict[i] = (branch_take[k] != issue_entries.branch[k].pred_taken);
                    ex_comp.dest_pr[i] = issue_entries.branch[k].dest_tag;
                    ex_comp.result[i] = issue_entries.branch[k].PC + 4;
                end
            end
        end
    end

endmodule