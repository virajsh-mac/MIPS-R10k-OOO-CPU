`include "sys_defs.svh"

module freelist #(
    parameter int ALLOC_WIDTH = `N,                // Number of allocation requests per cycle
    parameter int PR_COUNT    = `PHYS_REG_SZ_R10K
) (
    input clock,  // system clock
    input reset,  // system reset
    input logic mispredict,  // From retire for restore
    input logic [PR_COUNT-1:0] restore_mask,  // Mask to restore freelist on mispredict

    // From dispatch: allocation requests
    input logic [ALLOC_WIDTH-1:0] alloc_req,  // Request allocation for each dispatch lane

    // From retire: deallocation requests
    input logic [PR_COUNT-1:0] free_mask,  // Bit mask of registers being freed

    // Outputs to dispatch
    output logic [ALLOC_WIDTH-1:0][PR_COUNT-1:0] granted_regs,  // Granted physical registers for each allocation request
    output logic [$clog2(PR_COUNT+1)-1:0]        free_slots,   // Number of free physical registers
    output logic [PR_COUNT-1:0]                  available_regs // Available physical registers (for debug)
);

    // Grant matrix from allocator: grants[requester][resource]
    logic [ALLOC_WIDTH-1:0][PR_COUNT-1:0] grants;

    // Free slots counter
    logic [$clog2(PR_COUNT+1)-1:0] free_count, free_count_next;

    // Initial availability: arch regs busy (0), others free (1)
    localparam logic [PR_COUNT-1:0] INITIAL_AVAIL_MASK = {{PR_COUNT - `ARCH_REG_SZ{1'b1}}, {`ARCH_REG_SZ{1'b0}}};

    // Allocator instance for physical register allocation
    allocator #(
        .NUM_RESOURCES(PR_COUNT),
        .NUM_REQUESTS(ALLOC_WIDTH)
    ) reg_allocator (
        .reset(reset),
        .clock(clock),
        .req  (alloc_req),  // Allocation requests
        .clear(free_mask),  // Registers being freed
        .mispredict(mispredict),
        .initial_mask(INITIAL_AVAIL_MASK),
        .restore_mask(restore_mask),
        .grant(grants),     // Grant matrix
        .resource_status(available_regs)  // Available registers status
    );

    // Calculate next free count: +deallocations -allocations, reset on mispredict
    always_comb begin
        if (mispredict) begin
            free_count_next = unsigned'(PR_COUNT - `ARCH_REG_SZ);  // Free slots = total - arch regs (busy)
        end else begin
            free_count_next = free_count + unsigned'($countones(free_mask)) - unsigned'($countones(alloc_req));
        end
    end

    // Output grant matrix and free slots
    assign granted_regs = grants;
    assign free_slots = free_count;

    // Update free count register
    always_ff @(posedge clock) begin
        if (reset) begin
            free_count <= unsigned'(PR_COUNT - `ARCH_REG_SZ);  // Arch regs busy
        end else begin
            free_count <= free_count_next;
        end
    end

endmodule
