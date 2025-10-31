`include "sys_defs.svh"

module freelist #(
    parameter int ALLOC_WIDTH = `N,                // Number of allocation requests per cycle
    parameter int PR_COUNT    = `PHYS_REG_SZ_R10K
) (
    input clock,  // system clock
    input reset,  // system reset

    // From dispatch: allocation requests
    input logic [ALLOC_WIDTH-1:0] alloc_req,  // Request allocation for each dispatch lane

    // From retire: deallocation requests
    input logic [PR_COUNT-1:0] free_mask,  // Bit mask of registers being freed

    // Outputs to dispatch
    output logic [ALLOC_WIDTH-1:0][PR_COUNT-1:0] granted_regs  // Granted physical registers for each allocation request
);

    // Grant matrix from allocator: grants[requester][resource]
    logic [ALLOC_WIDTH-1:0][PR_COUNT-1:0] grants;

    // Create initial availability mask: first ARCH_REG_SZ registers are occupied (0),
    // remaining registers are available (1), resolved at compile time
    localparam logic [PR_COUNT-1:0] INITIAL_AVAIL_MASK = {{PR_COUNT - `ARCH_REG_SZ{1'b1}}, {`ARCH_REG_SZ{1'b0}}};

    // Allocator instance for physical register allocation
    allocator #(
        .NUM_RESOURCES(PR_COUNT),
        .NUM_REQUESTS(ALLOC_WIDTH),
        .INITIAL_AVAIL_MASK(INITIAL_AVAIL_MASK)
    ) reg_allocator (
        .reset(reset),
        .clock(clock),
        .req  (alloc_req),  // Allocation requests
        .clear(free_mask),  // Registers being freed
        .grant(grants)      // Grant matrix
    );

    // Output grant matrix directly
    assign granted_regs = grants;

endmodule
