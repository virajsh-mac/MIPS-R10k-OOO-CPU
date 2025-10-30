/*
    Generic Resource Allocator Module

    A parametrized resource allocation system that uses priority selectors to assign
    resources to incoming requests. Suitable for use in out-of-order CPU pipelines
    for managing resources such as:
    - Physical registers (register renaming)
    - Reservation station/issue queue entries
    - Reorder buffer entries
    - Load/store queue entries

    This implementation uses priority selectors instead of dependent for-loops for
    better synthesis and timing characteristics.

    Parameters:
    - NUM_RESOURCES: Number of allocatable resources (e.g., physical registers, queue slots)
    - NUM_REQUESTS: Number of simultaneous requesters (e.g., issue ports, decode width)

    Interface:
    - req: Incoming requests from NUM_REQUESTS requesters
    - clear: Bit vector indicating which resources are being released/freed
    - grant: 2D output showing which resource is granted to each requester

    Author: Genericized for out-of-order CPU pipeline use
*/

module allocator #(
    parameter NUM_RESOURCES = 64,
    parameter NUM_REQUESTS  = 3
) (
    input                           reset,
    input                           clock,
    input logic [ NUM_REQUESTS-1:0] req,
    input logic [NUM_RESOURCES-1:0] clear,

    output logic [NUM_REQUESTS-1:0][NUM_RESOURCES-1:0] grant
);

    // Represents the current status of all resources.
    // A '1' in a bit position indicates that the corresponding resource is free/available,
    // while a '0' indicates that the resource is allocated/occupied.
    logic [NUM_RESOURCES-1:0] resource_status;

    // Bit vector used to mark resources that will be allocated in the next cycle.
    // For example, if resources 1 and 2 are to be allocated in the next cycle, then
    // resource_allocated[1] and resource_allocated[2] should be 1 and all other bits should be 0.
    logic [NUM_RESOURCES-1:0] resource_allocated;

    // 2D bus output from the priority selector (gnt_bus) indicating
    // which resources are granted to incoming requests for each priority level.
    // This prioritizes available resources across NUM_REQUESTS requesters.
    logic [NUM_REQUESTS-1:0][NUM_RESOURCES-1:0] resource_gnt_bus;

    // 2D bus output from the priority selector (gnt_bus) indicating
    // which requesters are prioritized when multiple requests arrive simultaneously.
    logic [NUM_REQUESTS-1:0][NUM_REQUESTS-1:0] request_gnt_bus;


    // Priority selector for resource allocation: grants available resources
    // to incoming requests based on priority (alternating MSB/LSB priority scheme)
    psel_gen #(
        .WIDTH(NUM_RESOURCES),
        .REQS (NUM_REQUESTS)
    ) resource_psel (
        .req(resource_status),
        .gnt_bus(resource_gnt_bus)
    );


    // Priority selector for request prioritization: determines which requests
    // are processed when multiple requests arrive simultaneously
    psel_gen #(
        .WIDTH(NUM_REQUESTS),
        .REQS (NUM_REQUESTS)
    ) request_psel (
        .req(req),
        .gnt_bus(request_gnt_bus)
    );


    // Combinational logic for grant: assigns resources to requesters
    // by matching prioritized requesters with prioritized resources
    always_comb begin
        grant = '0;

        for (int i = 0; i < NUM_REQUESTS; i++) begin
            for (int j = 0; j < NUM_REQUESTS; j++) begin
                if (request_gnt_bus[i][j]) begin
                    // Assign the next highest priority resource to the current requester
                    // skip any requester that has no priority (i.e. no incoming request)
                    grant[j] = resource_gnt_bus[i];
                end
            end
        end
    end

    // Combinational logic for resource_allocated: accumulates all grants
    // to determine which resources will be allocated next cycle
    always_comb begin
        resource_allocated = '0;

        for (int i = 0; i < NUM_REQUESTS; i++) begin
            resource_allocated |= grant[i];
        end
    end


    // Sequential logic: update resource status based on allocations and releases
    always_ff @(posedge clock) begin
        if (reset) begin
            // On reset, mark all resources as free/available (1)
            resource_status <= '1;
        end else begin
            // Update resource status:
            // - OR with clear: resources being cleared become available (1)
            // - XOR with resource_allocated: resources being allocated become unavailable (0)
            // Note: XOR flips the bit when allocating; the OR with clear handles clearing
            //       This preserves the original parking lot logic behavior
            resource_status <= (resource_status | clear) ^ resource_allocated;
        end
    end

endmodule
