// Generic handshake interface for valid-ready protocol between modules

interface handshake_if #(
    parameter int DATA_WIDTH = 32  // Width of the data bus; customize as needed
) (
    input logic clk,               // Clock (optional)
    input logic rst_n              // Active-low reset (optional)
);

    logic valid;                   // Producer: data is valid
    logic ready;                   // Consumer: ready to accept data
    logic [DATA_WIDTH-1:0] data;   // Data payload

    // Modport for producer (sender) side
    modport producer (
        output valid, data,
        input  ready
    );

    // Modport for consumer (receiver) side
    modport consumer (
        input  valid, data,
        output ready
    );

    // Optional: Add clocking block for timing/simulation if needed
    // clocking cb @(posedge clk);
    //     default input #1step output #0;
    // endclocking

endinterface: handshake_if