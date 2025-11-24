module LFSR(input clk, rst, output reg [5:0] op);
  always@(posedge clk) begin
    if(rst) op <= 6'h00;
    else op = {op[4:0],(op[5]^op[4])};
  end
endmodule

