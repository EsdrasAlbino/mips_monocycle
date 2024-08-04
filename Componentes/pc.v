module PC(
  input  clock, 
  input      [31:0] PCin, 
  output reg [31:0] PCout 
);

  initial PCout = 0;
  always @(posedge clock) begin
  PCout <= PCin;
  end

endmodule
