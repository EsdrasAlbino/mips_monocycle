
module Mux3_ALU(
  input  ALUSrc,
  input  [31:0] ReadData2,
  input  [31:0] Extend32,
  output [31:0] SrcB
);

  assign SrcB = (ALUSrc) ? Extend32 : ReadData2;
endmodule;
