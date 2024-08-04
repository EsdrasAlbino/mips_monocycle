module Mux2_REG(
  input MemtoReg,
  input [31:0] ALUOut,
  input [31:0] ReadData,
  output [31:0] WriteData
);

 assign WriteData = MemtoReg ? ReadData : ALUOut;
endmodule;