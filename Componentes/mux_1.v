module Mux1_REG(
  input  [4:0] inst20_16,
  input  [4:0] inst15_11,
  input  RegDst,

  output [4:0] WriteReg
);

 assign WriteReg = RegDst ? inst15_11 : inst20_16;
endmodule;

