module Mux4_Scr(
  input  PCSrc,
  input  [31:0] PCPlus4,
  input  [31:0] PCBranch,
  output [31:0] PCin
);

  assign PCin = PCSrc ? PCBranch : PCPlus4;
endmodule;