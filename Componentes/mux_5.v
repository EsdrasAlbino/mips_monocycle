module Mux5_PC (
  input  [31:0] pcPlus4, 
  input  [31:0] Branch, 
  input         PCSrc, 
  output [31:0] PCoutMux 
);

  assign PCoutMux = (PCSrc) ? Branch : pcPlus4;
endmodule