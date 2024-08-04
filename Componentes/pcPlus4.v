module PCplus_4(
  input  [31:0] PCplusIn, 
  output [31:0] PCplusOut
);

  assign PCplusOut = PCplusIn + 4;
endmodule