module SignExtend(
  input  signed [15:0] inst15_0,
  output signed [31:0] Extend32
);
  
  assign Extend32 = {{16{inst15_0[15]}}, inst15_0};
endmodule