module PCBranch(
  input  [31:0] Extend32,
  input  [31:0] PCplus4,
  output [31:0] out
);
  
  assign out = PCplus4 + (Extend32 << 2); 
endmodule;