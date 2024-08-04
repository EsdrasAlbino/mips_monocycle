module ALU(
  input  [2:0] ALUCtl, 
  input  signed [31:0] SrcA, 
  input  signed [31:0] SrcB, 
  output Zero, 
  output signed [31:0] ALUOut 
);

  assign ALUOut = 
    (ALUCtl == 3'b000) ? (SrcA &  SrcB) :         // AND
    (ALUCtl == 3'b001) ? (SrcA |  SrcB) :         // OR
    (ALUCtl == 3'b010) ? (SrcA +  SrcB) :         // ADD
    (ALUCtl == 3'b100) ? (SrcA & ~SrcB) :         // AND com NOT
    (ALUCtl == 3'b101) ? (SrcA | ~SrcB) :         // OR com NOT
    (ALUCtl == 3'b110) ? (SrcA -  SrcB) :         // SUB
    (ALUCtl == 3'b111) ? ((SrcA < SrcB) ? 32'b1 : 32'b0) : // SLT
    32'b0;                                        // Default case

  assign Zero = (ALUOut == 0) ? 1 : 0;
endmodule
