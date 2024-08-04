module MIPS;
  // Clock signal
  reg clock = 0;

  // Instruction and control signals
  wire [5:0] W_InstrOpCode; 
  wire [4:0] W_Instr25_21; 
  wire [25:0] W_InstrJump; 
  wire [4:0] W_Instr20_16; 
  wire [4:0] W_InstrRd; 
  wire signed [15:0] W_Instr15_0; 
  wire [5:0] W_InstrFunct;

  // ALU
  wire W_Zero;
  wire signed [31:0] W_ALUOut;

  // Data Memory
  wire [31:0] W_ReadData;

  // Register File
  wire signed [31:0] W_Mux2_REG_out; 
  wire [31:0] W_ReadData1; 
  wire [31:0] W_ReadData2;
  
  // Control signals
  wire W_RegWrite;
  wire W_MemWrite;
  wire W_MemtoReg;
  wire W_Branch; 
  wire W_ALUSrc; 
  wire W_Jump;
  wire [2:0] W_ALUCtl;
  
  // Muxes
  wire [31:0] W_SrcB; 
  wire [31:0] W_PCSrcMuxOut; 
  wire [31:0] W_PCBranch; 
  wire [31:0] W_PCPlus4; 
  wire [31:0] W_PCJumpOut;
  wire [31:0] W_PC;
  wire [31:0] W_SignExtOut;
  wire [4:0] W_WriteReg; 
  wire W_RegDst;
  wire W_PCSrc;

  // Assignments
  assign W_InstrOpCode     = instr[31:26];
  assign W_Instr25_21         = instr[25:21];
  assign W_InstrJump       = instr[25:0];
  assign W_Instr20_16         = instr[20:16];
  assign W_InstrRd         = instr[15:11];
  assign W_Instr15_0  = instr[15:0];
  assign W_InstrFunct      = instr[5:0];

  assign W_PCSrc = W_Branch && W_Zero;

  // ALU
  ALU ALU_t (
    .ALUCtl(W_ALUCtl), 
    .SrcA(W_ReadData1), 
    .SrcB(W_SrcB), 
    .Zero(W_Zero), 
    .ALUOut(W_ALUOut)
  );
  
  // Data Memory
  DataMemory DataMemory_t (
    .clock(clock),
    .address(W_ALUOut), // Address of the word
    .WriteData(W_ReadData2),
    .WriteEnable(W_MemWrite),
    .ReadData(W_ReadData)
  );
  
  // Instruction Memory
  reg [31:0] instr;
  InstMem InstMem_t(
    .address(W_PC), 
    .ReadData(instr)
  );
  
  // Control Unit
  ControlUnit ControlUnit_t(
    .OPCode(W_InstrOpCode),
    .Funct(W_InstrFunct),
    .MemtoReg(W_MemtoReg), 
    .MemWrite(W_MemWrite), 
    .Branch(W_Branch),
    .ALUSrc(W_ALUSrc), 
    .RegDst(W_RegDst), 
    .RegWrite(W_RegWrite), 
    .Jump(W_Jump),
    .ALUCtl(W_ALUCtl)
  );

  // Register File
  RegFile RegFile_t(
    .ReadReg1(W_Instr25_21), 
    .ReadReg2(W_Instr20_16), 
    .WriteReg(W_WriteReg), 
    .WriteData(W_Mux2_REG_out),
    .RegWrite(W_RegWrite),
    .clock(clock),
    .ReadData1(W_ReadData1),
    .ReadData2(W_ReadData2)
  );

  // REG DST MUX
  Mux1_REG Mux1_REG_t(
    .inst20_16(W_Instr20_16), 
    .inst15_11(W_InstrRd), 
    .RegDst(W_RegDst), 
    .WriteReg(W_WriteReg)
  );

  // MEM TO REG MUX
  Mux2_REG Mux2_REG_t (
    .MemtoReg(W_MemtoReg), 
    .ALUOut(W_ALUOut), 
    .ReadData(W_ReadData), 
    .WriteData(W_Mux2_REG_out)
  );

  // ALU SRC MUX
  Mux3_ALU Mux3_ALU_t (
    .ALUSrc(W_ALUSrc), 
    .ReadData2(W_ReadData2), 
    .Extend32(W_SignExtOut), 
    .SrcB(W_SrcB)
  );

  // PC MUX
  Mux5_PC Mux5_PC_t(
    .pcPlus4(W_PCPlus4), 
    .Branch(W_PCBranch), 
    .PCSrc(W_PCSrc), 
    .PCin(W_PCSrcMuxOut)
  );

  // PC JUMP MUX
  Mux6_Jump MuxJump_t(
    .PCJumpIn(W_PCSrcMuxOut),
    .Jump(W_Jump),
    .Destination(W_InstrJump),
    .PCJumpOut(W_PCJumpOut)
  );

  // PC
  PC PC_t (
    .clock(clock), 
    .PCin(W_PCJumpOut), 
    .PCout(W_PC)
  );
  
  // SIGN EXTEND
  SignExtend SignExtend_t (
    .inst15_0(W_Instr15_0),
    .Extend32(W_SignExtOut)
  );

  // PC Branch
  PCBranch PCBranch_t (
    .Extend32(W_SignExtOut), 
    .PCplus4(W_PCPlus4), 
    .out(W_PCBranch)
  );

  // PC PLUS 4
  PCplus_4 PC_plus_4_t (
    .PCplusIn(W_PC), 
    .PCplusOut(W_PCPlus4)
  );

  // Clock generation
  initial forever #5 clock = ~clock;
  
  initial begin
    $monitor("Time: %t | PC: %h | Instruction: %h", $time, W_PC, instr);
    #300 $finish;
  end

endmodule