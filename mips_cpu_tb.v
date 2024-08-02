module testbench;
  reg clock;
  reg reset;
  
  wire [31:0] PCin, PCout, inst, ReadData1, ReadData2, Extend32, ALU_B, ShiftOut, ALUOut, ReadData, WriteData_Reg;
  wire RegDst, RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, Branch, Zero, pcPlus4,PCSrc;
  wire [1:0] ALUOp;
  wire [3:0] ALUCtl;
  wire [4:0] WriteReg;

  // Instantiate the MipsCPU
  MipsCPU uut (
    .clock(clock),
    .reset(reset),
    .PCin(PCin),
    .PCout(PCout),
    .inst(inst),
    .RegDst(RegDst),
    .RegWrite(RegWrite),
    .ALUSrc(ALUSrc),
    .MemtoReg(MemtoReg),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .Branch(Branch),
    .ALUControl(ALUOp),
    .WriteReg(WriteReg),
    .ReadData1(ReadData1),
    .ReadData2(ReadData2),
    .Extend32(Extend32),
    .ALU_B(ALU_B),
    .ShiftOut(ShiftOut),
    .ALUCtl(ALUCtl),
    .Zero(Zero),
    .ALUOut(ALUOut),
    .pcPlus4(pcPlus4),
    .PCSrc(PCSrc),
    .ReadData(ReadData),
    .WriteData_Reg(WriteData_Reg)
  );

  // Clock generation
  initial begin
    clock = 0;
    forever #5 clock = ~clock;  // 10 time units clock period
  end

  // Test sequence
  initial begin
    // Initialize reset
    reset = 1;
    #10;
    reset = 0;

    // Wait for some time and observe the output
    #80;

    // Finish simulation
    $finish;
  end

  // Monitor the changes
  initial begin
    $monitor("Time: %0t | PC: %h | Instr: %h | RegDst: %b | ALUOp: %b | RegWrite: %b | ALUSrc: %b | MemtoReg: %b | MemRead: %b | MemWrite: %b | Branch: %b | ALUOut: %h",
             $time, PCout, inst, RegDst, ALUOp, RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, Branch, ALUOut);
  end

endmodule