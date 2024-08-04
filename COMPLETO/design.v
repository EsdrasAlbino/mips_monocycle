//R Instructions 
`define R_TYPE    6'b000000 // Funct codes for R-Type Instructions
`define ADD       6'b100000
`define SUB       6'b100010

//I Instructions 	
`define ADDI    6'b001000 // Add Immediate
`define BEQ     6'b000100 // Branch Instructions
`define LW      6'b100011 // Load Instructions
`define SW      6'b101011 // Store Instructions

// J Instructions 
`define J       6'b000010 //Jump

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

module DataMemory (
  input clock,
  input signed [31:0] address, 
  input signed [31:0] WriteData, 
  input WriteEnable, 
  output reg signed [31:0] ReadData 
);
  reg signed [31:0] memory [0:31]; 
  
  // Bloco de escrita
  always @(posedge clock) begin
  if (WriteEnable) begin
    memory[address] <= WriteData;
    $display("Memory updated at address %0d: %0d", address, WriteData);
  end
  end

  // Bloco de leitura
  always @* begin
  ReadData = memory[address];
  end
endmodule

module InstMem(
  input [31:0] address,        
  output reg [31:0] ReadData   
);
  reg [7:0] memory [0:44]; 

initial begin
  // addi $at, $Zero, 5 
  memory[0] = 8'b001000_00;  
  memory[1] = 8'b000_00001;  
  memory[2] = 8'b00000000;  
  memory[3] = 8'b00000101;  

  // sw $at, 0($Zero)
  memory[4] = 8'b101011_00;  
  memory[5] = 8'b000_00001;  
  memory[6] = 8'b00000000;  
  memory[7] = 8'b00000000;  

  // addi $v0, $Zero, 10 
  memory[8] = 8'b001000_00;  
  memory[9] = 8'b000_00010;  
  memory[10] = 8'b00000000;  
  memory[11] = 8'b00001010;  

  // sw $v0, 4($Zero)
  memory[12] = 8'b101011_00;  
  memory[13] = 8'b000_00010;  
  memory[14] = 8'b00000000;  
  memory[15] = 8'b00000100;  

  // sub $t0, $v0, $at
  memory[16] = 8'b000000_00;  
  memory[17] = 8'b001_00001;  
  memory[18] = 8'b00001000;  
  memory[19] = 8'b00_100010;  

  // beq $t0, $Zero, end
  memory[20] = 8'b000100_00;  
  memory[21] = 8'b000_01000;  
  memory[22] = 8'b00000000;  
  memory[23] = 8'b00000100;  // Salta para o endereço 28

  // j end
  memory[24] = 8'b000010_00;  
  memory[25] = 8'b00000000;  
  memory[26] = 8'b00000000;  
  memory[27] = 8'b00100000;  

  // equal: addi $a0, $Zero, 1 (igual caso)
  memory[28] = 8'b001000_00;  
  memory[29] = 8'b000_00100;  
  memory[30] = 8'b00000000;  
  memory[31] = 8'b00000001;  

  // sw $a0, 8($Zero) (executado apenas se for igual)
  memory[32] = 8'b101011_00;  
  memory[33] = 8'b000_00100;  
  memory[34] = 8'b00000000;  
  memory[35] = 8'b00001000;  
end


  always @(address) begin
  ReadData[ 7: 0] = memory[address+3];
  ReadData[15: 8] = memory[address+2];
  ReadData[23:16] = memory[address+1];
  ReadData[31:24] = memory[address  ];
  end
endmodule

module ControlUnit(
  input [5:0] OPCode,
  input [5:0] Funct,
  output reg MemtoReg,
  output reg MemWrite,
  output reg Branch,
  output reg ALUSrc,
  output reg RegDst,
  output reg RegWrite,
  output reg Jump,
  output [2:0] ALUCtl
);
  reg [1:0] ALUOp;

  // Inicializa sinais de controle com valores padrão
  initial begin
  MemtoReg  = 1'b0;
  MemWrite  = 1'b0;
  Branch    = 1'b0;
  ALUSrc    = 1'b0;
  RegDst    = 1'b0;
  RegWrite  = 1'b0;
  ALUOp     = 2'b00;
  Jump      = 1'b0;
  end

  always @ (OPCode) begin
  case (OPCode)
    `R_TYPE: begin 
    MemtoReg  = 1'b0;    
    MemWrite  = 1'b0;    
    Branch    = 1'b0;    
    ALUSrc    = 1'b0;    
    RegDst    = 1'b1;    
    RegWrite  = 1'b1;    
    Jump      = 1'b0;    
    ALUOp     = 2'b10;   
    end

    `ADDI: begin 
    MemtoReg  = 1'b0;    
    MemWrite  = 1'b0;    
    Branch    = 1'b0;    
    ALUSrc    = 1'b1;    
    RegDst    = 1'b0;    
    RegWrite  = 1'b1;    
    Jump      = 1'b0;    
    ALUOp     = 2'b00;   
    end

    `BEQ: begin 
    MemtoReg  = 1'b0;    
    MemWrite  = 1'b0;    
    Branch    = 1'b1;    
    ALUSrc    = 1'b0;    
    RegDst    = 1'b0;    
    RegWrite  = 1'b0;    
    Jump      = 1'b0;    
    ALUOp     = 2'b01;   
    end

    `LW: begin 
    MemtoReg  = 1'b1;    
    MemWrite  = 1'b0;    
    Branch    = 1'b0;    
    ALUSrc    = 1'b1;    
    RegDst    = 1'b0;    
    RegWrite  = 1'b1;    
    Jump      = 1'b0;    
    ALUOp     = 2'b00;   
    end

    `SW: begin
    MemtoReg  = 1'b0;    
    MemWrite  = 1'b1;    
    Branch    = 1'b0;    
    ALUSrc    = 1'b1;    
    RegDst    = 1'b0;    
    RegWrite  = 1'b0;    
    Jump      = 1'b0;    
    ALUOp     = 2'b00;   
    end

    `J: begin 
    MemtoReg  = 1'b0;    
    MemWrite  = 1'b0;    
    Branch    = 1'b0;    
    ALUSrc    = 1'b0;    
    RegDst    = 1'b0;    
    RegWrite  = 1'b0;    
    Jump      = 1'b1;    
    ALUOp     = 2'b01;   
    end

  endcase
  end

  assign ALUCtl = (ALUOp == 2'b00) ? 3'b010 :  // ADD 
                  (ALUOp == 2'b01) ? 3'b110 :  // SUB 
                  (ALUOp == 2'b10) ? (         // Funct
                      (Funct == 6'b100000) ? 3'b010 : // ADD
                      (Funct == 6'b100010) ? 3'b110 : // SUB
                      (Funct == 6'b100100) ? 3'b000 : // AND
                      (Funct == 6'b100101) ? 3'b001 : // OR 
                      (Funct == 6'b101010) ? 3'b111 : // SLT 
                  3'b000 
                  ) : 3'b000; 

endmodule

module RegFile(
  input [4:0] ReadReg1,
  input [4:0] ReadReg2, 
  input [4:0] WriteReg, 
  input signed [31:0] WriteData, 
  input RegWrite, 
  input clock, 
  output reg signed [31:0] ReadData1, 
  output reg signed [31:0] ReadData2 
);

  reg signed [31:0] registers [0:31]; 

  integer i;
  initial begin
  for (i = 0; i < 32; i = i + 1) 
    registers[i] = 32'd0;
  end

  always @* begin
  if (ReadReg1 == 5'd0) 
    ReadData1 = 32'd0;
  else
    ReadData1 = registers[ReadReg1];
  
  if (ReadReg2 == 5'd0) 
    ReadData2 = 32'd0;
  else
    ReadData2 = registers[ReadReg2];
  end

  always @(posedge clock) begin
  if (RegWrite) begin
    if (WriteReg != 5'd0) 
    registers[WriteReg] <= RegWrite;
  end
  end
endmodule

module Mux1_REG(
  input  [4:0] inst20_16,
  input  [4:0] inst15_11,
  input  RegDst,

  output [4:0] WriteReg
);

 assign WriteReg = RegDst ? inst15_11 : inst20_16;
endmodule;

module Mux2_REG(
  input MemtoReg,
  input [31:0] ALUOut,
  input [31:0] ReadData,
  output [31:0] WriteData
);

 assign WriteData = MemtoReg ? ReadData : ALUOut;
endmodule;

module Mux3_ALU(
  input  ALUSrc,
  input  [31:0] ReadData2,
  input  [31:0] Extend32,
  output [31:0] SrcB
);

  assign SrcB = (ALUSrc) ? Extend32 : ReadData2;
endmodule;

module Mux4_Scr(
  input  PCSrc,
  input  [31:0] PCPlus4,
  input  [31:0] PCBranch,
  output [31:0] PCin
);

  assign PCin = PCSrc ? PCBranch : PCPlus4;
endmodule;

module Mux5_PC (
  input  [31:0] pcPlus4, 
  input  [31:0] Branch, 
  input         PCSrc, 
  output [31:0] PCin 
);

  assign PCin = (PCSrc) ? Branch : pcPlus4;
endmodule

module Mux6_Jump (
  input [31:0] PCJumpIn, 
  input Jump, 
  input [25:0] Destination,  
  output [31:0] PCJumpOut
);

  wire [31:0] DestinationShifted = {Destination, 2'b00}; 
  assign PCJumpOut = (Jump) ? {PCJumpIn[31:28], DestinationShifted[27:0]} : PCJumpIn;

endmodule

module PC(
  input  clock, 
  input      [31:0] PCin, 
  output reg [31:0] PCout 
);

  initial PCout = 0;
  always @(posedge clock) begin
  PCout <= PCin;
  end

endmodule

module SignExtend(
  input  signed [15:0] inst15_0,
  output signed [31:0] Extend32
);
  
  assign Extend32 = {{16{inst15_0[15]}}, inst15_0};
endmodule


module PCBranch(
  input  [31:0] Extend32,
  input  [31:0] PCplus4,
  output [31:0] out
);
  
  assign out = PCplus4 + (Extend32 << 2); 
endmodule;

module PCplus_4(
  input  [31:0] PCplusIn, 
  output [31:0] PCplusOut
);

  assign PCplusOut = PCplusIn + 4;
endmodule

