// Definições de códigos de operação
`define R_TYPE 6'b000000
`define SW 6'b101011
`define LW 6'b100011
`define ADDI 6'b001000
`define BEQ 6'b000100
`define J 6'b000010

// CONTROL UNIT
module controlUnit(
    input [5:0] op,
    input [5:0] funct,
    output reg menToReg, 
    output reg menWrite, 
    output reg branch, 
    output reg aluSrc, 
    output reg regDst, 
    output reg regWrite, 
    output reg jump,
    output [2:0] aluControl
);

    reg [1:0] aluOp; 

    initial begin
        menToReg = 0;
        menWrite = 0;
        branch = 0;
        aluSrc = 0;
        regDst = 0;
        regWrite = 0;
        aluOp = 2'b00;
        jump = 0;
    end

    always @ (op) begin
        case (op)
            // ADDI
            `ADDI: begin
                menToReg = 0;
                menWrite = 0;
                branch = 0;
                aluOp = 2'b00; 
                aluSrc = 1;
                regDst = 0;      
                regWrite = 1;
                jump = 0;
            end

            // BEQ
            `BEQ: begin
                menToReg = 0;
                menWrite = 0;
                branch = 1;
                aluOp = 2'b01; 
                aluSrc = 0;
                regDst = 0;      
                regWrite = 0;
                jump = 0;
            end

            // J 
            `J: begin
                menToReg = 0;
                menWrite = 0;
                branch = 1;
                aluOp = 2'b00; 
                aluSrc = 0;
                regDst = 0;
                regWrite = 0;
                jump = 1;
            end

            // LW (Load WoReadData)
            `LW: begin
                menToReg = 1;
                menWrite = 0;
                branch = 0;
                aluSrc = 1;
                aluOp = 2'b00;
                regDst = 0;
                regWrite = 1;
                jump = 0;
            end

            // R-type
            `R_TYPE: begin
                menToReg = 0;
                menWrite = 0;
                branch = 0;
                aluOp = 2'b10; 
                aluSrc = 0;
                regDst = 1;      
                regWrite = 1;
                jump = 0;
            end

            // SW (Store WoReadData)
            `SW: begin
                menToReg = 0;
                menWrite = 1;
                branch = 0;
                aluSrc = 1;
                aluOp = 2'b00;
                regDst = 0;
                regWrite = 0;
                jump = 0;
            end
        endcase
    end

  // Controle da ALU
  assign aluControl = (aluOp == 2'b00) ? 3'b010 :  // ADD
                      (aluOp == 2'b01) ? 3'b110 :  // SUBT
                      (aluOp == 2'b10) ? (
                          (funct == 6'b100000) ? 3'b010 :  // ADD
                          (funct == 6'b100010) ? 3'b110 :  // SUBT
                          (funct == 6'b100101) ? 3'b001 :  // OR
                          (funct == 6'b100100) ? 3'b000 :  // AND
                          (funct == 6'b101010) ? 3'b111 :  // SETLESSTHAN
                          3'b000
                      ) : 
                      3'b000;
endmodule

// PC SECTION
module PCBranch(
  input  [31:0] extend32,
  input  [31:0] PCplus4,
  output [31:0] out
);
  
  assign out = PCplus4 + (extend32<<2);   //<<2
endmodule;

module Mux4_Scr(
  input  PCSrc,
  input  [31:0] PCPlus4,
  input  [31:0] PCBranch,
  output [31:0] out
);

 assign out = PCSrc ? PCBranch : PCPlus4;
endmodule;

module PC(
  input  [31:0] PCIn, 
  input  clock, 
  output reg [31:0] PCOut 
);

  initial PCOut = 0;
  always @(posedge clock) begin
    PCOut = PCIn;
  end
endmodule

module Mux5_PC (
  input  [31:0] pcPlus4, 
  input  [31:0] branch, 
  input  PCSrc, 
  output [31:0] PCoutMux 
);

  assign PCoutMux = (PCSrc) ? branch : pcPlus4;
endmodule

module PCplus_4(
  input  [31:0] PCplusIn, 
  output [31:0] PCplusOut
);

  assign PCplusOut = PCplusIn + 4;
endmodule


module Mux6_Jump (
  input [31:0] PCJumpIn, 
  input jump, 
  input [25:0] destination,  
  output [31:0] PC_jump_output
);

  assign PC_jump_output = jump ? {PCJumpIn[31:28], destination, 2'b00} : PCJumpIn;

endmodule

module instMem(
  input [31:0] address,        
  output reg [31:0] ReadData       
);
  reg [7:0] memory [36:0]; 

initial begin
    // addi $at, $zero, 5 
    memory[0] = 8'b001000_00;  
    memory[1] = 8'b000_00001;  
    memory[2] = 8'b00000000;  
    memory[3] = 8'b00000101;  

    // sw $at, 0($zero)
    memory[4] = 8'b101011_00;  
    memory[5] = 8'b000_00001;  
    memory[6] = 8'b00000000;  
    memory[7] = 8'b00000000;  

    // addi $v0, $zero, 10 
    memory[8] = 8'b001000_00;  
    memory[9] = 8'b000_00010;  
    memory[10] = 8'b00000000;  
    memory[11] = 8'b00001010;  

    // sw $v0, 4($zero)
    memory[12] = 8'b101011_00;  
    memory[13] = 8'b000_00010;  
    memory[14] = 8'b00000000;  
    memory[15] = 8'b00000100;  

    // inst15_0ução sub $t0, $v0, $at
    memory[16] = 8'b000000_00;  // Parte do oPCode (6 bits) - deve ser preenchida corretamente
    memory[17] = 8'b001_00001;  // Parte do rs (5 bits) e rt (5 bits) - deve ser preenchida corretamente
    memory[18] = 8'b00001000;  // Parte do ReadData (5 bits) - deve ser preenchida corretamente
    memory[19] = 8'b00_100010;  // Parte do shamt (5 bits) e funct (6 bits) - deve ser preenchida corretamente

    // beq $t0, $zero, end
    memory[20] = 8'b00010000;  
    memory[21] = 8'b00001000;  
    memory[22] = 8'b00000000;  
    memory[23] = 8'b00000100;  

    // j end
    memory[24] = 8'b00001000;  
    memory[25] = 8'b00000000;  
    memory[26] = 8'b00000000;  
    memory[27] = 8'b00000111;  

    // equal: addi $a0, $zero, 1
    memory[28] = 8'b00100000;  
    memory[29] = 8'b00000100;  
    memory[30] = 8'b00000000;  
    memory[31] = 8'b00000001;  

    // sw $a0, 8($zero)
    memory[32] = 8'b10101100;  
    memory[33] = 8'b00000100;  
    memory[34] = 8'b00000000;  
    memory[35] = 8'b00001000;  

end

  always @(address) begin
      ReadData[7:0]   = memory[address+3];
      ReadData[15:8]  = memory[address+2];
      ReadData[23:16] = memory[address+1];
      ReadData[31:24] = memory[address];
  end
endmodule


module regFile(
  input [4:0] ReadReg1, 
  input [4:0] ReadReg2, 
  input [4:0] WriteReg, 
  input signed [31:0] WriteData3, 
  input WriteEnable3, 
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

  always @(*) begin
    case(ReadReg1)
      5'd0: ReadData1 = 32'd0;
      default: ReadData1 = registers[ReadReg1];
    endcase

    case(ReadReg2)
      5'd0: ReadData2 = 32'd0;
      default: ReadData2 = registers[ReadReg2];
    endcase
  end

  always @(posedge clock) begin
    if (WriteEnable3) begin
      case(WriteReg)
        5'd0: ; // Não escreve nada
        default: registers[WriteReg] <= WriteData3;
      endcase
    end
  end
endmodule


module MuxRegDst(
  input  RegDst,
  input  [4:0] inst20_16,
  input  [4:0] inst15_11,
  output [4:0] WriteReg
);

 assign WriteReg = RegDst ? inst15_11 : inst20_16;
endmodule;


module signExtend(
  input  signed [15:0] inst15_0,
  output signed [31:0] extend32
);
  
  assign extend32 = { {16{inst15_0[15]}}, inst15_0 };
endmodule


// ALU
module ALU(
  input [2:0] aluControl,
  input signed [31:0] SrcA,
  input signed [31:0] SrcB,
  output reg zero,
  output reg signed [31:0] aluOut
);

  always @(*) begin
    case (aluControl)
      3'b010: aluOut = SrcA + SrcB;         // ADD
      3'b110: aluOut = SrcA - SrcB;         // SUBTRACT
      3'b000: aluOut = SrcA & SrcB;         // AND
      3'b001: aluOut = SrcA | SrcB;         // OR
      3'b111: aluOut = (SrcA < SrcB) ? 32'd1 : 32'd0;  // SET LESS THAN
      default: aluOut = 32'd0;              // DEFAULT CASE
    endcase
  end

  always @(*) begin
    if (aluOut == 0)
      zero = 1;
    else
      zero = 0;
  end

endmodule

module MuxAluScr(
  input  ALUSrc,
  input  [31:0] ReadData2,
  input  [31:0] extend32,
  output [31:0] SrcB
);

  assign SrcB = (ALUSrc) ? extend32 : ReadData2;
endmodule;

//  DATA MEMORY
module dataMemory (
  input clock,
  input signed [31:0] address,
  input signed [31:0] WriteData,
  input WriteEnable,
  output reg signed [31:0] ReadData
);
  reg signed [31:0] memory [31:0];
  
  always @ (posedge clock) begin
    if (WriteEnable) begin
      memory[address] <= WriteData;
      $display("Time: %0t, Write: Address = %0d, Data = %0d", $time, address, WriteData);
    end else begin
      ReadData <= memory[address];
    end
  end
endmodule


module MuxMemToReg(
  input MemtoReg,
  input [31:0] ALUOut,
  input [31:0] ReadData,
  output [31:0] out
);

 assign out = MemtoReg ? ReadData : ALUOut;
endmodule;


module ttB;
    reg clock = 0;
 
    wire W_PCSrc;
    wire [31:0] W_Mux4ScrOut; 
    wire [31:0] W_PCBranch; 
    wire [31:0] W_PCplus_4; 
    wire [31:0] W_PCJumpOut;
    wire [31:0] W_PC;
    wire [31:0] W_Extend;
    

    Mux5_PC   Mux5_PC_t(
      W_PCplus_4, 
      W_PCBranch, 
      W_PCSrc, 
      W_Mux4ScrOut
    );
    
    PC   PC_t (
      W_PCJumpOut, 
      clock, 
      W_PC 
    );
    
    PCplus_4   PCplus_4_t (
      W_PC, 
      W_PCplus_4
    );
    
    PCBranch   PCBranch_t (
      W_Extend, 
      W_PCplus_4, 
      W_PCBranch
    );

    reg [31:0] inst15_0_w;
    instMem   instMem_t(
      W_PC, 
      inst15_0_w
    );

    wire  [5:0]  inst31_26; 
    wire  [4:0]  inst25_21; 
    wire [25:0]  inst25_0; 
    wire  [4:0]  inst20_16; 
    wire  [4:0]  inst15_11; 
    wire signed  [15:0] inst15_0; 
    wire  [5:0]  inst5_0;

    assign inst31_26 = inst15_0_w[31:26];
    assign inst25_21 = inst15_0_w[25:21];
    assign inst25_0  = inst15_0_w[25:0];
    assign inst20_16 = inst15_0_w[20:16];
    assign inst15_11 = inst15_0_w[15:11];
    assign inst15_0  = inst15_0_w[15:0];
    assign inst5_0   = inst15_0_w[5:0];


    wire [4:0] W_write_reg; 
    wire W_reg_dst;
    MuxRegDst   MuxRegDst_t(
      W_reg_dst, 
      inst20_16, 
      inst15_11, 
      W_write_reg
    );

    signExtend   signExtend_t (
      inst15_0,
      W_Extend
    );

    wire W_reg_write;
    wire W_mem_write;
    wire W_mem_to_reg;
    wire W_branch; 
    wire W_alu_src; 
    wire W_jump;
    wire [2:0] W_alu_control;
  
    controlUnit controlUnit_t(
      inst31_26,
      inst5_0,
      W_mem_to_reg, 
      W_mem_write, 
      W_branch,
      W_alu_src, 
      W_reg_dst, 
      W_reg_write, 
      W_jump,
      W_alu_control
    );

    Mux6_Jump Mux6_Jump_t(
      W_Mux4ScrOut,
      W_jump,
      inst25_0,
      W_PCJumpOut
    );

    wire signed [31:0] W_MuxMemToRegOutput; 
    wire [31:0] W_ReadData1; 
    wire [31:0] W_ReadData2;
    
    regFile   regFile_t(
      inst25_21, 
      inst20_16, 
      W_write_reg, 
      W_MuxMemToRegOutput,
      W_reg_write,
      clock,
      W_ReadData1,
      W_ReadData2
    );

    wire [31:0] W_SrcB;
    MuxAluScr   MuxAluScr_t (
      W_alu_src, 
      W_ReadData2, 
      W_Extend, 
      W_SrcB
    );

    wire W_zero;
    wire signed [31:0] W_aluOut;
    ALU   alu_t (
      W_alu_control, 
      W_ReadData1, 
      W_SrcB, 
      W_zero, 
      W_aluOut
    );

    assign W_PCSrc = W_branch && W_zero;

    wire [31:0] W_read_data;
    MuxMemToReg   MuxMemToReg_t (
      W_mem_to_reg, 
      W_aluOut, 
      W_read_data, 
      W_MuxMemToRegOutput
    );
  
    dataMemory   dataMemory_t (
      clock,
      W_aluOut,
      W_ReadData2,
      W_mem_write,
      W_read_data
    );

    initial forever #5 clock = ~clock;

    initial begin
      #300 $finish;
    end

endmodule