
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
    Branch    = 1'b1;    
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
