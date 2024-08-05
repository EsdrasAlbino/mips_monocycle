
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
