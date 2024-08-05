module RegFile(
  input [4:0] ReadReg1,
  input [4:0] ReadReg2, 
  input [4:0] WriteReg, 
  input signed [31:0] WD3, 
  input WE3, 
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