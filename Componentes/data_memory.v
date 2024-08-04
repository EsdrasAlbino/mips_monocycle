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