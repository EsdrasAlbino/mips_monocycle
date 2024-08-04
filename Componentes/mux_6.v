module Mux6_Jump (
  input [31:0] PCJumpIn, 
  input Jump, 
  input [25:0] Destination,  
  output [31:0] PCJumpOut
);

  wire [31:0] DestinationShifted = {Destination, 2'b00}; 
  assign PCJumpOut = (Jump) ? {PCJumpIn[31:28], DestinationShifted[27:0]} : PCJumpIn;

endmodule