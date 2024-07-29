module PCPlus4(
    input [31:0] PC,        // Entrada do contador de programa (PC)
    output [31:0] PCPlus4   // Saída do PC incrementado em 4
);

    assign PCPlus4 = PC + 4;

endmodule
