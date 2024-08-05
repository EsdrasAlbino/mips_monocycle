module ttB;
    reg clock = 0;
 
    wire W_PCSrc;
    wire [31:0] W_Mux4ScrOut; 
    wire [31:0] W_PCBranch; 
    wire [31:0] W_PCplus_4; 
    wire [31:0] W_PCJumpOut;
    wire [31:0] W_PC;
    wire [31:0] W_Extend;
    
    Mux5_PC Mux5_PC_t(
        W_PCplus_4, 
        W_PCBranch, 
        W_PCSrc, 
        W_Mux4ScrOut
    );
    
    PC PC_t (
        W_PCJumpOut, 
        clock, 
        W_PC
    );
    
    PCplus_4 PCplus_4_t (
        W_PC, 
        W_PCplus_4
    );
    
    PCBranch PCBranch_t (
        W_Extend, 
        W_PCplus_4, 
        W_PCBranch
    );

    reg [31:0] inst15_0_w;
    instMem instMem_t(
        W_PC, 
        inst15_0_w
    );

    wire [5:0] inst31_26; 
    wire [4:0] inst25_21; 
    wire [25:0] inst25_0; 
    wire [4:0] inst20_16; 
    wire [4:0] inst15_11; 
    wire signed [15:0] inst15_0; 
    wire [5:0] inst5_0;

    assign inst31_26 = inst15_0_w[31:26];
    assign inst25_21 = inst15_0_w[25:21];
    assign inst25_0  = inst15_0_w[25:0];
    assign inst20_16 = inst15_0_w[20:16];
    assign inst15_11 = inst15_0_w[15:11];
    assign inst15_0  = inst15_0_w[15:0];
    assign inst5_0   = inst15_0_w[5:0];

    wire [4:0] W_write_reg; 
    wire W_reg_dst;
    MuxRegDst MuxRegDst_t(
        W_reg_dst, 
        inst20_16, 
        inst15_11, 
        W_write_reg
    );

    signExtend signExtend_t (
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
    
    regFile regFile_t(
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
    MuxAluScr MuxAluScr_t (
        W_alu_src, 
        W_ReadData2, 
        W_Extend, 
        W_SrcB
    );

    wire W_zero;
    wire signed [31:0] W_aluOut;
    ALU alu_t (
        W_alu_control, 
        W_ReadData1, 
        W_SrcB, 
        W_zero, 
        W_aluOut
    );

    assign W_PCSrc = W_branch && W_zero;

    wire [31:0] W_read_data;
    MuxMemToReg MuxMemToReg_t (
        W_mem_to_reg, 
        W_aluOut, 
        W_read_data, 
        W_MuxMemToRegOutput
    );
  
    dataMemory dataMemory_t (
        clock,
        W_aluOut,
        W_ReadData2,
        W_mem_write,
        W_read_data
    );

    initial begin
        $monitor("Time: %0t | PC: %h | Instruction: %h", $time, W_PC, inst15_0_w);
        #70 $finish;
    end

    initial begin
        forever #5 clock = ~clock;
    end

endmodule
