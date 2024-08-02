// Code your design here
// Code your design here
 module ALU (ALUCtl, A, B, ALUOut, Zero);

	input [3:0] ALUCtl;
	input [31:0] A,B;
	
	output reg [31:0] ALUOut;
	output Zero;
	assign Zero = (ALUOut == 0);
	
	always @(ALUCtl, A, B) begin
		case (ALUCtl)
			0: ALUOut <= A & B;
			1: ALUOut <= A | B;
			2: ALUOut <= A + B;
			6: ALUOut <= A - B;
			7: ALUOut <= A < B ? 1:0;
			12: ALUOut <= ~(A | B);
			default: ALUOut <= 0;
		endcase
	end
endmodule 

 module AndGate(Branch, Zero, PCSrc);
	input Branch;
	input Zero;
	output reg PCSrc;
	
	always @(*) begin
		PCSrc <= Branch && Zero;
	end
endmodule 

 module DataMemory (clock, address, MemWrite/* , MemRead */, WriteData, ReadData);

	input clock;
	input [6:0] address;
	input MemWrite/* , MemRead */;
	input [31:0] WriteData; 
	
	output reg [31:0] ReadData;

	reg [31:0] Mem[0:127]; //32 bits memory with 128 entries

	initial begin
		Mem[0] = 5;
		Mem[1] = 6;
		Mem[2] = 7;
	end
	
	always @ (posedge clock) begin
	
		if (MemWrite == 1)
			Mem[address[6:2]] <= WriteData;
			$display("adress: %d, data: %d", address, WriteData);
	end
	
 	/*always @(negedge clock) begin
		if (MemRead == 1)
			ReadData <= Mem[address[6:2]];
	end */
endmodule
 
 module InstMem(address, inst);

	input [31:0] address;
	
	output reg [31:0]	inst;
	
	reg [31:0] Mem [0:127];
	
	initial begin
      $readmemh("program.txt", Mem, 0, 5);
	end
	always @(*) begin
		inst <= Mem[address[31:2]];
		$display("inst: %d", inst);

	end

endmodule 

module ALUControl (ALUControl, FuncCode, ALUCtl);

	input [1:0] ALUControl;
	input [5:0] FuncCode;
	output reg [3:0] ALUCtl;
	
	always @(ALUControl, FuncCode) begin
	if(ALUControl == 0)
		ALUCtl <= 2;    //LW and SW use add
	else if(ALUControl == 1)
		ALUCtl <= 6;		// branch use subtract
	else
		case(FuncCode)
			32: ALUCtl <= 2; //add
			34: ALUCtl <= 6; //subtract		
			36: ALUCtl <= 0; //and	
			37: ALUCtl <= 1; //or	
			39: ALUCtl <= 12; //nor
			42: ALUCtl <= 7; //slt
			default: ALUCtl <= 15; //should not happen
		endcase
	end
endmodule

module MainCOntrol(
	input [5:0] Opcode,
	
	output reg RegDst, RegWrite, ALUSrc,
	output reg MemtoReg, MemRead, MemWrite,
	output reg Branch,
	output reg [1:0] ALUControl); //ALUOp
	
	always @(*) begin
		case(Opcode)
			0: begin
				RegDst 		<= 1;
				ALUSrc 		<= 0;
				MemtoReg		<= 0;
				RegWrite		<= 1;
				MemRead		<= 0;
				MemWrite		<= 0;
				Branch		<= 0;
				ALUControl			<= 2'b10;;
			end
			35: begin
				RegDst 		<= 0;
				ALUSrc 		<= 1;
				MemtoReg		<= 1;
				RegWrite		<= 1;
				MemRead		<= 1;
				MemWrite		<= 0;
				Branch		<= 0;
				ALUControl			<= 2'b00;;
			end
			43: begin
				RegDst 		<= 0;
				ALUSrc 		<= 1;
				MemtoReg		<= 0;
				RegWrite		<= 0;
				MemRead		<= 0;
				MemWrite		<= 1;
				Branch		<= 0;
				ALUControl			<= 2'b00;
			end
			4: begin
				RegDst 		<= 0;
				ALUSrc 		<= 0;
				MemtoReg		<= 0;
				RegWrite		<= 0;
				MemRead		<= 0;
				MemWrite		<= 0;
				Branch		<= 1;
				ALUControl			<= 2'b01;
			end
		endcase
	end
endmodule

module ControlUnit(
	input [5:0] Opcode,
	input [5:0] Funct,
	
	output reg RegDst, RegWrite, ALUSrc,
	output reg MemtoReg, MemRead, MemWrite,
	output reg Branch,
	output reg [1:0] ALUOp,
	output reg [2:0] ALUCtl);
	
	// main decoder
	MainCOntrol main_control_0(
		.Opcode(Opcode),
		.RegDst(RegDst),
		.RegWrite(RegWrite),
		.ALUSrc(ALUSrc),
		.MemtoReg(MemtoReg),
		.MemRead(MemRead),
		.MemWrite(MemWrite),
		.Branch(Branch),
		.ALUControl(ALUOp)
	);

	// ALU decoder
	ALUControl alu_control_0(
		.ALUControl(ALUOp),
		.FuncCode(Funct),
		.ALUCtl(ALUCtl)
	);

	endmodule

module Mux2to1(
    input wire [31:0] in0,
    input wire [31:0] in1,
    input wire sel,
    output wire [31:0] out
);

    assign out = sel ? in1 : in0;

endmodule


module PC(clock, reset, PCin, PCout);

	input clock, reset;
	input [31:0] PCin;
	
	output reg [31:0] PCout;
	
	always @(posedge clock) begin
		if (reset == 1) 
			PCout <= 0;
		else 
			PCout <= PCin; 
	end
endmodule 

 module RegFile(clock, RegWrite, ReadReg1, ReadReg2, WriteReg, WriteData, ReadData1, ReadData2);

	input clock;
	input RegWrite;
	
	input [4:0] ReadReg1, ReadReg2, WriteReg;
	input [31:0] WriteData;
		
	output [31:0] ReadData1, ReadData2;
	
	reg [31:0] reg_mem [0:31];
	initial begin
		reg_mem[0] <= 0;
		reg_mem[1] <= 8;
		reg_mem[2] <= 20;
	end
	assign ReadData1 = reg_mem[ReadReg1];
	assign ReadData2 = reg_mem[ReadReg2];
	
	always @(posedge clock) begin
		if (RegWrite == 1)
			reg_mem[WriteReg] = WriteData;
	end	
endmodule 

 module ShiftLeft2 (ShiftIn, ShiftOut);

	input [31:0] ShiftIn;
	output reg [31:0] ShiftOut;
	
	always @(ShiftIn) begin
		ShiftOut = ShiftIn << 2;
	end 
	
endmodule 

 module SignExtend (inst15_0, Extend32);

	input [15:0] inst15_0;
	output reg [31:0] Extend32;

	always @(inst15_0) begin
		Extend32[31:0] <= inst15_0[15:0];
	end
endmodule 

module Add(ALUOut, A, B);

	input [31:0] A,B;
	output reg [31:0] ALUOut;
	
	always @(A, B) begin
		ALUOut = A + B;
	end
endmodule


module MipsCPU(clock, reset, 
					PCin,PCout,
					inst,
					RegDst, RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, Branch,
					ALUControl,
					WriteReg,
					ReadData1, ReadData2,
					Extend32,
					ALU_B,
					ShiftOut,
					ALUCtl,
					Zero,
					ALUOut,
					pcPlus4,
					PCSrc,
					ReadData,
					WriteData_Reg);
					
	input clock;
	input reset;

	always @(*) begin
    $display("PCout: %d", PCout);
  end
	
	//Connection of PC
	output wire [31:0] PCin, PCout;
	PC pc_0(
		//inputs
		.clock(clock),
		.reset(reset),
		.PCin(PCin),
		//outputs
		.PCout(PCout)	
	);

	//Connection of PCPlus4
	output wire [31:0] pcPlus4;
	Add pcplus4_0(
		//inputs
		.A(PCout),
		.B(4),
		//outputs
		.ALUOut(pcPlus4)
	);

	//Connection of InstMem
	output wire [31:0] inst;
	InstMem instmem_0(
		//inputs
		.address(PCout),
		//outputs
		.inst(inst)	
	);
	
	//Connection of ControlUnit
	output wire RegDst, RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, Branch, ALUCtl;
	output wire [1:0] ALUControl;
	ControlUnit main_control_0(
		//inputs
		.Opcode(inst[31:26]),
		.Funct(inst[5:0]),
		//outputs
		.RegDst(RegDst),
		.RegWrite(RegWrite),
		.ALUSrc(ALUSrc),
		.MemtoReg(MemtoReg),
		.MemRead(MemRead),
		.MemWrite(MemWrite),
		.Branch(Branch),
		.ALUCtl(ALUCtl)	
	);
	
	//Connection of the Mux between InstMem and RegisterFile
	output wire [4:0]  WriteReg;
	Mux2to1 mux1_0(
		//inputs
		.in0(inst[20:16]),
		.in1(inst[15:11]),
		.sel(RegDst),
		//outputs
		.out(WriteReg)	
	);
	
	//Connection of RegFile
	output wire [31:0] ReadData1, ReadData2;
	RegFile regfile_0(
		//inputs
		.clock(clock),
		.ReadReg1(inst[25:21]), //A1
		.ReadReg2(inst[20:16]), //A2
		.RegWrite(RegWrite),    // WE3
		.WriteReg(WriteReg),	//A3
		.WriteData(WriteData_Reg), //WD3
		//outputs
		.ReadData1(ReadData1), //RD1
		.ReadData2(ReadData2)	//RD2
	);
	
	//Connection of SignExtend
	output wire [31:0] Extend32;
	SignExtend sign_extend_0(
		//inputs
		.inst15_0(inst[15:0]),
		//outputs
		.Extend32(Extend32)
	);
	
	//Connection of Mux2
	output wire [31:0] ALU_B;
	Mux2to1 mux2_0(
		//inputs
		.in0(ReadData2), //ReadData2
		.in1(Extend32), //Extend32
		.sel(ALUSrc), //ALUSrc
		//outputs
		.out(ALU_B) //src B	
	);
	
	//Connection of ShiftLeft2
	output wire [31:0] ShiftOut;
	ShiftLeft2 shift_left2_0(
		//inputs
		.ShiftIn(Extend32),
		//outputs
		.ShiftOut(ShiftOut)
	);

	output wire[31:0] PCBranch;
	Add PcBranch(
		//inputs
		.A(pcPlus4),
		.B(ShiftOut),
		//outputs
		.ALUOut(PCBranch)
	);
		
	//Connection of ALU
	output wire Zero;
	output wire [31:0] ALUOut;
	ALU alu_0(
		//inputs
		.A(ReadData1), //SRC A
		.B(ALU_B), //SRC B
		.ALUCtl(ALUCtl),
		//outputs
		.ALUOut(ALUOut), // ALU Result
		.Zero(Zero)
	);
	
	//Connection of AndGate
	output wire PCSrc;
	AndGate and_gate_0(
		//inputs
		.Branch(Branch),
		.Zero(Zero),
		//outputs
		.PCSrc(PCSrc)
	);
	
	//Connection of Mux4
	Mux2to1 mux4_0(
		//inputs
		.in0(pcPlus4),
		.in1(PCBranch),
		.sel(PCSrc),
		//outputs
		.out(PCin)
	);
	
	//Connection of DataMemory
	output wire [31:0] ReadData;
	DataMemory  data_memory_0(
		//inputs
		.clock(clock),
		.address(ALUOut),
		.MemWrite(MemWrite),
		/* .MemRead(MemRead), */
		.WriteData(ReadData2),
		//outputs
		.ReadData(ReadData)
	);
	
	//Connection of Mux3
	output wire[31:0] WriteData_Reg;
	Mux2to1 mu3_0(
	//inputs
	.in0(ALUOut),
	.in1(ReadData),
	.sel(MemtoReg),
	//outputs
	.out(WriteData_Reg)
	);	
endmodule