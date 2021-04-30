`include "opcodes.v"
//`include "register_file.v" 
//`include "alu.v"
//`include "control_unit.v" 
//`include "branch_predictor.v"
//`include "hazard.v"

module datapath(clk, reset_n, read_m1, address1, data1, read_m2, write_m2, address2, data2, num_inst, output_port, is_halted);

	input clk;
	input reset_n;

	output read_m1;
	output [`WORD_SIZE-1:0] address1;
	output read_m2;    
	output write_m2;
	output [`WORD_SIZE-1:0] address2;

	input [`WORD_SIZE-1:0] data1;
	inout [`WORD_SIZE-1:0] data2;

	output reg [`WORD_SIZE-1:0] num_inst;
	output reg [`WORD_SIZE-1:0] output_port;
	output is_halted;

	//TODO: implement datapath of pipelined CPU

	// register for output
	reg read_m1_reg;
	reg [15:0] address1_reg;

	// register wires 
	//wire into IF ID
	wire [15:0] inputIR_IFID, inputPC_IFID;
	wire [15:0] outputIR_IFID, outputPC_IFID;

	//wire into ID EX
	wire [`WORD_SIZE - 1 : 0] inputPC_IDEX, inputData1_IDEX, inputData2_IDEX, inputImm_IDEX, inputInstr_IDEX;
	wire [1 : 0] inputWB_IDEX, outputWB_IDEX;
	wire [`WORD_SIZE - 1 : 0] outputPC_IDEX, outputData1_IDEX, outputData2_IDEX, outputImm_IDEX, outputInstr_IDEX;

	//wire into EX MEM
	wire [15:0] inputPC_EXMEM, inputALUOUT_EXMEM, inputB_EXMEM;
	wire [1 : 0] inputWB_EXMEM;
	wire [15:0] outputB_EXMEM, outputALUOUT_EXMEM, outputPC_EXMEM;
	wire [1 : 0] outputWB_EXMEM;

	//wire into MEMWB
	wire [`WORD_SIZE - 1 : 0] inputReadData_MEMWB, inputALUResult_MEMWB;
	wire [1 : 0] inputWB_MEMWB;
	wire [`WORD_SIZE - 1 : 0] outputReadData_MEMWB, outputALUResult_MEMWB;
	wire [1 : 0] outputWB_MEMWB;	
	
	// Register
	reg [`WORD_SIZE - 1 : 0] PC;
	
	// pipeline register_modules
	IFID ifid(clk, inputIR_IFID, inputPC_IFID, outputIR_IFID, outputPC_IFID);
	IDEX idex(clk, inputPC_IDEX, inputData1_IDEX, inputData2_IDEX, inputImm_IDEX, inputInstr_IDEX, inputWB_IDEX, outputPC_IDEX, outputData1_IDEX, outputData2_IDEX, outputImm_IDEX, outputInstr_IDEX, outputWB_IDEX);
	EXMEM exmem(clk, inputPC_EXMEM, inputALUOUT_EXMEM, inputB_EXMEM, outputB_EXMEM, outputALUOUT_EXMEM, outputPC_EXMEM, inputWB_EXMEM, outputWB_EXMEM);
	MEMWB memwb(clk, inputReadData_MEMWB, inputALUResult_MEMWB, inputWB_MEMWB, outputReadData_MEMWB, outputALUResult_MEMWB, outputWB_MEMWB);

	//wire for control unit
	wire [3:0] opcode;
	wire [5:0] func_code;
	wire pc_write_cond, pc_write, mem_read, mem_to_reg, mem_write, ir_write, pc_src;
	wire pc_to_reg, halt, wwd, new_inst;
	wire reg_write, alu_op, ALUsrc;

	//wire for hazard unit
	wire [`WORD_SIZE-1:0] IFID_IR;
	wire [1:0]  IDEX_rd;
	wire IDEX_M_mem_read;
	wire is_stall;

	//wire for forwarding unit
	wire [1:0] rs1, rs2, rd_EXMEM, rd_MEMWB;
	wire WB_EXMEM, WB_MEMWB;//WB is RegWrite
	wire [1:0] forward_A, forward_B;
	
	//wire for alu_control unit
	//wire [1:0] ALUOp; already stated from control unit
  	//input [3:0] opcode;//size can change
  	//input [5:0] func_code;//size can change
	wire [2:0] funcCode;
  	wire [1:0] branchType;

	//wire for regfile
	wire [1:0] read1;
	wire [1:0] read2;
	wire [1:0] dest;
	wire [`WORD_SIZE-1:0] write_data;
	wire [`WORD_SIZE-1:0] read_out1;
	wire [`WORD_SIZE-1:0] read_out2;

	// assign for IF data
	assign inputIR_IFID = data1;
	assign read1 = inputIR_IFID[11 : 10];
	assign read2 = inputIR_IFID[9 : 8];
	assign inputWB_IDEX = inputIR_IFID[7 : 6];
	assign opcode = data1[15:12];
	assign fuccode = data1[5:0];

	// assign for control
	assign read_m1 = read_m1_reg;
	assign address1 = PC;
//address1_reg;

	//assign for MEM data 
	assign inputWB_MEMWB = outputWB_IDEX;
	assign address2 = outputALUOUT_EXMEM;
	assign data2 = write_m2 ? outputB_EXMEM : 16'bz;
	assign inputALUResult_MEMWB = outputALUOUT_EXMEM;
	assign inputReadData_MEMWB = data2;//use data2 directly for forwarding unit and reg_write_data
	mux2_1 mux_LWD_or_aluoperation(mem_to_reg, data2, outputALUOUT_EXMEM, write_data);
	
	//assign for MEM control
	assign write_m2 = mem_write;
	assign read_m2 = mem_read;

	// datapath EX
	wire [15:0] ALU_a, ALU_b_temp, ALU_b;
	mux4_1 srcA(forward_A, outputData1_IDEX, write_data, outputALUOUT_EXMEM, 16'b0, ALU_a);
	mux4_1 srcB_temp(forward_B, outputData2_IDEX, write_data, outputALUOUT_EXMEM, 16'b0, ALU_b_temp);
	mux2_1 scrB(ALUsrc, ALU_b_temp, outputImm_IDEX ,ALU_b);
	
	//control modules
	control_unit control_unit_module(opcode, func_code, clk, reset_n, pc_write_cond, pc_write, mem_read, mem_to_reg, mem_write, ir_write, pc_to_reg, pc_src, halt, wwd, new_inst, reg_write, alu_op, ALUsrc);
	hazard_detect hazard_detection_module(IFID_IR, IDEX_rd, IDEX_M_mem_read, is_stall);
	forwarding_unit forwarding_module(clk, forward_A, forward_B, rs1, rs2, WB_EXMEM, WB_MEMWB, rd_EXMEM, rd_MEMWB);
	alu_control_unit alu_control_module(func_code, opcode, 2'b0, clk, funcCode, branchType);
	//ALU module
	alu alu_module(ALU_a, ALU_b, func_code, branch_type, C, overflow_flag, bcond);

	//Register file
	register_file register_module(read_out1, read_out2, read1, read2, outputWB_MEMWB, write_data, reg_write, clk, reset_n);

	//ImmGen Module
	ImmGen immgen_module(inputIR_IFID, inputImm_IDEX);	 

	// Initalize
	initial begin
		PC = 0;
		read_m1_reg = 1'b1;
	end

	integer count = 0;
	
	always @(posedge clk) begin
		if(!reset_n) begin
			PC <= 0;
			read_m1_reg <= 1'b1;
		end

		else begin
			if(count != 0) begin
				if(pc_src ==0)
					PC <= (PC + 1);
				else	
					PC <= inputImm_IDEX;//temporal for first jump
			end
			read_m1_reg <= 1'b1;
			address1_reg <= PC;
			count = count + 1;
		end

		if(wwd == 1'b1) begin
			output_port <= read_out1;
		end

		
	end


endmodule

