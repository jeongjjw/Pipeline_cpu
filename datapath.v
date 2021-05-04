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
	reg read_m1_reg, read_m2_reg, read_m2_reg_temp;
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

	//wire for wwd
	wire [15:0] outputWWD_ID, outputWWD_EX, outputWWD_MEM;

	// wire for EX
	wire [15:0] ALU_a, ALU_b_temp, ALU_b;

	//wire for control unit
	wire [3:0] opcode;
	wire [5:0] func_code;
	wire pc_write_cond, pc_write, mem_read, mem_to_reg, mem_write, ir_write;
	wire [1:0] pc_src;
	wire pc_to_reg, halt, wwd, new_inst;
	wire reg_write, alu_op, ALUsrc;
	
	// pipeline register_modules
	IFID ifid(clk, inputIR_IFID, inputPC_IFID, outputIR_IFID, outputPC_IFID, ir_write);
	IDEX idex(clk, inputPC_IDEX, inputData1_IDEX, inputData2_IDEX, inputImm_IDEX, inputInstr_IDEX, inputWB_IDEX, outputPC_IDEX, outputData1_IDEX, outputData2_IDEX, outputImm_IDEX, outputInstr_IDEX, outputWB_IDEX);
	// EXMEM exmem(clk, inputPC_EXMEM, inputALUOUT_EXMEM, inputB_EXMEM, outputB_EXMEM, outputALUOUT_EXMEM, outputPC_EXMEM, inputWB_EXMEM, outputWB_EXMEM);
	EXMEM exmem(clk, inputPC_EXMEM, inputALUOUT_EXMEM, inputB_EXMEM, inputWB_EXMEM, outputB_EXMEM, outputALUOUT_EXMEM, outputPC_EXMEM, outputWB_EXMEM, ALU_a, outputWWD_EX );
	MEMWB memwb(clk, inputReadData_MEMWB, inputALUResult_MEMWB, inputWB_MEMWB, outputReadData_MEMWB, outputALUResult_MEMWB, outputWB_MEMWB, outputWWD_EX, outputWWD_MEM);

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
	wire [3:0] funcCode;
  	wire [1:0] branchType;
	wire bcond;
	wire overflow_flag;
	
	//wire for regfile
	wire [1:0] read1;
	wire [1:0] read2;
	wire [1:0] dest;
	wire [`WORD_SIZE-1:0] write_data;
	wire [`WORD_SIZE-1:0] read_out1;
	wire [`WORD_SIZE-1:0] read_out2;

	//forwarding Register
	reg [`WORD_SIZE - 1 : 0] forwarding_ALUout;

	// assign for IF data
	assign inputIR_IFID = data1;
	//assign read1 = inputIR_IFID[11 : 10];
	//assign read2 = inputIR_IFID[9 : 8];
	//assign inputWB_IDEX = inputIR_IFID[7 : 6];
	
	// assign for control
	assign read_m1 = read_m1_reg;
	assign read_m2 = read_m2_reg;
	assign address1 = PC;
//address1_reg;

	//assign for IFID pipe
	assign inputPC_IFID = PC;

	//assign for ID data
	assign inputPC_IDEX = outputPC_IFID;
	assign inputData1_IDEX = read_out1;
	assign inputData2_IDEX = read_out2;
	//inputImm_IDEX already assigned from IMMmodule
	assign inputInstr_IDEX = outputIR_IFID;
	//assign inputWB_IDEX = outputIR_IFID[7:6];//write can happen to rd or rt, we need control paths too

	wire[1:0] rt, rs, rd, destination_reg_input_t;
	assign rs = inputIR_IFID[11 : 10];
	assign rt = inputIR_IFID[9 : 8];
	assign rd = inputIR_IFID[7 : 6];

	assign opcode = inputIR_IFID[15:12];
	assign func_code = inputIR_IFID[5:0];

	assign read1 = rs;
	assign read2 = rt;
	
	assign destination_reg_input_t = (opcode <= 4'd8 && opcode >= 4'd0) ? rt : rd;
	assign inputWB_IDEX = (wwd==1) ? outputWB_IDEX:((opcode == `JAL_OP || (opcode == `JRL_OP && func_code == `INST_FUNC_JRL)) ? 4'd2 : destination_reg_input_t);
	//assign 

	//assign for EX data
	assign inputPC_EXMEM = outputPC_IDEX;
	assign inputB_EXMEM = outputData2_IDEX;
	assign inputWB_EXMEM = outputWB_IDEX;


	//assign for MEM data 
	assign inputWB_MEMWB = outputWB_IDEX;
	assign address2 = outputALUOUT_EXMEM;
	assign data2 = write_m2 ? outputB_EXMEM : 16'bz;
	assign inputALUResult_MEMWB = outputALUOUT_EXMEM;
	assign inputReadData_MEMWB = data2;//use data2 directly for forwarding unit and reg_write_data
	
	//WB state
	mux2_1 mux_LWD_or_aluoperation(mem_to_reg_o_M, outputALUOUT_EXMEM, data2, write_data);//directly connect wire

	// control reg wires
	// ID / EX control
	// wire pc_write_cond_i, pc_write_i, mem_read_i, mem_to_reg_i, mem_write_i, ir_write_i, pc_src_i, pc_to_reg_i, halt_i, wwd_i, new_inst_i, reg_write_i, alu_op_i, ALUsrc_i;
	wire pc_write_cond_o, pc_write_o, mem_read_o, mem_to_reg_o, mem_write_o, ir_write_o,  pc_to_reg_o, halt_o, wwd_o, new_inst_o, reg_write_o, alu_op_o, ALUsrc_o;
	wire [1:0] pc_src_o;
	//EX/ MEM control
	// wire pc_write_cond_i_E, pc_write_i_E, i_or_d_i_E, mem_read_i_E, mem_to_reg_i_E;
	// wire mem_write_i_E, ir_write_i_E, pc_to_reg_i_E, pc_src_i_E, halt_i_E, wwd_i_E, new_inst_i_E, reg_write_i_E;
	wire pc_write_cond_o_E, pc_write_o_E, i_or_d_o_E, mem_read_o_E, mem_to_reg_o_E;
	wire mem_write_o_E, ir_write_o_E, pc_to_reg_o_E, halt_o_E, wwd_o_E, new_inst_o_E, reg_write_o_E;
	wire [1:0] pc_src_o_E;
	// mem/wb control
	// wire reg_write_i_M, new_inst_i_M, wwd_i_M, halt_i_M;
	wire reg_write_o_M, new_inst_o_M, wwd_o_M, halt_o_M;
	
	wire branch_signal;

	//assign for MEM control
	assign write_m2 = mem_write_o_E;
	//assign read_m2 = mem_read_o_E;

	// datapath EX
	mux4_1 srcA(forward_A, outputData1_IDEX, write_data, /*outputALUOUT_EXMEM*/ forwarding_ALUout, 16'b0, ALU_a);
	mux4_1 srcB_temp(forward_B, outputData2_IDEX, write_data, forwarding_ALUout, 16'b0, ALU_b_temp);
	mux2_1 scrB(ALUsrc_o, ALU_b_temp, outputImm_IDEX ,ALU_b);
	
	//control modules
	control_unit control_unit_module(opcode, func_code, clk, reset_n, pc_write_cond, /*pc_write,*/ mem_read, mem_to_reg, mem_write, /*ir_write,*/ pc_to_reg, pc_src, halt, wwd, new_inst, reg_write, alu_op, ALUsrc);
	hazard_detect hazard_detection_module(clk, inputInstr_IDEX, outputWB_EXMEM, mem_read_o_E, is_stall, pc_write, ir_write);
	forwarding_unit forwarding_module(clk, forward_A, forward_B, inputInstr_IDEX[11:10], inputInstr_IDEX[9:8], reg_write_o_E, reg_write_o_M, outputWB_EXMEM, outputWB_MEMWB);
	alu_control_unit alu_control_module(inputInstr_IDEX[5:0], inputInstr_IDEX[15:12], 2'b0, clk, funcCode, branchType);
	//ALU module
	alu alu_module(ALU_a, ALU_b, funcCode, branchType, inputALUOUT_EXMEM, overflow_flag, bcond);

	//Register file
	register_file register_module(read_out1, read_out2, read1, read2, outputWB_MEMWB, write_data, reg_write_o_M, clk, reset_n);

	//ImmGen Modul
	ImmGen immgen_module(inputIR_IFID, inputImm_IDEX);	 

	//control mux
	wire pc_write_cond_t, mem_read_t, mem_to_reg_t, mem_write_t, pc_to_reg_t, halt_t, wwd_t, new_inst_t, reg_write_t, alu_op_t, ALUsrc_t;
	wire [1:0] pc_src_t;

	reg branch_signal_t;

	assign pc_write_cond_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : pc_write_cond;
	assign mem_read_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : mem_read;
	assign mem_to_reg_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : mem_to_reg;
	assign mem_write_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : mem_write;
	assign pc_to_reg_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : pc_to_reg;
	assign pc_src_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : pc_src;
	assign halt_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : halt;
	assign wwd_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : wwd;
	assign new_inst_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : new_inst;
	assign reg_write_t = (is_stall == 1'b1 || branch_signal == 1'b1) ? 1'b0 : reg_write;
	assign alu_op_t = (is_stall == 1'b1|| branch_signal == 1'b1) ? 1'b0 : alu_op;
	assign ALUsrc_t = (is_stall == 1'b1  || branch_signal == 1'b1) ? 1'b0 : ALUsrc;

	//control reg modules
	IDEX_Control IDEX_Control_module(clk, pc_write_cond_t, /*pc_write,*/ mem_read_t, mem_to_reg_t, mem_write_t, /*ir_write,*/ pc_src_t, pc_to_reg_t, halt_t,
		wwd_t, new_inst_t, reg_write_t, alu_op_t, ALUsrc_t, 
		pc_write_cond_o, /*pc_write_o,*/ mem_read_o, mem_to_reg_o, mem_write_o, /*ir_write_o,*/ pc_src_o, pc_to_reg_o, halt_o,
		wwd_o, new_inst_o, reg_write_o, alu_op_o, ALUsrc_o, is_stall);
	

	EXMEM_Control EXMEM_Control_Module(clk, pc_write_cond_o, /*pc_write_o,*/ i_or_d_o, mem_read_o, mem_to_reg_o,
			mem_write_o, /*ir_write_o,*/ pc_to_reg_o, pc_src_o, halt_o, wwd_o, new_inst_o, reg_write_o,
			pc_write_cond_o_E, /*pc_write_o_E,*/ i_or_d_o_E, mem_read_o_E, mem_to_reg_o_E,
			mem_write_o_E, /*ir_write_o_E,*/ pc_to_reg_o_E, pc_src_o_E, halt_o_E, wwd_o_E, new_inst_o_E, reg_write_o_E);

	MEMWB_Control MEMWB_Control_module(clk, reg_write_o_M, reg_write_o_E, new_inst_o_E, new_inst_o_M, wwd_o_E, wwd_o_M, halt_o_M, halt_o_E, mem_to_reg_o_M, mem_to_reg_o_E);

	reg flagRegister, count;
	assign is_halted = halt_o_M;//check this

	// Branch Predictor
	wire [`WORD_SIZE - 1:0] nextBranchPC, correctPC;
	wire condition;
	
	branch_predictor BP(clk, PC, correctPC, condition, nextBranchPC);
	checkCondition checkCondition_module(clk, inputIR_IFID, read_out1, read_out2, condition);
	calc_correct calc_correct_module(clk, condition, inputImm_IDEX, outputPC_IFID, correctPC);
	branch_sig b_sig_module(clk, outputPC_IFID, correctPC, branch_signal, inputIR_IFID);
	
	// Initalize
	initial begin
		PC = 0;
		read_m1_reg = 1'b0;
		num_inst = 0;
		flagRegister = 1'b0;
		forwarding_ALUout = 16'b0;
		read_m2_reg = 1'b0;
		read_m2_reg_temp = 1'b0;
		count = 1'b0;
		branch_signal_t = 0;
	end

	// integer count = 0; // temporary
	
	always @(posedge clk) begin
		if(!reset_n) begin
			PC <= 0;
			read_m1_reg <= 1'b0;
			flagRegister <= 1'b0;
			read_m2_reg <= 1'b0;
			count <= 1'b0;
		end

		else begin
			if(count != 1'b0 && pc_write == 1'b1) begin
				if(branch_signal == 1'b1) begin
					PC <= correctPC;
				end
				else if(pc_src == 0) begin
					PC <= (PC + 1);
					//num_inst <= (num_inst + 1);//
				end
				else if(pc_src == 1) begin
					PC <= inputImm_IDEX;//temporal for first jump
				end
				else if(pc_src == 2) begin
					PC <= nextBranchPC;
				end
			end
			read_m1_reg <= 1'b1;
			address1_reg <= PC;
			count <= 1'b1;
		end
		
		if(mem_read_o_E == 1'b1) begin
			read_m2_reg <= 1;
			read_m2_reg_temp <= 1;
		end
		else if(read_m2_reg_temp ==1) begin
			read_m2_reg_temp <= 0;
			read_m2_reg <= 1;
		end
		else begin
			read_m2_reg <= 0;
		end

		if(is_stall  == 0) begin
			forwarding_ALUout <= outputALUOUT_EXMEM;
		end

		if(flagRegister == 1'b1) begin
			flagRegister <= 1'b0;
			num_inst <= (num_inst + 1);
		end

		if(new_inst_o_M == 1'b1) begin
			//num_inst <= (num_inst + 1);
			flagRegister <= 1'b1;
		end

		if(wwd_o_M == 1'b1) begin
			output_port <= outputWWD_MEM;
		end		
		/*
		if(branch_signal == 1) begin
			branch_signal_t <=1;
		end
		//delay branch_signal by one clock
		if(branch_signal_t ==1)begin
			branch_signal_t <=0;
		end*/
	end

endmodule

