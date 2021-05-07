`include "opcodes.v" 

module register_file (PC, num_inst, read_out1, read_out2, read1, read2, dest, write_data, reg_write, clk, reset_n);

	input [`WORD_SIZE - 1 : 0] PC;
	input [`WORD_SIZE - 1 : 0] num_inst;
	input clk, reset_n;
	input [1:0] read1;
	input [1:0] read2;
	input [1:0] dest;
	input reg_write;
	input [`WORD_SIZE-1:0] write_data;
	
	output reg [`WORD_SIZE-1:0] read_out1;
	output reg [`WORD_SIZE-1:0] read_out2;
	
	//TODO: implement register file
	reg [15:0] reg_file [0:3];
	reg [`WORD_SIZE - 1 : 0] write_data_reg;

	initial begin
		reg_file[0] = 0;
		reg_file[1] = 0;
		reg_file[2] = 0;
		reg_file[3] = 0;
		read_out1 = 0;
		read_out2 = 0;
		write_data_reg = 0;
	end

	always @(posedge clk) begin
		if(!reset_n) begin
			reg_file[0] <= 0;
			reg_file[1] <= 0;
			reg_file[2] <= 0;
			reg_file[3] <= 0;
			read_out1 <= 0;
			read_out2 <= 0;
			write_data_reg <= 0;
		end
		write_data_reg <= write_data;
	end

	always@(/*posedge clk*/*)begin
		if(reg_write == 1 && read1 == dest && read2 == dest) begin
			read_out1 = write_data_reg;
			read_out2 = write_data_reg;
		end
		else if (reg_write == 1 && read2 == dest && read1 != dest) begin
			read_out2 = write_data_reg;
			read_out1 = reg_file[read1];
		end
		else if (reg_write == 1 && read2 != dest && read1 == dest) begin
			read_out1 = write_data_reg;
			read_out2 = reg_file[read1];
		end
		else begin
			read_out1 = reg_file[read1];
			read_out2 = reg_file[read2];
		end
		$strobe("PC : %h, num: %h, <0: %h> <1: %h> <2: %h> <3: %h>", PC, num_inst + 1, reg_file[0], reg_file[1], reg_file[2], reg_file[3]);
	end

	always@(posedge clk) begin
		if(reg_write ==1) begin
			reg_file[dest] <= write_data;
		end

	end
    
endmodule

module IFID (clk, inputIR, inputPC, outputIR, outputPC, ir_write,  nextBranchPC, outputPredictPC_IFID);
	input clk, ir_write;
	input [15:0] inputIR, inputPC, nextBranchPC;
	output reg [15:0] outputIR, outputPC, outputPredictPC_IFID;

	initial begin
		outputIR = 0;
		outputPC = 0;
		outputPredictPC_IFID = 0;
	end
	
	always @(negedge clk) begin
		if(ir_write) begin
			outputIR <= inputIR;
			outputPC <= inputPC;
			outputPredictPC_IFID <= nextBranchPC;
		end
	end
	
endmodule

module IDEX (clk, inputPC, inputData1, inputData2, inputImm, inputInstr, inputWB, outputPC, outputData1, outputData2, outputImm, outputInstr, outputWB/*, is_flush*/);
	input clk;
	input [`WORD_SIZE - 1 : 0] inputPC, inputData1, inputData2, inputImm, inputInstr;
	input [1 : 0] inputWB;
	output reg [`WORD_SIZE - 1 : 0] outputPC, outputData1, outputData2, outputImm, outputInstr;
	output reg [1 : 0] outputWB;
	//input is_flush;
	initial begin 
		outputPC = 0; 
		outputData1 = 0;  
		outputData2 = 0;  
		outputImm = 0;  
		outputInstr = 0;  
		outputWB = 0;
	end

	always @(negedge clk) begin
		//if(is_flush ==0) begin
			outputPC <= inputPC; 
			outputData1 <= inputData1;  
			outputData2 <= inputData2;  
			outputImm <= inputImm;  
			outputInstr <= inputInstr;  
			outputWB <= inputWB;
	//	end
	end

endmodule

module EXMEM(clk, inputPC, inputALUOUT, inputB, inputWB, outputB, outputALUOUT, outputPC, outputWB, inputWWD, outputWWD, inputInstr_EXMEM, outputInstr_EXMEM);
	input clk;
	input [15:0] inputPC, inputALUOUT, inputB, inputInstr_EXMEM; 
	input [1 : 0] inputWB;
	output reg [15:0] outputB, outputALUOUT, outputPC, outputInstr_EXMEM;
	output reg [1 : 0] outputWB;

	input [15:0] inputWWD;
	output reg [15:0]outputWWD;

	wire [3 : 0] opcode;
	wire [5 : 0] func_code;

	assign opcode = inputInstr_EXMEM[15 : 12];
	assign func_code = inputInstr_EXMEM[5 : 0];

	//	input is_flush;
	initial begin
		outputB = 0;
		outputALUOUT = 0;
	end
	always@(negedge clk) begin
		outputB <= inputB;
		// outputALUOUT <= inputALUOUT;
		outputPC <= inputPC;
		outputWB <= inputWB;
		outputWWD <= inputWWD;
		outputInstr_EXMEM <= inputInstr_EXMEM;
		if(opcode == `JAL_OP || (opcode == `JRL_OP && func_code == `INST_FUNC_JRL)) begin
			outputALUOUT <= inputPC + 16'b1; 
		end
		else begin
			outputALUOUT <= inputALUOUT;
		end
	end
endmodule

module MEMWB(clk, inputReadData, inputALUResult, inputWB, outputReadData, outputALUResult, outputWB, inputWWD, outputWWD
	, outputPC_EXMEM, outputPC_WB,  outputInstr_EXMEM, outputInstr_MEMWB);
	input clk;
	input [15:0] inputReadData, inputALUResult;
	input [1 : 0] inputWB;
	output reg [15:0] outputReadData, outputALUResult;
	output reg [1 : 0]outputWB;
	input [15:0] outputPC_EXMEM;
	input [15:0] inputWWD;
	input  [15:0] outputInstr_EXMEM;
	output reg [15:0] outputInstr_MEMWB;
	output reg [15:0] outputWWD;
	output reg [15:0] outputPC_WB;
	initial begin
		outputReadData = 0;
		outputALUResult = 0;
		outputWB = 0;
		outputPC_WB = 0;
		outputInstr_MEMWB = 0;
	end

	always@(negedge clk) begin
		outputReadData <= inputReadData;
		outputALUResult <= inputALUResult;
		outputWB <= inputWB;
		outputWWD <= inputWWD;
		outputPC_WB <= outputPC_EXMEM;
		outputInstr_MEMWB <= outputInstr_EXMEM;
	end
endmodule

module IFID_Control (clk, Jsig_IFID_i, Jsig_IFID_o);
	input Jsig_IFID_i, clk;
	output reg Jsig_IFID_o;

	initial begin
		Jsig_IFID_o = 1'b0;
	end

	always @(negedge clk) begin
		Jsig_IFID_o <= Jsig_IFID_i;
	end 
endmodule

module IDEX_Control (clk, pc_write_cond_i, /*pc_write_i,*/ mem_read_i, mem_to_reg_i, mem_write_i, /*ir_write_i,*/ pc_src_i, pc_to_reg_i, halt_i,
		wwd_i, new_inst_i, reg_write_i, alu_op_i, ALUsrc_i, 
		pc_write_cond_o, /*pc_write_o,*/ mem_read_o, mem_to_reg_o, mem_write_o, /*ir_write_o,*/ pc_src_o, pc_to_reg_o, halt_o,
		wwd_o, new_inst_o, reg_write_o, alu_op_o, ALUsrc_o, is_stall, is_stall_o, Jsig_IDEX_i , Jsig_IDEX_o);
	input clk, is_stall, Jsig_IDEX_i;
	input pc_write_cond_i, /*pc_write_i,*/ mem_read_i, mem_to_reg_i, mem_write_i, /*ir_write_i,*/ pc_to_reg_i, halt_i, wwd_i, new_inst_i, reg_write_i, alu_op_i, ALUsrc_i;
	output reg pc_write_cond_o, /*pc_write_o,*/ mem_read_o, mem_to_reg_o, mem_write_o, /*ir_write_o,*/ pc_to_reg_o, halt_o, wwd_o, new_inst_o, reg_write_o, alu_op_o, ALUsrc_o;
	input [1:0] pc_src_i;
	output reg [1:0] pc_src_o;
	output reg is_stall_o, Jsig_IDEX_o;
	always @(negedge clk) begin
		pc_write_cond_o <= pc_write_cond_i;
		// pc_write_o <= pc_write_o; 
		mem_read_o <= mem_read_i;
		mem_to_reg_o <= mem_to_reg_i; 
		mem_write_o <= mem_write_i; 
		// ir_write_o <= ir_write_i; 
		pc_src_o <= pc_src_i; 
		pc_to_reg_o <= pc_to_reg_i; 
		halt_o <= halt_i; 
		wwd_o <= wwd_i; 
		new_inst_o <= new_inst_i; 
		reg_write_o <= reg_write_i; 
		alu_op_o <= alu_op_i; 
		ALUsrc_o <= ALUsrc_i;
		is_stall_o <=is_stall;
		Jsig_IDEX_o <= Jsig_IDEX_i;
	end

endmodule

module EXMEM_Control(clk, pc_write_cond_i, /*pc_write_i,*/ i_or_d_i, mem_read_i, mem_to_reg_i,
			mem_write_i, /*ir_write_i,*/ pc_to_reg_i, pc_src_i, halt_i, wwd_i, new_inst_i, reg_write_i,
			pc_write_cond_o, /*pc_write_o,*/ i_or_d_o, mem_read_o, mem_to_reg_o,
			mem_write_o, /*ir_write_o,*/ pc_to_reg_o, pc_src_o, halt_o, wwd_o, new_inst_o, reg_write_o, Jsig_EXMEM_i, Jsig_EXMEM_o);
	input clk;
	input pc_write_cond_i, /*pc_write_i,*/ i_or_d_i, mem_read_i, mem_to_reg_i;
	input mem_write_i, /*ir_write_i,*/ pc_to_reg_i,  halt_i, wwd_i, new_inst_i, reg_write_i, Jsig_EXMEM_i;

	output reg pc_write_cond_o, /*pc_write_o,*/ i_or_d_o, mem_read_o, mem_to_reg_o;
	output reg mem_write_o, /*ir_write_o,*/ pc_to_reg_o,  halt_o, wwd_o, new_inst_o, reg_write_o, Jsig_EXMEM_o;
	input [1:0] pc_src_i;
	output reg [1:0] pc_src_o;
	always@(negedge clk) begin
		pc_write_cond_o <= pc_write_cond_i;
		// pc_write_o <= pc_write_i;
		i_or_d_o <= i_or_d_i;
		mem_read_o <= mem_read_i;
		mem_to_reg_o <= mem_to_reg_i;
		mem_write_o <= mem_write_i;
		// ir_write_o <= ir_write_i;
		pc_to_reg_o <= pc_to_reg_i;
		pc_src_o <= pc_src_i;
		halt_o <= halt_i;
		wwd_o <= wwd_i;
		new_inst_o <= new_inst_i;
		reg_write_o <= reg_write_i;
		Jsig_EXMEM_o <= Jsig_EXMEM_i;
	end
endmodule

module MEMWB_Control(clk, reg_write_o, reg_write_i, new_inst_i, new_inst_o, wwd_i, wwd_o, halt_o, halt_i, mem_to_reg_o, mem_to_reg_i, pc_to_reg_o, pc_to_reg_i, Jsig_MEMWB_i, Jsig_MEMWB_o);
	input clk;
	input reg_write_i, new_inst_i, wwd_i, halt_i, mem_to_reg_i, pc_to_reg_i, Jsig_MEMWB_i;
	output reg reg_write_o, new_inst_o, wwd_o, halt_o, mem_to_reg_o, pc_to_reg_o, Jsig_MEMWB_o;

	always@(negedge clk) begin
		reg_write_o <= reg_write_i;
		halt_o <= halt_i;
		wwd_o <= wwd_i;
		new_inst_o <= new_inst_i;
		mem_to_reg_o <= mem_to_reg_i;
		pc_to_reg_o <= pc_to_reg_i;
		Jsig_MEMWB_o <= Jsig_MEMWB_i;
	end
	
endmodule


module last_signal_pipe(clk, Jsig_last_i, Jsig_last_o);
	input clk, Jsig_last_i;
	output reg Jsig_last_o;

	initial begin
		Jsig_last_o = 0;
	end
	always@(negedge clk) begin
		Jsig_last_o <= Jsig_last_i;
	end	
		
endmodule