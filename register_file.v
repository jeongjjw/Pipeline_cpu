`include "opcodes.v" 

module register_file (read_out1, read_out2, read1, read2, dest, write_data, reg_write, clk, reset_n);

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

	initial begin
		reg_file[0] = 0;
		reg_file[1] = 0;
		reg_file[2] = 0;
		reg_file[3] = 0;
		read_out1 = 0;
		read_out2 = 0;
	end

	always@(posedge clk)begin
		if(reg_write == 1 && read1 == dest && read2 == dest) begin
			read_out1 <= write_data;
			read_out2 <= write_data;
		end
		else if (reg_write == 1 && read2 == dest && read1 != dest) begin
			read_out2 <= write_data;
			read_out1 <= reg_file[read1];
		end
		else if (reg_write == 1 && read2 != dest && read1 == dest) begin
			read_out1 <= write_data;
			read_out2 <= reg_file[read1];
		end
		else begin
			read_out1 <= reg_file[read1];
			read_out2 <= reg_file[read2];
		end
		// $strobe("reg %d %d %d %d", reg_file[0], reg_file[1], reg_file[2], reg_file[3]);
	end

	always@(posedge clk) begin
		if(reg_write ==1) begin
			reg_file[dest] <= write_data;
		end

	end
    
endmodule

module IFID (clk, inputIR, inputPC, outputIR, outputPC);
	input clk;
	input [15:0] inputIR, inputPC;
	output reg [15:0] outputIR, outputPC;
	initial begin
		outputIR = 0;
		outputPC = 0;
	end
	
	always @(negedge clk) begin
		outputIR <= inputIR;
		outputPC <= inputPC;
	end
	
endmodule

module IDEX (clk, inputPC, inputData1, inputData2, inputImm, inputInstr, inputWB, outputPC, outputData1, outputData2, outputImm, outputInstr, outputWB);
	input clk;
	input [`WORD_SIZE - 1 : 0] inputPC, inputData1, inputData2, inputImm, inputInstr;
	input [1 : 0] inputWB;
	output reg [`WORD_SIZE - 1 : 0] outputPC, outputData1, outputData2, outputImm, outputInstr;
	output reg [1 : 0] outputWB;
	
	initial begin 
		outputPC = 0; 
		outputData1 = 0;  
		outputData2 = 0;  
		outputImm = 0;  
		outputInstr = 0;  
		outputWB = 0;
	end

	always @(negedge clk) begin
		outputPC <= inputPC; 
		outputData1 <= inputData1;  
		outputData2 <= inputData2;  
		outputImm <= inputImm;  
		outputInstr <= inputInstr;  
		outputWB <= inputWB;
	end

endmodule

module EXMEM(clk, inputPC, inputALUOUT, inputB, inputWB, outputB, outputALUOUT, outputPC, outputWB, inputWWD, outputWWD);
	input clk;
	input [15:0] inputPC, inputALUOUT, inputB; 
	input [1 : 0] inputWB;
	output reg [15:0] outputB, outputALUOUT, outputPC;
	output reg [1 : 0] outputWB;

	input [15:0] inputWWD;
	output reg [15:0]outputWWD;
	
	initial begin
		outputB = 0;
		outputALUOUT = 0;
	end
	always@(negedge clk) begin
		outputB <= inputB;
		outputALUOUT <= inputALUOUT;
		outputPC <= inputPC;
		outputWB <= inputWB;
		outputWWD <= inputWWD;
	end
endmodule

module MEMWB(clk, inputReadData, inputALUResult, inputWB, outputReadData, outputALUResult, outputWB, inputWWD, outputWWD);
	input clk;
	input [15:0] inputReadData, inputALUResult;
	input [1 : 0] inputWB;
	output reg [15:0] outputReadData, outputALUResult;
	output reg [1 : 0]outputWB;

	input [15:0] inputWWD;
	output reg [15:0] outputWWD;

	initial begin
		outputReadData = 0;
		outputALUResult = 0;
		outputWB = 0;
	end

	always@(negedge clk) begin
		outputReadData <= inputReadData;
		outputALUResult <= inputALUResult;
		outputWB <= inputWB;
		outputWWD <= inputWWD;
	end
	
endmodule


module IDEX_Control (clk, pc_write_cond_i, pc_write_i, mem_read_i, mem_to_reg_i, mem_write_i, ir_write_i, pc_src_i, pc_to_reg_i, halt_i,
		wwd_i, new_inst_i, reg_write_i, alu_op_i, ALUsrc_i, 
		pc_write_cond_o, pc_write_o, mem_read_o, mem_to_reg_o, mem_write_o, ir_write_o, pc_src_o, pc_to_reg_o, halt_o,
		wwd_o, new_inst_o, reg_write_o, alu_op_o, ALUsrc_o);
	input clk;
	input pc_write_cond_i, pc_write_i, mem_read_i, mem_to_reg_i, mem_write_i, ir_write_i, pc_src_i, pc_to_reg_i, halt_i, wwd_i, new_inst_i, reg_write_i, alu_op_i, ALUsrc_i;
	output reg pc_write_cond_o, pc_write_o, mem_read_o, mem_to_reg_o, mem_write_o, ir_write_o, pc_src_o, pc_to_reg_o, halt_o, wwd_o, new_inst_o, reg_write_o, alu_op_o, ALUsrc_o;

	always @(negedge clk) begin
		pc_write_cond_o <= pc_write_cond_i;
		pc_write_o <= pc_write_o; 
		mem_read_o <= mem_read_i;
		mem_to_reg_o <= mem_to_reg_i; 
		mem_write_o <= mem_write_i; 
		ir_write_o <= ir_write_i; 
		pc_src_o <= pc_src_i; 
		pc_to_reg_o <= pc_to_reg_i; 
		halt_o <= halt_i; 
		wwd_o <= wwd_i; 
		new_inst_o <= new_inst_i; 
		reg_write_o <= reg_write_i; 
		alu_op_o <= alu_op_i; 
		ALUsrc_o <= ALUsrc_i;
	end

endmodule

module EXMEM_Control(clk, pc_write_cond_i, pc_write_i, i_or_d_i, mem_read_i, mem_to_reg_i,
			mem_write_i, ir_write_i, pc_to_reg_i, pc_src_i, halt_i, wwd_i, new_inst_i, reg_write_i,
			pc_write_cond_o, pc_write_o, i_or_d_o, mem_read_o, mem_to_reg_o,
			mem_write_o, ir_write_o, pc_to_reg_o, pc_src_o, halt_o, wwd_o, new_inst_o, reg_write_o);
	input clk;
	input pc_write_cond_i, pc_write_i, i_or_d_i, mem_read_i, mem_to_reg_i;
	input mem_write_i, ir_write_i, pc_to_reg_i, pc_src_i, halt_i, wwd_i, new_inst_i, reg_write_i;

	output reg pc_write_cond_o, pc_write_o, i_or_d_o, mem_read_o, mem_to_reg_o;
	output reg mem_write_o, ir_write_o, pc_to_reg_o, pc_src_o, halt_o, wwd_o, new_inst_o, reg_write_o;
	
	always@(negedge clk) begin
		pc_write_cond_o <= pc_write_cond_i;
		pc_write_o <= pc_write_i;
		i_or_d_o <= i_or_d_i;
		mem_read_o <= mem_read_i;
		mem_to_reg_o <= mem_to_reg_i;
		mem_write_o <= mem_write_i;
		ir_write_o <= ir_write_i;
		pc_to_reg_o <= pc_to_reg_i;
		pc_src_o <= pc_src_i;
		halt_o <= halt_i;
		wwd_o <= wwd_i;
		new_inst_o <= new_inst_i;
		reg_write_o <= reg_write_i;
	end
endmodule

module MEMWB_Control(clk, reg_write_o, reg_write_i, new_inst_i, new_inst_o, wwd_i, wwd_o, halt_o, halt_i, mem_to_reg_o, mem_to_reg_i);
	input clk;
	input reg_write_i, new_inst_i, wwd_i, halt_i, mem_to_reg_i;
	output reg reg_write_o, new_inst_o, wwd_o, halt_o, mem_to_reg_o;

	always@(negedge clk) begin
		reg_write_o <= reg_write_i;
		halt_o <= halt_i;
		wwd_o <= wwd_i;
		new_inst_o <= new_inst_i;
		mem_to_reg_o <= mem_to_reg_i;
	end
	
endmodule