`include "opcodes.v" 
 
 module branch_predictor(/*clk, reset_n, PC, is_flush, is_BJ_type, actual_next_PC, actual_PC, next_PC*/
 						 count_B, clk, PC, always_taken_addr, prev_instr, prev_PC, update_taken, next_PC, branch_stall_signal, data1);

	//input clk;
	//input reset_n;
	//input [`WORD_SIZE-1:0] PC;
	//input is_flush;
	//input is_BJ_type;
	//input [`WORD_SIZE-1:0] actual_next_PC; //computed actual next PC from branch resolve stage
	//input [`WORD_SIZE-1:0] actual_PC; // PC from branch resolve stage
	
	input [3:0] count_B;
	input clk;
	input [`WORD_SIZE-1:0] PC;
	input update_taken;
	input [`WORD_SIZE-1:0] always_taken_addr;
	input [`WORD_SIZE-1:0] prev_instr;
	input [`WORD_SIZE-1:0] prev_PC;
	input branch_stall_signal;
	input [`WORD_SIZE-1:0] data1;

	output [`WORD_SIZE-1:0] next_PC;

	reg [23 : 0] BTB [255 : 0];//front 8 bits are Tag, 16 are BTB
	reg [1:0] global_2bit_state [255 : 0];//00: strongly not taken, 01: not taken, 10: taken, 11: strongly taken
	wire check_tag, check_initialize;
	wire [3:0] opcode;
	
	assign opcode = prev_instr[15 : 12];
	
	integer i;

	initial begin
		// global_2bit_state = 1;	
		for(i = 0; i <= 255; i = i+1) begin
			BTB[i][23 : 16] = 8'b0;
			BTB[i][15 : 0] = 16'hFFFF;
			global_2bit_state[i] = 2'b0;
		end
	end

	wire [7:0] index;
	wire [7:0] tag;
	wire mux_control_wire;
	assign tag = PC[15:8];
	assign index = PC[7:0];
	assign check_tag = (BTB[index][23:16] == tag);
	assign check_initialize = (BTB[index][15:0] != 16'hFFFF);
	assign mux_control_wire =  (check_tag && (global_2bit_state[index][1] ==1) && check_initialize);

	wire [7:0] prev_index;
	assign prev_index = prev_PC[7:0];

	assign next_PC = (mux_control_wire) ? (BTB[index][15:0]) : (PC + 1);

	//2bit global saturation counter
	always@(posedge clk) begin
		// $display("BTB index : %b", BTB[index][15:0]);
		
		if(count_B == 4 || ((opcode == `BNE_OP || opcode == `BEQ_OP || opcode == `BGZ_OP || opcode == `BLZ_OP) && count_B == 0 && branch_stall_signal == 0)) begin
			if(update_taken ==1) begin
				if(global_2bit_state[prev_index] !=3)
					global_2bit_state[prev_index] = global_2bit_state[prev_index] + 1;
			end

			if(update_taken ==0) begin
				if(global_2bit_state[prev_index] !=0)
					global_2bit_state[prev_index] = global_2bit_state[prev_index] - 1;
			end 

			if(opcode == 4'd0 || opcode == 4'd1 || opcode == 4'd2 || opcode == 4'd3) begin
				BTB[prev_index][15:0] <= always_taken_addr;
			end
		end
	end

endmodule

module checkCondition(clk, instr, read_out1, read_out2, condition);
	input clk;
	input [`WORD_SIZE - 1 : 0] instr, read_out1, read_out2;
	output reg condition;
	
	wire [3: 0] opcode;
	assign opcode = instr[15 : 12];

	initial begin
		condition = 0;
	end
	always @(*) begin
		case(opcode)
			`BNE_OP: begin
				if(read_out1 != read_out2)
					condition = 1'b1;
				else
					condition = 1'b0;
			end
			`BEQ_OP: begin
				if(read_out1 == read_out2) 
					condition = 1'b1;
				else 
					condition = 1'b0;
			end
			`BGZ_OP: begin
				if($signed(read_out1) > 0) 
					condition = 1'b1;
				else
					condition = 1'b0;
			end
			`BLZ_OP: begin
				if($signed(read_out1) < 0) 
					condition = 1'b1;
				else
					condition = 1'b0;
			end
		endcase
	end
endmodule


module calc_correct(clk, instr, bcond, Imm, PC, correctPC, always_taken_addr );
	input clk;
	input bcond;
	input [15:0] instr;
	input [15:0] Imm, PC;
	output reg [15:0] correctPC;
	output reg [`WORD_SIZE - 1: 0] always_taken_addr;

	initial begin
		correctPC = 16'b0;
		always_taken_addr = 16'b0;
	end

	wire [3:0] opcode;
	assign opcode = instr[15 : 12];
	always@(*/*posedge clk*/) begin
		if(opcode == `BNE_OP || opcode == `BEQ_OP || opcode == `BGZ_OP || opcode == `BLZ_OP) begin
			if(bcond == 1) begin
				correctPC = PC + Imm;
			end
			else begin
				correctPC = PC + 1;
			end

			always_taken_addr = PC + Imm;
		end
		

	end
endmodule


module branch_sig(clk, predictPC, correctPC, branch_signal, instr);
	input clk;
	input [`WORD_SIZE - 1 : 0] predictPC;
	input [`WORD_SIZE - 1 : 0] correctPC;
	input [15:0] instr;
	output reg branch_signal;

	initial begin
		branch_signal = 0;
	end

	wire [3:0] opcode;
	assign opcode = instr[15 : 12];

	always @(posedge clk) begin
		if(opcode == `BNE_OP || opcode == `BEQ_OP || opcode == `BGZ_OP || opcode == `BLZ_OP) begin
			if(predictPC == correctPC) begin
				branch_signal <= 1'b0;
			end
			else begin
				branch_signal <= 1'b1;
				// branch_signal <= 1'b0;
			end	
		end
		else begin
			branch_signal <= 1'b0;
		end
	end
endmodule


module branch_stall(data1, reg_write_o, inputWB_EXMEM, reg_write_o_E, inputWB_MEMWB, reg_write_o_M, outputWB_MEMWB, read1, read2, count_b, branch_stall);
	input [`WORD_SIZE - 1 : 0] data1;
	input reg_write_o, reg_write_o_E, reg_write_o_M;
	input [1 : 0] inputWB_EXMEM, inputWB_MEMWB, outputWB_MEMWB, read1, read2;
	input [3 : 0] count_b;
	output reg branch_stall;

	wire [3 : 0] opcode;
	assign opcode = data1[15 : 12];

	initial begin
		branch_stall = 1'b0;
	end

	always @(*) begin
		if(count_b == 0) begin
			case(opcode)
				`BNE_OP, `BEQ_OP: begin
					if(reg_write_o == 1'b1 && (read1 == inputWB_EXMEM || read2 == inputWB_EXMEM)) begin
						branch_stall = 1'b1;
					end
					else if (reg_write_o_E == 1'b1 && (read1 == inputWB_MEMWB || read2 == inputWB_MEMWB)) begin
						branch_stall = 1'b1;
					end
					else if (reg_write_o_M == 1'b1 && (read1 == outputWB_MEMWB || read2 == outputWB_MEMWB)) begin
						branch_stall = 1'b1;
					end
					else begin
						branch_stall = 1'b0;
					end
				end
				`BGZ_OP, `BLZ_OP: begin
					if(reg_write_o == 1'b1 && read1 == inputWB_EXMEM) begin
						branch_stall = 1'b1;
					end
					else if (reg_write_o_E == 1'b1 && read1 == inputWB_MEMWB) begin
						branch_stall = 1'b1;
					end
					else if (reg_write_o_M == 1'b1 && read1 == outputWB_MEMWB) begin
						branch_stall = 1'b1;
					end
					else begin
						branch_stall = 1'b0;
					end
				end
				default:
					branch_stall = 1'b0;
			endcase
		end
		else begin
			branch_stall = 1'b0;
		end
	end
endmodule


module JPR_JRL_Stall(data1, reg_write_o, inputWB_EXMEM, reg_write_o_E, inputWB_MEMWB, reg_write_o_M, outputWB_MEMWB, count_J, JPR_JRL_stall_signal, read1, count_J_limit);
	input [`WORD_SIZE - 1 : 0] data1;
	input reg_write_o, reg_write_o_E, reg_write_o_M;
	input [1 : 0] inputWB_EXMEM, inputWB_MEMWB, outputWB_MEMWB, read1;
	input [3 : 0] count_J;
	output reg JPR_JRL_stall_signal;
	output reg [3 : 0] count_J_limit;

	wire [3 : 0] opcode;
	wire [5 : 0] funcCode;
	assign opcode = data1[15 : 12];
	assign funcCode = data1[5 : 0];

	initial begin
		JPR_JRL_stall_signal = 1'b0;
		count_J_limit = 3;
	end

	always @(*) begin
		if(count_J == 0) begin
			if (opcode == 15 && (funcCode == `INST_FUNC_JPR || funcCode == `INST_FUNC_JRL)) begin
				if(reg_write_o == 1'b1 && read1 == inputWB_EXMEM) begin
					JPR_JRL_stall_signal = 1'b1;
					count_J_limit = 3;
				end
				else if (reg_write_o_E == 1'b1 && read1 == inputWB_MEMWB) begin
					JPR_JRL_stall_signal = 1'b1;
					count_J_limit = 3;
				end
				else if (reg_write_o_M == 1'b1 && read1 == outputWB_MEMWB) begin
					JPR_JRL_stall_signal = 1'b1;
					count_J_limit = 3;
				end
				else begin
					JPR_JRL_stall_signal = 1'b0;
					count_J_limit = 3;
				end
			end
		end
		else begin
			JPR_JRL_stall_signal = 1'b0;
		end
	end

endmodule