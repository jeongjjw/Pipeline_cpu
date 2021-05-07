`include "opcodes.v" 
 
 module branch_predictor(/*clk, reset_n, PC, is_flush, is_BJ_type, actual_next_PC, actual_PC, next_PC*/
 						 clk, PC, always_taken_addr, prev_instr, prev_PC, update_taken, next_PC);

	//input clk;
	//input reset_n;
	//input [`WORD_SIZE-1:0] PC;
	//input is_flush;
	//input is_BJ_type;
	//input [`WORD_SIZE-1:0] actual_next_PC; //computed actual next PC from branch resolve stage
	//input [`WORD_SIZE-1:0] actual_PC; // PC from branch resolve stage
	
	input clk;
	input [`WORD_SIZE-1:0] PC;
	input update_taken;
	input [`WORD_SIZE-1:0] always_taken_addr;
	input [`WORD_SIZE-1:0] prev_instr;
	input [`WORD_SIZE-1:0] prev_PC;

	output [`WORD_SIZE-1:0] next_PC;

	reg [23 : 0] BTB [255 : 0];//front 8 bits are Tag, 16 are BTB
	reg [1:0] global_2bit_state;//00: strongly not taken, 01: not taken, 10: taken, 11: strongly taken
	wire check_tag, check_initialize;
	wire [3:0] opcode;
	
	assign opcode = prev_instr[15 : 12];
	
	integer i;

	initial begin
		global_2bit_state = 3;	
		for(i = 0; i <= 255; i = i+1) begin
			BTB[i][23 : 16] = 8'b0;
			BTB[i][15 : 0] = 16'hFFFF;
		end
	end

	wire [7:0] index;
	wire [7:0] tag;
	wire mux_control_wire;
	assign tag = PC[15:8];
	assign index = PC[7:0];
	assign check_tag = (BTB[index][23:16] == tag);
	assign check_initialize = (BTB[index][15:0] != 16'hFFFF);
	assign mux_control_wire =  (check_tag && (global_2bit_state[1] ==1) && check_initialize);

	wire [7:0] prev_index;
	assign prev_index = prev_PC[7:0];

	assign next_PC = (mux_control_wire) ? (BTB[index][15:0]) : (PC + 1);

	//2bit global saturation counter
	always@(posedge clk) begin
		// $display("BTB index : %b", BTB[index][15:0]);
		/* if(update_taken ==1) begin
			if(global_2bit_state !=2)
				global_2bit_state = global_2bit_state + 1;
		end
		if(update_taken ==0) begin
			if(global_2bit_state !=0)
				global_2bit_state = global_2bit_state - 1;
		end */
		
		if(opcode == 4'd0 || opcode == 4'd1 || opcode == 4'd2 || opcode == 4'd3) begin
			BTB[prev_index][15:0] <= always_taken_addr;
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