`include "opcodes.v" 

module control_unit (opcode, func_code, clk, reset_n, pc_write_cond, /* pc_write, i_or_d,*/ mem_read, mem_to_reg, mem_write, /*ir_write,*/ pc_to_reg, pc_src, halt, wwd, new_inst, reg_write, alu_op, ALUSrc);

	input [3:0] opcode;
	input [5:0] func_code;

	input clk;
	input reset_n;
	

	output reg pc_write_cond, /*pc_write,*/ mem_read, mem_to_reg, mem_write /*ir_write,*/;
	output reg [1:0] pc_src;
	//unused regs
	//output reg i_or_d;
	//output reg alu_src_A, alu_src_B;
  	//additional control signals. pc_to_reg: to support JAL, JRL. halt: to support HLT. wwd: to support WWD. new_inst: new instruction start
  	output reg pc_to_reg, halt, wwd, new_inst;
  	output reg reg_write;
  	output reg alu_op;
	output reg ALUSrc;

	initial begin
		pc_write_cond = 0;
		//pc_write = 0;
		mem_read = 1;//for first instruction read
		mem_to_reg = 0;
		mem_write = 0;
		//ir_write = 0;
		pc_src = 0;
		pc_to_reg= 0;
		halt = 0;
		wwd = 0;
		new_inst = 0;
		reg_write = 0;
		alu_op = 0;
		ALUSrc = 0;
	end

	//TODO : implement control unit
    always@(*) begin
		//pc_write = 1;
		pc_src = 0;
		halt = 0; 
		case(opcode)
		`ALU_OP: begin//ALU, HLT, JPR, JRL, WWD
			// EX
			ALUSrc = 1'b0;
			// MEM
			mem_write = 1'b0; 
			mem_read = 1'b0;
			if(func_code == `INST_FUNC_WWD) begin
				wwd = 1'b1;
				reg_write = 1'b0;
			end
			else if(func_code == `INST_FUNC_HLT ) begin
				halt = 1;
				reg_write = 1'b0;
			end
			else begin
				wwd = 1'b0;
				reg_write = 1'b1;
				halt = 0;
			end		
			// WB
			mem_to_reg = 1'b0;
			new_inst = 1'b1;
		end
		`ADI_OP, `ORI_OP, `LHI_OP: begin
			// EX
			ALUSrc = 1'b1;
			// MEM
			mem_write = 1'b0; 
			mem_read = 1'b0;
			wwd = 1'b0;
			// WB
			mem_to_reg = 1'b0;
			reg_write = 1'b1;
			new_inst = 1'b1;
		end
		`LWD_OP: begin
			// EX
			ALUSrc = 1'b1;
			// MEM
			mem_write = 1'b0; 
			mem_read = 1'b1;
			wwd = 1'b0;
			// WB
			mem_to_reg = 1'b1;
			reg_write = 1'b1;
			new_inst = 1'b1;
		end
		`SWD_OP: begin
			// EX
			ALUSrc = 1'b1;
			// MEM
			mem_write= 1'b1; 
			mem_read = 1'b0;
			wwd = 1'b0;
			// WB
			mem_to_reg = 1'b0;
			reg_write = 1'b0;
			new_inst = 1'b1;
		end
		`BNE_OP, `BEQ_OP, `BGZ_OP, `BLZ_OP: begin
			// EX
			ALUSrc = 1'b0;
			// MEM
			mem_write = 1'b0; 
			mem_read = 1'b0;
			wwd = 1'b0;
			// WB
			mem_to_reg = 1'b0;
			reg_write = 1'b0;
			new_inst = 1'b1;
			
			pc_src = 2;
		end 
		`JMP_OP: begin
			// EX
			ALUSrc = 1'b0;
			// MEM
			mem_write = 1'b0; 
			mem_read = 1'b0;
			wwd = 1'b0;
			// WB
			mem_to_reg = 1'b0;
			reg_write = 1'b0;
			new_inst = 1'b1;
			
			//for jump instr, we need to write new pc value
			pc_src = 1;
		end
		`JAL_OP: begin
			// EX
			ALUSrc = 1'b0;
			// MEM
			mem_write = 1'b0; 
			mem_read = 1'b0;
			wwd = 1'b0;
			// WB
			mem_to_reg = 1'b0;
			reg_write = 1'b0;
			new_inst = 1'b1;

			pc_src = 1;
		end
		endcase
	end
endmodule


module alu_control_unit(func_code, opcode, ALUOp, clk, funcCode, branchType);
	input [1:0] ALUOp;
 	input clk;
  	input [3:0] opcode;//size can change
  	input [5:0] func_code;//size can change

  	output reg [3:0] funcCode;
  	output reg [1:0] branchType;

	always@(*) begin
		funcCode = 0;
		case(opcode)	
			`ALU_OP: begin 
				branchType = 2'b0;
				funcCode[3] = 0;
				case(func_code)
					`INST_FUNC_ADD:
						funcCode[2:0] = `FUNC_ADD;
					`INST_FUNC_SUB:
						funcCode[2:0] = `FUNC_SUB;
					`INST_FUNC_AND:
						funcCode[2:0] = `FUNC_AND;
					`INST_FUNC_ORR:
						funcCode[2:0] = `FUNC_ORR;
					`INST_FUNC_NOT:
						funcCode[2:0] = `FUNC_NOT;
					`INST_FUNC_TCP:
						funcCode[2:0] = `FUNC_TCP;
					`INST_FUNC_SHL:
						funcCode[2:0] = `FUNC_SHL;
					`INST_FUNC_SHR:	
						funcCode[2:0] = `FUNC_SHR;
					`INST_FUNC_WWD:
						funcCode = 4'b1001;
				endcase 
			end	
			`ADI_OP: begin
				branchType = 2'b0;
				funcCode[2:0] = `FUNC_ADD;
			end
			`ORI_OP: begin
				branchType = 2'b0;
				funcCode[2:0] = `FUNC_ORR;
			end
			`LHI_OP: begin
				branchType = 2'b0;
				funcCode = 4'b1000;
			end 
			`LWD_OP, `SWD_OP: begin
				branchType = 2'b0;
				funcCode = 4'b1010;
			end
			`BNE_OP: begin
				branchType = 2'b00;
				funcCode = `FUNC_ADD;
			end
			`BEQ_OP: begin
				branchType = 2'b01;
				funcCode = `FUNC_ADD;
			end
			`BGZ_OP: begin
				branchType = 2'b10;
				funcCode = `FUNC_ADD;
			end
			`BLZ_OP: begin
				branchType = 2'b11;
				funcCode = `FUNC_ADD;
			end
			`JMP_OP, `JAL_OP, `JPR_OP, `JRL_OP: begin
				branchType = 2'b0;
				funcCode = `FUNC_ADD;
			end
		endcase
	end
endmodule

module forwarding_unit(clk, forward_A, forward_B, rs1, rs2, WB_EXMEM, WB_MEMWB, rd_EXMEM, rd_MEMWB);
	input [1:0] rs1, rs2, rd_EXMEM, rd_MEMWB;
	input WB_EXMEM, WB_MEMWB, clk;//WB is RegWrite
	output reg [1:0] forward_A, forward_B;

	initial begin
		forward_A = 0;
		forward_B = 0;
	end
	always@(posedge clk) begin
		//forward_A = 0;
		//forward_B = 0;
		if((rs1== rd_EXMEM) && (WB_EXMEM == 1'b1)) begin//dist ==1 rs1!=0 condition not used for TSC?
			forward_A <= 2;
		end
		else if ((rs1== rd_MEMWB) && (WB_MEMWB == 1'b1)) begin//dist ==2
			forward_A <= 1;
		end	
		else
			forward_A <= 0;	

		if((rs2== rd_EXMEM) && (WB_EXMEM == 1'b1)) begin//rs1!=0 condition not used for TSC?
			forward_B <= 2;
		end
		else if ((rs2== rd_MEMWB) && (WB_MEMWB == 1'b1)) begin
			forward_B <= 1;
		end	
		else
			forward_B <= 0;
	end
	
endmodule 
