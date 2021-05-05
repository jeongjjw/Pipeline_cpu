`include "opcodes.v"
      
module hazard_detect(clk, IFID_IR, IDEX_rd, IDEX_M_mem_read, is_stall, pc_write, ir_write);

	input clk;
	input [`WORD_SIZE-1:0] IFID_IR;
	input [1:0]  IDEX_rd;
	input IDEX_M_mem_read;


	output reg is_stall;
	output reg pc_write, ir_write;
	//TODO: implement hazard detection unit
	//since we have forwarding, we only need hazard for LWD
	//my new regs
	reg pc_write_t;
	reg [1:0] rs1, rs2;
	initial begin
		rs1 = 0;
		rs2 = 0;
		is_stall = 0;
		pc_write = 1;
		ir_write = 1;
		
	end

	wire [3 : 0] opcode;
	wire [5 : 0] funccode;
	assign opcode = IFID_IR[15 : 12];
	assign funccode = IFID_IR[5 : 0];

	always @(posedge clk) begin
		if(is_stall == 0) begin
			pc_write <= 1;
			ir_write <= 1;
		end
		else begin
			pc_write <= 0;
			ir_write <= 0;
		end
	end

	always@(*) begin
		rs1 = IFID_IR[11:10];
		rs2 = IFID_IR[9:8];
		/*if((IDEX_M_mem_read == 1) && (IDEX_rd == rs1 || IDEX_rd == rs2)) begin
			is_stall = 1;
			ir_write = 0;
			pc_write = 0;
		end
		else begin
			is_stall = 0;
			ir_write = 1;
			pc_write = 1;
		end*/
		/*
		if((IFID_IR[5:0] == 25 || IFID_IR[5:0] == 26) && IFID_IR[15:12] == 15) begin
			is_stall = 1;
			ir_write = 0;
			pc_write = 0;
		end
		else begin
			is_stall = 0;
			ir_write = 1;
			pc_write = 1;
		end*/
		case(opcode)
			`ALU_OP: begin
				case(funccode)
					`INST_FUNC_ADD, `INST_FUNC_SUB, `INST_FUNC_AND, `INST_FUNC_ORR: begin
						if((IDEX_M_mem_read == 1) && (IDEX_rd == rs1 || IDEX_rd == rs2)) begin
							is_stall = 1;
							// ir_write = 0;
							// pc_write = 0;
						end
						else begin
							is_stall = 0;
							// ir_write = 1;
							// pc_write = 1;
						end
					end
					`INST_FUNC_NOT, `INST_FUNC_TCP, `INST_FUNC_SHL, `INST_FUNC_SHR, `INST_FUNC_JPR, `INST_FUNC_JRL, `INST_FUNC_WWD: begin
						if((IDEX_M_mem_read == 1) && (IDEX_rd == rs1)) begin
							is_stall = 1;
							// ir_write = 0;
							// pc_write = 0;
						end
						else begin
							is_stall = 0;
							// ir_write = 1;
							// pc_write = 1;
						end
					end
				endcase
			end
			`ADI_OP, `ORI_OP, `LHI_OP, `LWD_OP, `SWD_OP: begin
				if((IDEX_M_mem_read == 1) && (IDEX_rd == rs1)) begin
					is_stall = 1;
					// ir_write = 0;
					// pc_write = 0;
				end
				else begin
					is_stall = 0;
					// ir_write = 1;
					// pc_write = 1;
				end
			end
			`BNE_OP, `BEQ_OP, `BGZ_OP, `BLZ_OP: begin
				if((IDEX_M_mem_read == 1) && (IDEX_rd == rs1 || IDEX_rd == rs2)) begin
					is_stall = 1;
					// ir_write = 0;
					// pc_write = 0;
				end
				else begin
					is_stall = 0;
					//vir_write = 1;
					//vpc_write = 1;
				end
			end
		endcase
 		/*
		if(is_stall ==0) begin
			pc_write = 1;
			ir_write = 1;
		end
		*/
	end

	/*always@(*) begin
		if(is_stall == 1 && pc_write == 1) begin
			is_stall = 0;
		end
		else if(is_stall == 1) begin
			pc_write = 1;
			ir_write = 0;
			is_stall = 1;
		end
		else if(is_stall ==0) begin
			//pc_write <= 1;
			ir_write = 1;
		end
	end	*/


endmodule