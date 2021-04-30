`include "opcodes.v"
      
module hazard_detect(IFID_IR, IDEX_rd, IDEX_M_mem_read, is_stall);

	input [`WORD_SIZE-1:0] IFID_IR;
	input [1:0]  IDEX_rd;
	input IDEX_M_mem_read;

	output reg is_stall;

	//TODO: implement hazard detection unit
	//since we have forwarding, we only need hazard for LWD
	//my new regs
	reg [1:0] rs1, rs2;
	initial begin
		rs1 = 0;
		rs2 = 0;
		is_stall = 0;
	end

	always@(*) begin
		rs1 = IFID_IR[11:10];
		rs2 = IFID_IR[9:8];
		if(IDEX_M_mem_read == 1) begin
			if(IDEX_rd == rs1 || IDEX_rd == rs2)
				is_stall = 1;
		end

	end


endmodule