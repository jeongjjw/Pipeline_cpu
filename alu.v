`include "opcodes.v"

`define NumBits 16

module alu (A, B, func_code, branch_type, C, overflow_flag, bcond);
   input [`NumBits-1:0] A; //input data A.
   input [`NumBits-1:0] B; //input data B.
   input [2:0] func_code; //function code for the operation
   input [1:0] branch_type; //branch type for bne, beq, bgz, blz
   output reg [`NumBits-1:0] C; //output data C
   output reg overflow_flag; 
   output reg bcond; //1 if branch condition met, else 0

initial begin
	overflow_flag = 0;
	bcond = 0;
	C = 0;
end
//add funccode for aluop = 0, just let data flow
//TODO: implement ALU
	always@(*) begin
		//bcond =0;
		overflow_flag = 0;
		case(func_code)
			`FUNC_ADD: begin
				C  = $signed(A) + $signed(B);
				overflow_flag = (A[15]==B[15]) && (A[15]!=C[15]);
			end
			`FUNC_SUB: begin
				C  = $signed(A) - $signed(B);
				overflow_flag = (A[15]!=B[15]) && (A[15]!=C[15]);
			end
			`FUNC_AND: C  = A & B;
			`FUNC_ORR: C  = A | B;
			`FUNC_NOT: C  = ~A; //NOT
			`FUNC_TCP: C  = ~$signed(A) + 1;
			`FUNC_SHL: C  = A << 1; //SHL
			`FUNC_SHR: C = A >> 1; //SHR
		    // : C  = B << 8; //LHI
			//4'b1001: C = A; // let value just flow, no operations
			//4'b1010: C = B; // let value B flow
		endcase

		case(branch_type)
			2'b00:	bcond = (A!=B); //BNE
			2'b01: bcond = (A==B); //BEQ
			2'b10: bcond = ($signed(A)>0); //BGZ
			2'b11: bcond = ($signed(A)<0); //BLZ
		endcase
		//branch calculation
		//if(branch_type ==0)$display("yes %d", branch_type);
	end
   
endmodule