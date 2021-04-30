`include "opcodes.v"

module ImmGen(in, out);
    input [`WORD_SIZE - 1 : 0] in;
    output reg [`WORD_SIZE - 1 : 0] out;

    reg [3:0] opcode;

    always @(*) begin
        opcode = in[15 : 12];
        if(opcode == `LHI_OP) begin
            out = {8'b0, in[7 : 0]};
        end

        else if(opcode == `JMP_OP || opcode == `JAL_OP) begin
            out = {4'b0, in[11 : 0]};
        end

        else if(opcode == `ADI_OP) begin
            out = {{8{in[7]}}, in[7 : 0]};
        end

        else if(opcode == `ORI_OP) begin
            out = {8'b0, in[7:0]};
        end

        else if(opcode == `LWD_OP || opcode == `SWD_OP) begin
            out = {{8{in[7]}}, in[7 : 0]};
        end

        else if((opcode == `BNE_OP) || (opcode == `BGZ_OP) || (opcode == `BLZ_OP) || (opcode == `BEQ_OP)) begin
			out = {{8{in[7]}}, in[7 : 0]} + 1;
		end
    end
endmodule