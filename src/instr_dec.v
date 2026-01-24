`include "block.vh"
`include "instr_dec.vh"

module instr_decoder #(parameter data_width = 16)
	(
        input wire clk,

		input wire [`BLOCK_INSTR_WIDTH 	- 1 : 0] instr,
		
		output logic [`BLOCK_INSTR_OP_WIDTH - 1 : 0] operation,
		
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_a,
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_b,
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_c,
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] dest,
		
		output logic src_a_reg,
		output logic src_b_reg,
		output logic src_c_reg,

		output logic saturate,

		output logic [`SHIFT_WIDTH - 1 : 0] instr_shift,
		
		output logic [`BLOCK_RES_ADDR_WIDTH - 1 : 0] res_addr
	);
	
	localparam operand_type_start_index = 4 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH;
	localparam pms_start_index = operand_type_start_index + 4;

	assign operation 		  = instr[4:0];
	wire instr_format 		  = instr[5];
	assign {src_a_reg, src_a} = instr[10: 6];
	assign {src_b_reg, src_b} = instr[15:11];
	assign {src_c_reg, src_c} = (instr_format) ? 		5'b0  : instr[20:16];
	assign dest 			  = (instr_format) ? instr[19:16] : instr[24:21];
	assign instr_shift		  = (instr_format) ? 		5'b0  : instr[29:25];
	assign saturate			  = (instr_format) ? 		   1  : instr[30];
	assign res_addr			  = (instr_format) ? instr[27:20] : 8'b0;
endmodule
