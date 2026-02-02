`include "block.vh"
`include "instr_dec.vh"

module instr_decoder #(parameter data_width = 16)
	(
        input wire clk,

		input wire [31 : 0] instr,
		
		output logic [4 : 0] operation,
		
		output logic [3 : 0] src_a,
		output logic [3 : 0] src_b,
		output logic [3 : 0] src_c,
		output logic [3 : 0] dest,
		
		output logic src_a_reg,
		output logic src_b_reg,
		output logic src_c_reg,

		output logic saturate,
		output logic use_accumulator,
		output logic subtract,
		output logic signedness,
		output logic dest_acc,

		output logic [4 : 0] instr_shift,
		output logic no_shift,
		
		output logic [7 : 0] res_addr,
		
		output logic src_a_needed,
		output logic src_b_needed,
		output logic src_c_needed
	);
	
	localparam operand_type_start_index = 4 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH;
	localparam pms_start_index = operand_type_start_index + 4;

	assign operation 		  = instr[4:0];
	wire instr_format 		  = instr[5];
	assign {src_a_reg, src_a} = instr[10: 6];
	assign {src_b_reg, src_b} = instr[15:11];
	assign no_shift		  	  = instr[31];
	
	assign src_a_needed =
		(operation != `BLOCK_INSTR_NOP 			&&
		 operation != `BLOCK_INSTR_LOAD 		&&
		 operation != `BLOCK_INSTR_MOV_ACC 		&&
		 operation != `BLOCK_INSTR_FRAC_DELAY 	&&
		 operation != `BLOCK_INSTR_LOAD_ACC 	&&
		 operation != `BLOCK_INSTR_SAVE_ACC 	&&
		 operation != `BLOCK_INSTR_CLEAR_ACC 	&&
		 operation != `BLOCK_INSTR_MOV_UACC);
	
	assign src_b_needed = (
		operation == `BLOCK_INSTR_ADD 	||
		operation == `BLOCK_INSTR_SUB 	||
		operation == `BLOCK_INSTR_MUL 	||
		operation == `BLOCK_INSTR_MADD 	||
		operation == `BLOCK_INSTR_CLAMP ||
		operation == `BLOCK_INSTR_MACZ 	||
		operation == `BLOCK_INSTR_MAC 	||
		operation == `BLOCK_INSTR_LINTERP);
	
	assign src_c_needed = (
		operation == `BLOCK_INSTR_MADD 	||
		operation == `BLOCK_INSTR_CLAMP ||
		operation == `BLOCK_INSTR_MACZ 	||
		operation == `BLOCK_INSTR_MAC 	||
		operation == `BLOCK_INSTR_LINTERP);
	
	assign {src_c_reg, src_c} = (instr_format) ? 		5'b0  :  instr[20:16];
	assign dest 			  = (instr_format) ? instr[19:16] :  instr[24:21];
	assign instr_shift		  = (instr_format) ? 		5'b0  :  instr[29:25];
	assign saturate		      = (instr_format) ? 		   1  : ~instr[30];
	assign res_addr		      = (instr_format) ? instr[27:20] :  8'b0;
	
	assign use_accumulator = (
		operation == `BLOCK_INSTR_MACZ 	  ||
		operation == `BLOCK_INSTR_MAC 	  ||
		operation == `BLOCK_INSTR_MOV_ACC ||);
	assign subtract			= (operation == `BLOCK_INSTR_SUB);
	assign signedness 		= 1;
	assign dest_acc 		= (operation == `BLOCK_INSTR_MACZ || operation == `BLOCK_INSTR_MAC);
endmodule
