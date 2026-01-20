`include "block.vh"
`include "instr_dec.vh"

module instr_decoder #(parameter data_width = 16)
	(
		input wire [`BLOCK_INSTR_WIDTH 	- 1 : 0] instr,
		
		output logic [`BLOCK_INSTR_OP_WIDTH - 1 : 0] operation,
		
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_a,
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_b,
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] src_c,
		output logic [`BLOCK_REG_ADDR_WIDTH - 1 : 0] dest,
		
		output logic src_a_reg,
		output logic src_b_reg,
		output logic src_c_reg,
		output logic dest_reg,

		output logic saturate,

		output logic [`SHIFT_WIDTH - 1 : 0] instr_shift,
		
		output logic [`BLOCK_RES_ADDR_WIDTH - 1 : 0] res_addr
	);
	
	assign operation = instr[`BLOCK_INSTR_OP_WIDTH - 1 : 0];
	
	wire instr_format = (operation == `BLOCK_INSTR_DELAY ||
						 operation == `BLOCK_INSTR_SAVE  ||
						 operation == `BLOCK_INSTR_LOAD  ||
						 operation == `BLOCK_INSTR_MOV)
		? `BLOCK_INSTR_FORMAT_B : `BLOCK_INSTR_FORMAT_A;
	
	localparam operand_type_start_index = 4 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH;
	localparam pms_start_index = operand_type_start_index + 5;

	always @(*) begin
		case (instr_format)
			`BLOCK_INSTR_FORMAT_A: begin
				src_a = instr[1 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 0 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
				src_b = instr[2 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 1 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
				src_c = instr[3 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 2 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
				dest  = instr[4 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 3 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];

				src_a_reg = instr[operand_type_start_index + 0];
				src_b_reg = instr[operand_type_start_index + 1];
				src_c_reg = instr[operand_type_start_index + 2];
				dest_reg  = instr[operand_type_start_index + 3];
				
				saturate = ~instr[operand_type_start_index + 4];
				instr_shift = {{(`SHIFT_WIDTH - `BLOCK_PMS_WIDTH){1'b0}}, instr[pms_start_index + `BLOCK_PMS_WIDTH - 1 : pms_start_index]};
				
				res_addr = 0;
			end
			
			`BLOCK_INSTR_FORMAT_B: begin
				src_a = instr[1 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 0 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
				src_b = instr[2 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 1 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
				src_c = 0;
				dest  = instr[3 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH - 1 : 2 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];

				src_a_reg = 0;
				src_b_reg = 0;
				src_c_reg = 0;
				dest_reg  = 0;
				
				saturate = 0;
				instr_shift = 0;
				
				res_addr = instr[3 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH + `BLOCK_RES_ADDR_WIDTH - 1 : 3 * `BLOCK_REG_ADDR_WIDTH + `BLOCK_INSTR_OP_WIDTH];
			end
		endcase
	end
endmodule
