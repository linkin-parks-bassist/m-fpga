`include "instr_dec.vh"
`include "block.vh"
`include "lut.vh"
`include "seq.vh"
`include "alu.vh"


module instr_fetch_decode_stage #(parameter data_width = 16, parameter n_blocks = 256, parameter n_block_regs = 2)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
	
		input wire sample_tick,
	
		input wire [$clog2(n_blocks) - 1 : 0] n_blocks_running,
		input wire [$clog2(n_blocks) - 1 : 0] last_block,
		output reg [$clog2(n_blocks) - 1 : 0] instr_read_addr,
		
		input wire [31 : 0] instr_read_val,
		
		output reg out_valid,
		input wire out_ready,
		
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		output reg [4 : 0] operation_out,
		
		output reg [3 : 0] src_a_out,
		output reg [3 : 0] src_b_out,
		output reg [3 : 0] src_c_out,
		output reg [3 : 0] dest_out,
		
		output reg src_a_reg_out,
		output reg src_b_reg_out,
		output reg src_c_reg_out,

		output reg saturate_out,
		output reg use_accumulator_out,
		output reg subtract_out,
		output reg signedness_out,
		output reg dest_acc_out,

		output reg [4 : 0] instr_shift_out,
		output reg no_shift_out,
		
		output reg [7 : 0] res_addr_out,
		
		output reg arg_a_needed_out,
		output reg arg_b_needed_out,
		output reg arg_c_needed_out,
		
		output reg [`N_INSTR_BRANCHES - 1 : 0] branch_out,
		output reg commits_out,
		
		output reg ext_write_out
	);
	
	reg [31 : 0] current_instr;
	reg [$clog2(n_blocks) - 1 : 0] current_block;
	reg current_instr_valid;
	reg instr_read_valid_next;
	reg instr_read_valid;
	
	wire [4 : 0] operation;

	wire [3 : 0] src_a;
	wire [3 : 0] src_b;
	wire [3 : 0] src_c;
	wire [3 : 0] dest;

	wire src_a_reg;
	wire src_b_reg;
	wire src_c_reg;

	wire saturate;
	wire use_accumulator;
	wire subtract;
	wire signedness;
	wire dest_acc;

	wire [4 : 0] instr_shift;
	wire no_shift;

	wire [7 : 0] res_addr;

	wire arg_a_needed;
	wire arg_b_needed;
	wire arg_c_needed;
	
	wire [`N_INSTR_BRANCHES - 1 : 0] branch;
	wire commits;
	wire ext_write;
	
	instr_decoder #(.data_width(data_width)) dec
	(
		.instr(current_instr),
		
		.operation(operation),
		
		.src_a(src_a),
		.src_b(src_b),
		.src_c(src_c),
		.dest(dest),
		
		.src_a_reg(src_a_reg),
		.src_b_reg(src_b_reg),
		.src_c_reg(src_c_reg),

		.saturate(saturate),
		.use_accumulator(use_accumulator),
		.subtract(subtract),
		.signedness(signedness),
		.dest_acc(dest_acc),

		.instr_shift(instr_shift),
		.no_shift(no_shift),
		
		.res_addr(res_addr),
		
		.src_a_needed(arg_a_needed),
		.src_b_needed(arg_b_needed),
		.src_c_needed(arg_c_needed),
		
		.branch(branch),
		.commits(commits),
		.ext_write(ext_write)
	);
	
	// if the current output has been taken
	wire out_consumed = out_valid && out_ready;
	// if the current output can be replaced
	wire out_free = out_consumed || !out_valid;
	// if the current instruction can be replaced
	wire current_free = !current_instr_valid || out_consumed;
	
	reg out_valid_next;
	reg instr_read_pending;
	
	always @(posedge clk) begin
		if (reset) begin
			out_valid   <= 0;
			current_block <= 0;
			out_valid_next  <= 0;
			instr_read_addr   <= 0;
			instr_read_valid    <= 0;
			instr_read_pending    <= 1;
			current_instr_valid     <= 0;
			instr_read_valid_next     <= 0;
		end else if (enable) begin
			instr_read_pending <= 0;
			
			if (n_blocks_running > 0) begin
				if (out_valid_next)
					out_valid <= 1;
				
				instr_read_valid <= 1;
				instr_read_pending <= 0;
				
				if (instr_read_val[4:0] == `BLOCK_INSTR_NOP && !instr_read_pending) begin
					instr_read_valid <= 0;
					instr_read_pending <= 1;
					instr_read_addr  <= (instr_read_addr == last_block) ? 0 : instr_read_addr + 1;
				end
				
				if (out_consumed)
					out_valid <= 0;
				
				if (out_free && current_instr_valid) begin
					block_out <= current_block;
				
					operation_out <= operation;
			
					src_a_out <= src_a;
					src_b_out <= src_b;
					src_c_out <= src_c;
					dest_out  <= dest;
					
					src_a_reg_out <= src_a_reg;
					src_b_reg_out <= src_b_reg;
					src_c_reg_out <= src_c_reg;

					saturate_out 		<= saturate;
					use_accumulator_out <= use_accumulator;
					subtract_out 		<= subtract;
					signedness_out 		<= signedness;
					dest_acc_out 		<= dest_acc;

					instr_shift_out <= instr_shift;
					no_shift_out 	<= no_shift;
					
					res_addr_out <= res_addr;
					
					arg_a_needed_out <= arg_a_needed;
					arg_b_needed_out <= arg_b_needed;
					arg_c_needed_out <= arg_c_needed;
					
					branch_out  	<= branch;
					commits_out 	<= commits;
					ext_write_out 	<= ext_write;
					
					out_valid_next <= 1;
				end
				
				if (current_free && instr_read_valid) begin
					current_instr <= instr_read_val;
					current_block <= instr_read_addr;
					current_instr_valid <= 1;
					instr_read_valid <= 0;
					instr_read_pending <= 1;
					instr_read_addr <= (instr_read_addr == last_block) ? 0 : instr_read_addr + 1;
				end
			end
		end
	end
endmodule
