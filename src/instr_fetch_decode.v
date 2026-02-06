`include "instr_dec.vh"

`include "lut.vh"
`include "core.vh"



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
		
		output reg saturate_disable_out,
		output reg shift_disable_out,
		output reg signedness_out,
		
		output reg [4  : 0] shift_out,
		output reg [11 : 0] res_addr_out,
		
		output reg arg_a_needed_out,
		output reg arg_b_needed_out,
		output reg arg_c_needed_out,
		
		output reg acc_needed_out,
		
		output reg writes_channel_out,
		output reg writes_acc_out,
		output reg commit_flag_out,
		output reg writes_external_out,
		
		output reg [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_out
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
	
	wire saturate_disable;
	wire shift_disable;
	wire signedness;
	
	wire [4 : 0] shift;
	wire [11 : 0] res_addr;
	
	wire arg_a_needed;
	wire arg_b_needed;
	wire arg_c_needed;
	
	wire acc_needed;
	
	wire writes_channel;
	wire writes_acc;
	wire commit_flag;
	wire writes_external;
	
	wire [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch;
	
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
		
		.saturate_disable(saturate_disable),
		.shift_disable(shift_disable),
		.signedness(signedness),
		
		.shift(shift),
		.res_addr(res_addr),
		
		.arg_a_needed(arg_a_needed),
		.arg_b_needed(arg_b_needed),
		.arg_c_needed(arg_c_needed),
		
		.acc_needed(acc_needed),
		
		.writes_channel(writes_channel),
		.writes_acc(writes_acc),
		.commit_flag(commit_flag),
		.writes_external(writes_external),
		
		.branch(branch)
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
					dest_out <= dest;
					
					src_a_reg_out <= src_a_reg;
					src_b_reg_out <= src_b_reg;
					src_c_reg_out <= src_c_reg;
					
					saturate_disable_out <= saturate_disable;
					shift_disable_out <= shift_disable;
					signedness_out <= signedness;
					
					shift_out <= shift;
					res_addr_out <= res_addr;
					
					arg_a_needed_out <= arg_a_needed;
					arg_b_needed_out <= arg_b_needed;
					arg_c_needed_out <= arg_c_needed;
					
					acc_needed_out <= acc_needed;
					
					writes_channel_out <= writes_channel;
					writes_acc_out <= writes_acc;
					commit_flag_out <= commit_flag;
					writes_external_out <= writes_external;
					
					branch_out <= branch;
					
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
