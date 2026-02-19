`include "instr_dec.vh"

`default_nettype none

module misc_branch_stage_1 #(parameter data_width = 16, parameter n_blocks = 256, parameter full_width = full_width + 8)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
				
		input  wire in_valid,
		output wire in_ready,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		input wire signed [full_width - 1 : 0] accumulator_in,
		
		input wire [8 : 0] operation_in,
		input wire [4 : 0] operation_out,

		input wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_in,
		output reg [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out,
		
		output reg signed [full_width - 1 : 0] acc_shift_out,
		
		output reg signed [data_width - 1 : 0] upper_acc_out,
		output reg signed [data_width - 1 : 0] lower_acc_out,
		
		output reg signed [data_width - 1 : 0] abs_out,
		
		output reg signed [data_width - 1 : 0] min_out,
		output reg signed [data_width - 1 : 0] max_out,
		
		output reg signed [data_width - 1 : 0] lsh_out,
		output reg signed [data_width - 1 : 0] rsh_out,
		
		output reg signed [data_width - 1 : 0] clamp_min_out,
		output reg signed [data_width - 1 : 0] clamp_max_out,
		
		output reg signed [data_width - 1 : 0] clamp_out,
		
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		input wire [4 : 0] shift_in,
		output reg [4 : 0] shift_out,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,
		
		input wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_in,
		output reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out
	);

	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;

	wire signed [data_width - 1 : 0] lsh_1 = shift_in[3] ? (arg_a_in << 8) : arg_a_in;
	wire signed [data_width - 1 : 0] lsh_2 = shift_in[2] ? (lsh_1	<< 4) : lsh_1;

	wire [data_width - 1 : 0] rsh_1 = shift_in[3] ? (arg_a_in >> 8) : arg_a_in;
	wire [data_width - 1 : 0] rsh_2 = shift_in[2] ? (rsh_1	>> 4) : rsh_1;

	wire clamp_swap = (arg_c_in < arg_b_in);

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
			
			block_out <= 0;
			misc_op_out <= 0;
			acc_shift_out <= 0;
			upper_acc_out <= 0;
			lower_acc_out <= 0;
			abs_out <= 0;
			min_out <= 0;
			max_out <= 0;
			lsh_out <= 0;
			rsh_out <= 0;
			clamp_min_out <= 0;
			clamp_max_out <= 0;
			clamp_out <= 0;
			saturate_disable_out <= 0;
			shift_out <= 0;
			dest_out <= 0;
			commit_id_out <= 0;
			commit_flag_out <= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				misc_op_out <= misc_op_in;

				block_out <= block_in;
				
				acc_shift_out <= accumulator_in >>> (data_width - 1);
				
				upper_acc_out <= accumulator_in[2 * data_width - 1 : data_width];
				lower_acc_out <= accumulator_in[	data_width - 1 : 	  0    ];

				abs_out <= (arg_a_in < 0) ? -arg_a_in : arg_a_in;

				min_out <= (arg_a_in < arg_b_in) ? arg_a_in : arg_b_in;
				max_out <= (arg_a_in > arg_b_in) ? arg_a_in : arg_b_in;

				lsh_out <= lsh_2;
				rsh_out <= rsh_2;
				
				clamp_min_out <= clamp_swap ? arg_c_in : arg_b_in;
				clamp_max_out <= clamp_swap ? arg_b_in : arg_c_in;
				
				clamp_out <= arg_a_in;
				
				saturate_disable_out <= saturate_disable_in;
				
				shift_out <= shift_in;
				
				dest_out <= dest_in;
				
				commit_id_out		 <= commit_id_in;
				commit_flag_out 	 <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module misc_branch_stage_2 #(parameter data_width = 16, parameter n_blocks = 256, parameter full_width = full_width + 8)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
				
		input  wire in_valid,
		output wire in_ready,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		input wire signed [full_width - 1 : 0] accumulator_in,
		
		input wire [8 : 0] operation_in,
		input wire [4 : 0] operation_out,

		input wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_in,
		output reg [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out,
		
		input wire signed [full_width - 1 : 0] acc_shift_in,
		output reg signed [full_width - 1 : 0] acc_shift_out,
		
		input wire signed [data_width - 1 : 0] upper_acc_in,
		output reg signed [data_width - 1 : 0] upper_acc_out,
		input wire signed [data_width - 1 : 0] lower_acc_in,
		output reg signed [data_width - 1 : 0] lower_acc_out,
		
		input wire signed [data_width - 1 : 0] abs_in,
		output reg signed [data_width - 1 : 0] abs_out,
		
		input wire signed [data_width - 1 : 0] min_in,
		output reg signed [data_width - 1 : 0] min_out,
		input wire signed [data_width - 1 : 0] max_in,
		output reg signed [data_width - 1 : 0] max_out,
		
		input wire signed [data_width - 1 : 0] lsh_in,
		output reg signed [data_width - 1 : 0] lsh_out,
		input wire signed [data_width - 1 : 0] rsh_in,
		output reg signed [data_width - 1 : 0] rsh_out,
		
		input wire signed [data_width - 1 : 0] clamp_min_in,
		input wire signed [data_width - 1 : 0] clamp_max_in,
		
		input wire signed [data_width - 1 : 0] clamp_in,
		output reg signed [data_width - 1 : 0] clamp_out,
		
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		
		input wire [4 : 0] shift_in,
		output reg [4 : 0] shift_out,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,
		
		input wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_in,
		output reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out
	);

	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;

	wire signed [data_width - 1 : 0] lsh_1 = shift_in[1] ? (arg_a_in << 2) : arg_a_in;
	wire signed [data_width - 1 : 0] lsh_2 = shift_in[0] ? (lsh_1	<< 1) : lsh_1;

	wire [data_width - 1 : 0] rsh_1 = shift_in[1] ? (arg_a_in >> 2) : arg_a_in;
	wire [data_width - 1 : 0] rsh_2 = shift_in[0] ? (rsh_1	>> 1) : rsh_1;

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
			
			block_out <= 0;
			misc_op_out <= 0;
			acc_shift_out <= 0;
			upper_acc_out <= 0;
			lower_acc_out <= 0;
			abs_out <= 0;
			min_out <= 0;
			max_out <= 0;
			lsh_out <= 0;
			rsh_out <= 0;
			clamp_out <= 0;
			saturate_disable_out <= 0;
			shift_out <= 0;
			dest_out <= 0;
			commit_id_out <= 0;
			commit_flag_out <= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				misc_op_out <= misc_op_in;

				block_out <= block_in;
				
				acc_shift_out <= acc_shift_in;
				upper_acc_out <= upper_acc_in;
				lower_acc_out <= lower_acc_in;

				abs_out <= abs_in;

				min_out <= min_in;
				max_out <= max_in;

				lsh_out <= lsh_2;
				rsh_out <= rsh_2;
				
				clamp_out <= (clamp_in < clamp_min_in) ? clamp_min_in : ((clamp_in > clamp_max_in) ? clamp_max_in : clamp_in);
				
				saturate_disable_out <= saturate_disable_in;
				
				shift_out <= shift_in;
				
				dest_out 			 <= dest_in;
				
				commit_id_out		 <= commit_id_in;
				commit_flag_out 	 <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule


module misc_branch_stage_3 #(parameter data_width = 16, parameter n_blocks = 256, parameter full_width = full_width + 8)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
				
		input  wire in_valid,
		output wire in_ready,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		input wire [4 : 0] operation_in,

		input wire signed [full_width - 1 : 0] acc_shift_in,
		
		input wire signed [data_width - 1 : 0] upper_acc_in,
		input wire signed [data_width - 1 : 0] lower_acc_in,
		
		input wire signed [data_width - 1 : 0] abs_in,
		
		input wire signed [data_width - 1 : 0] min_in,
		input wire signed [data_width - 1 : 0] max_in,
		
		input wire signed [data_width - 1 : 0] lsh_in,
		input wire signed [data_width - 1 : 0] rsh_in,
		
		input wire signed [data_width - 1 : 0] clamp_in,
		
		output reg signed [full_width - 1 : 0] result_out,

		input wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_in,
		
		input wire saturate_disable_in,
		input wire [4 : 0] shift_in,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,
		
		input wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_in,
		output reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out
	);
	
	localparam signed [full_width - 1 : 0] sat_max = ( 1 << (2 * data_width - 1)) - 1;
	localparam signed [full_width - 1 : 0] sat_min = (-1 << (2 * data_width - 1));
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	wire signed [full_width - 1 : 0] results [`N_MISC_OPS - 1 : 0];
	
	assign results[`BLOCK_INSTR_LSH	  - `MISC_OPCODE_MIN] = lsh_in;
	assign results[`BLOCK_INSTR_RSH	  - `MISC_OPCODE_MIN] = rsh_in;
	assign results[`BLOCK_INSTR_ABS	  - `MISC_OPCODE_MIN] = abs_in;
	assign results[`BLOCK_INSTR_MIN	  - `MISC_OPCODE_MIN] = min_in;
	assign results[`BLOCK_INSTR_MAX	  - `MISC_OPCODE_MIN] = max_in;
	assign results[`BLOCK_INSTR_CLAMP	- `MISC_OPCODE_MIN] = clamp_in;
	assign results[`BLOCK_INSTR_MOV_ACC  - `MISC_OPCODE_MIN] = acc_shift_in;
	assign results[`BLOCK_INSTR_MOV_LACC - `MISC_OPCODE_MIN] = upper_acc_in;
	assign results[`BLOCK_INSTR_MOV_UACC - `MISC_OPCODE_MIN] = lower_acc_in;
	
	wire signed [full_width - 1 : 0] result;
	
	assign result = results[misc_op_in];
	
	wire signed [full_width - 1 : 0] result_sat = (result < sat_min) ? sat_min : ((result > sat_max) ? sat_max : result);
	
	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
			
			block_out <= 0;
			result_out <= 0;
			dest_out <= 0;
			commit_id_out <= 0;
			commit_flag_out <= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				block_out <= block_in;
				
				result_out <= saturate_disable_in ? result : result_sat;
				
				dest_out 			 <= dest_in;
				
				commit_id_out		 <= commit_id_in;
				commit_flag_out 	 <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end

endmodule

module misc_branch #(parameter data_width = 16, parameter n_blocks = 256, parameter full_width = full_width + 8)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
				
		input  wire in_valid,
		output wire in_ready,
		
		output wire out_valid,
		input  wire out_ready,
		
		input  wire [$clog2(n_blocks) - 1 : 0] block_in,
		output wire [$clog2(n_blocks) - 1 : 0] block_out,
		

		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		input wire signed [full_width - 1 : 0] accumulator_in,
		
		input wire [4 : 0] operation_in,
		input wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_in,
		
		input wire saturate_disable_in,
		input wire [4 : 0] shift_in,
		
		input  wire [3 : 0] dest_in,
		output wire [3 : 0] dest_out,
		
		output wire signed [full_width - 1 : 0] result_out,
		
		input  wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_in,
		output wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,
		
		input  wire commit_flag_in,
		output wire commit_flag_out
	);
	
	assign in_ready = ~out_valid_1 | in_ready_1;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid_1 & in_ready_1;
	
	wire in_ready_1;
	wire out_valid_1;
	wire out_ready_1;
	wire [$clog2(n_blocks) - 1 : 0] block_1_out;
	wire [4 : 0] operation_1_out;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_1_out;
	wire signed [full_width - 1 : 0] acc_shift_1_out;
	wire signed [full_width - 1 : 0] upper_acc_1_out;
	wire signed [full_width - 1 : 0] lower_acc_1_out;
	wire signed [data_width - 1 : 0] abs_1_out;
	wire signed [data_width - 1 : 0] min_1_out;
	wire signed [data_width - 1 : 0] max_1_out;
	wire signed [data_width - 1 : 0] lsh_1_out;
	wire signed [data_width - 1 : 0] rsh_1_out;
	wire signed [data_width - 1 : 0] clamp_min_1_out;
	wire signed [data_width - 1 : 0] clamp_max_1_out;
	wire signed [data_width - 1 : 0] clamp_1_out;
	wire saturate_disable_1_out;
	wire shift_disable_1_out;
	wire [4 : 0] shift_1_out;
	wire [3 : 0] dest_1_out;
	wire signed [full_width - 1 : 0] result_1_out;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_1_out;
	wire commit_flag_1_out;
	
	misc_branch_stage_1 #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width)) stage_1
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
				
		.in_valid(in_valid),
		.in_ready(in_ready_1),
		
		.out_valid(out_valid_1),
		.out_ready(in_ready_2),
		
		.block_in(block_in),
		.block_out(block_1_out),
		
		.arg_a_in(arg_a_in),
		.arg_b_in(arg_b_in),
		.arg_c_in(arg_c_in),
		
		.accumulator_in(accumulator_in),
		
		.operation_in(operation_in),
		.operation_out(operation_1_out),
		
		.misc_op_in(misc_op_in),
		.misc_op_out(misc_op_1_out),
		
		.acc_shift_out(acc_shift_1_out),
		.upper_acc_out(upper_acc_1_out),
		.lower_acc_out(lower_acc_1_out),

		.abs_out(abs_1_out),

		.min_out(min_1_out),
		.max_out(max_1_out),

		.lsh_out(lsh_1_out),
		.rsh_out(rsh_1_out),
		
		.clamp_min_out(clamp_min_1_out),
		.clamp_max_out(clamp_max_1_out),
		
		.clamp_out(clamp_1_out),
		
		.saturate_disable_in(saturate_disable_in),
		.saturate_disable_out(saturate_disable_1_out),
		.shift_in(shift_in),
		.shift_out(shift_1_out),
		
		.dest_in(dest_in),
		.dest_out(dest_1_out),
		
		.commit_id_in(commit_id_in),
		.commit_id_out(commit_id_1_out),
		
		.commit_flag_in(commit_flag_in),
		.commit_flag_out(commit_flag_1_out)
	);
	
	
	wire in_ready_2;
	wire out_valid_2;
	wire out_ready_2;
	wire [$clog2(n_blocks) - 1 : 0] block_2_out;
	wire [4 : 0] operation_2_out;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_2_out;
	wire signed [full_width - 1 : 0] acc_shift_2_out;
	wire signed [full_width - 1 : 0] upper_acc_2_out;
	wire signed [full_width - 1 : 0] lower_acc_2_out;
	wire signed [full_width - 1 : 0] abs_2_out;
	wire signed [full_width - 1 : 0] min_2_out;
	wire signed [full_width - 1 : 0] max_2_out;
	wire signed [full_width - 1 : 0] lsh_2_out;
	wire signed [full_width - 1 : 0] rsh_2_out;
	wire signed [	data_width - 1 : 0] clamp_2_out;
	wire saturate_disable_2_out;
	wire shift_disable_2_out;
	wire [4 : 0] shift_2_out;
	wire [3 : 0] dest_2_out;
	wire signed [full_width - 1 : 0] result_2_out;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_2_out;
	wire commit_flag_2_out;
	

	misc_branch_stage_2 #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width)) stage_2
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
				
		.in_valid(out_valid_1),
		.in_ready(in_ready_2),
		
		.out_valid(out_valid_2),
		.out_ready(in_ready_3),
		
		.block_in(block_1_out),
		.block_out(block_2_out),
		
		.arg_a_in(arg_a_in),
		.arg_b_in(arg_b_in),
		.arg_c_in(arg_c_in),
		
		.accumulator_in(accumulator_in),
		
		.operation_in(operation_1_out),
		.operation_out(operation_2_out),
		
		.misc_op_in(misc_op_1_out),
		.misc_op_out(misc_op_2_out),
		
		.acc_shift_in (acc_shift_1_out),
		.acc_shift_out(acc_shift_2_out),
		.upper_acc_in (upper_acc_1_out),
		.upper_acc_out(upper_acc_2_out),
		.lower_acc_in (lower_acc_1_out),
		.lower_acc_out(lower_acc_2_out),

		.abs_in (abs_1_out),
		.abs_out(abs_2_out),

		.min_in (min_1_out),
		.min_out(min_2_out),
		.max_in (max_1_out),
		.max_out(max_2_out),

		.lsh_in (lsh_1_out),
		.lsh_out(lsh_2_out),
		.rsh_in (rsh_1_out),
		.rsh_out(rsh_2_out),
	
		.clamp_min_in(clamp_min_1_out),
		.clamp_max_in(clamp_max_1_out),
		
		.clamp_in(clamp_1_out),
		.clamp_out(clamp_2_out),
		
		.saturate_disable_in(saturate_disable_in),
		.saturate_disable_out(saturate_disable_2_out),
		.shift_in(shift_in),
		.shift_out(shift_2_out),
		
		.dest_in (dest_1_out),
		.dest_out(dest_2_out),
		
		.commit_id_in (commit_id_1_out),
		.commit_id_out(commit_id_2_out),
		
		.commit_flag_in (commit_flag_1_out),
		.commit_flag_out(commit_flag_2_out)
	);
	
	wire in_ready_3;
	wire [$clog2(n_blocks) - 1 : 0] block_3_out;
	wire signed [full_width - 1 : 0] accumulator_3_out;
	wire [4 : 0] operation_3_out;
	wire saturate_disable_3_out;
	wire [4 : 0] shift_3_out;
	wire [3 : 0] dest_3_out;
	wire signed [full_width - 1 : 0] result_3_out;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_3_out;
	wire commit_flag_3_out;
	
	misc_branch_stage_3 #(.data_width(data_width), .n_blocks(n_blocks), .full_width(full_width)) stage_3
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
				
		.in_valid(out_valid_2),
		.in_ready(in_ready_3),
		
		.out_valid(out_valid),
		.out_ready(out_ready),
		
		.block_in(block_2_out),
		.block_out(block_out),
		
		.operation_in(operation_2_out),
		.misc_op_in(misc_op_2_out),
		
		.acc_shift_in(acc_shift_2_out),
		.upper_acc_in(upper_acc_2_out),
		.lower_acc_in(lower_acc_2_out),

		.abs_in(abs_2_out),

		.min_in(min_2_out),
		.max_in(max_2_out),

		.lsh_in(lsh_2_out),
		.rsh_in(rsh_2_out),
		
		.clamp_in(clamp_2_out),
		
		.result_out(result_out),
		
		.saturate_disable_in(saturate_disable_2_out),
		.shift_in(shift_2_out),
		
		.dest_in(dest_2_out),
		.dest_out(dest_out),
		
		.commit_id_in(commit_id_2_out),
		.commit_id_out(commit_id_out),
		
		.commit_flag_in(commit_flag_2_out),
		.commit_flag_out(commit_flag_out)
	);
endmodule

`default_nettype wire
