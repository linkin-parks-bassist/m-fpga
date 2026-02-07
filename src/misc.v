`include "instr_dec.vh"

`default_nettype none

module misc_branch_stage_1 #(parameter data_width = 16, parameter n_blocks = 256)
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
		input wire signed [data_width - 1 : 0] arg_a_out,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_b_out,
		input wire signed [data_width - 1 : 0] arg_c_in,
		input wire signed [data_width - 1 : 0] arg_c_out,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		
		input wire [4 : 0] operation_in,
		input wire [4 : 0] operation_out,
		
		input wire saturate_disable_in,
		input wire [4 : 0] shift_in,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,
		
		output reg signed [2 * data_width - 1 : 0] result_out,
		
		input wire [8 : 0] commit_id_in,
		output reg [8 : 0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out
	);

	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	logic signed [2 * data_width - 1 : 0] result;
	
	wire signed [2 * data_width - 1 : 0] upper_acc = {{(data_width){1'b0}}, accumulator_in[2 * data_width - 1 : data_width]};
	wire signed [2 * data_width - 1 : 0] lower_acc = {{(data_width){1'b0}}, accumulator_in[    data_width - 1 :          0]};
	
	wire signed [2 * data_width - 1 : 0] abs = (arg_a_in < 0) ? -arg_a_in : arg_a_in;
	
	wire signed [2 * data_width - 1 : 0] min = (arg_a_in < arg_b_in) ? arg_a_in : arg_b_in;
	wire signed [2 * data_width - 1 : 0] max = (arg_a_in > arg_b_in) ? arg_a_in : arg_b_in;
	
	wire signed [2 * data_width - 1 : 0] lsh = arg_a_in << shift_in;
	wire signed [2 * data_width - 1 : 0] rsh = arg_a_in >> shift_in;
	
	always_comb begin
		case (operation_in)
			`BLOCK_INSTR_MOV_ACC: 	result = accumulator_in >>> shift_in;
			`BLOCK_INSTR_ABS: 		result = abs;
			`BLOCK_INSTR_MIN: 	    result = min;
			`BLOCK_INSTR_MAX: 	    result = max;
			`BLOCK_INSTR_LSH: 		result = lsh;
			`BLOCK_INSTR_RSH: 		result = rsh;
			`BLOCK_INSTR_MOV_UACC: 	result = upper_acc;
			`BLOCK_INSTR_MOV_LACC: 	result = lower_acc;
		endcase
	end

	//wire signed [2 * data_width - 1 : 0] result_sat = (result < sat_min) ? sat_min : ((result > sat_max) ? sat_max : result);

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				block_out <= block_in;
				
				result_out <= result;
				
				dest_out 			 <= dest_in;
				
				commit_id_out		 <= commit_id_in;
				commit_flag_out 	 <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end

endmodule


module misc_branch_stage_2 #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input wire saturate_disable_in,
		input wire [4 : 0] shift_in,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,
		
		input wire signed [2 * data_width - 1 : 0] result_in,
		output reg signed [2 * data_width - 1 : 0] result_out,
		
		input wire [8 : 0] commit_id_in,
		output reg [8 : 0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out
	);
	
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	wire signed [2 * data_width - 1 : 0] result_sat = (result_in < sat_min) ? sat_min : ((result_in > sat_max) ? sat_max : result_in);
	
	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				block_out <= block_in;
				
				result_out <= saturate_disable_in ? result_in : result_sat;
				
				dest_out 			 <= dest_in;
				
				commit_id_out		 <= commit_id_in;
				commit_flag_out 	 <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end

endmodule

module misc_branch #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		
		input wire [4 : 0] operation_in,
		
		input wire saturate_disable_in,
		input wire [4 : 0] shift_in,
		
		input  wire [3 : 0] dest_in,
		output wire [3 : 0] dest_out,
		
		output wire signed [2 * data_width - 1 : 0] result_out,
		
		input  wire [8 : 0] commit_id_in,
		output wire [8 : 0] commit_id_out,
		
		input  wire commit_flag_in,
		output wire commit_flag_out
	);
	
	assign in_ready = ~out_valid_0 | in_ready_1;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid_0 & in_ready_1;
	
	wire in_ready_1;
	reg  out_valid_0;
	wire out_valid_1;
	wire out_ready_1;
	
	reg  [$clog2(n_blocks) - 1 : 0] block_latched;
	wire [$clog2(n_blocks) - 1 : 0] block_1_out;
	
	reg  signed [data_width - 1 : 0] arg_a_latched;
	wire signed [data_width - 1 : 0] arg_a_1_out;
	reg  signed [data_width - 1 : 0] arg_b_latched;
	wire signed [data_width - 1 : 0] arg_b_1_out;
	reg  signed [data_width - 1 : 0] arg_c_latched;
	wire signed [data_width - 1 : 0] arg_c_1_out;
	
	reg signed  [2 * data_width - 1 : 0] accumulator_latched;
	wire signed [2 * data_width - 1 : 0] accumulator_1_out;
	
	reg  [4 : 0] operation_latched;
	wire [4 : 0] operation_1_out;
	
	reg  saturate_disable_latched;
	wire saturate_disable_1_out;
	reg  [4 : 0] shift_latched;
	wire [4 : 0] shift_1_out;
	
	reg  [3 : 0] dest_latched;
	wire [3 : 0] dest_1_out;
	
	wire signed [2 * data_width - 1 : 0] result_1_out;
	
	reg  [8 : 0] commit_id_latched;
	wire [8 : 0] commit_id_1_out;
	
	reg commit_flag_latched;
	wire commit_flag_1_out;
	
	misc_branch_stage_1 #(.data_width(data_width), .n_blocks(n_blocks)) stage_1
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
				
		.in_valid(out_valid_0),
		.in_ready(in_ready_1),
		
		.out_valid(out_valid_1),
		.out_ready(in_ready_2),
		
		.block_in(block_latched),
		.block_out(block_1_out),
		
		.arg_a_in(arg_a_latched),
		.arg_a_out(arg_a_1_out),
		.arg_b_in(arg_b_latched),
		.arg_b_out(arg_b_1_out),
		.arg_c_in(arg_c_latched),
		.arg_c_out(arg_c_1_out),
		
		.accumulator_in(accumulator_latched),
		
		.operation_in(operation_latched),
		.operation_out(operation_1_out),
		
		.saturate_disable_in(saturate_disable_latched),
		.shift_in(shift_latched),
		
		.dest_in(dest_latched),
		.dest_out(dest_1_out),
		
		.result_out(result_1_out),
		
		.commit_id_in(commit_id_latched),
		.commit_id_out(commit_id_1_out),
		
		.commit_flag_in(commit_flag_latched),
		.commit_flag_out(commit_flag_1_out)
	);
	
	wire in_ready_2;
	
	wire out_valid_2;
	wire out_ready_2;
	
	wire [$clog2(n_blocks) - 1 : 0] block_2_out;
	
	wire signed [data_width - 1 : 0] arg_a_out;
	wire signed [data_width - 1 : 0] arg_b_out;
	wire signed [data_width - 1 : 0] arg_c_out;
	
	wire signed [2 * data_width - 1 : 0] accumulator_2_out;
	
	wire [4 : 0] operation_2_out;
	
	wire saturate_disable_2_out;
	wire [4 : 0] shift_2_out;
	
	wire [3 : 0] dest_2_out;
	
	wire signed [2 * data_width - 1 : 0] result_2_out;
	
	wire [8 : 0] commit_id_2_out;
	
	wire commit_flag_2_out;
	
	misc_branch_stage_2 #(.data_width(data_width), .n_blocks(n_blocks)) stage_2
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
				
		.in_valid(out_valid_1),
		.in_ready(in_ready_2),
		
		.out_valid(out_valid),
		.out_ready(out_ready),
		
		.block_in(block_1_out),
		.block_out(block_out),
		
		.arg_a_in(arg_a_1_out),
		.arg_b_in(arg_b_1_out),
		.arg_c_in(arg_c_1_out),
		
		.operation_in(operation_1_out),
		
		.saturate_disable_in(saturate_disable_1_out),
		.shift_in(shift_1_out),
		
		.dest_in(dest_1_out),
		.dest_out(dest_out),
		
		.result_in(result_1_out),
		.result_out(result_out),
		
		.commit_id_in(commit_id_1_out),
		.commit_id_out(commit_id_out),
		
		.commit_flag_in(commit_flag_1_out),
		.commit_flag_out(commit_flag_out)
	);

	always @(posedge clk) begin
		if (reset) begin
			out_valid_0 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid_0 <= 1;
				
				block_latched <= block_in;
				
				arg_a_latched <= arg_a_in;
				arg_b_latched <= arg_b_in;
				arg_c_latched <= arg_c_in;
				
				accumulator_latched <= accumulator_in;
				
				operation_latched <= operation_in;
				
				saturate_disable_latched <= saturate_disable_in;
				shift_latched <= shift_in;
				
				dest_latched <= dest_in;
				
				commit_id_latched <= commit_id_in;
				commit_flag_latched <= commit_flag_in;
			end else if (take_out) begin
				out_valid_0 <= 0;
			end
		end
	end
endmodule

`default_nettype wire
