`default_nettype none

module multiply_stage #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input wire [4:0] shift_in,
		output reg [4:0] shift_out,
		input wire shift_disable_in,
		output reg shift_disable_out,
		input wire signedness_in,
		output reg signedness_out,
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		
		input wire signed [    data_width - 1 : 0] arg_a_in,
		input wire signed [    data_width - 1 : 0] arg_b_in,
		output reg signed [2 * data_width - 1 : 0] product_out,
		
		input wire signed [data_width - 1 : 0] arg_c_in,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		input wire [3:0] dest_in,
		output reg [3:0] dest_out,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out,

		input wire commit_flag_in,
		output reg commit_flag_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				block_out <= block_in;
				
				shift_out 			 <= shift_in;
				shift_disable_out 	 <= shift_disable_in;
				saturate_disable_out <= saturate_disable_in;
				signedness_out 		 <= signedness_in;
				
				product_out 		 <= signedness_in ? $signed(arg_a_in) * $signed(arg_b_in) : arg_a_in * arg_b_in;
				arg_c_out 			 <= arg_c_in;
				
				dest_out 			 <= dest_in;
				
				commit_id_out		 <= commit_id_in;
				commit_flag_out 	 <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module shift_stage #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input wire [4:0] shift_in,
		input wire shift_disable_in,
		
		input wire signedness_in,
		output reg signedness_out,
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		
		input wire signed [2 * data_width - 1 : 0] product_in,
		output reg signed [2 * data_width - 1 : 0] product_out,
		
		input wire signed [data_width - 1 : 0] arg_c_in,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		input wire [3:0] dest_in,
		output reg [3:0] dest_out,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out,

		input wire commit_flag_in,
		output reg commit_flag_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	wire [4 : 0] shift = data_width - shift_in - 1;
	wire [2 * data_width - 1 : 0] rounding_bias = (shift == 0) ? 0 : (1 << (shift - 1));
	
	wire [2 * data_width - 1 : 0] result = (shift_disable_in) ? product_in : (product_in + rounding_bias) >>> shift;

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				block_out <= block_in;
				
				saturate_disable_out 	<= saturate_disable_in;
				signedness_out 			<= signedness_in;
				
				product_out 	<= result;
				arg_c_out 		<= arg_c_in;
				
				dest_out 		<= dest_in;
				
				commit_id_out	<= commit_id_in;
				commit_flag_out <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module add_stage #(parameter data_width = 16, parameter n_blocks = 256)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input  wire in_valid,
		output wire in_ready,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		
		input wire signedness_in,
		
		input wire signed [2 * data_width - 1 : 0] product_in,
		input wire signed [    data_width - 1 : 0] arg_c_in,
		
		output reg signed [2 * data_width - 1 : 0] result_out,
		
		input wire [3:0] dest_in,
		output reg [3:0] dest_out,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out,

		input wire commit_flag_in,
		output reg commit_flag_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  =  in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	wire signed [2 * data_width - 1 : 0] arg_c_ext = {{(data_width){signedness_in & arg_c_in[data_width-1]}}, arg_c_in};
	
	wire signed [2 * data_width - 1 : 0] result = product_in + arg_c_ext;

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				block_out <= block_in;
				
				saturate_disable_out 	<= saturate_disable_in;
				result_out		<= result;
				dest_out 		<= dest_in;
				
				commit_id_out	<= commit_id_in;
				commit_flag_out <= commit_flag_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module saturate_stage #(parameter data_width = 16, parameter n_blocks = 256)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input  wire in_valid,
		output wire in_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire saturate_disable_in,
		
		input wire signed [2 * data_width - 1 : 0] result_in,
		output reg signed [2 * data_width - 1 : 0] result_out,
		
		input wire [3:0] dest_in,
		output reg [3:0] dest_out,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out,

		input wire commit_flag_in,
		output reg commit_flag_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  =  in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	wire signed [2 * data_width - 1 : 0] result_in_sat = (result_in > sat_max) ? sat_max : ((result_in < sat_min) ? sat_min : result_in);

	always @(posedge clk) begin
		if (reset) begin
			out_valid <= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= 1;
				
				result_out <= (saturate_disable_in) ? result_in : result_in_sat;
				
				dest_out 	  	<= dest_in;
				
				block_out 	  	<= block_in;
				commit_id_out 	<= commit_id_in;
				commit_flag_out <= commit_flag_in;
				
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module mac_pipeline #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input  wire [4:0] shift,
		input  wire shift_disable,
		input  wire signedness_in,
		output wire signedness_out,
		input  wire saturate_disable_in,
		output wire saturate_disable_out,
		
		input  wire signed [data_width - 1 : 0] arg_a_in,
		input  wire signed [data_width - 1 : 0] arg_b_in,
		input  wire signed [data_width - 1 : 0] arg_c_in,
		output wire signed [data_width - 1 : 0] arg_c_out,
		
		output wire signed [2 * data_width - 1 : 0] result_out,
		
		input  wire [3:0] dest_in,
		output wire [3:0] dest_out,
		
		input  wire [8:0] commit_id_in,
		output wire [8:0] commit_id_out,

		input  wire commit_flag_in,
		output wire commit_flag_out
	);
	
	wire out_valid_muls;
	wire [4:0] shift_out_muls;
	wire shift_disable_out_muls;
	wire saturate_disable_out_muls;
	wire signedness_out_muls;
	wire acc_needed_out_muls;
	wire subtract_out_muls;
	wire signed [data_width - 1 : 0] arg_a_out_muls;
	wire signed [data_width - 1 : 0] arg_b_out_muls;
	wire signed [data_width - 1 : 0] arg_c_out_muls;
	wire signed [2 * data_width - 1 : 0] product_out_muls;
	wire signed [2 * data_width - 1 : 0] accumulator_out_muls;
	wire [3:0] dest_out_muls;
	wire writes_acc_out_muls;
	wire [8:0] commit_id_out_muls;
	wire commit_flag_out_muls;
	
	multiply_stage #(.data_width(data_width)) multiply_stage
		(
			.clk(clk),
			.reset(reset),
			
			.enable(enable),
			
			.in_valid(in_valid),
			.in_ready(in_ready),
			
			.out_valid(out_valid_muls),
			.out_ready(in_ready_shs),
			
			.shift_in(shift),
			.shift_out(shift_out_muls),
			
			.shift_disable_in(shift_disable),
			.shift_disable_out(shift_disable_out_muls),
			
			.signedness_in(signedness_in),
			.signedness_out(signedness_out_muls),
			
			.saturate_disable_in(saturate_disable_in),
			.saturate_disable_out(saturate_disable_out_muls),
			
			.arg_a_in(arg_a_in),
			.arg_b_in(arg_b_in),
			.arg_c_in(arg_c_in),
			
			.product_out(product_out_muls),
			.arg_c_out(arg_c_out_muls),
			
			.dest_in(dest_in),
			.dest_out(dest_out_muls),
			
			.commit_id_in(commit_id_in),
			.commit_id_out(commit_id_out_muls),
			
			.commit_flag_in(commit_flag_in),
			.commit_flag_out(commit_flag_out_muls)
		);
	
	wire in_ready_shs;

	shift_stage #(.data_width(data_width)) shift_stage
		(
			.clk(clk),
			.reset(reset),
			
			.enable(enable),
			
			.in_valid(out_valid_muls),
			.in_ready(in_ready_shs),
			
			.out_valid(out_valid),
			.out_ready(out_ready),
			
			.shift_in(shift_out_muls),
			.shift_disable_in(shift_disable_out_muls),
			
			.saturate_disable_in(saturate_disable_out_muls),
			.saturate_disable_out(saturate_disable_out),
			
			.signedness_in(signedness_out_muls),
			.signedness_out(signedness_out),
			
			.product_in(product_out_muls),
			.product_out(result_out),
			
			.arg_c_in(arg_c_out_muls),
			.arg_c_out(arg_c_out),
			
			.dest_in(dest_out_muls),
			.dest_out(dest_out),
			
			.commit_id_in(commit_id_out_muls),
			.commit_id_out(commit_id_out),
			
			.commit_flag_in(commit_flag_out_muls),
			.commit_flag_out(commit_flag_out)
		);
	
endmodule

module madd_pipeline #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input wire [4:0] shift,
		input wire shift_disable,
		input wire signedness,
		input wire saturate_disable,
		
		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		output wire signed [2 * data_width - 1 : 0] result_out,
		
		input  wire [3:0] dest_in,
		output wire [3:0] dest_out,
		
		input  wire [8:0] commit_id_in,
		output wire [8:0] commit_id_out,

		input wire commit_flag_in,
		output wire commit_flag_out
	);

	wire [$clog2(n_blocks) - 1 : 0] block_out_mac;
	wire in_ready_mac;
	wire out_valid_mac;
	wire signedness_out_mac;
	wire saturate_disable_out_mac;
	wire signed [data_width - 1 : 0] arg_c_out_mac;
	wire signed [2 * data_width - 1 : 0] accumulator_out_mac;
	wire signed [2 * data_width - 1 : 0] result_out_mac;
	wire [8:0] commit_id_out_mac;
	wire [3:0] dest_out_mac;
	wire commit_flag_out_mac;

	mac_pipeline #(.data_width(data_width), .n_blocks(n_blocks)) madd_main
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
		
		.in_valid(in_valid),
		.in_ready(in_ready),
		
		.out_valid(out_valid_mac),
		.out_ready(in_ready_add),
		
		.block_in(block_in),
		.block_out(block_out_mac),
		
		.shift(shift),
		.shift_disable(shift_disable),
		.signedness_in(signedness),
		.signedness_out(signedness_out_mac),
		.saturate_disable_in(saturate_disable),
		.saturate_disable_out(saturate_disable_out_mac),
		
		.arg_a_in(arg_a_in),
		.arg_b_in(arg_b_in),
		.arg_c_in(arg_c_in),
		.arg_c_out(arg_c_out_mac),
		
		.result_out(result_out_mac),
		
		.dest_in(dest_in),
		.dest_out(dest_out_mac),
		
		.commit_id_in(commit_id_in),
		.commit_id_out(commit_id_out_mac),

		.commit_flag_in(commit_flag_in),
		.commit_flag_out(commit_flag_out_mac)
	);
	
	wire saturate_disable_out_add;
	wire in_ready_add;
	wire out_valid_add;
	wire [2 * data_width - 1 : 0] result_out_add;
	wire [3:0] dest_out_add;
	wire [8:0] commit_id_out_add;
	wire commit_flag_out_add;

	add_stage #(.data_width(data_width)) add_stage
		(
			.clk(clk),
			.reset(reset),
			
			.enable(enable),
			
			.in_valid(out_valid_mac),
			.in_ready(in_ready_add),
			
			.out_valid(out_valid_add),
			.out_ready(in_ready_sats),
			
			.signedness_in(signedness_out_mac),
			.saturate_disable_in(saturate_disable_out_mac),
			.saturate_disable_out(saturate_disable_out_add),
			
			.product_in(result_out_mac),
			.arg_c_in(arg_c_out_mac),
			
			.dest_in(dest_out_mac),
			.dest_out(dest_out_add),
			
			.result_out(result_out_add),
			
			.commit_id_in(commit_id_out_mac),
			.commit_id_out(commit_id_out_add),
			
			.commit_flag_in(commit_flag_out_mac),
			.commit_flag_out(commit_flag_out)
		);

	wire in_ready_sats;

	saturate_stage #(.data_width(data_width)) saturate_stage
		(
			.clk(clk),
			.reset(reset),
			
			.enable(enable),
			
			.in_valid(out_valid_add),
			.in_ready(in_ready_sats),
			
			.out_valid(out_valid),
			.out_ready(out_ready),
			
			.saturate_disable_in(saturate_disable_out_add),
			
			.dest_in(dest_out_add),
			.dest_out(dest_out),
			
			.result_in(result_out_add),
			.result_out(result_out),
			
			.commit_id_in(commit_id_out_add),
			.commit_id_out(commit_id_out),
			
			.commit_flag_in(commit_flag_out_add),
			.commit_flag_out(commit_flag_out)
		);
endmodule

`default_nettype wire
