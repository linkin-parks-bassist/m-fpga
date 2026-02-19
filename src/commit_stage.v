`default_nettype none

module commit_stage #(parameter data_width = 16, parameter n_blocks = 256, parameter full_width = 2 * data_width + 8)
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
		
		input  wire signed [full_width - 1 : 0] result_in,
		output wire signed [full_width - 1 : 0] result_out,
		
		input  wire [3:0] dest_in,
		output wire [3:0] dest_out,
		
		input  wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_in,
		output wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,

		input  wire commit_flag_in,
		output wire commit_flag_out
	);
	
	localparam payload_width = $clog2(n_blocks) + full_width + 4 + `COMMIT_ID_WIDTH + 1;

	wire [payload_width - 1 : 0] skid_payload_in = 
		{block_in, result_in, dest_in, commit_id_in, commit_flag_in};
	wire [payload_width - 1 : 0] skid_payload_out;

	assign {block_out, result_out, dest_out, commit_id_out, commit_flag_out} = skid_payload_out;

	skid_buffer #(.payload_width(payload_width)) skid_buffer
		(.clk(clk), .reset(reset), .enable(enable),
		  .in_ready(in_ready),   .in_valid(in_valid),
		 .out_ready(out_ready), .out_valid(out_valid),
		 .payload_in(skid_payload_in), .payload_out(skid_payload_out));
endmodule

`default_nettype wire
