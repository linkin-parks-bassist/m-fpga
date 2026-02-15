`default_nettype none

`include "instr_dec.vh"
`include "core.vh"

module branch_router #(parameter data_width = 16, parameter n_blocks = 256, parameter n_block_regs = 2)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
	
		input wire sample_tick,
		
		input wire in_valid,
		output reg in_ready,
		
		output reg [`N_INSTR_BRANCHES - 1 : 0] out_valid,
		input wire [`N_INSTR_BRANCHES - 1 : 0] out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire [4 : 0] operation_in,
		output reg [4 : 0] operation_out,

		input wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_in,
		output reg [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,

		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		
		input wire signedness_in,
		output reg signedness_out,
		
		input wire writes_external_in,
		output reg writes_external_out,
		
		input wire accumulator_needed_in,
		output reg accumulator_needed_out,

		input wire [4 : 0] shift_in,
		output reg [4 : 0] shift_out,
		input wire shift_disable_in,
		output reg shift_disable_out,
		
		input wire [7 : 0] res_addr_in,
		output reg [7 : 0] res_addr_out,
		
		input wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_in,
		output reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		output reg signed [2 * data_width - 1 : 0] accumulator_out,

		input wire [`N_INSTR_BRANCHES - 1 : 0] branch
	);
	
	reg [`N_INSTR_BRANCHES - 1 : 0] branch_out;
	
	assign in_ready = ~(|out_valid) | out_ready[branch_out];
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid[branch_out] & out_ready[branch_out];
	
	
	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				out_valid <= (1 << branch);
				
				branch_out <= branch;
				
				block_out <= block_in;
				operation_out <= operation_in;
				misc_op_out <= misc_op_in;
				dest_out <= dest_in;
				
				arg_a_out <= arg_a_in;
				arg_b_out <= arg_b_in;
				arg_c_out <= arg_c_in;
				saturate_disable_out <= saturate_disable_in;
				signedness_out <= signedness_in;
				accumulator_needed_out <= accumulator_needed_in;
				shift_out <= shift_in;
				shift_disable_out <= shift_disable_in;
				res_addr_out <= res_addr_in;
				
				writes_external_out <= writes_external_in;
				
				commit_id_out <= commit_id_in;
				commit_flag_out <= commit_flag_in;
				accumulator_out <= accumulator_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

`default_nettype wire
