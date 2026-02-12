`include "instr_dec.vh"
`include "lut.vh"
`include "core.vh"

`default_nettype none

module block_fetcher #(parameter data_width = 16, parameter n_blocks = 256)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] n_blocks_running,
		output reg [$clog2(n_blocks) - 1 : 0] block_read_addr,
		
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire [data_width - 1 : 0] register_0_in,
		output reg [data_width - 1 : 0] register_0_out,
		input wire [data_width - 1 : 0] register_1_in,
		output reg [data_width - 1 : 0] register_1_out,
		
		input wire [31 : 0] instr_in,
		output reg [31 : 0] instr_out
	);
	
	reg in_valid;
	
	wire in_ready = ~out_valid | out_ready;
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	reg skid;
	reg [31 : 0] instr_skid;
	reg [data_width - 1 : 0] register_0_skid;
	reg [data_width - 1 : 0] register_1_skid;
	reg [$clog2(n_blocks) - 1 : 0] block_skid;
	
	reg [$clog2(n_blocks) - 1 : 0] block_r;
	
	always @(posedge clk) begin
		if (reset) begin
			block_read_addr <= 0;
			in_valid <= 0;
			out_valid <= 0;
			skid <= 0;
		end else if (n_blocks_running == 0) begin
			block_read_addr <= 0;
			in_valid <= 0;
			out_valid <= 0;
			skid <= 0;
		end else if (enable) begin
			
			in_valid <= 0;
			
			if ((~out_valid | take_out) & ~skid) begin
				if (block_read_addr >= n_blocks_running - 1)
					block_read_addr <= 0;
				else
					block_read_addr <= block_read_addr + 1;
				
				block_r <= block_read_addr;
				in_valid <= 1;
			end else if (skid & take_out) begin
				instr_out 	   <= instr_skid;
				register_0_out <= register_0_skid;
				register_1_out <= register_1_skid;
				block_out 	   <= block_skid;
				out_valid <= 1;
				skid <= 0;
			end
			
			if (in_valid) begin
				if (out_valid & ~out_ready) begin // Record skid, freeze frame
					// Yup, that's me. I bet you're wondering how I ended up in this situation
					instr_skid 	    <= instr_in;
					register_0_skid <= register_0_in;
					register_1_skid <= register_1_in;
					block_skid 	    <= block_r;
					skid <= 1;
				end else begin
					instr_out 	   <= instr_in;
					register_0_out <= register_0_in;
					register_1_out <= register_1_in;
					block_out <= block_r;
				end
				
				out_valid <= 1;
			end else if (take_out & ~skid) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module block_buffer #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input wire [data_width - 1 : 0] register_0_in,
		output reg [data_width - 1 : 0] register_0_out,
		input wire [data_width - 1 : 0] register_1_in,
		output reg [data_width - 1 : 0] register_1_out,
		
		input wire [31 : 0] instr_in,
		output reg [31 : 0] instr_out
	);
	
	assign in_ready = ~buf_valid | (out_valid & out_ready);
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	reg buf_valid;
	
	reg [31 : 0] instr_buf;
	reg [data_width - 1 : 0] register_0_buf;
	reg [data_width - 1 : 0] register_1_buf;
	reg [$clog2(n_blocks) - 1 : 0] block_buf;
	
	always @(posedge clk) begin
		if (reset) begin
			buf_valid <= 0;
			out_valid <= 0;
		end else begin
			out_valid <= out_valid;
			buf_valid <= buf_valid;
			
			case ({take_in, take_out})
				2'b10: begin
					if (out_valid) begin
						instr_buf <= instr_in;
						register_0_buf <= register_0_in;
						register_1_buf <= register_1_in;
						block_buf <= block_in;
						buf_valid <= 1;
					end else begin
						instr_out <= instr_in;
						register_0_out <= register_0_in;
						register_1_out <= register_1_in;
						block_out <= block_in;
						out_valid <= 1;
					end
				end
				
				2'b01: begin
					if (buf_valid) begin
						instr_out <= instr_buf;
						register_0_out <= register_0_buf;
						register_1_out <= register_1_buf;
						block_out <= block_buf;
						buf_valid <= 0;
					end else begin
						out_valid <= 0;
					end
				end
				
				2'b11: begin
					if (buf_valid) begin
						instr_buf <= instr_in;
						register_0_buf <= register_0_in;
						register_1_buf <= register_1_in;
						block_buf <= block_in;
						instr_out <= instr_buf;
						register_0_out <= register_0_buf;
						register_1_out <= register_1_buf;
						block_out <= block_buf;
						
						out_valid <= 1;
						buf_valid <= 1;
					end else begin
						instr_out <= instr_in;
						register_0_out <= register_0_in;
						register_1_out <= register_1_in;
						block_out <= block_in;
						
						out_valid <= 1;
						buf_valid <= 0;
					end
				end
				
				default: begin end
			endcase
		end
	end
endmodule

module instr_decode_stage #(parameter data_width = 16, parameter n_blocks = 256)
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
		
		input wire [data_width - 1 : 0] register_0_in,
		output reg [data_width - 1 : 0] register_0_out,
		input wire [data_width - 1 : 0] register_1_in,
		output reg [data_width - 1 : 0] register_1_out,
		
		input wire [31 : 0] instr_in,
		
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
		
		output reg accumulator_needed_out,
		
		output reg writes_channel_out,
		output reg writes_accumulator_out,
		output reg commit_flag_out,
		output reg writes_external_out,
		
		output reg [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_out,

        output reg [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;

	wire [4 : 0] operation;
	
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op;
	
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
	
	wire accumulator_needed;
	
	wire writes_channel;
	wire writes_acc;
	wire commit_flag;
	wire writes_external;
	
	wire [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch;
	
	instr_decoder #(.data_width(data_width)) dec
	(
		.instr(instr_in),
		
		.operation(operation),
		
		.misc_op(misc_op),
		
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
		
		.accumulator_needed(accumulator_needed),
		
		.writes_channel(writes_channel),
		.writes_acc(writes_acc),
		.commit_flag(commit_flag),
		.writes_external(writes_external),
		
		.branch(branch)
	);

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else if (enable) begin
			if (take_in) begin
				
				block_out <= block_in;
		
				register_0_out <= register_0_in;
				register_1_out <= register_1_in;
				
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
				
				accumulator_needed_out <= accumulator_needed;
				
				writes_channel_out <= writes_channel;
				writes_accumulator_out <= writes_acc;
				commit_flag_out <= commit_flag;
				writes_external_out <= writes_external;
				
				branch_out <= branch;

				misc_op_out <= misc_op;
				
				out_valid <= 1;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule


module block_fetch_decode_stage #(parameter data_width = 16, parameter n_blocks = 256, parameter n_block_regs = 2)
	(
		input  wire clk,
		input  wire reset,
		
		input  wire enable,
	
		output wire out_valid,
		input  wire out_ready,
		
		input  wire [$clog2(n_blocks) - 1 : 0] n_blocks_running,
		output wire [$clog2(n_blocks) - 1 : 0] block_read_addr,
		
		input  wire [data_width - 1 : 0] register_0_in,
		output wire [data_width - 1 : 0] register_0_out,
		input  wire [data_width - 1 : 0] register_1_in,
		output wire [data_width - 1 : 0] register_1_out,
		
		input  wire [31 : 0] instr_read_val,
		
		output wire [$clog2(n_blocks) - 1 : 0] block_out,
		
		output wire [4 : 0] operation_out,
		
		output wire [3 : 0] src_a_out,
		output wire [3 : 0] src_b_out,
		output wire [3 : 0] src_c_out,
		output wire [3 : 0] dest_out,
		
		output wire src_a_reg_out,
		output wire src_b_reg_out,
		output wire src_c_reg_out,
		
		output wire saturate_disable_out,
		output wire shift_disable_out,
		output wire signedness_out,
		
		output wire [4  : 0] shift_out,
		output wire [11 : 0] res_addr_out,
		
		output wire arg_a_needed_out,
		output wire arg_b_needed_out,
		output wire arg_c_needed_out,
		
		output wire accumulator_needed_out,
		
		output wire writes_channel_out,
		output wire writes_accumulator_out,
		output wire commit_flag_out,
		output wire writes_external_out,
		
		output wire [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_out,

        output wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out
	);
	
	wire out_valid_1;
	wire [$clog2(n_blocks) - 1 : 0] block_1_out;
	wire [31 : 0] instr_1_out;
	wire [data_width - 1 : 0] register_0_1_out;
	wire [data_width - 1 : 0] register_1_1_out;
	
	block_fetcher #(.data_width(data_width), .n_blocks(n_blocks)) fetcher
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
		
		.out_valid(out_valid_1),
		.out_ready(in_ready_2),
		
		.n_blocks_running(n_blocks_running),
		.block_read_addr(block_read_addr),
		
		.block_out(block_1_out),
		
		.register_0_in(register_0_in),
		.register_0_out(register_0_1_out),
		.register_1_in(register_1_in),
		.register_1_out(register_1_1_out),
		
		.instr_in(instr_read_val),
		.instr_out(instr_1_out)
	);
	
	wire in_ready_2;
	wire out_valid_2;
	wire [$clog2(n_blocks) - 1 : 0] block_2_out;
	wire [31 : 0] instr_2_out;
	wire [data_width - 1 : 0] register_0_2_out;
	wire [data_width - 1 : 0] register_1_2_out;
	
	block_buffer #(.data_width(data_width), .n_blocks(n_blocks)) buffer
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
		
		.register_0_in(register_0_1_out),
		.register_0_out(register_0_2_out),
		.register_1_in(register_1_1_out),
		.register_1_out(register_1_2_out),
		
		.instr_in(instr_1_out),
		.instr_out(instr_2_out)
	);
	
	wire in_ready_3;
	
	instr_decode_stage #(.data_width(data_width), .n_blocks(n_blocks)) decoder
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
		
		.register_0_in(register_0_2_out),
		.register_0_out(register_0_out),
		.register_1_in(register_1_2_out),
		.register_1_out(register_1_out),
		
		.instr_in(instr_2_out),
		
		.operation_out(operation_out),
		
		.src_a_out(src_a_out),
		.src_b_out(src_b_out),
		.src_c_out(src_c_out),
		.dest_out(dest_out),
		
		.src_a_reg_out(src_a_reg_out),
		.src_b_reg_out(src_b_reg_out),
		.src_c_reg_out(src_c_reg_out),
		
		.saturate_disable_out(saturate_disable_out),
		.shift_disable_out(shift_disable_out),
		.signedness_out(signedness_out),
		
		.shift_out(shift_out),
		.res_addr_out(res_addr_out),
		
		.arg_a_needed_out(arg_a_needed_out),
		.arg_b_needed_out(arg_b_needed_out),
		.arg_c_needed_out(arg_c_needed_out),
		
		.accumulator_needed_out(accumulator_needed_out),
		
		.writes_channel_out(writes_channel_out),
		.writes_accumulator_out(writes_accumulator_out),
		.commit_flag_out(commit_flag_out),
		.writes_external_out(writes_external_out),
		
		.branch_out(branch_out),

        .misc_op_out(misc_op_out)
	);
	
endmodule

`default_nettype wire
