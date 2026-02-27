`include "instr_dec.vh"

`include "lut.vh"
`include "core.vh"

`default_nettype none

module operand_fetch_substage #(parameter data_width = 16, parameter n_blocks = 256, parameter bit last = 0)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
	
		input wire sample_tick,
		
		input  wire in_valid,
		output wire in_ready,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] n_blocks_running,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire signed [data_width - 1 : 0] register_0_in,
		output reg signed [data_width - 1 : 0] register_0_out,
		input wire signed [data_width - 1 : 0] register_1_in,
		output reg signed [data_width - 1 : 0] register_1_out,
		
		input wire [4 : 0] operation_in,
		output reg [4 : 0] operation_out,

		input wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_in,
		output reg [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,
		
		input wire arg_needed,
		input wire [3 : 0] src,
		input wire src_reg,
		output reg signed [data_width - 1 : 0] fetched_out,		
		
		input wire arg_a_needed_in,
		input wire [3 : 0] src_a_in,
		input wire src_a_reg_in,
		input wire signed [data_width - 1 : 0] arg_a_in,
		output reg arg_a_needed_out,
		output reg [3 : 0] src_a_out,
		output reg src_a_reg_out,
		output reg signed [data_width - 1 : 0] arg_a_out,
		
		input wire arg_b_needed_in,
		input wire [3 : 0] src_b_in,
		input wire src_b_reg_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		output reg arg_b_needed_out,
		output reg [3 : 0] src_b_out,
		output reg src_b_reg_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		
		input wire arg_c_needed_in,
		input wire [3 : 0] src_c_in,
		input wire src_c_reg_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		output reg arg_c_needed_out,
		output reg [3 : 0] src_c_out,
		output reg src_c_reg_out,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		
		input wire signedness_in,
		output reg signedness_out,
		
		input wire accumulator_needed_in,
		output reg accumulator_needed_out,

		input wire [4 : 0] shift_in,
		output reg [4 : 0] shift_out,
		input wire shift_disable_in,
		output reg shift_disable_out,
		
		input wire [7 : 0] res_addr_in,
		output reg [7 : 0] res_addr_out,
		
		input wire writes_external_in,
		output reg writes_external_out,
		
		input wire writes_channel_in,
		output reg writes_channel_out,
		input wire writes_accumulator_in,
		output reg writes_accumulator_out,
		
		output reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out,

		input wire [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_in,
		output reg [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_out,
		
		input wire [3 : 0] channel_write_addr,
		input wire signed [data_width - 1 : 0] channel_write_val,
		input wire channel_write_enable,
		
		input wire accumulator_write_enable
	);
	
	reg [data_width - 1 : 0] channels [15 : 0];
	
	wire take_in = in_ready & in_valid;
	wire [data_width - 1 : 0] channel_read_val = channels[src_live];
	
	integer j;
	always @(posedge clk) begin
		if (reset) begin
			for (j = 0; j < 16; j = j + 1) begin
				channels[j] <= 0;
			end
		end else begin
			if (channel_write_enable)
				channels[channel_write_addr] <= channel_write_val;
		end
	end
	
	reg [3 : 0] channels_scoreboard [15 : 0];
	reg [3 : 0] accumulator_pending_writes;
	
	reg [15 : 0] busy_bits;
	reg accumulator_busy;

	integer i;
	always @(posedge clk) begin
		for (i = 0; i < 16; i = i + 1)
			channels_scoreboard[i] <= channels_scoreboard[i];
	
		if (reset) begin
			busy_bits <= 0;
			accumulator_busy <= 0;
			
			for (i = 0; i < 16; i = i + 1)
				channels_scoreboard[i] <= 0;
			
			accumulator_pending_writes <= 0;
		end else if (enable) begin
			case ({inject_pending_ch0_write, (add_pending_write && dest_live == 0 && !writes_accumulator_live),
					channel_write_enable && channel_write_addr == 0})
				3'b010: begin
					channels_scoreboard[0] <= channels_scoreboard[0] + 1;
					busy_bits[0] <= 1;
				end
				
				3'b001: begin
					if (channels_scoreboard[0] != 0) begin
						channels_scoreboard[0] <= channels_scoreboard[0] - 1;
						busy_bits[0] <= (channels_scoreboard[0] != 1);
					end else begin
						channels_scoreboard[0] <= channels_scoreboard[0];
						busy_bits[0] <= 0;
					end
				end
				
				3'b100: begin
					channels_scoreboard[0] <= channels_scoreboard[0] + 1;
					busy_bits[0] <= 1;
				end
				
				3'b110: begin
					channels_scoreboard[0] <= channels_scoreboard[0] + 2;
					busy_bits[0] <= 1;
				end
				
				3'b111: begin
					channels_scoreboard[0] <= channels_scoreboard[0] + 1;
					busy_bits[0] <= 1;
				end
				
				default: begin
					channels_scoreboard[0] <= channels_scoreboard[0];
					busy_bits[0] <= (channels_scoreboard[0] != 0);
				end
			endcase
		
			for (i = 1; i < 16; i = i + 1) begin
				case ({(add_pending_write && dest_live == i && !writes_accumulator_live),
						channel_write_enable && channel_write_addr == i})
					2'b10: begin
						channels_scoreboard[i] <= channels_scoreboard[i] + 1;
						busy_bits[i] <= 1;
					end
					
					2'b01: begin
						if (channels_scoreboard[i] != 0) begin
							channels_scoreboard[i] <= channels_scoreboard[i] - 1;
							busy_bits[i] <= (channels_scoreboard[i] != 1);
						end else begin
							channels_scoreboard[i] <= channels_scoreboard[i];
							busy_bits[i] <= 0;
						end
					end
					
					default: begin
						channels_scoreboard[i] <= channels_scoreboard[i];
						busy_bits[i] <= (channels_scoreboard[i] != 0);
					end
				endcase
			end
			
			case ({add_pending_write & writes_accumulator_live, accumulator_write_enable})
				2'b10: begin
					accumulator_pending_writes <= accumulator_pending_writes + 1;
					accumulator_busy <= 1;
				end
				
				2'b01: begin
					if (accumulator_pending_writes != 0) begin
						accumulator_pending_writes <= accumulator_pending_writes - 1;
						accumulator_busy <= (accumulator_pending_writes != 1);
					end else begin
						accumulator_pending_writes <= accumulator_pending_writes;
						accumulator_busy <= 0;
					end
				end
				
				default: begin
					accumulator_pending_writes <= accumulator_pending_writes;
					accumulator_busy <= (accumulator_pending_writes != 0);
				end
			endcase
		end
	end
	
	reg [$clog2(n_blocks) - 1 : 0] block_latched;
	
	reg signed [data_width - 1 : 0] register_0_latched;
	reg signed [data_width - 1 : 0] register_1_latched;
	
	reg [4 : 0] operation_latched;
	
	reg [3 : 0] dest_latched;
	reg writes_channel_latched;
	reg accumulator_needed_latched;
	reg writes_accumulator_latched;

	reg [3 : 0] src_latched;
	reg arg_needed_latched;
	reg src_reg_latched;
	reg arg_valid;
	
	reg arg_a_needed_latched;
	reg [3 : 0] src_a_latched;
	reg src_a_reg_latched;
	reg signed [data_width - 1 : 0] arg_a_latched;

	reg arg_b_needed_latched;
	reg [3 : 0] src_b_latched;
	reg src_b_reg_latched;
	reg signed [data_width - 1 : 0] arg_b_latched;

	reg arg_c_needed_latched;
	reg [3 : 0] src_c_latched;
	reg src_c_reg_latched;
	reg signed [data_width - 1 : 0] arg_c_latched;
	
	wire [$clog2(n_blocks) - 1 : 0] block_live = busy ? block_latched : block_in;
	
	wire [3 : 0] dest_live = (busy) ? dest_latched : dest_in;
	wire writes_channel_live = (busy) ? writes_channel_latched : writes_channel_in;
	wire accumulator_needed_live = (busy) ? accumulator_needed_latched : accumulator_needed_in;
	wire writes_accumulator_live = (busy) ? writes_accumulator_latched : writes_accumulator_in;

	wire [3 : 0] src_live = (busy) ? src_latched : src;
	wire arg_needed_live = (busy) ? arg_needed_latched : arg_needed;
	wire src_reg_live = (busy) ? src_reg_latched : src_reg;
	
	reg saturate_disable_latched;
	reg signedness_latched;

	reg [4 : 0] shift_latched;
	reg shift_disable_latched;
	
	reg [7 : 0] res_addr_latched;
	
	reg [$clog2(`N_INSTR_BRANCHES) - 1 : 0] branch_latched;
	reg writes_external_latched;

	reg commit_flag_latched;
	
	reg [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_latched;

	wire  [3 : 0] arg_pending_writes = channels_scoreboard[src_latched];
	
	reg signed [data_width - 1 : 0] arg_latched;
	wire arg_resolved = ~arg_needed_latched | arg_valid;
	
	always @(posedge clk) begin
		if (reset) begin
			arg_valid <= 0;
			arg_latched <= 0;
		end else if (enable && busy) begin
			if (!arg_needed_latched) begin
				arg_latched <= 0;
				arg_valid <= 1;
			end
			
			if (!arg_resolved) begin
				if (src_reg_latched) begin
					if (src_latched[0])
						arg_latched <= register_1_latched;
					else
						arg_latched <= register_0_latched;
					arg_valid <= 1;
				end else if (arg_pending_writes == 0) begin
					arg_latched <= channel_read_val;
					arg_valid <= 1;
				end else if (arg_pending_writes == 1 && channel_write_enable
						  && channel_write_addr == src_latched) begin
					arg_latched <= channel_write_val;
					arg_valid <= 1;
				end
			end
		end else begin
			arg_valid <= 0;
		end
	end
	
	localparam IDLE = 2'd0;
	localparam BUSY = 2'd1;
	localparam DONE = 2'd2;
	
	assign in_ready = ~busy & (~out_valid | out_ready);
	
	reg busy;
	
	reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id;
	
	logic signed [data_width - 1 : 0] reg_value;
	
	always_comb begin
		case (src)
			4'd0: reg_value = register_0_in;
			4'd1: reg_value = register_1_in;
			
			4'd3: reg_value = (1 << (data_width - 2));
			4'd4: reg_value = (1 << (data_width - 1));
			default: reg_value = 0;
		endcase
	end
	
	wire arg_pending_write = busy_bits[src];
	wire accumulator_stall = last & accumulator_needed_live & accumulator_busy;
	wire needs_stall = (arg_needed & ~src_reg & arg_pending_write) | accumulator_stall;
	wire signed [data_width - 1 : 0] single_cycle_result = src_reg ? reg_value : channels[src];
	wire stall = ~stall_done & (busy ? 1 : needs_stall);
	wire stall_done = busy & arg_resolved & ~accumulator_stall;
	wire proceed = (busy ? stall_done : ~needs_stall);
	wire last_cycle = busy ? proceed : (~needs_stall & take_in);
	wire send = stall_done | (~busy & ~needs_stall);
	wire creates_dependency = writes_channel_live | writes_accumulator_live;
	wire last_block = (block_live == n_blocks_running - 1);
	
	wire add_pending_write = last_cycle & creates_dependency;
	wire inject_pending_ch0_write = last_cycle & last_block;
	
	always @(posedge clk) begin
		if (reset) begin
			busy	<= 0;
			out_valid <= 0;
			commit_id   <= 0;
			branch_out    <= 0;
		end else if (enable) begin
			
			branch_out <= branch_out;

			if (take_in) begin
				if (!needs_stall) begin
					operation_out <= operation_in;
					misc_op_out <= misc_op_in;
	
					dest_out <= dest_in;

					block_out <= block_in;
					
					register_0_out <= register_0_in;
					register_1_out <= register_1_in;
					
					fetched_out <= single_cycle_result;
					
					accumulator_needed_out <= accumulator_needed_in;
					saturate_disable_out <= saturate_disable_in;
					signedness_out 		 <= signedness_in;
					shift_out 			 <= shift_in;
					shift_disable_out 	 <= shift_disable_in;
					res_addr_out 		 <= res_addr_in;
					commit_flag_out		 <= commit_flag_in;
					
					writes_channel_out   	<= writes_channel_in;
					writes_external_out  	<= writes_external_in;
					writes_accumulator_out 	<= writes_accumulator_in;
				
					arg_a_needed_out 	<= arg_a_needed_in;
					src_a_out 			<= src_a_in;
					src_a_reg_out 		<= src_a_reg_in;
					arg_a_out 			<= arg_a_in;

					arg_b_needed_out 	<= arg_b_needed_in;
					src_b_out 			<= src_b_in;
					src_b_reg_out 		<= src_b_reg_in;
					arg_b_out 			<= arg_b_in;

					arg_c_needed_out 	<= arg_c_needed_in;
					src_c_out 			<= src_c_in;
					src_c_reg_out 		<= src_c_reg_in;
					arg_c_out 			<= arg_c_in;
					
					branch_out <= branch_in;
					
					out_valid <= 1;
					
					if (writes_channel_in | writes_accumulator_in) begin
						commit_id_out <= commit_id;
						commit_id <= commit_id + 1;
						
						dest_latched <= dest_in;
						writes_accumulator_latched <= writes_accumulator_in;
						writes_channel_latched <= writes_channel_in;
					end
				end else begin
					
					block_latched <= block_in;
					
					register_0_latched <= register_0_in;
					register_1_latched <= register_1_in;
					
					operation_latched <= operation_in;
					misc_op_latched <= misc_op_in;

					dest_latched <= dest_in;
					
					saturate_disable_latched 	<= saturate_disable_in;
					accumulator_needed_latched	<= accumulator_needed_in;
					signedness_latched 			<= signedness_in;
					shift_latched 				<= shift_in;
					shift_disable_latched 		<= shift_disable_in;
					
					res_addr_latched <= res_addr_in;
					
					writes_external_latched 	<= writes_external_in;
					writes_channel_latched 		<= writes_channel_in;
					writes_accumulator_latched 	<= writes_accumulator_in;
					commit_flag_latched 		<= commit_flag_in;
					branch_latched 				<= branch_in;
					
					src_latched 		<= src;
					src_reg_latched 	<= src_reg;
					arg_needed_latched 	<= arg_needed;
					
									
					arg_a_needed_latched 	<= arg_a_needed_in;
					src_a_latched 			<= src_a_in;
					src_a_reg_latched 		<= src_a_reg_in;
					arg_a_latched 			<= arg_a_in;

					arg_b_needed_latched 	<= arg_b_needed_in;
					src_b_latched 			<= src_b_in;
					src_b_reg_latched 		<= src_b_reg_in;
					arg_b_latched 			<= arg_b_in;

					arg_c_needed_latched 	<= arg_c_needed_in;
					src_c_latched 			<= src_c_in;
					src_c_reg_latched		<= src_c_reg_in;
					arg_c_latched 			<= arg_c_in;
					
					out_valid <= ~out_ready;
					busy <= 1;
				end
			end else begin
				if (out_ready)
					out_valid <= 0;
				
				if (busy & arg_resolved & ~(last & accumulator_stall)) begin
					operation_out <= operation_latched;
					misc_op_out <= misc_op_latched;
	
					dest_out <= dest_latched;

					block_out <= block_latched;
					
					register_0_out <= register_0_latched;
					register_1_out <= register_1_latched;

					fetched_out <= arg_latched;
					
					saturate_disable_out <= saturate_disable_latched;
					accumulator_needed_out <= accumulator_needed_latched;
					signedness_out 		 <= signedness_latched;
					shift_out 			 <= shift_latched;
					shift_disable_out 	 <= shift_disable_latched;
					res_addr_out 		 <= res_addr_latched;
					commit_flag_out		 <= commit_flag_latched;
													
					arg_a_needed_out 	<= arg_a_needed_latched;
					src_a_out 			<= src_a_latched;
					src_a_reg_out 		<= src_a_reg_latched;
					arg_a_out 			<= arg_a_latched;

					arg_b_needed_out 	<= arg_b_needed_latched;
					src_b_out 			<= src_b_latched;
					src_b_reg_out 		<= src_b_reg_latched;
					arg_b_out 			<= arg_b_latched;

					arg_c_needed_out 	<= arg_c_needed_latched;
					src_c_out 			<= src_c_latched;
					src_c_reg_out		<= src_c_reg_latched;
					arg_c_out 			<= arg_c_latched;
					
					writes_channel_out   	<= writes_channel_latched;
					writes_external_out  	<= writes_external_latched;
					writes_accumulator_out 	<= writes_accumulator_latched;
					
					branch_out <= branch_latched;
					
					if (writes_channel_latched | writes_accumulator_latched) begin
						commit_id_out <= commit_id;
						commit_id <= commit_id + 1;
					end
					
					out_valid <= 1;
					busy <= 0;
				end
			end
		end
	end
endmodule

module operand_fetch_stage #(parameter data_width = 16, parameter n_blocks = 256)
	(
		input  wire clk,
		input  wire reset,
		
		input  wire enable,
	
		input  wire sample_tick,
		
		input  wire in_valid,
		output wire in_ready,
		
		output wire out_valid,
		input  wire out_ready,
		
		input  wire [$clog2(n_blocks) - 1 : 0] n_blocks_running,
		
		input  wire [$clog2(n_blocks) - 1 : 0] block_in,
		output wire [$clog2(n_blocks) - 1 : 0] block_out,
		
		input  wire signed [data_width - 1 : 0] register_0_in,
		output wire signed [data_width - 1 : 0] register_0_out,
		input  wire signed [data_width - 1 : 0] register_1_in,
		output wire signed [data_width - 1 : 0] register_1_out,
		
		input  wire [4 : 0] operation_in,
		output wire [4 : 0] operation_out,

		input  wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_in,
		output wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_out,
		
		input  wire [3 : 0] dest_in,
		output wire [3 : 0] dest_out,

		input  wire signed [3 : 0] src_a_in,
		input  wire signed [3 : 0] src_b_in,
		input  wire signed [3 : 0] src_c_in,

		input  wire src_a_reg_in,
		input  wire src_b_reg_in,
		input  wire src_c_reg_in,
		
		input  wire arg_a_needed_in,
		input  wire arg_b_needed_in,
		input  wire arg_c_needed_in,

		output wire signed [data_width - 1 : 0] arg_a_out,
		output wire signed [data_width - 1 : 0] arg_b_out,
		output wire signed [data_width - 1 : 0] arg_c_out,
		
		input  wire saturate_disable_in,
		output wire saturate_disable_out,
		
		input  wire signedness_in,
		output wire signedness_out,
		
		input  wire accumulator_needed_in,
		output wire accumulator_needed_out,

		input  wire [4 : 0] shift_in,
		output wire [4 : 0] shift_out,
		input  wire shift_disable_in,
		output wire shift_disable_out,
		
		input  wire [7 : 0] res_addr_in,
		output wire [7 : 0] res_addr_out,
		
		input  wire writes_external_in,
		output wire writes_external_out,
		
		input  wire writes_channel_in,
		output wire writes_channel_out,
		input  wire writes_accumulator_in,
		
		output wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out,
		
		input  wire commit_flag_in,
		output wire commit_flag_out,
		
		input  wire [`N_INSTR_BRANCHES - 1 : 0] branch_in,
		output wire [`N_INSTR_BRANCHES - 1 : 0] branch_out,
		
		input  wire [3 : 0] channel_write_addr,
		input  wire signed [data_width - 1 : 0] channel_write_val,
		input  wire channel_write_enable,
		
		input  wire signed [data_width - 1 : 0] channel_read_val,
		
		input  wire accumulator_write_enable
	);
	
	wire out_valid_1;
	wire  [$clog2(n_blocks) - 1 : 0] block_1_out;
	wire signed [data_width - 1 : 0] register_0_1_out;
	wire signed [data_width - 1 : 0] register_1_1_out;
	wire [4 : 0] operation_1_out;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_1_out;
	wire [3 : 0] dest_1_out;
	
	wire [3 : 0] src_a_1_out;
	wire [3 : 0] src_b_1_out;
	wire [3 : 0] src_c_1_out;
	
	wire src_a_reg_1_out;
	wire src_b_reg_1_out;
	wire src_c_reg_1_out;
	
	wire arg_a_needed_1_out;
	wire arg_b_needed_1_out;
	wire arg_c_needed_1_out;
	
	wire signed [data_width - 1 : 0] arg_a_1_out;
	wire signed [data_width - 1 : 0] arg_b_1_out;
	wire signed [data_width - 1 : 0] arg_c_1_out;
	
	wire saturate_disable_1_out;
	wire signedness_1_out;
	wire [4 : 0] shift_1_out;
	wire shift_disable_1_out;
	wire [7 : 0] res_addr_1_out;
	wire writes_external_1_out;
	wire writes_channel_1_out;
	wire writes_accumulator_1_out;
	wire accumulator_needed_1_out;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_1_out;
	wire commit_flag_1_out;
	wire [`N_INSTR_BRANCHES - 1 : 0] branch_1_out;
	
	operand_fetch_substage #(.data_width(data_width), .n_blocks(n_blocks), .last(0)) fetch_1
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
	
		.sample_tick(sample_tick),
		
		.in_valid(in_valid),
		.in_ready(in_ready),
		
		.out_valid(out_valid_1),
		.out_ready(in_ready_2),
		
		.n_blocks_running(n_blocks_running),
		
		.block_in(block_in),
		.block_out(block_1_out),
		
		.register_0_in(register_0_in),
		.register_0_out(register_0_1_out),
		.register_1_in(register_1_in),
		.register_1_out(register_1_1_out),
		
		.operation_in(operation_in),
		.operation_out(operation_1_out),

		.misc_op_in(misc_op_in),
		.misc_op_out(misc_op_1_out),
		
		.dest_in(dest_in),
		.dest_out(dest_1_out),

		.arg_a_needed_in (arg_a_needed_in),
		.arg_a_needed_out(arg_a_needed_1_out),
		.arg_b_needed_in (arg_b_needed_in),
		.arg_b_needed_out(arg_b_needed_1_out),
		.arg_c_needed_in (arg_c_needed_in),
		.arg_c_needed_out(arg_c_needed_1_out),
		.src_a_in (src_a_in),
		.src_a_out(src_a_1_out),
		.src_b_in (src_b_in),
		.src_b_out(src_b_1_out),
		.src_c_in (src_c_in),
		.src_c_out(src_c_1_out),
		.src_a_reg_in (src_a_reg_in),
		.src_a_reg_out(src_a_reg_1_out),
		.src_b_reg_in (src_b_reg_in),
		.src_b_reg_out(src_b_reg_1_out),
		.src_c_reg_in (src_c_reg_in),
		.src_c_reg_out(src_c_reg_1_out),
		.arg_a_in (),
		//.arg_a_out(arg_a_1_out),
		.arg_b_in (),
		.arg_b_out(arg_b_1_out),
		.arg_c_in (),
		.arg_c_out(arg_c_1_out),
		
		.arg_needed(arg_a_needed_in),
		.src(src_a_in),
		.src_reg(src_a_reg_in),
		.fetched_out(arg_a_1_out),
		
		.saturate_disable_in(saturate_disable_in),
		.saturate_disable_out(saturate_disable_1_out),
		
		.signedness_in(signedness_in),
		.signedness_out(signedness_1_out),
		
		.accumulator_needed_in(accumulator_needed_in),
		.accumulator_needed_out(accumulator_needed_1_out),

		.shift_in(shift_in),
		.shift_out(shift_1_out),
		.shift_disable_in(shift_disable_in),
		.shift_disable_out(shift_disable_1_out),
		
		.res_addr_in(res_addr_in),
		.res_addr_out(res_addr_1_out),
		
		.writes_external_in(writes_external_in),
		.writes_external_out(writes_external_1_out),
		
		.writes_channel_in(writes_channel_in),
		.writes_channel_out(writes_channel_1_out),
		.writes_accumulator_in(writes_accumulator_in),
		.writes_accumulator_out(writes_accumulator_1_out),
		
		.commit_id_out(),
		
		.commit_flag_in(commit_flag_in),
		.commit_flag_out(commit_flag_1_out),

		.branch_in(branch_in),
		.branch_out(branch_1_out),
		
		.channel_write_addr(channel_write_addr),
		.channel_write_val(channel_write_val),
		.channel_write_enable(channel_write_enable),
		
		.accumulator_write_enable(accumulator_write_enable)
	);
	
	wire in_ready_2;
	wire out_valid_2;
	wire [$clog2(n_blocks)  - 1 : 0] block_2_out;
	wire signed [data_width - 1 : 0] register_0_2_out;
	wire signed [data_width - 1 : 0] register_1_2_out;
	wire [4 : 0] operation_2_out;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_2_out;
	wire [3 : 0] dest_2_out;
	
	wire [3 : 0] src_a_2_out;
	wire [3 : 0] src_b_2_out;
	wire [3 : 0] src_c_2_out;
	
	wire src_a_reg_2_out;
	wire src_b_reg_2_out;
	wire src_c_reg_2_out;
	
	wire arg_a_needed_2_out;
	wire arg_b_needed_2_out;
	wire arg_c_needed_2_out;
	
	wire signed [data_width - 1 : 0] arg_a_2_out;
	wire signed [data_width - 1 : 0] arg_b_2_out;
	wire signed [data_width - 1 : 0] arg_c_2_out;
	
	wire saturate_disable_2_out;
	wire signedness_2_out;
	wire [4 : 0] shift_2_out;
	wire shift_disable_2_out;
	wire [7 : 0] res_addr_2_out;
	wire writes_external_2_out;
	wire writes_channel_2_out;
	wire writes_accumulator_2_out;
	wire accumulator_needed_2_out;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_2_out;
	wire commit_flag_2_out;
	wire [`N_INSTR_BRANCHES - 1 : 0] branch_2_out;
	
	operand_fetch_substage #(.data_width(data_width), .n_blocks(n_blocks), .last(0)) fetch_2
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
	
		.sample_tick(sample_tick),
		
		.in_valid(out_valid_1),
		.in_ready(in_ready_2),
		
		.out_valid(out_valid_2),
		.out_ready(in_ready_3),
		
		.n_blocks_running(n_blocks_running),
		
		.block_in(block_1_out),
		.block_out(block_2_out),
		
		.register_0_in(register_0_1_out),
		.register_0_out(register_0_2_out),
		.register_1_in(register_1_1_out),
		.register_1_out(register_1_2_out),
		
		.operation_in(operation_1_out),
		.operation_out(operation_2_out),

		.misc_op_in(misc_op_1_out),
		.misc_op_out(misc_op_2_out),
		
		.dest_in(dest_1_out),
		.dest_out(dest_2_out),
		
		.arg_a_needed_in (arg_a_needed_1_out),
		.arg_a_needed_out(arg_a_needed_2_out),
		.arg_b_needed_in (arg_b_needed_1_out),
		.arg_b_needed_out(arg_b_needed_2_out),
		.arg_c_needed_in (arg_c_needed_1_out),
		.arg_c_needed_out(arg_c_needed_2_out),
		.src_a_in (src_a_1_out),
		.src_a_out(src_a_2_out),
		.src_b_in (src_b_1_out),
		.src_b_out(src_b_2_out),
		.src_c_in (src_c_1_out),
		.src_c_out(src_c_2_out),
		.src_a_reg_in (src_a_reg_1_out),
		.src_a_reg_out(src_a_reg_2_out),
		.src_b_reg_in (src_b_reg_1_out),
		.src_b_reg_out(src_b_reg_2_out),
		.src_c_reg_in (src_c_reg_1_out),
		.src_c_reg_out(src_c_reg_2_out),
		.arg_a_in (arg_a_1_out),
		.arg_a_out(arg_a_2_out),
		.arg_b_in (arg_b_1_out),
		//.arg_b_out(arg_b_2_out),
		.arg_c_in (arg_c_1_out),
		.arg_c_out(arg_c_2_out),
		
		.arg_needed(arg_b_needed_1_out),
		.src(src_b_1_out),
		.src_reg(src_b_reg_1_out),
		.fetched_out(arg_b_2_out),
		
		.saturate_disable_in(saturate_disable_1_out),
		.saturate_disable_out(saturate_disable_2_out),
		
		.signedness_in(signedness_1_out),
		.signedness_out(signedness_2_out),
		
		.accumulator_needed_in(accumulator_needed_1_out),
		.accumulator_needed_out(accumulator_needed_2_out),

		.shift_in(shift_1_out),
		.shift_out(shift_2_out),
		.shift_disable_in(shift_disable_1_out),
		.shift_disable_out(shift_disable_2_out),
		
		.res_addr_in(res_addr_1_out),
		.res_addr_out(res_addr_2_out),
		
		.writes_external_in(writes_external_1_out),
		.writes_external_out(writes_external_2_out),
		
		.writes_channel_in(writes_channel_1_out),
		.writes_channel_out(writes_channel_2_out),
		.writes_accumulator_in(writes_accumulator_1_out),
		.writes_accumulator_out(writes_accumulator_2_out),
		
		.commit_id_out(),
		
		.commit_flag_in(commit_flag_1_out),
		.commit_flag_out(commit_flag_2_out),

		.branch_in(branch_1_out),
		.branch_out(branch_2_out),
		
		.channel_write_addr(channel_write_addr),
		.channel_write_val(channel_write_val),
		.channel_write_enable(channel_write_enable),
		
		.accumulator_write_enable(accumulator_write_enable)
	);

	wire in_ready_3;
	wire signed [data_width - 1 : 0] arg_c_fetched_out;
	
	wire out_valid_3;
	wire [$clog2(n_blocks)  - 1 : 0] block_3_out;
	wire signed [data_width - 1 : 0] register_0_3_out;
	wire signed [data_width - 1 : 0] register_1_3_out;
	wire [4 : 0] operation_3_out;
	wire [$clog2(`N_MISC_OPS) - 1 : 0] misc_op_3_out;
	wire [3 : 0] dest_3_out;
	wire [3 : 0] src_a_3_out;
	wire [3 : 0] src_b_3_out;
	wire [3 : 0] src_c_3_out;
	wire src_a_reg_3_out;
	wire src_b_reg_3_out;
	wire src_c_reg_3_out;
	wire arg_a_needed_3_out;
	wire arg_b_needed_3_out;
	wire arg_c_needed_3_out;
	wire signed [data_width - 1 : 0] arg_a_3_out;
	wire signed [data_width - 1 : 0] arg_b_3_out;
	wire signed [data_width - 1 : 0] arg_c_3_out;
	wire saturate_disable_3_out;
	wire signedness_3_out;
	wire [4 : 0] shift_3_out;
	wire shift_disable_3_out;
	wire [7 : 0] res_addr_3_out;
	wire writes_external_3_out;
	wire writes_channel_3_out;
	wire writes_accumulator_3_out;
	wire accumulator_needed_3_out;
	wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_3_out;
	wire commit_flag_3_out;
	wire [`N_INSTR_BRANCHES - 1 : 0] branch_3_out;

	operand_fetch_substage #(.data_width(data_width), .n_blocks(n_blocks), .last(1)) fetch_3
	(
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
	
		.sample_tick(sample_tick),
		
		.in_valid(out_valid_2),
		.in_ready(in_ready_3),
		
		.out_valid(out_valid_3),
		.out_ready(in_ready_skid),
		
		.n_blocks_running(n_blocks_running),
		
		.block_in(block_2_out),
		.block_out(block_3_out),
		
		.register_0_in(register_0_2_out),
		.register_0_out(register_0_3_out),
		.register_1_in(register_1_2_out),
		.register_1_out(register_1_3_out),
		
		.operation_in(operation_2_out),
		.operation_out(operation_3_out),

		.misc_op_in(misc_op_2_out),
		.misc_op_out(misc_op_3_out),
		
		.dest_in(dest_2_out),
		.dest_out(dest_3_out),

		.arg_a_needed_in (arg_a_needed_2_out),
		.arg_a_needed_out(arg_a_needed_3_out),
		.arg_b_needed_in (arg_b_needed_2_out),
		.arg_b_needed_out(arg_b_needed_3_out),
		.arg_c_needed_in (arg_c_needed_2_out),
		.arg_c_needed_out(arg_c_needed_3_out),
		.src_a_in (src_a_2_out),
		.src_a_out(src_a_3_out),
		.src_b_in (src_b_2_out),
		.src_b_out(src_b_3_out),
		.src_c_in (src_c_2_out),
		.src_c_out(src_c_3_out),
		.src_a_reg_in (src_a_reg_2_out),
		.src_a_reg_out(src_a_reg_3_out),
		.src_b_reg_in (src_b_reg_2_out),
		.src_b_reg_out(src_b_reg_3_out),
		.src_c_reg_in (src_c_reg_2_out),
		.src_c_reg_out(src_c_reg_3_out),
		.arg_a_in (arg_a_2_out),
		.arg_a_out(arg_a_3_out),
		.arg_b_in (arg_b_2_out),
		.arg_b_out(arg_b_3_out),
		.arg_c_in (arg_c_2_out),
		//.arg_c_out(arg_c_3_out),
		
		.arg_needed(arg_c_needed_2_out),
		.src(src_c_2_out),
		.src_reg(src_c_reg_2_out),
		.fetched_out(arg_c_3_out),

		.saturate_disable_in(saturate_disable_2_out),
		.saturate_disable_out(saturate_disable_3_out),
		
		.signedness_in(signedness_2_out),
		.signedness_out(signedness_3_out),
		
		.accumulator_needed_in(accumulator_needed_2_out),

		.shift_in(shift_2_out),
		.shift_out(shift_3_out),
		.shift_disable_in(shift_disable_2_out),
		.shift_disable_out(shift_disable_3_out),
		
		.res_addr_in(res_addr_2_out),
		.res_addr_out(res_addr_3_out),
		
		.writes_external_in(writes_external_2_out),
		.writes_external_out(writes_external_3_out),
		
		.writes_channel_in(writes_channel_2_out),
		//.writes_channel_out(writes_channel_3_out),
		.writes_accumulator_in(writes_accumulator_2_out),
		//.writes_accumulator_out(writes_accumulator_3_out),
		
		.commit_id_out(commit_id_3_out),
		
		.commit_flag_in(commit_flag_2_out),
		.commit_flag_out(commit_flag_3_out),

		.branch_in(branch_2_out),
		.branch_out(branch_3_out),
		
		.channel_write_addr(channel_write_addr),
		.channel_write_val(channel_write_val),
		.channel_write_enable(channel_write_enable),
		
		.accumulator_write_enable(accumulator_write_enable)
	);
	
	localparam payload_width = 
		$clog2(n_blocks)+data_width+data_width+5+$clog2(`N_MISC_OPS)+4+data_width+data_width+data_width+1+1+5+1+8+1+1+1+$clog2(`N_INSTR_BRANCHES)+6;
	
	wire in_ready_skid;
	
	wire [payload_width - 1 : 0] skid_buffer_payload_in = 
		{
			block_3_out,
			register_0_3_out,
			register_1_3_out,
			operation_3_out,
			misc_op_3_out,
			dest_3_out,
			arg_a_3_out,
			arg_b_3_out,
			arg_c_3_out,
			saturate_disable_3_out,
			signedness_3_out,
			shift_3_out,
			shift_disable_3_out,
			res_addr_3_out,
			writes_external_3_out,
			commit_id_3_out,
			commit_flag_3_out,
			branch_3_out
		};
	wire [payload_width - 1 : 0] skid_buffer_payload_out;
	
	assign {
			block_out,
			register_0_out,
			register_1_out,
			operation_out,
			misc_op_out,
			dest_out,
			arg_a_out,
			arg_b_out,
			arg_c_out,
			saturate_disable_out,
			signedness_out,
			shift_out,
			shift_disable_out,
			res_addr_out,
			writes_external_out,
			commit_id_out,
			commit_flag_out,
			branch_out
		} = skid_buffer_payload_out;
	
	skid_buffer #(.payload_width(payload_width)) skidder
		(.clk(clk), .reset(reset), .enable(enable),
		.in_ready(in_ready_skid), .in_valid(out_valid_3),
		.out_valid(out_valid), .out_ready(out_ready),
		.payload_in(skid_buffer_payload_in), .payload_out(skid_buffer_payload_out));
	
endmodule

`default_nettype wire
