`include "instr_dec.vh"
`include "block.vh"
`include "lut.vh"
`include "seq.vh"
`include "alu.vh"

module instr_fetch_decode_stage #(parameter data_width = 16, parameter n_blocks = 256, parameter n_block_regs = 2)
	(
		input wire clk,
		input wire reset,
	
		input wire sample_tick,
	
		input wire [$clog2(n_blocks) - 1 : 0] last_block,
		output reg [$clog2(n_blocks) - 1 : 0] instr_read_addr,
		
		input wire [31 : 0] instr_read_val,
		
		output reg out_valid,
		input wire out_ready,
		
		output reg [$clog2(n_blocks) - 1 : 0] block,
		
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
		output reg commits_out
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
		
		.src_a_needed(arc_a_needed),
		.src_b_needed(arc_b_needed),
		.src_c_needed(arc_c_needed),
		
		.branch(branch),
		.commits(commits)
	);
	
	// if the current output has been taken
	wire out_consumed = out_valid && out_ready;
	// if the current output can be replaced
	wire out_free = out_consumed || !out_valid;
	// if the current instruction can be replaced
	wire current_free = !current_instr_valid || out_consumed;
	
	always @(posedge clk) begin
		if (reset) begin
			out_valid   <= 0;
			current_block <= 0;
			instr_read_addr <= 0;
			instr_read_valid  <= 0;
			current_instr_valid <= 0;
			instr_read_valid_next <= 0;
		end else begin
			
			// fragile to refactoring
			if (!instr_read_valid && !instr_read_valid_next)
				instr_read_valid_next <= 1;
			else
				instr_read_valid_next <= 0;
			
			if (instr_read_valid_next)
				instr_read_valid <= 1;
			
			if (out_consumed)
				out_valid <= 0;
			
			if (out_free && current_instr_valid) begin
				block <= current_block;
			
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
				
				branch_out  <= branch;
				commits_out <= commits;
				
				out_valid <= 1;
			end
			
			if (current_free && instr_read_valid) begin
				current_instr <= instr_read_val;
				current_block <= instr_read_addr;
				current_instr_valid <= 1;
				instr_read_valid <= 0;
				instr_read_addr <= (instr_read_addr == last_block) ? 0 : instr_read_addr + 1;
			end else if (out_valid && out_ready) begin
				current_instr_valid <= 0;
			end
		end
	end
endmodule

module operand_fetch_stage #(parameter data_width = 16, parameter n_blocks = 256)
	(
		input wire clk,
		input wire reset,
	
		input wire sample_tick,
	
		input wire [$clog2(n_blocks) - 1 : 0] last_block,
		output reg [$clog2(n_blocks) - 1 : 0] instr_read_addr,
		
		input wire [31 : 0] instr_read_val,
		
		input wire in_valid,
		output reg in_ready,
		
		input wire [$clog2(n_blocks) : 0] block,
		
		input wire [4 : 0] operation_in,
		
		input wire [3 : 0] dest_in,
		input wire dest_acc_in,

		input wire signed [data_width - 1 : 0] src_a_in,
		input wire signed [data_width - 1 : 0] src_b_in,
		input wire signed [data_width - 1 : 0] src_c_in,

		input wire src_a_reg_in,
		input wire src_b_reg_in,
		input wire src_c_reg_in,
		
		input wire saturate_in,
		input wire use_accumulator_in,
		input wire subtract_in,
		input wire signedness_in,

		input wire [4 : 0] instr_shift_in,
		input wire no_shift_in,
		
		input wire [7 : 0] res_addr_in,
		
		input wire arg_a_needed_in,
		input wire arg_b_needed_in,
		input wire arg_c_needed_in,
		
		input wire [`N_INSTR_BRANCHES - 1 : 0] branch,
		input wire commits,
		
		output reg [4 : 0] operation_out,
		
		output reg [3 : 0] dest_out,
		output reg dest_acc_out,

		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		output reg signed [data_width - 1 : 0] arg_c_out,

		output reg saturate_out,
		output reg use_accumulator_out,
		output reg subtract_out,
		output reg signedness_out,

		output reg [4 : 0] instr_shift_out,
		output reg no_shift_out,
		
		output reg [7 : 0] res_addr_out,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		output reg signed [2 * data_width - 1 : 0] accumulator_out,
		
		input wire [3 : 0] channel_write_addr,
		input wire signed [data_width - 1 : 0] channel_write_val,
		input wire channel_write_enable,
		
		output reg [3 : 0] channel_read_addr,
		input wire signed [data_width - 1 : 0] channel_read_val,
		
		output reg [$clog2(n_blocks) + $clog2(n_block_regs) - 1 : 0] reg_read_addr,
		input wire signed [data_width - 1 : 0] reg_read_val,
		
		input wire [2 * data_width - 1 : 0] acc_write_val,
		input wire acc_write_enable,
		
		output reg out_valid [`N_INSTR_BRANCHES - 1 : 0],
		input wire out_ready [`N_INSTR_BRANCHES - 1 : 0]
	);
	
	reg   [3 : 0] channels_scoreboard [15 : 0];
	reg   [3 : 0] acc_pending_writes;
	
	wire accumulator_stall = (use_accumulator_latched && acc_pending_writes != 0);
	
	reg add_pending_write;

	integer i;
	always @(posedge clk) begin
		if (reset) begin
			for (i = 0; i < 16; i = i + 1)
				channels_scoreboard[i] <= 0;
			
			acc_pending_writes <= 0;
		end else begin
			for (i = 0; i < 16; i = i + 1) begin
				case ({add_pending_write && dest_latched == i && !dest_acc_latched,
						channel_write_enable && channel_write_addr == i})
					2'b10: begin
						channels_scoreboard[i] <= channels_scoreboard[i] + 1;
					end
					
					2'b01: begin
						if (channels_scoreboard[i] != 0)
							channels_scoreboard[i] <= channels_scoreboard[i] - 1;
					end
					
					default: begin
						channels_scoreboard[i] <= channels_scoreboard[i];
					end
				endcase
			end
			
			case ({add_pending_write & dest_acc_latched, acc_write_enable})
				2'b10: begin
					acc_pending_writes <= acc_pending_writes + 1;
				end
				
				2'b01: begin
					if (acc_pending_writes != 0)
						acc_pending_writes <= acc_pending_writes - 1;
				end
				
				default: begin
					acc_pending_writes <= acc_pending_writes;
				end
			endcase
		end
	end
	
	reg [$clog2(n_blocks) - 1 : 0] block_latched;
	
	reg [4 : 0] operation_latched;
	
	reg [3 : 0] dest_latched;
	reg dest_acc_latched;

	reg signed [data_width - 1 : 0] src_a_latched;
	reg signed [data_width - 1 : 0] src_b_latched;
	reg signed [data_width - 1 : 0] src_c_latched;
	
	reg src_a_reg_latched;
	reg src_b_reg_latched;
	reg src_c_reg_latched;
	
	reg arg_a_needed_latched;
	reg arg_b_needed_latched;
	reg arg_c_needed_latched;

	reg saturate_latched;
	reg use_accumulator_latched;
	reg subtract_latched;
	reg signedness_latched;

	reg [4 : 0] instr_shift_latched;
	reg no_shift_latched;
	
	reg [7 : 0] res_addr_latched;
	
	reg [`N_INSTR_BRANCHES - 1 : 0] branch_latched;
	reg commits_latched;
	
	wire arg_a_resolved = ~arg_a_needed_latched | arg_a_valid;
	wire arg_b_resolved = ~arg_b_needed_latched | arg_b_valid;
	wire arg_c_resolved = ~arg_c_needed_latched | arg_c_valid;
	
	wire  [3 : 0] current_arg = arg_a_resolved ? (arg_b_resolved ? (src_c_latched) : src_b_latched) : src_a_latched;
	
	wire  [3 : 0] arg_pending_writes = channels_scoreboard[current_arg];
	
	reg 		arg_a_read_issued;
	reg [2 : 0] arg_a_read_shift_register;
	
	reg signed [data_width - 1 : 0] arg_a_latched;
	reg arg_a_valid;
	
	reg 		arg_b_read_issued;
	reg [2 : 0] arg_b_read_shift_register;
	
	reg signed [data_width - 1 : 0] arg_b_latched;
	reg arg_b_valid;
	
	reg 		arg_c_read_issued;
	reg [2 : 0] arg_c_read_shift_register;
	
	reg signed [data_width - 1 : 0] arg_c_latched;
	reg arg_c_valid;
	
	always @(posedge clk) begin
		if (reset) begin
			arg_a_read_issued 		  <= 0;
			arg_a_read_shift_register <= 0;
			arg_a_valid 			  <= 0;
		end else if (state == BUSY) begin
			if (!arg_a_needed_latched) arg_a_valid <= 1;
			if (!arg_b_needed_latched) arg_b_valid <= 1;
			if (!arg_c_needed_latched) arg_c_valid <= 1;
			
			arg_a_read_shift_register <= arg_a_read_shift_register >> 1;
			
			if (!arg_a_resolved) begin
				if (arg_a_read_issued) begin
					if (arg_a_read_shift_register[1]) begin
						arg_a_latched 				<= src_a_reg_latched ? reg_read_val : channel_read_val;
						arg_a_valid 				<= 1;
						arg_a_read_issued 			<= 0;
						arg_a_read_shift_register 	<= 0;
					end
				end else begin
					if (src_a_reg_latched) begin
						reg_read_addr <= {block_latched, src_a_latched[$clog2(n_block_regs) - 1 : 0]};
						arg_a_read_issued <= 1;
						arg_a_read_shift_register <= 3'b100;
					end else begin
						if (arg_pending_writes == 0) begin
							channel_read_addr <= src_a_latched;
							arg_a_read_issued <= 1;
							arg_a_read_shift_register <= 3'b100;
						end else if (arg_pending_writes == 1) begin
							if (channel_write_enable && channel_write_addr == src_a_latched) begin
								arg_a_latched <= channel_read_val;
								arg_a_valid <= 1;
							end
						end
					end
				end
			end else if (!arg_b_resolved) begin
				if (arg_b_read_issued) begin
					if (arg_b_read_shift_register[1]) begin
						arg_b_latched 				<= src_b_reg_latched ? reg_read_val : channel_read_val;
						arg_b_valid 				<= 1;
						arg_b_read_issued 			<= 0;
						arg_b_read_shift_register 	<= 0;
					end
				end else begin
					if (src_b_reg_latched) begin
						reg_read_addr <= {block_latched, src_b_latched[$clog2(n_block_regs) - 1 : 0]};
						arg_b_read_issued <= 1;
						arg_b_read_shift_register <= 3'b100;
					end else begin
						if (arg_pending_writes == 0) begin
							channel_read_addr <= src_b_latched;
							arg_b_read_issued <= 1;
							arg_b_read_shift_register <= 3'b100;
						end else if (arg_pending_writes == 1) begin
							if (channel_write_enable && channel_write_addr == src_b_latched) begin
								arg_b_latched <= channel_read_val;
								arg_b_valid <= 1;
							end
						end
					end
				end
			end else if (!arg_c_resolved) begin
				if (arg_c_read_issued) begin
					if (arg_c_read_shift_register[1]) begin
						arg_c_latched 				<= src_c_reg_latched ? reg_read_val : channel_read_val;
						arg_c_valid 				<= 1;
						arg_c_read_issued 			<= 0;
						arg_c_read_shift_register 	<= 0;
					end
				end else begin
					if (src_c_reg_latched) begin
						reg_read_addr <= {block_latched, src_c_latched[$clog2(n_block_regs) - 1 : 0]};
						arg_c_read_issued <= 1;
						arg_c_read_shift_register <= 3'b100;
					end else begin
						if (arg_pending_writes == 0) begin
							channel_read_addr <= src_c_latched;
							arg_c_read_issued <= 1;
							arg_c_read_shift_register <= 3'b100;
						end else if (arg_pending_writes == 1) begin
							if (channel_write_enable && channel_write_addr == src_c_latched) begin
								arg_c_latched <= channel_read_val;
								arg_c_valid <= 1;
							end
						end
					end
				end
			end
		end else begin
			arg_a_valid <= 0;
			arg_b_valid <= 0;
			arg_c_valid <= 0;
		end
	end
	
	localparam IDLE = 2'd0;
	localparam BUSY = 2'd1;
	localparam DONE = 2'd2;
	
	assign in_ready = (state == IDLE);
	
	reg [1:0] state;
	
	reg [8:0] commit_id;
	
	always @(posedge clk) begin
		if (reset) begin
			state   <= 0;
			out_valid <= '{default:0};
			commit_id   <= 0;
		end else begin
			if (sample_tick) begin
				commit_id <= 0;
			
			add_pending_write <= 0;
		
			case (state)
				IDLE: begin
					if (in_valid) begin
					
						block_latched <= block;
						
						operation_latched <= operation_in;
	
						dest_latched 	 <= dest_in;
						dest_acc_latched <= dest_acc_in;

						src_a_latched <= src_a_in;
						src_b_latched <= src_b_in;
						src_c_latched <= src_c_in;

						src_a_reg_latched <= src_a_reg_in;
						src_b_reg_latched <= src_b_reg_in;
						src_c_reg_latched <= src_c_reg_in;

						arg_a_needed_latched <= arg_a_needed_in;
						arg_b_needed_latched <= arg_b_needed_in;
						arg_c_needed_latched <= arg_c_needed_in;

						saturate_latched 		<= saturate_in;
						use_accumulator_latched <= use_accumulator_in;
						subtract_latched 		<= subtract_in;
						signedness_latched 		<= signedness_in;
						instr_shift_latched 	<= instr_shift_in;
						no_shift_latched 		<= no_shift_in;
						
						res_addr_latched <= res_addr_in;
						
						branch_latched  <= branch;
						commits_latched <= commits;
						
						state <= BUSY;
					end
				end
				
				BUSY: begin
					if (arg_a_resolved && arg_b_resolved && arg_c_resolved && !accumulator_stall) begin
						operation_out <= operation_latched;
		
						dest_out <= dest_latched;
						dest_acc_out <= dest_acc_latched;

						arg_a_out <= arg_a_latched;
						arg_b_out <= arg_b_latched;
						arg_c_out <= arg_c_latched;

						saturate_out 		<= saturate_latched;
						use_accumulator_out <= use_accumulator_latched;
						subtract_out 		<= subtract_latched;
						signedness_out 		<= signedness_latched;

						instr_shift_out <= instr_shift_latched;
						no_shift_out 	<= no_shift_latched;
						
						res_addr_out <= res_addr_latched;
						
						out_valid[branch_latched] <= 1;
						
						add_pending_write <= 1;
						
						state <= DONE;
					end
				end
				
				DONE: begin
					if (out_ready[branch_latched]) begin
						out_valid <= '{default:0};
						state <= IDLE;
					end
				end
			endcase
		end
	end
endmodule


module multiply_stage #(parameter data_width = 16)
	(
		input wire clk,
		input wire reset,
		
		input  wire in_valid,
		output wire in_ready,
		
		input wire [7 : 0] operation_in,
		
		input wire [4:0] shift_in,
		input wire no_shift_in,
		input wire saturate_in,
		input wire signedness_in,
		input wire use_accumulator_in,
		input wire subtract_in,
		
		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		
		input wire [3:0] dest_in,
		input wire dest_acc_in,
		
		output reg [7 : 0] operation_out,
		
		output reg [4:0] shift_out,
		output reg no_shift_out,
		output reg saturate_out,
		output reg signedness_out,
		output reg use_accumulator_out,
		output reg subtract_out,
		
		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		output reg signed [2 * data_width - 1 : 0] product_out,
		
		output reg signed [2 * data_width - 1 : 0] accumulator_out,
		
		output reg [3:0] dest_out,
		output reg dest_acc_out,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else begin
			if (take_in) begin
				out_valid <= 1;
				
				operation_out 		<= operation_in;
				shift_out 			<= shift_in;
				no_shift_out 		<= no_shift_in;
				saturate_out 		<= saturate_in;
				signedness_out 		<= signedness_in;
				use_accumulator_out <= use_accumulator_in;
				subtract_out		<= subtract_in;
				
				product_out 		<= arg_a_in * arg_b_in;
				arg_a_out 			<= arg_a_in;
				arg_b_out 			<= arg_b_in;
				arg_c_out 			<= arg_c_in;
				
				accumulator_out 	<= accumulator_in;
				dest_out 			<= dest_in;
				dest_acc_out 		<= dest_acc_in;
				
				commit_id_out		<= commit_id_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule


module shift_stage #(parameter data_width = 16)
	(
		input wire clk,
		input wire reset,
		
		input  wire in_valid,
		output wire in_ready,
		
		input wire [7 : 0] operation_in,
		
		input wire [4:0] shift_in,
		input wire no_shift_in,
		input wire saturate_in,
		input wire signedness_in,
		input wire use_accumulator_in,
		input wire subtract_in,
		
		input wire signed [2 * data_width - 1 : 0] product_in,
		
		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		
		input wire [3:0] dest_in,
		input wire dest_acc_in,
		
		output reg [7 : 0] operation_out,
		
		output reg saturate_out,
		output reg signedness_out,
		output reg use_accumulator_out,
		output reg subtract_out,
		
		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		output reg signed [2 * data_width - 1 : 0] product_out,
		output reg signed [2 * data_width - 1 : 0] accumulator_out,
		
		output reg [3:0] dest_out,
		output reg dest_acc_out,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	wire shift_arg_a = (operation_in == `BLOCK_INSTR_LSH
					 || operation_in == `BLOCK_INSTR_RSH
					 || operation_in == `BLOCK_INSTR_ARSH);
	
	wire [1:0] arg_a_shift_type =
		(operation_in == `BLOCK_INSTR_ARSH) ? 2'b11 :
			((operation_in == `BLOCK_INSTR_RSH) ? 2'b10 :
				((operation_in == `BLOCK_INSTR_LSH) ? 2'b01
					: 2'b00));
	
	wire [data_width - 1 : 0] arg_a_shifts [3:0];
	
	assign arg_a_shifts[2'b00] = arg_a_in;
	assign arg_a_shifts[2'b01] = arg_a_in  << shift;
	assign arg_a_shifts[2'b10] = arg_a_in  >> shift;
	assign arg_a_shifts[2'b11] = arg_a_in >>> shift;
	
	wire [data_width - 1 : 0] arg_a_shift = arg_a_shifts[arg_a_shift_type];

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else begin
			if (take_in) begin
				out_valid <= 1;
				
				operation_out 		<= operation_in;
				saturate_out 		<= saturate_in;
				signedness_out 		<= signedness_in;
				use_accumulator_out <= use_accumulator_in;
				subtract_out		<= subtract_in;
				
				product_out 		<= (no_shift_in) ? product_in : product_in >>> shift_in;
				arg_a_out 			<= arg_a_shift;
				arg_b_out 			<= arg_b_in;
				arg_c_out 			<= arg_c_in;
				
				accumulator_out 	<= accumulator_in;
				dest_out 			<= dest_in;
				dest_acc_out 		<= dest_acc_in;
				
				commit_id_out		<= commit_id_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module arithmetic_stage #(parameter data_width = 16)
	(
		input wire clk,
		input wire reset,
		
		input  wire in_valid,
		output wire in_ready,
		
		input wire [7 : 0] operation_in,
		
		input wire saturate_in,
		input wire signedness_in,
		input wire use_accumulator_in,
		input wire subtract_in,
		
		input wire signed [2 * data_width - 1 : 0] product_in,
		
		input wire signed [data_width - 1 : 0] arg_a_in,
		input wire signed [data_width - 1 : 0] arg_b_in,
		input wire signed [data_width - 1 : 0] arg_c_in,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		
		input wire [3:0] dest_in,
		input wire dest_acc_in,
		
		output reg saturate_out,
		
		output reg signed [2 * data_width - 1 : 0] result_out,
		
		output reg [3:0] dest_out,
		output reg dest_acc_out,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  =  in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	wire signed [2 * data_width - 1 : 0] arg_a_ext = {{(data_width){signedness_in & arg_a_in[data_width-1]}}, arg_a_in};
	wire signed [2 * data_width - 1 : 0] arg_b_ext = {{(data_width){signedness_in & arg_b_in[data_width-1]}}, arg_b_in};
	wire signed [2 * data_width - 1 : 0] arg_c_ext = {{(data_width){signedness_in & arg_c_in[data_width-1]}}, arg_c_in};
	
	wire signed [2 * data_width - 1 : 0] r_summand = use_accumulator_in ? accumulator_in : arg_c_ext;
	
	wire signed [2 * data_width - 1 : 0] sum = product_in + (subtract_in) ? -r_summand : r_summand;
	
	wire signed [2 * data_width - 1 : 0] clamp = (arg_a_e < arg_b_in) ? arg_b_ext : ((arg_a_in > arg_c_in) ? arg_c_ext : arg_a_ext);
	wire signed [2 * data_width - 1 : 0] abs   = (arg_a_in < 0) ? -arg_a_ext : arg_a_ext;
	
	wire signed [2 * data_width - 1 : 0] result = (operation_in == `BLOCK_INSTR_ABS) ? abs : ((operation_in == `BLOCK_INSTR_CLAMP) ? clamp : sum);

	always @(posedge clk) begin
		if (reset) begin
			out_valid 	<= 0;
		end else begin
			if (take_in) begin
				out_valid <= 1;
				
				saturate_out 		<= saturate_in;
				use_accumulator_out <= use_accumulator_in;
				
				result_out			<= result;
				
				dest_out 			<= dest_in;
				dest_acc_out 		<= dest_acc_in;
				
				commit_id_out		<= commit_id_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module saturate_stage #(parameter data_width = 16)
	(
		input wire clk,
		input wire reset,
		
		input  wire in_valid,
		output wire in_ready,
		
		input wire saturate_in,
		
		input wire [3:0] dest_in,
		input wire dest_acc_in,
		
		input wire signed [2 * data_width - 1 : 0] result_in,
		output reg signed [2 * data_width - 1 : 0] result_out,
		
		output reg [3:0] dest_out,
		output reg dest_acc_out,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out
	);
	
	assign in_ready = ~out_valid | out_ready;
	
	wire take_in  =  in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	wire signed [2 * data_width - 1 : 0] result_in_sat = (result_in > sat_max) ? sat_max : ((result_in < sat_min) ? sat_min : result_in);

	always @(posedge clk) begin
		if (reset) begin
			out_valid 		<= 0;
		end else begin
			if (take_in) begin
				out_valid <= 1;
				
				result_out <= (saturate_in) ? result_in_sat : result_in;
				
				dest_out     <= dest_in;
				dest_acc_out <= dest_acc_in;
				
				commit_id_out <= commit_id_in;
			end else if (take_out) begin
				out_valid <= 0;
			end
		end
	end
endmodule

module resource_branch #(parameter data_width = 16, parameter handle_width = 8)
	(
		input wire clk,
		input wire reset,
		
		input  wire in_valid,
		output wire in_ready,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire write,
		
		input wire [handle_width - 1 : 0] handle_in,
		input wire [data_width   - 1 : 0] arg_a_in,
		input wire [data_width   - 1 : 0] arg_b_in,
		
		output wire read_req,
		output wire write_req,
		
		output reg 		[handle_width - 1 : 0] handle_out,
		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		input wire signed [data_width - 1 : 0] data_in,
		
		input wire read_ready,
		input wire write_ack,
		
		output reg signed [data_width - 1 : 0] data_out,
		
		input wire [8:0] commit_id_in,
		output reg [8:0] commit_id_out
	);
	
	localparam IDLE = 2'd0;
	localparam REQ  = 2'd1;
	localparam DONE = 2'd2;
	
	assign in_ready = (state == IDLE);
	
	assign read_req  = (state == REQ) & ~write_latched;
	assign write_req = (state == REQ) &  write_latched;
	
	assign out_valid = (state == DONE);
	
	reg write_latched;
	reg [1:0] state;
	
	always @(posedge clk) begin
		if (reset) begin
			state   <= 0;
			out_valid <= 0;
			commits_out <= 0;
			commit_id_out <= 0;
		end else begin
			case (state)
				IDLE: begin
					if (in_valid && in_ready) begin
						handle_out <= handle_in;
						
						arg_a_out <= arg_a_in;
						arg_b_out <= arg_b_in;
						
						commit_id_out <= commit_id_in;
						
						write_latched <= write;
						
						state <= 1;
					end
				end
				
				REQ: begin
					if (write_latched && write_ack) begin
						state <= 0;
					end else if (!write_latched && read_ready) begin
						data_out <= data_in;
						out_valid <= 1;
						state <= 2;
					end
				end
				
				DONE: begin
					if (out_ready) begin
						state <= 0;
					end
				end
			endcase
		end
	end
endmodule

module commit_master #(parameter data_width = 16,
					   parameter `N_INSTR_BRANCHES = 2)
	(
		input wire clk,
		input wire reset,
		
		input wire sample_tick,
		
		input wire 						 out_valid [`N_INSTR_BRANCHES - 1 : 0],
		input wire [2 * data_width - 1 : 0] result [`N_INSTR_BRANCHES - 1 : 0],
		input wire [3 : 0] 					  dest [`N_INSTR_BRANCHES - 1 : 0],
		input wire 		 				  dest_acc [`N_INSTR_BRANCHES - 1 : 0],
		input wire 						   commits [`N_INSTR_BRANCHES - 1 : 0],
		input wire [8 : 0]				 commit_id [`N_INSTR_BRANCHES - 1 : 0],
		output reg 						  in_ready [`N_INSTR_BRANCHES - 1 : 0],
		
		output reg [3 : 0] 				channel_write_addr,
		output reg [data_width - 1 : 0] channel_write_val,
		output reg 						channel_write_enable,
		
		output reg [2 * data_width - 1 : 0] acc_write_val,
		output reg acc_write_enable
	);
	
	reg [8:0] next_commit_id;
	
	bit found;
	
	integer i;
	always @(posedge clk) begin	
		
		in_ready <= 0;
		
		acc_write_enable <= 0;
		channel_write_enable <= 0;
		
		found = 0;
		
		if (reset) begin
			next_commit_id <= 0;
		end else if (sample_tick) begin
			next_commit_id <= 0;
		end else begin
			for (i = 0; i < `N_INSTR_BRANCHES && !found; i = i + 1) begin
				if (out_valid[i] && commit_id[i] == next_commit_id) begin
					if (dest_acc[i]) begin
						acc_write_val <= result[i];
						acc_write_enable <= 1;
					end else begin
						channel_write_addr <= dest[i];
						channel_write_val  <= result[i][data_width - 1 : 0];
						channel_write_enable <= 1;
					end
					
					next_commit_id <= next_commit_id + 1;
					in_ready[i] <= 1;
					
					found = 1;
				end
			end
		end
	end
endmodule

module dsp_core_2 #(
		parameter integer data_width 	= 16,
		parameter integer n_blocks		= 256,
		parameter integer n_channels   	= 16,
		parameter integer n_block_regs  = 2,
		parameter integer memory_size	= n_blocks
	) (
		input wire clk,
		input wire reset,
		
		input wire enable,
		input wire tick,
	
		input wire signed [data_width - 1 : 0] sample_in,
		output reg signed [data_width - 1 : 0] sample_out,
		
		output reg ready,
		
		input wire command_reg_write,
		input wire command_instr_write,
		
		input wire [$clog2(n_blocks)  	 - 1 : 0] command_block_target,
		input wire [$clog2(n_block_regs) - 1 : 0] command_reg_target,
		input wire [31					     : 0] command_instr_write_val,
		input wire signed [data_width 	 - 1 : 0] command_reg_write_val,
		
		output reg lut_req,
		output reg signed [data_width - 1 : 0] lut_handle,
		output reg signed [data_width - 1 : 0] lut_arg,
		input wire signed [data_width - 1 : 0] lut_data,
		input wire lut_ready,
		
		output reg delay_read_req,
		output reg delay_write_req,
		output reg signed [data_width - 1 : 0] delay_req_handle,
		output reg signed [data_width - 1 : 0] delay_req_arg,
		input wire signed [data_width - 1 : 0] delay_req_data_in,
		input wire delay_read_ready,
		input wire delay_write_ack,
		
		input wire full_reset,
		output reg resetting
	);
	
	localparam signed sat_max = {{(data_width + 1){1'b0}}, {(data_width - 1){1'b1}}};
	localparam signed sat_min = {{(data_width + 1){1'b1}}, {(data_width - 1){1'b0}}};
	
	localparam block_addr_w 	= $clog2(n_blocks);
	localparam mem_addr_w 		= $clog2(memory_size);
	localparam ch_addr_w 		= $clog2(n_channels);
	localparam reg_addr_w		= $clog2(n_blocks) + $clog2(n_block_regs);
	
	reg [31 			  : 0] instrs [n_blocks - 1 : 0];
	
	reg [block_addr_w - 1 : 0] instr_read_addr;
	reg [block_addr_w - 1 : 0] instr_write_addr;
	reg [31 			  : 0] instr_read_val;
	reg [31 			  : 0] instr_write_val;
	
	reg instr_write_enable;
	
	reg [data_width   - 1 : 0] block_regs [n_block_regs * n_blocks - 1 : 0];
	
	reg [reg_addr_w - 1 : 0] reg_read_addr;
	reg [reg_addr_w - 1 : 0] reg_write_addr;
	reg [data_width - 1 : 0] reg_read_val;
	reg [data_width - 1 : 0] reg_write_val;
	reg reg_write_enable;
	
	reg signed [data_width - 1 : 0] channels [n_channels - 1 : 0];
	
	reg [ch_addr_w  - 1 : 0] channel_read_addr;
	reg [ch_addr_w  - 1 : 0] channel_write_addr;
	reg [data_width - 1 : 0] channel_read_val;
	reg [data_width - 1 : 0] channel_write_val;
	
	reg channel_write_enable;
	
	reg signed [data_width - 1 : 0] mem [memory_size - 1 : 0];
	
	reg [mem_addr_w - 1 : 0] mem_read_addr;
	reg [mem_addr_w - 1 : 0] mem_write_addr;
	reg [data_width - 1 : 0] mem_read_val;
	reg [data_width - 1 : 0] mem_write_val;
	
	reg mem_write_enable;
	
	always @(posedge clk) begin
		instr_read_val <= instrs[instr_read_addr];
		if (instr_write_enable)
            instrs[instr_write_addr] <= instr_write_val;
    end

    always @(posedge clk) begin
		channel_read_val <= channels[channel_read_addr];
		if (channel_write_enable)
            channels[channel_write_addr] <= channel_write_val;
    end

    always @(posedge clk) begin
		reg_read_val <= block_regs[reg_read_addr];
		if (reg_write_enable)
            block_regs[reg_write_addr] <= reg_write_val;
    end

    always @(posedge clk) begin
		mem_read_val <= mem[mem_read_addr];
		if (mem_write_enable)
            mem[mem_write_addr] <= mem_write_val;
    end
	
	reg [31 : 0] instr;
	
	reg latch_instr;
	reg latch_instr_next;
	reg latch_instr_now;
	
	reg [31 : 0] next_instr;
	
	reg latch_next_instr;
	reg latch_next_instr_next;
	reg latch_next_instr_now;
		
	wire [4 : 0] operation;
	
	wire [3 : 0] src_a;
	wire [3 : 0] src_b;
	wire [3 : 0] src_c;
	wire [3 : 0] dest;
	
	wire src_a_reg;
	wire src_b_reg;
	wire src_c_reg;

	wire src_a_needed;
	wire src_b_needed;
	wire src_c_needed;

	wire saturate;

	wire [4 : 0] instr_shift;
	wire no_shift;
	
	wire [7 : 0] res_addr;
	
	instr_decoder #(.data_width(data_width)) dec(
		.clk(clk),
		
		.instr(instr),
		
		.operation(operation),
		
		.src_a(src_a),
		.src_b(src_b),
		.src_c(src_c),
		.dest(dest),
		
		.src_a_reg(src_a_reg),
		.src_b_reg(src_b_reg),
		.src_c_reg(src_c_reg),
		
		.src_a_needed(src_a_needed),
		.src_b_needed(src_b_needed),
		.src_c_needed(src_c_needed),

		.saturate(saturate),

		.instr_shift(instr_shift),
		
		.res_addr(res_addr),
		
		.no_shift(no_shift)
	);
	
	reg alu_trigger;
	
	reg  [7:0] alu_op;
	reg  signed [data_width - 1 : 0] alu_a;
	reg  signed [data_width - 1 : 0] alu_b;
	reg  signed [data_width - 1 : 0] alu_c;
	
	reg  signed [2 * data_width - 1 : 0] alu_a_wide;
	reg  signed [2 * data_width - 1 : 0] alu_b_wide;
	
	reg [$clog2(data_width) - 1 : 0] alu_shift;

	reg alu_no_shift;
	reg alu_saturate;
	
	wire signed [    data_width - 1 : 0] alu_result;
	wire signed [2 * data_width - 1 : 0] alu_result_wide;
	
	reg signed [data_width - 1 : 0] alu_result_latched;
	reg write_alu_result;
	
	reg signed [data_width - 1 : 0] lut_result_latched;
	reg write_lut_result;
	
	reg signed [data_width - 1 : 0] delay_result_latched;
	reg write_delay_result;
	
	reg signed [data_width - 1 : 0] mem_result_latched;
	reg write_mem_result;
	
	wire alu_result_valid;
	wire alu_ready;

	// (input|output)[ \t]*(wire|reg)[ \t]*(signed|)[ \t]*(\[[a-z_0-9: \-\+\*]*\]|)[ \t]*([a-z_]*)
	// .\5(\5)
	dsp_core_alu #(.data_width(data_width)) alu
	(
		.clk(clk),
		.reset(reset),
		
		.trigger(alu_trigger),
		
		.op(alu_op),
		.a(alu_a),
		.b(alu_b),
		.c(alu_c),
		
		.a_wide(alu_a_wide),
		.b_wide(alu_b_wide),
		
		.shift(alu_shift),
		.no_shift(no_shift),
		.saturate(saturate),
		
		.result(alu_result),
		.result_wide(alu_result_wide),
		
		.result_valid(alu_result_valid),
		.ready(alu_ready)
	);
	
	reg  [$clog2(n_blocks) : 0] current_block;
	reg  [$clog2(n_blocks) : 0]    last_block;
	wire [$clog2(n_blocks) : 0]    next_block = (current_block == last_block) ? 0 : current_block + 1;

	reg latch_src_a_next;
	reg latch_src_a;
	reg src_a_valid;
	
	reg signed [data_width - 1 : 0] src_a_latched;

	reg latch_src_b_next;
	reg latch_src_b;
	reg src_b_valid;
	
	reg signed [data_width - 1 : 0] src_b_latched;

	reg latch_src_c_next;
	reg latch_src_c;
	reg src_c_valid;
	
	reg signed [data_width - 1 : 0] src_c_latched;
	
	always @(posedge clk) begin
		if (reset) begin
			latch_src_a_next <= 0;
			latch_src_a		 <= 0;
			src_a_valid	 	 <= 0;
			latch_src_b_next <= 0;
			latch_src_b		 <= 0;
			src_b_valid	 	 <= 0;
			latch_src_c_next <= 0;
			latch_src_c		 <= 0;
			src_c_valid	 	 <= 0;
        end else if (full_reset | exec_done | (tick & enable)) begin
			latch_src_a_next <= 0;
			latch_src_a		 <= 0;
			src_a_valid	 	 <= 0;
			latch_src_b_next <= 0;
			latch_src_b		 <= 0;
			src_b_valid	 	 <= 0;
			latch_src_c_next <= 0;
			latch_src_c		 <= 0;
			src_c_valid	 	 <= 0;
		end else if (latch_src_a_next) begin
			latch_src_a_next <= 0;
			latch_src_a 	 <= 1;
		end else if (latch_src_a) begin
			src_a_latched 	<= (src_a_reg) ? reg_read_val : channel_read_val;
			latch_src_a 	<= 0;
			src_a_valid 	<= 1;
		end else if (src_a_needed & ~src_a_valid) begin
			if (src_a_reg)
				reg_read_addr <= {current_block, src_a[0]};
			else
				channel_read_addr <= src_a;
			latch_src_a_next <= 1;
		end
		
		if (latch_src_b_next) begin
			latch_src_b_next <= 0;
			latch_src_b 	 <= 1;
		end else if (latch_src_b) begin
			src_b_latched 	<= (src_b_reg) ? reg_read_val : channel_read_val;
			latch_src_b 	<= 0;
			src_b_valid 	<= 1;
		end else if (src_b_needed & ~src_b_valid && (!(src_a_needed & ~src_a_valid) | latch_src_a_next)) begin
			if (src_b_reg)
				reg_read_addr <= {current_block, src_b[0]};
			else
				channel_read_addr <= src_b;
			latch_src_b_next <= 1;
		end
		
		if (latch_src_c_next) begin
			latch_src_c_next <= 0;
			latch_src_c 	 <= 1;
		end else if (latch_src_c) begin
			src_c_latched 	<= (src_c_reg) ? reg_read_val : channel_read_val;
			latch_src_c 	<= 0;
			src_c_valid 	<= 1;
		end else if (src_c_needed & ~src_c_valid
			&& (!(src_a_needed & ~src_a_valid) | latch_src_a) && (!(src_b_needed & ~src_b_valid) | latch_src_b_next))
		begin
			if (src_c_reg)
				reg_read_addr <= {current_block, src_c[0]};
			else
				channel_read_addr <= src_c;
			latch_src_c_next <= 1;
		end
	end
	
	reg [7:0] state;
	reg executing;
	reg exec_done;
	reg block_boundary;
	
	// Sequential reset counters
	reg [$clog2(memory_size) : 0] mem_reset_ctr;
	reg [$clog2(n_blocks) 	 : 0] blk_reset_ctr;
	
	always @(posedge clk) begin
		if (reset) begin
			ready		<= 1;
			latch_instr <= 1;
			
			executing 	   <= 0;
			block_boundary <= 0;
			
			current_block <= 0;
			state 		  <= 0;
        end else if (full_reset) begin
            resetting <= 1;
				
            blk_reset_ctr <= 0;
            
            ready <= 0;
            
            executing 	   <= 0;
			block_boundary <= 0;
			
			current_block <= 0;
			state 		  <= 0;
		end else if (resetting) begin
			ready <= 0;
			
			if (blk_reset_ctr < n_blocks) begin
				instr_write_addr 	<= blk_reset_ctr;
				instr_write_val 	<= 0;
				instr_write_enable	<= 1;
				
				blk_reset_ctr <= blk_reset_ctr + 1;
			end
			
			if (mem_reset_ctr >= memory_size && blk_reset_ctr >= n_blocks) begin
				resetting 	<= 0;
				state 		<= 0;
				latch_instr <= 1;
				ready		<= 1;
			end
			
		end else begin
            latch_next_instr <= 0;
            block_boundary   <= 0;

            instr_write_enable   	<= 0;
            channel_write_enable 	<= 0;
            reg_write_enable     	<= 0;

            latch_instr <= 0;
            latch_instr_next <= 0;
            latch_instr_now <= 0;

            latch_next_instr <= 0;
            latch_next_instr_next <= 0;
            latch_next_instr_now <= 0;

            if (latch_instr_now) instr <= instr_read_val;
            if (latch_instr_next) latch_instr_now <= 1;
            if (latch_instr) begin
                instr_read_addr <= current_block;
                latch_instr_next <= 1;
                latch_next_instr <= 1;
            end

            if (latch_next_instr_now) next_instr <= instr_read_val;
            if (latch_next_instr_next) latch_next_instr_now <= 1;
            if (latch_next_instr) begin
                instr_read_addr <= next_block;
                latch_next_instr_next <= 1;
            end

            if (command_reg_write || command_instr_write) begin
                if (command_reg_write) begin
                    reg_write_addr   <= {command_block_target, command_reg_target[0]};
                    reg_write_val    <= command_reg_write_val;
                    reg_write_enable <= 1;
                end
                
                if (command_instr_write) begin
                    instr_write_addr   <= command_block_target;
                    instr_write_val    <= command_instr_write_val;
                    instr_write_enable <= 1;
                    
                    last_block <= (command_block_target > last_block) ? command_block_target : last_block;
                    latch_instr <= 1;
                end
            end else begin
                case (state)
                    0: begin
                        if (tick && enable) begin
                            channel_write_addr 	 <= 0;
                            channel_write_val  	 <= sample_in;
                            channel_write_enable <= 1;
                            latch_next_instr <= 1;
                            
                            executing <= 1;
                            state <= 1;
                            ready <= 0;
                        end
                    end
                    
                    1: begin
                        state <= 2;
                    end
                    
                    2: begin
                        if (exec_done) begin
                            block_boundary 	 <= 1;
                            latch_next_instr <= 1;
                            instr 			 <= next_instr;
                            current_block 	 <= next_block;
                            
                            state 			 <= 1;
                            
                            channel_write_addr <= dest;
                            
                            if (write_result) begin
                                channel_write_val 	 <= result_sat;
                                channel_write_enable <= 1;
                            end
                            
                            if (write_alu_result) begin
                                channel_write_val 	 <= alu_result_latched;
                                channel_write_enable <= 1;
                            end
                            
                            if (write_lut_result) begin
                                channel_write_val 	 <= lut_result_latched;
                                channel_write_enable <= 1;
                            end
                            
                            if (write_delay_result) begin
                                channel_write_val 	 <= delay_result_latched;
                                channel_write_enable <= 1;
                            end
                            
                            if (write_mem_result) begin
                                channel_write_val 	 <= mem_result_latched;
                                channel_write_enable <= 1;
                            end
                            
                            if (current_block == last_block) begin
                                state <= 3;
                            end
                        end
                    end
                    
                    3: begin
                        state <= 4;
                    end
                    
                    4: begin
                        ready <= 1;
                        executing <= 0;
                        sample_out <= channels[0];
                        state <= 0;
                    end
                endcase
            end
        end
    end
	
	reg [7:0] exec_state;
	
	reg signed [data_width - 1 : 0] shift;
	
	reg latch_shift_next;
	reg latch_shift;
	
	reg  signed [2 * data_width - 1 : 0] accumulator;
	reg  signed [2 * data_width - 1 : 0] accumulator_sat = (accumulator > sat_max) ? sat_max : ((accumulator < sat_min) ? sat_min : accumulator);
	wire signed [    data_width - 1 : 0] upper_accumulator = accumulator[2 * data_width - 1 : data_width];
	wire signed [    data_width - 1 : 0] lower_accumulator = accumulator[    data_width - 1 :          0];
	
	always @(posedge clk) begin
		latch_shift_next <= 0;
		latch_shift 	 <= 0;
		
		if (latch_shift_next)
			latch_shift <= 1;
		
		if (latch_shift)
			shift <= data_width - 1 - instr_shift;
		
		if (block_boundary | tick)
			latch_shift_next <= 1;
	end
	
	reg  signed [    data_width - 1 : 0] result;
	reg  signed [2 * data_width - 1 : 0] result_wide;
	wire signed [data_width - 1 : 0] result_sat = saturate ? ((result > sat_max) ? sat_max : ((result < sat_min) ? sat_min : result)) : result;
	reg write_result;
	
    reg alu_a_hold = 0;
    reg alu_b_hold = 0;

    reg alu_a_latch_delay = 0;
    reg alu_b_latch_delay = 0;

	always @(posedge clk) begin
		if (reset) begin
			exec_done  <= 0;
			exec_state <= 0;

            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;
        end else if (full_reset) begin
            exec_done  <= 0;
			exec_state <= 0;

            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;

            if (full_reset)
                mem_reset_ctr <= 0;
		end else if (resetting) begin
            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;

			if (mem_reset_ctr < memory_size) begin
				mem_write_addr 	 <= mem_reset_ctr;
				mem_write_val 	 <= 0;
				mem_write_enable <= 1;
				
				mem_reset_ctr <= mem_reset_ctr + 1;
			end
        end else begin
            write_result <= 0;
            alu_trigger  <= 0;
            
            write_alu_result 	<= 0;
            write_lut_result 	<= 0;
            write_delay_result 	<= 0;
            write_mem_result 	<= 0;

            mem_write_enable    <= 0;

            if (!alu_a_hold) begin
                alu_a <= src_a_latched;
                if (alu_a_latch_delay)
                    alu_a <= delay_req_data_in;
            end

            if (!alu_b_hold) begin
                alu_b <= src_b_latched;
                if (alu_b_latch_delay)
                    alu_b <= delay_req_data_in;
            end

            delay_req_handle <= res_addr;

            lut_handle  <= res_addr;
            lut_arg		<= src_a_latched;

            if (exec_done | block_boundary | tick) begin
                exec_done  <= 0;
                exec_state <= 0;
            end else if (executing) begin
                case (operation)
                    `BLOCK_INSTR_NOP: begin
                        if (exec_state == 0) exec_state <= 1;
                        else exec_done <= 1;
                    end
                
                    `BLOCK_INSTR_ADD: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid) begin
                                    alu_op <= `ALU_OP_ADD;
                                    exec_state <= 1;
                                end
                            end
                            1: exec_state <= 2;
                            2: begin
                                alu_result_latched <= alu_result;
                                
                                exec_done 	 <= 1;
                                write_alu_result <= 1;
                                exec_state   <= 3;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_SUB: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid) begin
                                    alu_op <= `ALU_OP_SUB;
                                    exec_state <= 1;
                                end
                            end
                            1: exec_state <= 2;
                            2: begin
                                alu_result_latched <= alu_result;
                                write_alu_result <= 1;
                                
                                exec_done 	 <= 1;
                                exec_state   <= 3;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_CLAMP: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid) begin
                                    alu_op <= `ALU_OP_CLAMP;
                                    exec_state <= 1;
                                end
                            end
                            1: exec_state <= 2;
                            2: begin
                                alu_result_latched <= alu_result;
                                write_alu_result <= 1;
                                
                                exec_done 	 <= 1;
                                exec_state   <= 3;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_ABS: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid) begin
                                    alu_op <= `ALU_OP_ABS;
                                    exec_state <= 1;
                                end
                            end
                            1: exec_state <= 2;
                            2: begin
                                alu_result_latched <= alu_result;
                                write_alu_result <= 1;
                                
                                exec_done 	 <= 1;
                                exec_state   <= 3;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_LSH: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid) begin
                                    alu_op <= `ALU_OP_LSH;
                                    alu_b  <= src_b;
                                    alu_trigger <= 1;
                                    exec_state  <= 1;
                                end
                            end
                            1: begin
                                if (alu_result_valid) begin
                                    alu_result_latched <= alu_result;
                                    write_alu_result <= 1;
                                    exec_done 	 <= 1;
                                    exec_state	 <= 2;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_RSH: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid) begin
                                    alu_op <= `ALU_OP_RSH;
                                    alu_b  <= src_b;
                                    alu_trigger <= 1;
                                    exec_state  <= 1;
                                end
                            end
                            1: begin
                                if (alu_result_valid) begin
                                    alu_result_latched <= alu_result;
                                    write_alu_result <= 1;
                                    exec_done 	 <= 1;
                                    exec_state	 <= 2;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_ARSH: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid) begin
                                    alu_op <= `ALU_OP_ARSH;
                                    alu_b  <= src_b;
                                    alu_trigger <= 1;
                                    exec_state  <= 1;
                                end
                            end
                            1: begin
                                if (alu_result_valid) begin
                                    alu_result_latched <= alu_result;
                                    write_alu_result <= 1;
                                    exec_done 	 <= 1;
                                    exec_state	 <= 2;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_MUL: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid) begin
                                    alu_op 		<= `ALU_OP_MUL;
                                    alu_shift 	<= shift;
                                    alu_trigger <= 1;
                                    
                                    exec_state <= 1;
                                end
                            end
                            
                            1: begin
                                if (alu_result_valid) begin
                                    alu_result_latched <= alu_result;
                                    write_alu_result <= 1;
                                    exec_done 	 <= 1;
                                    exec_state	 <= 2;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_MADD: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid && src_c_valid) begin
                                    alu_op 		<= `ALU_OP_MADD;
                                    alu_c  		<= src_c_latched;
                                    alu_shift 	<= shift;
                                    alu_trigger <= 1;
                                    
                                    exec_state <= 1;
                                end
                            end
                            
                            1: begin
                                if (alu_result_valid) begin
                                    alu_result_latched <= alu_result;
                                    write_alu_result <= 1;
                                    exec_done 	 <= 1;
                                    exec_state	 <= 2;
                                end
                            end
                        endcase
                    end
                        
                    `BLOCK_INSTR_MACZ: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid) begin
                                    alu_op 		<= `ALU_OP_MAC;
                                    alu_b_wide  <= 0;
                                    alu_no_shift<= no_shift;
                                    alu_shift 	<= shift;
                                    alu_trigger <= 1;
                                    
                                    exec_state <= 1;
                                end
                            end
                            
                            1: begin
                                if (alu_result_valid) begin
                                    accumulator <= alu_result_wide;
                                    exec_done 	<= 1;
                                    exec_state 	<= 2;
                                end
                            end
                        endcase
                    end
                        
                    `BLOCK_INSTR_MAC: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid) begin
                                    alu_op 		<= `ALU_OP_MAC;
                                    alu_b_wide	<= accumulator;
                                    alu_shift 	<= shift;
                                    alu_trigger <= 1;
                                    
                                    exec_state <= 1;
                                end
                            end
                            
                            1: begin
                                if (alu_result_valid) begin
                                    accumulator <= alu_result_wide;
                                    exec_done 	<= 1;
                                    exec_state 	<= 2;
                                end
                            end
                        endcase
                    end
                        
                    `BLOCK_INSTR_LINTERP: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid && src_b_valid && src_c_valid) begin
                                    alu_op 		<= `ALU_OP_LINTERP;
                                    alu_c 		<= src_c_latched;
                                    alu_trigger <= 1;
                                    
                                    exec_state <= 1;
                                end
                            end
                            
                            1: begin
                                if (alu_result_valid) begin
                                    accumulator <= alu_result;
                                    exec_done 	<= 1;
                                    exec_state 	<= 2;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_MOV: begin
                        if (src_a_valid) begin
                            result 		 <= src_a_latched;
                            exec_done 	 <= 1;
                            write_result <= 1;
                        end
                    end
                    
                    `BLOCK_INSTR_MOV_ACC: begin
                        case (exec_state)
                            0: begin
                                exec_state <= 1;
                            end
                            
                            1: begin
                                result <= (saturate) ? accumulator_sat[data_width - 1 : 0] : accumulator[data_width - 1 : 0];
                                write_result <= 1;
                                exec_done <= 1;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_MOV_UACC: begin
                        case (exec_state)
                            0: begin
                                exec_state <= 1;
                            end
                            
                            1: begin
                                result 		 <= accumulator[2 * data_width - 1 : data_width];
                                write_result <= 1;
                                exec_done 	 <= 1;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_CLEAR_ACC: begin
                        case (exec_state)
                            0: begin
                                exec_state <= 1;
                                accumulator <= 0;
                            end
                            
                            1: begin
                                exec_done <= 1;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_ACC: begin
                        case (exec_state)
                            0: begin
                                exec_state <= 1;
                            end
                        
                            1: begin
                                if (src_a_valid) begin
                                    accumulator <= (no_shift) ? accumulator + {{(data_width){src_a_latched[data_width-1]}}, src_a_latched} : accumulator + {{(data_width){src_a_latched[data_width - 1]}}, src_a_latched};
                                    exec_done <= 1;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_LOAD_ACC: begin
                        case (exec_state)
                            0: begin
                                mem_read_addr <= res_addr;
                                exec_state <= 1;
                            end
                            
                            1: begin
                                mem_read_addr <= res_addr + 1;
                                exec_state <= 2;
                            end
                            
                            2: begin
                                accumulator <= {accumulator[data_width - 1 : 0], mem_read_val};
                                exec_state <= 3;
                            end
                            
                            3: begin
                                accumulator <= {accumulator[data_width - 1 : 0], mem_read_val};
                                exec_done 	<= 1;
                                exec_state 	<= 4;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_SAVE_ACC: begin
                        case (exec_state)
                            0: begin
                                mem_write_addr 		<= res_addr;
                                mem_write_val 		<= accumulator[2 * data_width - 1 : data_width];
                                mem_write_enable 	<= 1;
                                exec_state 			<= 1;
                            end
                            1: begin
                                mem_write_addr 		<= res_addr + 1;
                                mem_write_val 		<= accumulator[data_width - 1 : 0];
                                mem_write_enable 	<= 1;
                                exec_state 			<= 2;
                                exec_done 			<= 1;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_SAVE: begin
                        if (src_a_valid) begin
                            mem_write_addr   <= res_addr;
                            mem_write_val    <= src_a_latched;
                            mem_write_enable <= 1;
                            
                            exec_done 	 <= 1;
                        end
                    end
                    
                    `BLOCK_INSTR_LOAD: begin
                        case (exec_state)
                            0: begin
                                mem_read_addr <= res_addr;
                                exec_state <= 1;
                            end
                            1: exec_state <= 2;
                            2: begin
                                mem_result_latched <= mem_read_val;
                                write_mem_result <= 1;
                                exec_done <= 1;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_LUT: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid) begin
                                    lut_req		<= 1;
                                    exec_state 	<= 1;
                                end
                            end
                            1: exec_state <= 2;
                            2: begin
                                if (lut_ready) begin
                                    lut_result_latched <= lut_data;
                                    write_lut_result <= 1;
                                    exec_done <= 1;
                                    lut_req <= 0;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_DELAY_READ: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid) begin
                                    delay_req_arg		<= src_a_latched;
                                    delay_read_req		<= 1;
                                    exec_state 			<= 1;
                                end
                            end
                            1: exec_state <= 2;
                            2: begin
                                if (delay_read_ready) begin
                                    delay_result_latched <= delay_req_data_in;
                                    write_delay_result 	<= 1;
                                    exec_done 		<= 1;
                                    delay_read_req 	<= 0;
                                end
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_DELAY_WRITE: begin
                        case (exec_state)
                            0: begin
                                if (src_a_valid) begin
                                    delay_req_arg		<= src_a_latched;
                                    delay_write_req		<= 1;
                                    exec_state 			<= 1;
                                end
                            end
                            1: begin
                                delay_write_req <= 0;
                                exec_done		<= 1;
                            end
                        endcase
                    end
                    
                    `BLOCK_INSTR_FRAC_DELAY: begin
                        case (exec_state)
                            0: begin
                                delay_req_arg 	 <= accumulator[2*data_width-1] ? 0 : upper_accumulator;
                                delay_read_req 	 <= 1;

                                alu_a_latch_delay <= 1;
                                alu_b_latch_delay <= 1;
                                
                                exec_state <= 1;
                            end
                            
                            1: begin
                                exec_state <= 2;
                            end
                            
                            2: begin
                                if (delay_read_ready) begin
                                    alu_a_hold <= 1;
                                    alu_a_latch_delay <= 0;

                                    delay_req_arg 	<= delay_req_arg + 1;
                                    delay_read_req 	<= 1;
                                    exec_state 		<= 3;
                                end
                            end
                            
                            3: begin
                                if (delay_read_ready) begin
                                    alu_b_hold <= 1;
                                    alu_b_latch_delay <= 0;

                                    delay_read_req 	<= 0;
                                    alu_c 			<= lower_accumulator;
                                    alu_op 			<= `ALU_OP_LINTERP;
                                    alu_trigger 	<= 1;
                                    
                                    exec_state <= 4;
                                end
                            end
                            
                            4: begin
                                if (alu_result_valid) begin
                                    alu_result_latched <= alu_result;
                                    
                                    alu_a_hold <= 0;
                                    alu_b_hold <= 0;

                                    write_alu_result <= 1;
                                    
                                    exec_done  <= 1;
                                    exec_state <= 5;
                                end
                            end
                        endcase
                    end
                endcase
            end
        end
    end
endmodule

