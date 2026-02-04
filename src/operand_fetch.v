`include "instr_dec.vh"
`include "block.vh"
`include "lut.vh"
`include "seq.vh"
`include "alu.vh"

module operand_fetch_stage #(parameter data_width = 16, parameter n_blocks = 256, parameter n_block_regs = 2)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
	
		input wire sample_tick,
		
		input wire  in_valid,
		output wire in_ready,
		
		output reg [`N_INSTR_BRANCHES - 1 : 0] out_valid,
		input wire [`N_INSTR_BRANCHES - 1 : 0] out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] n_blocks_running,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire [4 : 0] operation_in,
		
		input wire [3 : 0] dest_in,
		input wire dest_acc_in,

		input wire signed [3 : 0] src_a_in,
		input wire signed [3 : 0] src_b_in,
		input wire signed [3 : 0] src_c_in,

		input wire src_a_reg_in,
		input wire src_b_reg_in,
		input wire src_c_reg_in,
		
		input wire saturate_in,
		input wire use_accumulator_in,
		input wire subtract_in,
		input wire signedness_in,

		input wire [4 : 0] shift_in,
		input wire no_shift_in,
		
		input wire [7 : 0] res_addr_in,
		
		input wire arg_a_needed_in,
		input wire arg_b_needed_in,
		input wire arg_c_needed_in,
		
		input wire [`N_INSTR_BRANCHES - 1 : 0] branch,
		input wire commits,
		
		input wire ext_write_in,
		
		output reg [4 : 0] operation_out,
		
		output reg [3 : 0] dest_out,
		output reg dest_acc_out,

		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		output reg signed [2 * data_width - 1 : 0] accumulator_out,

		output reg saturate_out,
		output reg use_accumulator_out,
		output reg subtract_out,
		output reg signedness_out,

		output reg [4 : 0] shift_out,
		output reg no_shift_out,
		
		output reg [7 : 0] res_addr_out,
		
		output reg commits_out,
		output reg [8:0] commit_id_out,
		output reg ext_write_out,
		
		input wire [3 : 0] channel_write_addr,
		input wire signed [data_width - 1 : 0] channel_write_val,
		input wire channel_write_enable,
		
		output reg [3 : 0] channel_read_addr,
		input wire signed [data_width - 1 : 0] channel_read_val,
		
		output reg [$clog2(n_blocks) + $clog2(n_block_regs) - 1 : 0] reg_read_addr,
		input wire signed [data_width - 1 : 0] reg_read_val,
		
		input wire [2 * data_width - 1 : 0] acc_write_val,
		input wire acc_write_enable
	);
	
	reg   [3 : 0] channels_scoreboard [15 : 0];
	reg   [3 : 0] acc_pending_writes;
	
	wire accumulator_stall = (use_accumulator_latched && acc_pending_writes != 0);
	
	reg add_pending_write;
	
	reg inject_pending_ch0_write;

	integer i;
	always @(posedge clk) begin
		if (reset) begin
			for (i = 0; i < 16; i = i + 1)
				channels_scoreboard[i] <= 0;
			
			acc_pending_writes <= 0;
		end else if (enable) begin
			for (i = 0; i < 16; i = i + 1) begin
				case ({(add_pending_write && dest_latched == i && !dest_acc_latched) ||
						(inject_pending_ch0_write && i == 0),
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

	reg signed [data_width - 1 : 0] accumulator_latched;
	
	reg saturate_latched;
	reg use_accumulator_latched;
	reg subtract_latched;
	reg signedness_latched;

	reg [4 : 0] shift_latched;
	reg no_shift_latched;
	
	reg [7 : 0] res_addr_latched;
	
	reg [`N_INSTR_BRANCHES - 1 : 0] branch_latched;
	reg commits_latched;
	reg ext_write_latched;
	
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
	
	reg caught;
	
	always @(posedge clk) begin
		if (reset) begin
			arg_a_read_issued 		  <= 0;
			arg_a_read_shift_register <= 0;
			arg_a_valid 			  <= 0;
			arg_b_read_issued 		  <= 0;
			arg_b_read_shift_register <= 0;
			arg_b_valid 			  <= 0;
			arg_c_read_issued 		  <= 0;
			arg_c_read_shift_register <= 0;
			arg_c_valid 			  <= 0;
			
			caught <= 0;
		end else if (enable && state == BUSY) begin
			caught <= 0;
			
			if (!arg_a_needed_latched) arg_a_valid <= 1;
			if (!arg_b_needed_latched) arg_b_valid <= 1;
			if (!arg_c_needed_latched) arg_c_valid <= 1;
			
			arg_a_read_shift_register <= arg_a_read_shift_register >> 1;
			arg_b_read_shift_register <= arg_b_read_shift_register >> 1;
			arg_c_read_shift_register <= arg_c_read_shift_register >> 1;
			
			if (!arg_a_resolved) begin
				if (arg_a_read_issued) begin
					if (arg_a_read_shift_register[1]) begin
						arg_a_latched 				<= src_a_reg_latched ? reg_read_val : channel_read_val;
						arg_a_valid 				<= 1;
						arg_a_read_issued 			<= 0;
					end
				end else begin
					if (src_a_reg_latched) begin
						reg_read_addr <= {block_latched, src_a_latched[$clog2(n_block_regs) - 1 : 0]};
						arg_a_read_issued <= 1;
						arg_a_read_shift_register <= 3'b100;
					end else begin
						caught <= arg_pending_writes == 1;
						if (arg_pending_writes == 0) begin
							channel_read_addr <= src_a_latched;
							arg_a_read_issued <= 1;
							arg_a_read_shift_register <= 3'b100;
						end else if (arg_pending_writes == 1) begin
							if (channel_write_enable && channel_write_addr == src_a_latched) begin
								arg_a_latched <= channel_write_val;
								arg_a_valid <= 1;
							end
						end
					end
				end
			end
			
			if (!arg_b_resolved) begin
				if (arg_b_read_issued) begin
					if (arg_b_read_shift_register[1]) begin
						arg_b_latched 				<= src_b_reg_latched ? reg_read_val : channel_read_val;
						arg_b_valid 				<= 1;
						arg_b_read_issued 			<= 0;
						arg_b_read_shift_register 	<= 0;
					end
				end else if (arg_a_resolved | arg_a_read_issued | (src_b_reg_latched != src_a_reg_latched)) begin
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
								arg_b_latched <= channel_write_val;
								arg_b_valid <= 1;
							end
						end
					end
				end
			end
			
			if (!arg_c_resolved) begin
				if (arg_c_read_issued) begin
					if (arg_c_read_shift_register[1]) begin
						arg_c_latched 				<= src_c_reg_latched ? reg_read_val : channel_read_val;
						arg_c_valid 				<= 1;
						arg_c_read_issued 			<= 0;
						arg_c_read_shift_register 	<= 0;
					end
				end else if ((arg_a_resolved || arg_a_read_issued || (src_b_reg_latched != src_a_reg_latched))
						  && (arg_b_resolved || arg_b_read_issued || (src_c_reg_latched != src_b_reg_latched))) begin
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
								arg_c_latched <= channel_write_val;
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
			inject_pending_ch0_write <= 0;
		end else if (enable) begin
			//if (sample_tick) commit_id <= 0;
			
			add_pending_write <= 0;
			inject_pending_ch0_write <= 0;
		
			case (state)
				IDLE: begin
					if (in_valid) begin
					
						block_latched <= block_in;
						
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
						shift_latched 	<= shift_in;
						no_shift_latched 		<= no_shift_in;
						
						res_addr_latched <= res_addr_in;
						
						branch_latched  <= branch;
						commits_latched <= commits;
						
						if (block_in == 0)
							inject_pending_ch0_write <= 1;
						
						ext_write_latched <= ext_write_in;
						
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

						accumulator_out <= accumulator_in;

						saturate_out 		<= saturate_latched;
						use_accumulator_out <= use_accumulator_latched;
						subtract_out 		<= subtract_latched;
						signedness_out 		<= signedness_latched;

						shift_out <= shift_latched;
						no_shift_out 	<= no_shift_latched;
						
						res_addr_out <= res_addr_latched;
						
						out_valid[branch_latched] <= 1;
						
						ext_write_out <= ext_write_latched;
						
						block_out <= block_latched;
						
						if (commits_latched) begin
							commit_id_out <= commit_id;
							commit_id <= commit_id + 1;
							
							add_pending_write <= 1;
						end
						
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
