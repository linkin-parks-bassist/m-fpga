`include "instr_dec.vh"

`include "lut.vh"
`include "core.vh"


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
		output reg [4 : 0] operation_out,
		
		input wire [3 : 0] dest_in,
		output reg [3 : 0] dest_out,

		input wire signed [3 : 0] src_a_in,
		input wire signed [3 : 0] src_b_in,
		input wire signed [3 : 0] src_c_in,

		input wire src_a_reg_in,
		input wire src_b_reg_in,
		input wire src_c_reg_in,
		
		input wire arg_a_needed_in,
		input wire arg_b_needed_in,
		input wire arg_c_needed_in,

		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		output reg signed [data_width - 1 : 0] arg_c_out,
		
		input wire saturate_disable_in,
		output reg saturate_disable_out,
		
		input wire signedness_in,
		output reg signedness_out,
		
		input wire acc_needed_in,

		input wire [4 : 0] shift_in,
		output reg [4 : 0] shift_out,
		input wire shift_disable_in,
		output reg shift_disable_out,
		
		input wire [7 : 0] res_addr_in,
		output reg [7 : 0] res_addr_out,
		
		input wire writes_external_in,
		output reg writes_external_out,
		
		input wire writes_channel_in,
		input wire writes_acc_in,
		
		output reg [8:0] commit_id_out,
		
		input wire commit_flag_in,
		output reg commit_flag_out,
		
		input wire signed [2 * data_width - 1 : 0] accumulator_in,
		output reg signed [2 * data_width - 1 : 0] accumulator_out,

		input wire [`N_INSTR_BRANCHES - 1 : 0] branch,
		
		input wire [3 : 0] channel_write_addr,
		input wire signed [data_width - 1 : 0] channel_write_val,
		input wire channel_write_enable,
		
		output reg 		  [3              : 0] channel_read_addr,
		input wire signed [data_width - 1 : 0] channel_read_val,
		
		output reg [$clog2(n_blocks) + 4 - 1 : 0] reg_read_addr,
		input wire signed 	 [data_width - 1 : 0] reg_read_val,
		
		input wire [2 * data_width - 1 : 0] acc_write_val,
		input wire acc_write_enable
	);
	
	reg   [3 : 0] channels_scoreboard [15 : 0];
	reg   [3 : 0] acc_pending_writes;
	
	wire accumulator_stall = (acc_needed_latched && acc_pending_writes != 0);
	
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
				case ({(add_pending_write && dest_latched == i && !writes_acc_latched) ||
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
			
			case ({add_pending_write & writes_acc_latched, acc_write_enable})
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
	reg writes_channel_latched;
	reg writes_acc_latched;

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
	
	reg saturate_disable_latched;
	reg acc_needed_latched;
	reg signedness_latched;

	reg [4 : 0] shift_latched;
	reg shift_disable_latched;
	
	reg [7 : 0] res_addr_latched;
	
	reg [`N_INSTR_BRANCHES - 1 : 0] branch_latched;
	reg writes_external_latched;
	
	reg commit_flag_latched;
	
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
			arg_b_read_issued 		  <= 0;
			arg_b_read_shift_register <= 0;
			arg_b_valid 			  <= 0;
			arg_c_read_issued 		  <= 0;
			arg_c_read_shift_register <= 0;
			arg_c_valid 			  <= 0;
		end else if (enable && state == BUSY) begin
			if (!arg_a_needed_latched) begin
				arg_a_latched <= 0;
				arg_a_valid <= 1;
			end
			if (!arg_b_needed_latched) begin
				arg_b_latched <= 0;
				arg_b_valid <= 1;
			end
			if (!arg_c_needed_latched) begin
				arg_c_latched <= 0;
				arg_c_valid <= 1;
			end
			
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
						reg_read_addr <= {block_latched, 3'b000, src_a_latched[0]};
						arg_a_read_issued <= 1;
						arg_a_read_shift_register <= 3'b100;
					end else begin
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
						reg_read_addr <= {block_latched, 3'b000, src_b_latched[0]};
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
						reg_read_addr <= {block_latched, 3'b000, src_c_latched[0]};
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
			
			add_pending_write <= 0;
			inject_pending_ch0_write <= 0;
		
			case (state)
				IDLE: begin
					if (in_valid) begin
					
						block_latched <= block_in;
						
						operation_latched <= operation_in;
						
						src_a_latched <= src_a_in;
						src_b_latched <= src_b_in;
						src_c_latched <= src_c_in;

						src_a_reg_latched <= src_a_reg_in;
						src_b_reg_latched <= src_b_reg_in;
						src_c_reg_latched <= src_c_reg_in;

						arg_a_needed_latched <= arg_a_needed_in;
						arg_b_needed_latched <= arg_b_needed_in;
						arg_c_needed_latched <= arg_c_needed_in;

						dest_latched <= dest_in;
						
						saturate_disable_latched 	<= saturate_disable_in;
						acc_needed_latched			<= acc_needed_in;
						signedness_latched 			<= signedness_in;
						shift_latched 				<= shift_in;
						shift_disable_latched 		<= shift_disable_in;
						
						res_addr_latched 		<= res_addr_in;
						
						writes_external_latched <= writes_external_in;
						writes_channel_latched 	<= writes_channel_in;
						writes_acc_latched 		<= writes_acc_in;
						commit_flag_latched 	<= commit_flag_in;
						branch_latched 			<= branch;
						
						if (block_in == 0)
							inject_pending_ch0_write <= 1;
						
						state <= BUSY;
					end
				end
				
				BUSY: begin
					if (arg_a_resolved && arg_b_resolved && arg_c_resolved && !accumulator_stall) begin
						operation_out <= operation_latched;
		
						dest_out <= dest_latched;

						block_out 			 <= block_latched;
						arg_a_out 			 <= arg_a_latched;
						arg_b_out 			 <= arg_b_latched;
						arg_c_out 			 <= arg_c_latched;
						saturate_disable_out <= saturate_disable_latched;
						signedness_out 		 <= signedness_latched;
						shift_out 			 <= shift_latched;
						shift_disable_out 	 <= shift_disable_latched;
						res_addr_out 		 <= res_addr_latched;
						writes_external_out  <= writes_external_latched;
						commit_flag_out		 <= commit_flag_latched;
						
						out_valid[branch_latched] <= 1;
						
						accumulator_out <= accumulator_in;
						
						if (writes_channel_latched | writes_acc_latched) begin
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
