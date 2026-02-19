`include "controller.vh"
`include "instr_dec.vh"
`include "core.vh"

`default_nettype none

module control_unit
	#(
		parameter n_blocks 			= 256,
		parameter data_width 		= 16
	)
	(
		input wire clk,
		input wire reset,

		output wire [7:0] control_state,
		
		input wire [7:0] in_byte,
		input wire in_valid,
		
		output reg [$clog2(n_blocks)	  - 1 : 0] block_target,
		output reg reg_target,
		output reg [`BLOCK_INSTR_WIDTH	  - 1 : 0] instr_out,
		output reg [data_width 			  - 1 : 0] data_out,
		output reg [2 * data_width 		  - 1 : 0] delay_size_out,
		output reg [2 * data_width 		  - 1 : 0] init_delay_out,
		
		output reg [1:0] block_instr_write,
		output reg [1:0] block_reg_write,
		output reg [1:0] block_reg_update,
		output reg [1:0] reg_writes_commit,
		input wire [1:0] pipeline_regfiles_syncing,
		output reg [1:0] alloc_delay,
		output reg [1:0] pipeline_full_reset,
		output reg [1:0] pipeline_resetting,
		output reg [1:0] pipeline_enables,
		output reg [1:0] pipeline_reset,
		
		output reg swap_pipelines,
		input wire pipelines_swapping,
		output reg current_pipeline,
		
		output reg set_input_gain,
		output reg set_output_gain,
		
		output reg next,
		
		output reg invalid,
		
		output reg [7:0] spi_output
	);
	
	reg [7:0] in_byte_latched = 0;
	reg [7:0] command = 0;
	
	localparam instr_n_bytes = `BLOCK_INSTR_WIDTH / 8;
	
	assign control_state = state;
	reg [7:0] state = `CONTROLLER_STATE_READY;
	reg [7:0] ret_state;

	wire [data_width - 1 + 8 : 0] data_out_in_byte  = {data_out, in_byte};
	wire [`BLOCK_INSTR_WIDTH - 1 + 8 : 0] instr_out_in_byte = {instr_out, in_byte};
	
	reg load_block_number;
	reg load_reg_number;
	reg load_block_instr;
	reg load_data;
	reg load_buf_delay;
	
	reg wait_one = 0;
	
	wire target_pipeline_inst = ~in_byte[3];
	wire target_pipeline = ~command[3];

	wire ready = (state == `CONTROLLER_STATE_READY);
	
	localparam block_bytes 		= (n_blocks > 256) ? 2 : 1;
	localparam data_bytes		= (data_width == 24) ? 3 : 2;
	localparam instr_bytes   	= 4;
	localparam delay_addr_bytes = 3;
	localparam max_bytes_needed = 6;
	
	reg [$clog2(max_bytes_needed) - 1 : 0] byte_ctr;
	reg [$clog2(max_bytes_needed) - 1 : 0] bytes_needed;
	reg [max_bytes_needed * 8 - 1 : 0] bytes_in;
	
	wire [7:0] byte_0_in = bytes_in[7:0];
	wire [7:0] byte_1_in = bytes_in[15:8];
	wire [7:0] byte_2_in = bytes_in[23:16];
	wire [7:0] byte_3_in = bytes_in[31:24];
	wire [7:0] byte_4_in = bytes_in[39:32];
	wire [7:0] byte_5_in = bytes_in[47:40];
	
	always @(posedge clk) begin
		reg_writes_commit <= 0;
		wait_one <= 0;
		
		next 	<= 0;
		invalid <= 0;
		swap_pipelines <= 0;
		pipeline_reset <= 0;
		pipeline_full_reset <= 0;
		
		block_instr_write <= 0;
		block_reg_write   <= 0;
		
		alloc_delay <= 0;
		
		set_input_gain  <= 0;
		set_output_gain <= 0;
		
		if (reset) begin
			state <= `CONTROLLER_STATE_READY;
			spi_output <= 0;
			pipeline_enables <= 2'b01;
			current_pipeline <= 0;
			
			byte_ctr <= 0;
			bytes_in <= 0;
		end
		else begin
			case (state)
				`CONTROLLER_STATE_READY: begin
					if (in_valid) begin
						command <= in_byte;
						wait_one <= 1;
						next <= 1;
						
						state <= `CONTROLLER_STATE_LISTEN;
						
						byte_ctr <= 0;
						bytes_in <= 0;
						
						case (in_byte)
							`COMMAND_WRITE_BLOCK_INSTR: begin
								bytes_needed <= block_bytes + instr_bytes;
							end
							
							`COMMAND_WRITE_BLOCK_REG_0: begin
								block_target <= 0;
								command <= `COMMAND_WRITE_BLOCK_REG;
								bytes_needed <= block_bytes + data_bytes;
							end
							
							`COMMAND_UPDATE_BLOCK_REG_0: begin
								block_target <= 0;
								command <= `COMMAND_UPDATE_BLOCK_REG;
								bytes_needed <= block_bytes + data_bytes;
							end
							
							`COMMAND_WRITE_BLOCK_REG_1: begin
								block_target <= 1;
								command <= `COMMAND_WRITE_BLOCK_REG;
								bytes_needed <= block_bytes + data_bytes;
							end
							
							`COMMAND_UPDATE_BLOCK_REG_1: begin
								block_target <= 1;
								command <= `COMMAND_UPDATE_BLOCK_REG;
								bytes_needed <= block_bytes + data_bytes;
							end
							
							`COMMAND_ALLOC_DELAY: begin
								bytes_needed <= 2 * delay_addr_bytes;
							end
							
							`COMMAND_SWAP_PIPELINES: begin
								swap_pipelines  	 <= 1;
								reg_writes_commit[1] <= 1;
								pipeline_enables[1]  <= 1;
								state <= `CONTROLLER_STATE_SWAP_WAIT;
							end
							
							`COMMAND_RESET_PIPELINE: begin
								pipeline_reset[target_pipeline_inst] <= 1;
								state <= `CONTROLLER_STATE_READY;
							end
							
							`COMMAND_SET_INPUT_GAIN: begin
								bytes_needed <= data_bytes;
							end
							
							`COMMAND_SET_OUTPUT_GAIN: begin
								bytes_needed <= data_bytes;
							end
							
							`COMMAND_COMMIT_REG_UPDATES: begin
								reg_writes_commit[target_pipeline_inst] <= 1;
								state <= `CONTROLLER_STATE_READY;
							end
						endcase
					end
				end
				
				`CONTROLLER_STATE_LISTEN: begin
					if (!wait_one && in_valid) begin
						bytes_in <= (bytes_in << 8) | in_byte;
						
						if (byte_ctr == bytes_needed - 1)
							state <= `CONTROLLER_STATE_ACT;
						else
							byte_ctr <= byte_ctr + 1;
						
						next <= 1;
						wait_one <= 1;
					end
				
				end
				
				`CONTROLLER_STATE_ACT: begin
					case (command)
						`COMMAND_WRITE_BLOCK_INSTR: begin
							if (block_bytes == 2)
								block_target <= {byte_5_in, byte_4_in};
							else
								block_target <= byte_4_in;
							
							instr_out 	 <= {byte_3_in, byte_2_in, byte_1_in, byte_0_in};
							block_instr_write[target_pipeline] <= 1;
							state <= `CONTROLLER_STATE_READY;
							wait_one <= 1;
						end

						`COMMAND_WRITE_BLOCK_REG: begin
							if (!pipelines_swapping && !pipeline_regfiles_syncing[target_pipeline]) begin
								if (block_bytes == 2)
									block_target <= {byte_3_in, byte_2_in};
								else
									block_target <= byte_2_in;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[target_pipeline] <= 1;
								state <= `CONTROLLER_STATE_READY;
							end
						end
						
						`COMMAND_UPDATE_BLOCK_REG: begin
							if (!pipelines_swapping && !pipeline_regfiles_syncing[target_pipeline]) begin
								if (block_bytes == 2)
									block_target <= {byte_3_in, byte_2_in};
								else
									block_target <= byte_2_in;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[target_pipeline] <= 1;
								state <= `CONTROLLER_STATE_READY;
							end
						end

						`COMMAND_ALLOC_DELAY: begin
							delay_size_out <= {byte_5_in, byte_4_in, byte_3_in};
							init_delay_out <= {byte_2_in, byte_1_in, byte_0_in};
							alloc_delay[target_pipeline] <= 1;
							state <= `CONTROLLER_STATE_READY;
						end

						`COMMAND_SET_INPUT_GAIN: begin
							data_out <= {byte_1_in, byte_0_in};
							set_input_gain <= 1;
							state <= `CONTROLLER_STATE_READY;
						end

						`COMMAND_SET_OUTPUT_GAIN: begin
							data_out <= {byte_1_in, byte_0_in};
							set_output_gain <= 1;
							state <= `CONTROLLER_STATE_READY;
						end
					endcase
				end
				
				`CONTROLLER_STATE_SWAP_WAIT: begin
					if (!wait_one && !pipelines_swapping) begin
						current_pipeline 		<= ~current_pipeline;
						pipeline_full_reset[1] 	<= 1;
						pipeline_enables[1] 	<= 0;
						
						wait_one <= 1;
						state <= `CONTROLLER_STATE_RESET_WAIT;
					end
				end
				
				`CONTROLLER_STATE_RESET_WAIT: begin
					if (!wait_one && pipeline_resetting == 2'b00) begin
						state <= `CONTROLLER_STATE_READY;
					end
				end
			endcase
		end
	end
endmodule

`default_nettype wire
