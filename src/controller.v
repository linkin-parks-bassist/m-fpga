`include "controller.vh"
`include "instr_dec.vh"
`include "core.vh"

module control_unit
	#(
		parameter n_blocks 			= 256,
		parameter n_block_registers = 2,
		parameter data_width 		= 16
	)
	(
		input wire clk,
		input wire reset,

		output wire [7:0] control_state,
		
		input wire [7:0] in_byte,
		input wire in_valid,
		
		output reg [$clog2(n_blocks)	  - 1 : 0] block_target,
		output reg [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_target,
		output reg [`BLOCK_INSTR_WIDTH	- 1 : 0] instr_out,
		output reg [data_width 			  - 1 : 0] data_out,
		output reg [2 * data_width 		  - 1 : 0] buf_init_delay,
		
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
		
		output reg [7:0] spi_output,

		input wire [$clog2(n_blocks) - 1 : 0] pipeline_n_blocks [1:0],
		input wire [31 : 0] pipeline_n_commits [1:0],
		input wire [7 : 0] pipeline_byte_probe [1:0]
	);
	
	reg [7:0] in_byte_latched = 0;
	reg [7:0] command = 0;
	
	localparam instr_n_bytes = `BLOCK_INSTR_WIDTH / 8;
	
	assign control_state = state;
	reg [7:0] state = `CONTROLLER_STATE_READY;
	reg [7:0] ret_state;
	
	reg [5:0] byte_ctr;

	wire [data_width - 1 + 8 : 0] data_out_in_byte  = {data_out, in_byte};
	wire [`BLOCK_INSTR_WIDTH - 1 + 8 : 0] instr_out_in_byte = {instr_out, in_byte};
	
	reg load_block_number;
	reg load_reg_number;
	reg load_block_instr;
	reg load_data;
	reg load_buf_delay;
	
	reg wait_one = 0;
	
	wire target_pipeline = ~command[3];

	assign ready = (state == `CONTROLLER_STATE_READY);
	
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
		
		spi_output <= pipeline_byte_probe[current_pipeline] | (1 << 7);
		
		if (reset) begin
			state <= `CONTROLLER_STATE_READY;
			spi_output <= 0;
			pipeline_enables <= 2'b01;
			current_pipeline <= 0;
		end
		else begin
			case (state)
				`CONTROLLER_STATE_READY: begin
					if (in_valid) begin
						command <= in_byte;
						next <= 1;
						
						state <= `CONTROLLER_STATE_BEGIN;
					end
				end
				
				`CONTROLLER_STATE_BEGIN: begin
					case (command)
						`COMMAND_WRITE_BLOCK_INSTR: begin
							load_block_number <= 1;
							load_reg_number	  <= 0;
							load_data 		  <= 0;
							load_block_instr  <= 1;
							load_buf_delay	<= 0;
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_WRITE_BLOCK_INSTR;
						end

						`COMMAND_WRITE_BLOCK_REG: begin
							load_block_number <= 1;
							load_reg_number	  <= 1;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							load_buf_delay	<= 0;
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_WRITE_BLOCK_REG;
						end

						`COMMAND_UPDATE_BLOCK_REG: begin
							load_block_number <= 1;
							load_reg_number	  <= 1;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							load_buf_delay	<= 0;
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_WRITE_BLOCK_REG;
						end
						
						`COMMAND_COMMIT_REG_UPDATES: begin
							reg_writes_commit[target_pipeline] <= 1;
							state <= `CONTROLLER_STATE_READY;
						end

						`COMMAND_ALLOC_DELAY: begin
							load_block_number <= 0;
							load_reg_number	  <= 0;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							load_buf_delay	<= 1;
							
							state <= `CONTROLLER_STATE_GET_DATA;
							ret_state <= `CONTROLLER_STATE_ALLOC_DELAY;
						end

						`COMMAND_SWAP_PIPELINES: begin
							swap_pipelines  	 <= 1;
							reg_writes_commit[1] <= 1;
							pipeline_enables[1]  <= 1;
							wait_one 			 <= 1;
							state <= `CONTROLLER_STATE_SWAP_WAIT;
						end

						`COMMAND_RESET_PIPELINE: begin
							pipeline_reset[target_pipeline] <= 1;
							state <= `CONTROLLER_STATE_READY;
						end

						`COMMAND_SET_INPUT_GAIN: begin
							load_block_number <= 0;
							load_reg_number	  <= 0;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							load_buf_delay	<= 0;
							
							state <= `CONTROLLER_STATE_GET_DATA;
							ret_state <= `CONTROLLER_STATE_SET_INPUT_GAIN;
						end

						`COMMAND_SET_OUTPUT_GAIN: begin
							load_block_number <= 0;
							load_reg_number	  <= 0;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							load_buf_delay	<= 0;
							
							state <= `CONTROLLER_STATE_GET_DATA;
							ret_state <= `CONTROLLER_STATE_SET_OUTPUT_GAIN;
						end

						default: begin
							invalid <= 1;
							state <= `CONTROLLER_STATE_READY;
						end
					endcase
				end
				
				`CONTROLLER_STATE_GET_BLOCK_NUMBER: begin
					if (!wait_one && in_valid) begin
						block_target <= in_byte[$clog2(n_blocks) - 1 : 0];
						next <= 1;
						
						byte_ctr <= 0;
						
						if (load_reg_number)  		state <= `CONTROLLER_STATE_GET_REG_NUMBER;
						else if (load_data)	  		state <= `CONTROLLER_STATE_GET_DATA;
						else if (load_block_instr) 	state <= `CONTROLLER_STATE_GET_INSTR;
						else 						state <= ret_state;
						
						wait_one <= 1;
					end
				end
				
				
				`CONTROLLER_STATE_GET_REG_NUMBER: begin
					if (!wait_one && in_valid) begin
						reg_target <= in_byte[`BLOCK_REG_ADDR_WIDTH - 1 : 0];
						next <= 1;
						
						byte_ctr <= 0;
						
						if (load_data)	  			state <= `CONTROLLER_STATE_GET_DATA;
						else if (load_block_instr) 	state <= `CONTROLLER_STATE_GET_INSTR;
						else 						state <= ret_state;
						
						wait_one <= 1;
					end
				end
				
				`CONTROLLER_STATE_GET_DATA: begin
					if (!wait_one && in_valid) begin
						data_out <= data_out_in_byte[data_width - 1 : 0];
						next <= 1;
						
						if (byte_ctr >= (data_width / 8) - 1) begin
							byte_ctr <= 0;
							
							if (load_block_instr)
								state <= `CONTROLLER_STATE_GET_INSTR;
							else if (load_buf_delay)
								state <= `CONTROLLER_STATE_GET_DELAY;
							else
								state <= ret_state;
						end
						else begin
							byte_ctr <= byte_ctr + 1;
						end
						wait_one <= 1;
					end
				end
				
				`CONTROLLER_STATE_GET_INSTR: begin
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_valid) begin
						instr_out  <= instr_out_in_byte[`BLOCK_INSTR_WIDTH - 1 : 0];
						next <= 1;
						
						if (byte_ctr >= (`BLOCK_INSTR_WIDTH / 8) - 1) begin
							byte_ctr <= 0;
							
							state <= ret_state;
						end
						else begin
							byte_ctr <= byte_ctr + 1;
						end
						wait_one <= 1;
					end
				end
				
				`CONTROLLER_STATE_GET_DELAY: begin
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_valid) begin
						buf_init_delay  <= {buf_init_delay[2 * data_width - 8 - 1 : 0], in_byte};
						next <= 1;
						
						if (byte_ctr > 2) begin
							byte_ctr <= 0;
							
							state <= ret_state;
						end
						else begin
							byte_ctr <= byte_ctr + 1;
						end
						wait_one <= 1;
					end
				end
				
				`CONTROLLER_STATE_WRITE_BLOCK_INSTR: begin
					block_instr_write[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
					wait_one <= 1;
				end

				`CONTROLLER_STATE_WRITE_BLOCK_REG: begin
					if (!pipelines_swapping && !pipeline_regfiles_syncing[target_pipeline]) begin
						block_reg_write[target_pipeline] <= 1;
						state <= `CONTROLLER_STATE_READY;
					end
				end

				`CONTROLLER_STATE_ALLOC_DELAY: begin
					alloc_delay[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_SET_INPUT_GAIN: begin
					set_input_gain <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_SET_OUTPUT_GAIN: begin
					set_output_gain <= 1;
					state <= `CONTROLLER_STATE_READY;
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
