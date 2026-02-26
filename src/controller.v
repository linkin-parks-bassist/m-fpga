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
		output reg [1:0] reg_writes_commit,
		input wire [1:0] pipeline_regfiles_syncing,
		output reg [1:0] alloc_delay,
		output reg [1:0] pipeline_full_reset,
		input wire [1:0] pipeline_resetting,
		output reg [1:0] pipeline_enables,
		output reg [1:0] pipeline_reset,
		
		output reg swap_pipelines,
		input wire pipelines_swapping,
		output reg current_pipeline,
		
		output reg set_input_gain,
		output reg set_output_gain,
		
		output reg next,
		
		output reg invalid,
		
		output wire [7:0] control_state
	);
	
	reg [7:0] in_byte_latched;
	reg [7:0] command = 0;
	
	localparam instr_n_bytes = `BLOCK_INSTR_WIDTH / 8;
	
	reg [7:0] state = READY;
    assign control_state = {bytes_in[1:0], bytes_needed[1:0], state[3:0]};

	reg load_block_number;
	reg load_reg_number;
	reg load_block_instr;
	reg load_data;
	reg load_buf_delay;
	
	reg wait_one = 0;
	
	wire target_pipeline_inst = ~in_byte[3];
	wire target_pipeline = ~command[3];

	wire ready = (state == READY);
	
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
	
	wire [8 * block_bytes - 1 : 0] instr_write_block;
	wire [8 * block_bytes - 1 : 0] reg_write_block;
	generate
		if (block_bytes == 2) begin
			assign instr_write_block = {byte_5_in, byte_4_in};
			assign reg_write_block = {byte_3_in, byte_2_in};
		end else begin
			assign instr_write_block = byte_4_in;
			assign reg_write_block = byte_2_in;
		end
	endgenerate
	
    localparam READY      = 8'd0;
    localparam LISTEN     = 8'd1;
    localparam EXECUTE    = 8'd2;
    localparam SWAP_WAIT  = 8'd3;
    localparam RESET_WAIT = 8'd4;

	wire front_pipeline = current_pipeline;
	wire back_pipeline = ~current_pipeline;

    reg [15 : 0] total_bytes;
    
    reg [31:0] timeout_ctr;
    reg timeout;
    
    reg programming;

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

        timeout <= 0;
		
		if (reset) begin
			timeout_ctr <= 0;
			state <= READY;
			pipeline_enables <= 2'b01;
			current_pipeline <= 0;
			
			byte_ctr <= 0;
			bytes_in <= 0;

			pipeline_full_reset <= 2'b11;

            total_bytes <= 0;
            
            programming <= 0;
		end else if (timeout) begin
			pipeline_full_reset[back_pipeline] <= 1;
			programming <= 0;
			timeout_ctr <= 0;
			state <= RESET_WAIT;
		end else begin
			if (!programming || in_valid) begin
				timeout_ctr <= 0;
			end else begin
				if (timeout_ctr == `CONTROLLER_TIMEOUT_CYCLES - 1)
					timeout <= 1;
				else
					timeout_ctr <= timeout_ctr + 1;
			end
		
			case (state)
				READY: begin
					if (!wait_one && in_valid) begin
						command <= in_byte;
						wait_one <= 1;
						next <= 1;
						
						state <= LISTEN;
						
						byte_ctr <= 0;
						bytes_in <= 0;

                        total_bytes <= total_bytes + 1;
						
						case (in_byte)
							`COMMAND_BEGIN_PROGRAM: begin
								programming <= 1;
								state <= READY;
							end
						
							`COMMAND_WRITE_BLOCK_INSTR: begin
								bytes_needed <= block_bytes + instr_bytes;
								
								if (!programming) state <= READY;
							end
							
							`COMMAND_WRITE_BLOCK_REG_0: begin
								reg_target <= 0;
								bytes_needed <= block_bytes + data_bytes;
								
								if (!programming) state <= READY;
							end
							
							
							`COMMAND_WRITE_BLOCK_REG_1: begin
								reg_target <= 1;
								bytes_needed <= block_bytes + data_bytes;
								
								if (!programming) state <= READY;
							end
							
							`COMMAND_ALLOC_DELAY: begin
								bytes_needed <= 2 * delay_addr_bytes;
								
								if (!programming) state <= READY;
							end
							
							`COMMAND_UPDATE_BLOCK_REG_0: begin
								reg_target <= 0;
								bytes_needed <= block_bytes + data_bytes;
							end
							
							`COMMAND_UPDATE_BLOCK_REG_1: begin
								reg_target <= 1;
								bytes_needed <= block_bytes + data_bytes;
							end
							
							`COMMAND_COMMIT_REG_UPDATES: begin
								reg_writes_commit[front_pipeline] <= 1;
								state <= READY;
							end
							
							`COMMAND_END_PROGRAM: begin
								if (programming) begin
									programming    <= 0;
									swap_pipelines <= 1;
									
									reg_writes_commit[back_pipeline] <= 1;
									pipeline_enables [back_pipeline] <= 1;
									
									state <= SWAP_WAIT;
								end else begin
									state <= READY;
								end
							end
							
							`COMMAND_SET_INPUT_GAIN: begin
								bytes_needed <= data_bytes;
							end
							
							`COMMAND_SET_OUTPUT_GAIN: begin
								bytes_needed <= data_bytes;
							end
							
							default: begin
								state <= READY;
							end
						endcase
					end
				end
				
				LISTEN: begin
					if (!wait_one && in_valid) begin
						bytes_in <= (bytes_in << 8) | in_byte;
						
						if (byte_ctr == bytes_needed - 1)
							state <= EXECUTE;
						else
							byte_ctr <= byte_ctr + 1;
						
						next <= 1;
						wait_one <= 1;
					end
				end
				
				EXECUTE: begin
					case (command)
						`COMMAND_WRITE_BLOCK_INSTR: begin
							block_target <= instr_write_block;
							
							instr_out 	 <= {byte_3_in, byte_2_in, byte_1_in, byte_0_in};
							block_instr_write[back_pipeline] <= 1;
							state <= READY;
							wait_one <= 1;
						end

						`COMMAND_WRITE_BLOCK_REG_0: begin
							if (!pipelines_swapping && !pipeline_regfiles_syncing[back_pipeline]) begin
								block_target <= reg_write_block;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[back_pipeline] <= 1;
								state <= READY;
							end
						end
						
						`COMMAND_WRITE_BLOCK_REG_1: begin
							if (!pipelines_swapping && !pipeline_regfiles_syncing[back_pipeline]) begin
								block_target <= reg_write_block;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[back_pipeline] <= 1;
								state <= READY;
							end
						end

						`COMMAND_ALLOC_DELAY: begin
							delay_size_out <= {8'd0, byte_5_in, byte_4_in, byte_3_in};
							init_delay_out <= {8'd0, byte_2_in, byte_1_in, byte_0_in};
							alloc_delay[back_pipeline] <= 1;
							state <= READY;
						end

						`COMMAND_UPDATE_BLOCK_REG_0: begin
							if (!pipelines_swapping && !pipeline_regfiles_syncing[front_pipeline]) begin
								block_target <= reg_write_block;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[front_pipeline] <= 1;
								state <= READY;
							end
						end

						`COMMAND_UPDATE_BLOCK_REG_1: begin
							if (!pipelines_swapping && !pipeline_regfiles_syncing[front_pipeline]) begin
								block_target <= reg_write_block;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[front_pipeline] <= 1;
								state <= READY;
							end
						end
						
						`COMMAND_SET_INPUT_GAIN: begin
							data_out <= {byte_1_in, byte_0_in};
							set_input_gain <= 1;
							state <= READY;
						end

						`COMMAND_SET_OUTPUT_GAIN: begin
							data_out <= {byte_1_in, byte_0_in};
							set_output_gain <= 1;
							state <= READY;
						end
					endcase
				end
				
				SWAP_WAIT: begin
					if (!wait_one && !pipelines_swapping) begin
						current_pipeline 		<= ~current_pipeline;
						pipeline_full_reset[front_pipeline] <= 1;
						pipeline_enables   [front_pipeline] <= 0;
						
						wait_one <= 1;
						state <= RESET_WAIT;
					end
				end
				
				RESET_WAIT: begin
					if (!wait_one && pipeline_resetting[back_pipeline]) begin
						state <= READY;
					end
				end
			endcase
		end
	end
endmodule

`default_nettype wire
