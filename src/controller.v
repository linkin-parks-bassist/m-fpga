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
		
		output reg health_monitor_enable,
		output reg health_monitor_reset,
		input wire health,
		
		output reg invalid,
		
		output wire [7:0] control_state,
		output reg  [7:0] spi_byte_out
	);
	
	reg [7:0] in_byte_latched;
	reg [7:0] command = 0;
	
	localparam instr_n_bytes = `BLOCK_INSTR_WIDTH / 8;
	
	reg [2:0] state = READY;
	reg [2:0] state_prev = READY;
    assign control_state = {4'd0, timeout_blinker, timeout_active, programming, |state};
	
	reg wait_one = 0;

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
	
    localparam READY      		  = 3'd0;
    localparam LISTEN     		  = 3'd1;
    localparam EXECUTE    		  = 3'd2;
    localparam SWAP_WARMUP  	  = 3'd3;
    localparam SWAP_WAIT  		  = 3'd4;
    localparam RESET_WAIT 		  = 3'd5;
    localparam INITIAL_RESET_WAIT = 3'd6;

	wire front_pipeline = current_pipeline;
	wire back_pipeline = ~current_pipeline;

    reg [15 : 0] total_bytes;
    
    reg [31:0] timeout_ctr;
    reg [31:0] timeout_max;
    reg timeout;
    
    wire timeout_blinker = |timeout_blinker_ctr;
    reg [31:0] timeout_blinker_ctr;
    
    reg programming;
    reg ignore_command;
    reg timeout_active;
    
    localparam warmup_cycles = 326530;
    
    reg warmup;
    reg [31:0] warmup_ctr;
    
    localparam SPI_RESPONSE_OK 				= 8'd0;
    localparam SPI_RESPONSE_INITIALISING	= 8'd1;
    localparam SPI_RESPONSE_PROGRAMMING 	= 8'd2;
    localparam SPI_RESPONSE_REJECTED		= 8'd3;
    localparam SPI_RESPONSE_TIMEOUT			= 8'd4;

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
        
        state_prev <= state;
        
        health_monitor_reset <= 0;
		
		pipeline_enables <= 2'b00;
		
		if (reset) begin
			state 		<= INITIAL_RESET_WAIT;
            state_prev  <= INITIAL_RESET_WAIT;
            
			pipeline_full_reset <= 2'b11;
			wait_one <= 1;
			current_pipeline <= 0;
			
			byte_ctr <= 0;
			bytes_in <= 0;

			health_monitor_enable <= 0;

            total_bytes <= 0;
            
            programming <= 0;
            reg_target <= 0;
            
            ignore_command <= 0;
            timeout_active <= 0;
			
			ignore_command <= 0;
            
			timeout_ctr <= 0;
            timeout_max <= `CONTROLLER_TIMEOUT_CYCLES;
            
            spi_byte_out <= SPI_RESPONSE_INITIALISING;
		end else if (timeout) begin
			pipeline_full_reset[back_pipeline] <= 1;
			programming 	<= 0;
			timeout_active 	<= 0;
			timeout_ctr 	<= 0;
			ignore_command  <= 0;
			state 			<= RESET_WAIT;
            timeout_max 	<= `CONTROLLER_TIMEOUT_CYCLES;
            spi_byte_out 	<= SPI_RESPONSE_TIMEOUT;
            
            timeout_blinker_ctr <= 32'd112500000;
		end else begin
			if (timeout_active | programming) begin
				if ((wait_one && in_valid) || state_prev != state)
					timeout_ctr <= 0;
				else if (timeout_ctr == timeout_max - 1)
					timeout <= 1;
				else
					timeout_ctr <= timeout_ctr + 1;
			end
			
			if (timeout_blinker_ctr != 0)
				timeout_blinker_ctr <= timeout_blinker_ctr - 1;
		
			case (state)
				READY: begin
					timeout_active <= 0;
					ignore_command <= 0;
					
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
								spi_byte_out <= 0;
								
								spi_byte_out <= SPI_RESPONSE_PROGRAMMING;
							end
						
							`COMMAND_WRITE_BLOCK_INSTR: begin
								bytes_needed <= block_bytes + instr_bytes;
								
								if (!programming) ignore_command <= 1;
							end
							
							`COMMAND_WRITE_BLOCK_REG_0: begin
								reg_target <= 0;
								bytes_needed <= block_bytes + data_bytes;
								
								if (!programming) ignore_command <= 1;
							end
							
							
							`COMMAND_WRITE_BLOCK_REG_1: begin
								reg_target <= 1;
								bytes_needed <= block_bytes + data_bytes;
								
								if (!programming) ignore_command <= 1;
							end
							
							`COMMAND_ALLOC_DELAY: begin
								bytes_needed <= 2 * delay_addr_bytes;
								
								if (!programming) ignore_command <= 1;
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
									programming <= 0;
									
									reg_writes_commit[back_pipeline] <= 1;
									pipeline_enables [back_pipeline] <= 1;
									
									health_monitor_enable <= 1;
									health_monitor_reset <= 1;
									
									warmup_ctr <= 0;
									
									state <= SWAP_WARMUP;
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
					timeout_active <= 1;
					if (!wait_one && in_valid) begin
						bytes_in <= (bytes_in << 8) | in_byte;
						
						if (byte_ctr == bytes_needed - 1) begin
							state <= ignore_command ? READY : EXECUTE;
							timeout_active <= 0;
						end else begin
							byte_ctr <= byte_ctr + 1;
						end
						
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
							timeout_active <= 1;
							if (!pipelines_swapping && !pipeline_resetting[back_pipeline] && !pipeline_regfiles_syncing[back_pipeline]) begin
								block_target <= reg_write_block;
								reg_target <= 0;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[back_pipeline] <= 1;
								state <= READY;
							end
						end
						
						`COMMAND_WRITE_BLOCK_REG_1: begin
							timeout_active <= 1;
							if (!pipelines_swapping && !pipeline_resetting[back_pipeline] && !pipeline_regfiles_syncing[back_pipeline]) begin
								block_target <= reg_write_block;
								reg_target <= 1;
								
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
							timeout_active <= 1;
							if (pipelines_swapping) begin
								state <= READY;
							end else if (!pipeline_regfiles_syncing[front_pipeline]) begin
								block_target <= reg_write_block;
								reg_target <= 0;
								
								data_out <= {byte_1_in, byte_0_in};
								block_reg_write[front_pipeline] <= 1;
								state <= READY;
							end
						end

						`COMMAND_UPDATE_BLOCK_REG_1: begin
							timeout_active <= 1;
							if (pipelines_swapping) begin
								state <= READY;
							end else if (!pipeline_regfiles_syncing[front_pipeline]) begin
								block_target <= reg_write_block;
								reg_target <= 1;
								
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
				
				SWAP_WARMUP: begin
					timeout_active <= 0;
					if (warmup_ctr == warmup_cycles) begin
						if (health) begin
							swap_pipelines <= 1;
							
							
							swap_pipelines <= 1;
							
							state <= SWAP_WAIT;
							spi_byte_out <= SPI_RESPONSE_OK;
						end else begin
							pipeline_full_reset[back_pipeline] <= 1;
							state <= READY;
							spi_byte_out <= SPI_RESPONSE_REJECTED;
						end
						
						health_monitor_enable <= 0;
						health_monitor_reset <= 1;
						
						warmup_ctr <= 0;
					end else begin
						warmup_ctr <= warmup_ctr + 1;
					end
				end
				
				SWAP_WAIT: begin
					timeout_active <= 1;
					if (!wait_one && !pipelines_swapping) begin
						current_pipeline <= ~current_pipeline;
						pipeline_full_reset[front_pipeline] <= 1;
						pipeline_enables   [front_pipeline] <= 0;
						
						wait_one <= 1;
						state <= RESET_WAIT;
					end
				end
				
				RESET_WAIT: begin
					timeout_active <= 1;
					if (!wait_one && !(|pipeline_resetting)) begin
						state <= READY;
					end
				end
				
				INITIAL_RESET_WAIT: begin
					timeout_active <= 1;
					if (!wait_one && !(|pipeline_resetting[front_pipeline])) begin
						state <= READY;
						pipeline_enables[front_pipeline] <= 1;
						
						spi_byte_out <= SPI_RESPONSE_OK;
					end
				end
			endcase
		end
	end
endmodule

`default_nettype wire
