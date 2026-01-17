`include "controller.vh"
`include "block.vh"

module control_unit
	#(
		parameter n_blocks 		= 32,
		parameter n_block_registers 	= 16,
		parameter data_width 	= 16
	)
	(
		input wire clk,
		input wire reset,
		
		input wire [7:0] in_byte,
		input wire in_ready,
		
		output reg [$clog2(n_blocks)	  - 1 : 0] block_target,
		output reg [`BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_target,
		output reg [`BLOCK_INSTR_WIDTH    - 1 : 0] instr_out,
		output reg [data_width 			  - 1 : 0] data_out,
		
		output reg [1:0] block_instr_write,
		output reg [1:0] block_reg_write,
		output reg [1:0] block_reg_update,
		output reg [1:0] alloc_sram_delay,
		
		output reg swap_pipelines,
		input wire pipelines_swapping,
		output reg [1:0] reset_pipeline,
		
		output reg next,
		
		output reg invalid
	);
	
	reg [7:0] in_byte_latched = 0;
	reg [7:0] command = 0;
	
	localparam instr_n_bytes = `BLOCK_INSTR_WIDTH / 8;
	
	reg [7:0] state = `CONTROLLER_STATE_READY;
	reg [7:0] ret_state;
	
	reg [5:0] byte_ctr;

    wire [data_width - 1 + 8 : 0] data_out_in_byte  = {data_out, in_byte};
    wire [`BLOCK_INSTR_WIDTH - 1 + 8 : 0] instr_out_in_byte = {instr_out, in_byte};
	
	reg load_block_number;
	reg load_reg_number;
	reg load_block_instr;
	reg load_data;
	
	reg wait_one = 0;
	
	wire target_pipeline = ~command[3];
	
	always @(posedge clk) begin
		next 	<= 0;
		invalid <= 0;
		swap_pipelines <= 0;
		reset_pipeline <= 0;
		
		block_instr_write 	<= 0;
		block_reg_write 	<= 0;
		block_reg_update 	<= 0;
		
		alloc_sram_delay <= 0;
		
		if (reset) begin
			state <= `CONTROLLER_STATE_READY;
		end
		else begin
			case (state)
				`CONTROLLER_STATE_READY: begin
					if (in_ready) begin
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
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_WRITE_BLOCK_INSTR;
						end

						`COMMAND_WRITE_BLOCK_REG: begin
							load_block_number <= 1;
							load_reg_number	  <= 1;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_WRITE_BLOCK_REG;
						end

						`COMMAND_UPDATE_BLOCK_REG: begin
							load_block_number <= 1;
							load_reg_number	  <= 1;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_UPDATE_BLOCK_REG;
						end

						`COMMAND_ALLOC_SRAM_DELAY: begin
							load_block_number <= 0;
							load_reg_number	  <= 0;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							
							state <= `CONTROLLER_STATE_GET_DATA;
							ret_state <= `CONTROLLER_STATE_ALLOC_SRAM_DELAY;
						end

						`COMMAND_SWAP_PIPELINES: begin
							swap_pipelines  <= 1;
							wait_one 		<= 1;
							state <= `CONTROLLER_STATE_SWAP_WAIT;
						end

						`COMMAND_RESET_PIPELINE: begin
							reset_pipeline[target_pipeline] <= 1;
							state <= `CONTROLLER_STATE_READY;
						end

						default: begin
							invalid <= 1;
							state <= `CONTROLLER_STATE_READY;
						end
					endcase
				end
				
				`CONTROLLER_STATE_GET_BLOCK_NUMBER: begin
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_ready) begin
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
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_ready) begin
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
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_ready) begin
						data_out <= data_out_in_byte[data_width - 1 : 0];
						next <= 1;
						
						if (byte_ctr >= (data_width / 8) - 1) begin
							byte_ctr <= 0;
							
							if (load_block_instr)
								state <= `CONTROLLER_STATE_GET_INSTR;
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
					else if (in_ready) begin
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
				
				`CONTROLLER_STATE_WRITE_BLOCK_INSTR: begin
					block_instr_write[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_WRITE_BLOCK_REG: begin
					block_reg_write[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_UPDATE_BLOCK_REG: begin
					block_reg_update[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_ALLOC_SRAM_DELAY: begin
					alloc_sram_delay[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_SWAP_WAIT: begin
					if (wait_one) begin
						if (pipelines_swapping) wait_one <= 0;
					end
					else begin
						if (!pipelines_swapping) begin
							state <= `CONTROLLER_STATE_READY;
						end
					end
				end
			endcase
		end
	end
	
endmodule

module control_unit_seq
	#(
		parameter n_blocks 		= 32,
		parameter n_block_registers 	= 16,
		parameter data_width 	= 16
	)
	(
		input wire clk,
		input wire reset,
		
		input wire [7:0] in_byte,
		input wire in_ready,
		
		output reg [$clog2(n_blocks)	  - 1 : 0] block_target,
		output reg [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_target,
		output reg [`BLOCK_INSTR_WIDTH    - 1 : 0] instr_out,
		output reg [data_width 			  - 1 : 0] data_out,
		
		input wire [1:0] reg_write_ack,
		
		output reg [1:0] block_instr_write,
		output reg [1:0] block_reg_write,
		output reg [1:0] block_reg_update,
		output reg [1:0] alloc_sram_delay,
		
		output reg swap_pipelines,
		input wire pipelines_swapping,
		output reg [1:0] reset_pipeline,
		
		output reg next,
		
		output reg invalid
	);
	
	reg [7:0] in_byte_latched = 0;
	reg [7:0] command = 0;
	
	localparam instr_n_bytes = `BLOCK_INSTR_WIDTH / 8;
	
	reg [7:0] state = `CONTROLLER_STATE_READY;
	reg [7:0] ret_state;
	
	reg [5:0] byte_ctr;

    wire [data_width - 1 + 8 : 0] data_out_in_byte  = {data_out, in_byte};
    wire [`BLOCK_INSTR_WIDTH - 1 + 8 : 0] instr_out_in_byte = {instr_out, in_byte};
	
	reg load_block_number;
	reg load_reg_number;
	reg load_block_instr;
	reg load_data;
	
	reg wait_one = 0;
	
	wire target_pipeline = ~command[3];
	
	always @(posedge clk) begin
		wait_one <= 0;
		
		next 	<= 0;
		invalid <= 0;
		swap_pipelines <= 0;
		reset_pipeline <= 0;
		
		block_instr_write 	<= 0;
		block_reg_update 	<= 0;
		
		alloc_sram_delay <= 0;
		
		if (reset) begin
			state <= `CONTROLLER_STATE_READY;
			block_reg_write <= 0;
		end
		else begin
			case (state)
				`CONTROLLER_STATE_READY: begin
					if (in_ready) begin
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
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_WRITE_BLOCK_INSTR;
						end

						`COMMAND_WRITE_BLOCK_REG: begin
							load_block_number <= 1;
							load_reg_number	  <= 1;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_WRITE_BLOCK_REG;
						end

						`COMMAND_UPDATE_BLOCK_REG: begin
							load_block_number <= 1;
							load_reg_number	  <= 1;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							
							state <= `CONTROLLER_STATE_GET_BLOCK_NUMBER;
							ret_state <= `CONTROLLER_STATE_UPDATE_BLOCK_REG;
						end

						`COMMAND_ALLOC_SRAM_DELAY: begin
							load_block_number <= 0;
							load_reg_number	  <= 0;
							load_data 		  <= 1;
							load_block_instr  <= 0;
							
							state <= `CONTROLLER_STATE_GET_DATA;
							ret_state <= `CONTROLLER_STATE_ALLOC_SRAM_DELAY;
						end

						`COMMAND_SWAP_PIPELINES: begin
							swap_pipelines  <= 1;
							wait_one 		<= 1;
							state <= `CONTROLLER_STATE_SWAP_WAIT;
						end

						`COMMAND_RESET_PIPELINE: begin
							reset_pipeline[target_pipeline] <= 1;
							state <= `CONTROLLER_STATE_READY;
						end

						default: begin
							invalid <= 1;
							state <= `CONTROLLER_STATE_READY;
						end
					endcase
				end
				
				`CONTROLLER_STATE_GET_BLOCK_NUMBER: begin
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_ready) begin
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
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_ready) begin
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
					if (wait_one) begin
						wait_one <= 0;
					end
					else if (in_ready) begin
						data_out <= data_out_in_byte[data_width - 1 : 0];
						next <= 1;
						
						if (byte_ctr >= (data_width / 8) - 1) begin
							byte_ctr <= 0;
							
							if (load_block_instr)
								state <= `CONTROLLER_STATE_GET_INSTR;
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
					else if (in_ready) begin
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
				
				`CONTROLLER_STATE_WRITE_BLOCK_INSTR: begin
					block_instr_write[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
					wait_one <= 1;
				end

				`CONTROLLER_STATE_WRITE_BLOCK_REG: begin
					block_reg_write[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_WRITE_BLOCK_REG_WAIT;
				end
					
				`CONTROLLER_STATE_WRITE_BLOCK_REG_WAIT: begin
					if (!wait_one && reg_write_ack[target_pipeline]) begin
						block_reg_write[target_pipeline] <= 0;
						state <= `CONTROLLER_STATE_READY;
					end
				end

				`CONTROLLER_STATE_UPDATE_BLOCK_REG: begin
					block_reg_update[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_ALLOC_SRAM_DELAY: begin
					alloc_sram_delay[target_pipeline] <= 1;
					state <= `CONTROLLER_STATE_READY;
				end

				`CONTROLLER_STATE_SWAP_WAIT: begin
					if (wait_one) begin
						if (pipelines_swapping) wait_one <= 0;
					end
					else begin
						if (!pipelines_swapping) begin
							state <= `CONTROLLER_STATE_READY;
						end
					end
				end
			endcase
		end
	end
	
endmodule
