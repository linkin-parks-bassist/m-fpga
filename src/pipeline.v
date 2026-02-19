`include "instr_dec.vh"
`include "core.vh"
`include "lut.vh"

`define PIPELINE_READY 			0
`define PIPELINE_PROCESSING 	1
`define PIPELINE_INVALID	 	2

module dsp_pipeline #(
		parameter data_width 		= 16,
		parameter n_blocks 			= 256
	) (
		input wire clk,
		input wire reset,
		
		input wire full_reset,
		input wire enable,
		
		input wire signed [data_width - 1:0] in_sample,
		input wire in_valid,
		
		output wire [data_width - 1:0] out_sample,
		output reg ready,
		
		output wire error,
		
		input wire [$clog2(n_blocks) 	  - 1 : 0] block_target,
		input wire [$clog2(n_blocks) + `BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_target,
	
		input wire [`BLOCK_INSTR_WIDTH - 1 : 0] instr_val,
		input wire instr_write,
	
		input wire [data_width - 1 : 0] ctrl_data,
		input wire [2 * data_width - 1 : 0] delay_size,
		input wire [2 * data_width - 1 : 0] init_delay,
		input wire reg_update,
		input wire reg_write,
		
		input wire reg_writes_commit,
		
		output wire reg_write_ack,
		output wire instr_write_ack,
	
		input wire alloc_delay,
		output wire resetting,

		output wire[7:0] out,

		output wire [$clog2(n_blocks) - 1 : 0] n_blocks_running,
		output wire [31:0] commits_accepted,
		output wire [7 : 0] byte_probe
	);
	
	/*******************/
	/* Processing core */
	/*******************/
	
	dsp_core #(
		.data_width(data_width),
		.n_blocks(n_blocks)
	) core (
		.clk(clk),
		.reset(reset),
		
		.enable(enable),
		
		.tick(in_valid),
		
		.sample_in(in_sample),
		.sample_out(out_sample),
		
		.ready(core_ready),
		
		.command_reg_write(reg_write),
		.command_instr_write(instr_write),
		
		.command_instr_write_val(instr_val),
		
		.command_block_target(block_target),
		.command_reg_target(reg_target),
		.command_reg_write_val(ctrl_data),
		
		.lut_req(lut_req),
		.lut_handle(lut_req_handle),
		.lut_arg(lut_req_arg),
		.lut_data(lut_data),
		.lut_valid(lut_valid),
		
		.delay_read_req  (delay_read_req),
		.delay_write_req (delay_write_req),
		.delay_req_handle(delay_req_handle),
		.delay_write_data(delay_write_data),
		.delay_write_inc (delay_write_inc),
		.delay_read_data (delay_read_data),
		.delay_read_valid(delay_read_valid),
		.delay_write_ack (delay_write_ack),
		
		.reg_writes_commit(reg_writes_commit),
		
		.full_reset(full_reset),
		.resetting(resetting)
	);
	
	/************************/
	/* Peripheral resources */
	/************************/
	
	// Lookup tables, for function calls
	lut_master #(.data_width(data_width)) luts (
		.clk(clk),
		.reset(reset | full_reset),
		
		.lut_handle(lut_req_handle),
		.req_arg(lut_req_arg),
		.req(lut_req),
		
		.data_out(lut_data),
		.valid(lut_valid),
		
		.invalid_request(invalid_lut_request)
	);
	
	// Delay buffers
	delay_master #(
		.data_width(data_width), 
		.n_buffers(8),
		.memory_size(8192)
	) delays (
		.clk(clk),
		.reset(reset | full_reset),
		
		.enable(1),
		
		.alloc_req  (alloc_delay),
		.alloc_size (delay_size),
		.alloc_delay(init_delay),
		
		.read_req(delay_read_req),
		.write_req(delay_write_req),
		
		.write_handle(delay_req_handle),
		.read_handle (delay_req_handle),
		.write_data  (delay_write_data),
		.write_inc   (delay_write_inc),
			
		.data_out(delay_read_data),
		
		.read_valid(delay_read_valid),
		.write_ack(delay_write_ready)
	);
	
	/**********/
	/* Wiring */
	/**********/
	
	reg signed [data_width - 1 : 0] sample_latched;
	
	reg [15:0] state;
	
	reg invalid = 0;
	assign error = invalid;
	
	reg [63 : 0] sample_ctr = 0;
	
	reg wait_one = 0;
	
	wire core_ready;

	wire lut_req;
	wire [`LUT_HANDLE_WIDTH - 1 : 0] lut_req_handle;
	wire signed [data_width - 1 : 0] lut_req_arg;
	wire signed [data_width - 1 : 0] lut_data;
	wire lut_valid;
	
	wire controller_valid;
	wire invalid_command;
	
	wire block_reg_write;
	wire invalid_lut_request;

	wire delay_read_req;
	wire delay_write_req;
	wire [data_width - 1 : 0] delay_req_handle;
	wire [data_width - 1 : 0] delay_write_data;
	wire [data_width - 1 : 0] delay_write_inc;
	wire [data_width - 1 : 0] delay_read_data;
	wire delay_read_valid;
	wire delay_write_ack;
	
	wire invalid_delay_read;
	wire invalid_delay_write;
	wire invalid_delay_alloc;
	
	// FSM deprecated; remove carefully without breaking anything later
	always @(posedge clk) begin
		wait_one <= 0;
		if (reset | full_reset) begin
			state 			<= `PIPELINE_READY;
			sample_latched 	<= 0;
			ready 			<= 1;
			invalid 		<= 0;
			sample_ctr 		<= 0;
		end
		else begin
			case (state)
				`PIPELINE_READY: begin
					ready <= 1;
					
					if (in_valid) begin
						state 	<= `PIPELINE_PROCESSING;
						ready 	<= 0;
						sample_ctr <= sample_ctr + 1;
						wait_one <= 1;
					end
				end
			
				`PIPELINE_PROCESSING: begin
					if (!wait_one) begin
						ready <= core_ready;
						if (ready) begin
							state <= `PIPELINE_READY;
						end
					end
				end
				
				`PIPELINE_INVALID: begin
					invalid <= 1;
					ready 	<= 0;
				end
				
				default: begin
					invalid <= 1;
					state 	<= `PIPELINE_INVALID;
				end
			endcase
		end
	end
endmodule


