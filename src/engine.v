`include "engine.vh"
`include "core.vh"

`default_nettype none

module dsp_engine #(
		parameter n_blocks 			= 255,
		parameter data_width 		= 16,
		parameter spi_fifo_length	= 32
	) (
		input wire clk,
		input wire reset,

		input wire [data_width - 1 : 0]  in_sample,
		output reg [data_width - 1 : 0] out_sample,
		
		input wire sample_valid,
		
		input  wire [7:0] command_in,
		input  wire command_in_valid,
		output wire invalid_command,
		
		output reg ready,
		
		output wire [$clog2(spi_fifo_length) : 0] fifo_count,

		output wire current_pipeline,

		output wire [7:0] out
	);

	assign out = {4'b0, pipelines_swapping | (|pipeline_resetting), control_state[2:0]};

	/*******/
	/*******/
	/* DSP */
	/*******/
	/*******/

	/***************************************************************************/
	/* Dual DSP piplines for atomic, artifact-free runtime DSP reconfiguration */
	/***************************************************************************/
	
    wire [7:0] bute_probe = current_pipeline ? byte_probe_b : byte_probe_a;
	wire [7:0] byte_probe_a;
	wire [7:0] byte_probe_b;
	
	dsp_pipeline #(.data_width(data_width), .n_blocks(n_blocks)) pipeline_a (
		.clk(clk),
		.reset(reset | pipeline_a_reset),
		
		.in_sample(in_sample_amped),
		.in_valid(pipeline_tick),
		.out_sample(out_samples[0]),
		
		.ready(pipeline_a_ready),
		.error(pipeline_a_error),
		
		.block_target(block_target),
		.reg_target(reg_target),

		.instr_val(ctrl_instr_out),
		.instr_write(pipeline_a_block_instr_write),
	
		.ctrl_data(ctrl_data_out),
		.delay_size(delay_alloc_size),
		.init_delay(delay_init_delay),
		.reg_write(pipeline_a_block_reg_write),
		.reg_write_ack(reg_write_acks[0]),
		.reg_update(pipeline_a_block_reg_update),
		
		.reg_writes_commit(pipeline_a_reg_writes_commit),
		.regfile_syncing(pipeline_b_regfile_syncing),
	
		.alloc_delay(pipeline_a_alloc_delay),
		
		.full_reset(pipeline_a_full_reset),
		.enable(pipeline_a_enable),
		
		.resetting(pipeline_a_resetting),
		
		.byte_probe(byte_probe_a)
	);
	
	dsp_pipeline #(.data_width(data_width), .n_blocks(n_blocks)) pipeline_b (
		.clk(clk),
		.reset(reset | pipeline_b_reset),
		
		.in_sample(in_sample_amped),
		.in_valid(pipeline_tick),
		.out_sample(out_samples[1]),
		
		.ready(pipeline_b_ready),
		.error(pipeline_b_error),
		
		.block_target(block_target),
		.reg_target(reg_target),

		.instr_val(ctrl_instr_out),
		.instr_write(pipeline_b_block_instr_write),
	
		.ctrl_data(ctrl_data_out),
		.delay_size(delay_alloc_size),
		.init_delay(delay_init_delay),
		.reg_write(pipeline_b_block_reg_write),
		.reg_update(pipeline_b_block_reg_update),
		
		.reg_writes_commit(pipeline_b_reg_writes_commit),
		.regfile_syncing(pipeline_a_regfile_syncing),
	
		.alloc_delay(pipeline_b_alloc_delay),

		.full_reset(pipeline_b_full_reset),
		.enable(pipeline_b_enable),
		
		.resetting(pipeline_b_resetting),
		
		.byte_probe(byte_probe_b)
	);
	
	/**********************************************************/
	/* Mixer; applies input/output gain, crossfades pipelines */
	/**********************************************************/
	
	mixer #(.data_width(data_width), .gain_shift(5)) mixerr (
		.clk(clk),
		.reset(reset),
		
		.in_sample(in_sample_latched),
		.in_sample_out(in_sample_amped),
		
		.out_sample_in_a(out_samples[0]),
		.out_sample_in_b(out_samples[1]),
		
		.out_sample(out_sample_mixed),
		
		.data_in(ctrl_data_out),
		
		.in_sample_valid(apply_input_gain),
		.out_samples_valid(mix_outputs),
		
		.in_sample_mixed(in_sample_valid),
		.out_sample_valid(out_sample_valid),
		
		.set_input_gain(set_input_gain),
		.set_output_gain(set_output_gain),
		
		.swap_pipelines(swap_pipelines),
		.pipelines_swapping(pipelines_swapping),
		.current_pipeline(current_pipeline)
	);
	
	/*****************/
	/*****************/
	/* Input/control */
	/*****************/
	/*****************/
	
	// A FIFO to store incoming SPI transfers
	fifo_buffer #(.data_width(8), .n(spi_fifo_length)) spi_fifo (
		.clk(clk),
		.reset(reset),
		
		.data_in(command_in),
		.data_out(command_byte),
		
		.write(command_in_valid),
		.next(inp_fifo_next),
		
		.nonempty(inp_fifo_nonempty),
		.full(inp_fifo_full),
		
		.count(fifo_count)
	);
	
	/************************************************************/
	/* Global control unit; executes commands recieved over SPI */
	/************************************************************/
	
	wire [7:0] control_state;
	
	control_unit #(.n_blocks(n_blocks), .data_width(data_width)) controller (
		.clk(clk),
		.reset(reset),
		
		.in_byte(command_byte),
		.in_valid(inp_fifo_nonempty),
		.next(inp_fifo_next),
		
		.current_pipeline(current_pipeline),
		
		.block_target(block_target),
		.reg_target(reg_target),
		.instr_out(ctrl_instr_out),
		.data_out(ctrl_data_out),
		
		.block_instr_write(block_instr_write),
		.block_reg_write(block_reg_write),
		
		.reg_writes_commit(reg_writes_commit),
		
		.alloc_delay(alloc_delay),
		.delay_size_out(delay_alloc_size),
		.init_delay_out(delay_init_delay),
		
		.swap_pipelines(swap_pipelines),
		.pipelines_swapping(pipelines_swapping),
		.pipeline_regfiles_syncing(pipeline_regfiles_syncing),
		.pipeline_reset(pipeline_reset),
		.pipeline_full_reset(pipeline_full_reset),
		.pipeline_resetting(pipeline_resetting),
		.pipeline_enables(pipeline_enables),
		.set_input_gain(set_input_gain),
		.set_output_gain(set_output_gain),
		
		.invalid(invalid_command),
		
		.control_state(control_state)
	);
	
	/*******/
	/* FSM */
	/*******/
	always @(posedge clk) begin
		pipeline_tick 		<= 0;
		
		apply_input_gain 	<= 0;
		mix_outputs			<= 0;
		
		case (state)
			`ENGINE_STATE_READY: begin
				if (sample_valid) begin
					in_sample_latched <= in_sample;
					apply_input_gain <= 1;
					ready <= 0;
				end
				
				if (in_sample_valid) begin
					apply_input_gain <= 0;
					
					sample_ctr <= sample_ctr + 1;
					pipeline_tick <= 1;	
					
					ready <= 0;
					state <= `ENGINE_STATE_PROCESSING_WAIT;
				end
			end
			
			`ENGINE_STATE_PROCESSING_WAIT: begin
				state <= `ENGINE_STATE_PROCESSING;
			end
			
			`ENGINE_STATE_PROCESSING: begin
				if (pipeline_a_ready && pipeline_b_ready) begin
					mix_outputs <= 1;
					
					state <= `ENGINE_STATE_MIXING;
				end
			end
			
			`ENGINE_STATE_MIXING: begin
				if (out_sample_valid) begin
					out_sample <= out_sample_mixed;
					ready <= 1;
					state <= `ENGINE_STATE_READY;
				end
			end
		endcase
	end
	
	/**********/
	/* Wiring */
	/**********/
	
	reg  signed [data_width - 1 : 0]  in_sample_latched;
	wire signed [data_width - 1 : 0]  in_sample_amped;
	wire signed [data_width - 1 : 0] out_samples [1:0];
	wire signed [data_width - 1 : 0] out_sample_mixed;

	wire in_valid;

	wire pipeline_a_ready;
	wire pipeline_b_ready;

	wire pipeline_a_error;
	wire pipeline_b_error;

	wire pipelines_swapping;

	wire [1:0] block_instr_write;
	wire [1:0] block_reg_write;
	wire [1:0] block_reg_update;
	wire [1:0] reg_writes_commit;
	wire [1:0] alloc_delay;
	wire [1:0] pipeline_reset;
	wire [1:0] pipeline_full_reset;
	wire [1:0] pipeline_enables;
	wire [1:0] pipeline_resetting;
	wire [1:0] pipeline_regfiles_syncing;

	reg pipeline_tick = 0;

	wire [$clog2(n_blocks) - 1 : 0] block_target;
	wire reg_target;

	wire [data_width 		 - 1 : 0] ctrl_data_out;
	wire [`BLOCK_INSTR_WIDTH - 1 : 0] ctrl_instr_out;

	wire swap_pipelines;
	wire controller_ready;

	reg  ctrl_inp_ready = 0;
	wire ctrl_inp_req;
	wire ctrl_inp_ack;

	wire [7:0] command_byte;
	wire inp_fifo_nonempty;
	wire inp_fifo_full;

	wire inp_fifo_next;

	reg [63 : 0] sample_ctr = 0;

	reg apply_input_gain = 0;
	reg mix_outputs = 0;

	reg out_sample_ready;
	wire in_sample_valid;
	wire out_sample_valid;

	reg [7:0] state = `ENGINE_STATE_READY;

	reg inp_fifo_waiting = 0;

	wire [1:0] reg_write_acks;
	
	wire set_input_gain;
	wire set_output_gain;
	
	wire [2 * data_width - 1 : 0] delay_alloc_size;
	wire [2 * data_width - 1 : 0] delay_init_delay;
	
	wire pipeline_a_block_instr_write 	= block_instr_write		[0];
	wire pipeline_a_block_reg_write 	= block_reg_write  		[0];
	wire pipeline_a_block_reg_update 	= block_reg_update 		[0];
	wire pipeline_a_reg_writes_commit 	= reg_writes_commit 	[0];
	wire pipeline_a_regfile_syncing;
	wire pipeline_a_alloc_delay 		= alloc_delay 			[0];
	wire pipeline_a_enable 				= pipeline_enables 		[0];
	wire pipeline_a_full_reset 			= pipeline_full_reset	[0];
	wire pipeline_a_resetting;
	wire pipeline_a_reset	 			= pipeline_reset		[0];

	wire pipeline_b_block_instr_write 	= block_instr_write		[1];
	wire pipeline_b_block_reg_write 	= block_reg_write  		[1];
	wire pipeline_b_block_reg_update 	= block_reg_update 		[1];
	wire pipeline_b_reg_writes_commit 	= reg_writes_commit 	[1];
	wire pipeline_b_regfile_syncing;
	wire pipeline_b_alloc_delay 		= alloc_delay 			[1];
	wire pipeline_b_enable 				= pipeline_enables 		[1];
	wire pipeline_b_full_reset 			= pipeline_full_reset	[1];
	wire pipeline_b_resetting;
	wire pipeline_b_reset	 			= pipeline_reset		[1];
	
	assign pipeline_resetting = {pipeline_b_resetting, pipeline_a_resetting};
	assign pipeline_regfiles_syncing = {pipeline_b_regfile_syncing, pipeline_a_regfile_syncing};
endmodule

`default_nettype wire
