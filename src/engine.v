`include "engine.vh"

module dsp_engine
	#(
		parameter n_blocks 			= 4,
		parameter n_block_registers = 16,
		parameter data_width 		= 16,
		parameter n_channels 		= 16,
		parameter n_sram_banks 		= 64,
		parameter sram_bank_size 	= 1024,
		parameter spi_fifo_length	= 32
	)
	(
        input wire clk,
        input wire reset,

        input wire [data_width - 1 : 0]  in_sample,
        output reg [data_width - 1 : 0] out_sample,
        
        input wire sample_ready,
        
        input  wire [7:0] command_in,
        input  wire command_in_ready,
        output wire invalid_command,
        
        output reg ready,
        
        output wire [$clog2(spi_fifo_length) : 0] fifo_count
    );
    
    reg  signed [data_width - 1 : 0]  in_sample_latched;
    wire signed [data_width - 1 : 0]  in_sample_amped;
    wire signed [data_width - 1 : 0] out_samples [1:0];
    wire signed [data_width - 1 : 0] out_sample_mixed;

    wire in_valid;

    wire pipeline_a_ready;
    wire pipeline_b_ready;

    wire pipeline_a_error;
    wire pipeline_b_error;

    wire current_pipeline;
    wire pipelines_swapping;

    wire [1:0] block_instr_write;
    wire [1:0] block_reg_write;
    wire [1:0] block_reg_update;
    wire [1:0] alloc_sram_delay;

    wire pipeline_a_block_instr_write 	= block_instr_write[current_pipeline];
    wire pipeline_a_block_reg_write 	= block_reg_write  [current_pipeline];
    wire pipeline_a_block_reg_update 	= block_reg_update [current_pipeline];
    wire pipeline_a_alloc_sram_delay 	= alloc_sram_delay [current_pipeline];

    wire pipeline_b_block_instr_write 	= block_instr_write[~current_pipeline];
    wire pipeline_b_block_reg_write 	= block_reg_write  [~current_pipeline];
    wire pipeline_b_block_reg_update 	= block_reg_update [~current_pipeline];
    wire pipeline_b_alloc_sram_delay 	= alloc_sram_delay [~current_pipeline];

    reg pipeline_tick = 0;

    wire [$clog2(n_blocks) 	    - 1 : 0] block_target;
    wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_target;

    wire [data_width 		 - 1 : 0] ctrl_data_out;
    wire [`BLOCK_INSTR_WIDTH - 1 : 0] ctrl_instr_out;

    wire swap_pipelines;
    wire [1:0] reset_pipeline;
    wire controller_ready;

    reg ctrl_inp_ready = 0;
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
    
    pipeline
		#(
			.n_blocks(n_blocks),
			.n_block_registers(n_block_registers),
			.data_width(data_width),
			.n_channels(n_channels),
			.n_sram_banks(n_sram_banks),
			.sram_bank_size(sram_bank_size)
		)
		pipeline_a
		(
			.clk(clk),
			.reset(reset | reset_pipeline[~current_pipeline]),
			
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
			.reg_write(pipeline_a_block_reg_write),
			.reg_update(pipeline_a_block_reg_update),
		
			.alloc_sram_delay(pipeline_a_alloc_sram_delay)
		);
    
    pipeline
		#(
			.n_blocks(n_blocks),
			.n_block_registers(n_block_registers),
			.data_width(data_width),
			.n_channels(n_channels),
			.n_sram_banks(n_sram_banks),
			.sram_bank_size(sram_bank_size)
		)
		pipeline_b
		(
			.clk(clk),
			.reset(reset | reset_pipeline[current_pipeline]),
			
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
			.reg_write(pipeline_b_block_reg_write),
			.reg_update(pipeline_b_block_reg_update),
		
			.alloc_sram_delay(pipeline_b_alloc_sram_delay)
		);
	
	
	
	fifo_buffer #(.data_width(8), .n(spi_fifo_length)) spi_fifo
		(
			.clk(clk),
			.reset(reset),
			
			.data_in(command_in),
			.data_out(command_byte),
			
			.write(command_in_ready),
			.next(inp_fifo_next),
			
			.nonempty(inp_fifo_nonempty),
			.full(inp_fifo_full),
			
			.count(fifo_count)
		);
	
	
    
    control_unit #(.n_blocks(n_blocks), .data_width(data_width), .n_block_registers(n_block_registers)) controller
		(
			.clk(clk),
			.reset(reset),
			
			.in_byte(command_byte),
			.in_ready(inp_fifo_nonempty),
			.next(inp_fifo_next),
			
			.block_target(block_target),
			.reg_target(reg_target),
			.instr_out(ctrl_instr_out),
			.data_out(ctrl_data_out),
			
			.block_instr_write(block_instr_write),
			.block_reg_write(block_reg_write),
			.block_reg_update(block_reg_update),
			
			.alloc_sram_delay(alloc_sram_delay),
			
			.swap_pipelines(swap_pipelines),
			.pipelines_swapping(pipelines_swapping),
			.reset_pipeline(reset_pipeline),
			
			.invalid(invalid_command)
		);
	
	mixer #(.data_width(data_width), .gain_shift(5)) mixerr (
			.clk(clk),
			.reset(reset),
			
			.in_sample(in_sample_latched),
			.in_sample_out(in_sample_amped),
			
			.out_sample_in_a(out_samples[0]),
			.out_sample_in_b(out_samples[1]),
			
			.out_sample(out_sample_mixed),
			
			.data_in(0),
			
			.in_sample_valid(apply_input_gain),
			.out_samples_valid(mix_outputs),
			
			.in_sample_ready(in_sample_valid),
			.out_sample_ready(out_sample_valid),
			
			.set_input_gain(0),
			.set_output_gain(0),
			
			.swap_pipelines(swap_pipelines),
			.pipelines_swapping(pipelines_swapping),
			.current_pipeline(current_pipeline)
		);
	
	always @(posedge clk) begin
		pipeline_tick 		<= 0;
		
		out_sample_ready 	<= 0;
		apply_input_gain 	<= 0;
		mix_outputs			<= 0;
		
		case (state)
			`ENGINE_STATE_READY: begin
				if (sample_ready) begin
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
					out_sample_ready <= 0;
					ready <= 1;
					state <= `ENGINE_STATE_READY;
				end
			end
		endcase
	end
endmodule