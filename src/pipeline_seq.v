module pipeline_seq
	#(
		parameter n_blocks 			= 256,
		parameter n_block_registers = 16,
		parameter data_width 		= 16,
		parameter n_channels 		= 16,
		parameter n_sram_banks 		= 8,
		parameter sram_bank_size 	= 1024
	)
	(
		input wire clk,
		input wire reset,
		
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
		input wire reg_update,
		input wire reg_write,
		output wire reg_write_ack,
	
		input wire alloc_sram_delay
	);
	
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
	wire lut_ready;
	
	wire controller_ready;
	wire invalid_command;
	
	wire sram_read;
	wire sram_write;

	localparam sram_capacity = n_sram_banks * sram_bank_size;
	localparam sram_addr_width = $clog2(sram_capacity + 1);

	wire [sram_addr_width - 1 : 0] sram_read_addr;
	wire [sram_addr_width - 1 : 0] sram_write_addr;
	wire [data_width - 1 : 0] sram_data_in;
	wire [data_width - 1 : 0] sram_data_out;
	
	wire sram_read_ready;
	wire sram_write_ready;
	
	wire sram_read_invalid;
	wire sram_write_invalid;
	
	wire block_reg_write;
	wire invalid_lut_request;

    wire delay_read_req;
	wire delay_write_req;
	wire [data_width - 1 : 0] delay_req_handle;
	wire [data_width - 1 : 0] delay_req_arg;
	wire [data_width - 1 : 0] delay_read_data;
	wire delay_read_ready;
	wire delay_write_ready;
	
	wire invalid_delay_read;
	wire invalid_delay_write;
	wire invalid_delay_alloc;
	
	wire [sram_addr_width - 1 : 0] ctrl_data_addr_width;
	
	always @(posedge clk) begin
		wait_one <= 0;
		if (reset) begin
			state 			<= `PIPELINE_READY;
			sample_latched 	<= 0;
			ready 			<= 1;
			invalid 		<= 0;
			sample_ctr <= 0;
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
	
	generate
		if (sram_addr_width < data_width) begin : ADDR_SMALL
			assign ctrl_data_addr_width = ctrl_data[sram_addr_width - 1 : 0];
		end
		else if (sram_addr_width > data_width) begin : ADDR_BIG
			assign ctrl_data_addr_width =  {{(sram_addr_width - data_width){1'b0}}, ctrl_data};
		end
		else begin : ADDR_SAME
			assign ctrl_data_addr_width = ctrl_data;
		end
	endgenerate
	
	dsp_core #(.data_width(data_width), .n_blocks(n_blocks), .n_channels(n_channels), .n_registers(n_block_registers)) core (
		.clk(clk),
		.reset(reset),
		
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
		
		.reg_write_ack(reg_write_ack),
		
		.lut_req(lut_req),
		.lut_handle(lut_req_handle),
		.lut_arg(lut_req_arg),
		.lut_data(lut_data),
		.lut_ready(lut_ready),
		
		.delay_read_req(delay_read_req),
		.delay_write_req(delay_write_req),
		.delay_req_handle(delay_req_handle),
		.delay_req_arg(delay_req_arg),
		.delay_req_data_in(delay_read_data),
		.delay_read_ready(delay_read_ready),
		.delay_write_ready(delay_write_ready)
	);
	
	lut_master #(.data_width(data_width)) luts
		(
			.clk(clk),
			.reset(reset),
			
			.lut_handle(lut_req_handle),
			.req_arg(lut_req_arg),
			.req(lut_req),
			
			.data_out(lut_data),
			.ready(lut_ready),
			
			.invalid_request(invalid_lut_request)
		);
	
	contiguous_sram #(.data_width(data_width), .addr_width(sram_addr_width), .bank_size(sram_bank_size), .n_banks(n_sram_banks)) sram
		(
			.clk(clk),
			.reset(reset),
			
			.read(sram_read),
			.write(sram_write),
			
			.read_addr(sram_read_addr),
			.write_addr(sram_write_addr),
			
			.data_in(sram_data_in),
			.data_out(sram_data_out),
			
			.read_ready(sram_read_ready),
			.write_ready(sram_write_ready),
			
			.invalid_read(sram_read_invalid),
			.invalid_write(sram_write_invalid)
		);
	
	delay_master #(
			.data_width(data_width), 
			.n_sram_buffers(8),
			.sram_addr_width(sram_addr_width),
			.sram_capacity(n_sram_banks * sram_bank_size)
		)
		delays (
			.clk(clk),
			.reset(reset),
			
			.read_req(delay_read_req),
			.write_req(delay_write_req),
			
			.alloc_sram_req(alloc_sram_delay),
			.alloc_size(ctrl_data_addr_width),
			
			.read_req_handle(delay_req_handle),
			.read_req_arg(delay_req_arg),
			
			.write_req_handle(delay_req_handle),
			.write_req_arg(delay_req_arg),
			
			.req_sram_read(sram_read),
			.req_sram_write(sram_write),
			.req_sram_read_addr(sram_read_addr),
			.req_sram_write_addr(sram_write_addr),
			.sram_read_ready(sram_read_ready),
			.sram_write_ready(sram_write_ready),
			.data_to_sram(sram_data_in),
			.data_from_sram(sram_data_out),
			.sram_read_invalid(sram_read_invalid),
			.sram_write_invalid(sram_write_invalid),
				
			.data_out(delay_read_data),
			
			.read_ready(delay_read_ready),
			.write_ready(delay_write_ready),
			
			.invalid_read(invalid_delay_read),
			.invalid_write(invalid_delay_write),
			.invalid_alloc(invalid_delay_alloc)
		);
endmodule


