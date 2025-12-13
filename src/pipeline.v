`include "block.vh"
`include "lut.vh"

`define PIPELINE_READY 			0
`define PIPELINE_PROCESSING 	1
`define PIPELINE_INVALID	 	2

module distributed_multiplier
	#(
		parameter width = 18,
		parameter n_blocks = 32
	)
	(
		input wire clk,
		input wire reset,
		
		input wire signed [width - 1:0] x [0 : n_blocks - 1],
		input wire signed [width - 1:0] y [0 : n_blocks - 1],
		
		output reg signed [2 * width - 1 : 0] prod,
		
		output reg  [n_blocks - 1 : 0] done
	);
	
	localparam index_width = $clog2(n_blocks);
	localparam [index_width - 1 : 0] initial_index = index_width'(n_blocks - 1);
	
	reg [index_width - 1 : 0] index = initial_index;
	
	integer i;
	always @(posedge clk) begin
		if (reset) begin
			index <= initial_index;
			
			for (i = 0; i < n_blocks; i = i + 1) begin
				done[i] <= 0;
			end
			
			prod <= 0;
		end
		else begin
			prod <= x[index] * y[index];
			
			done <= '0;
			done[index] <= 1;
			
			if (index == 0) begin
				index <= initial_index;
			end
			else begin
				index <= index - 1;
			end
		end
	end
endmodule

module pipeline
	#(
		parameter n_blocks 			= 2,
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
		input wire [`BLOCK_REG_ADDR_WIDTH - 1 : 0] reg_target,
	
		input wire [`BLOCK_INSTR_WIDTH - 1 : 0] instr_val,
		input wire instr_write,
	
		input wire [data_width - 1 : 0] ctrl_data,
		input wire reg_update,
		input wire reg_write,
	
		input wire alloc_sram_delay
	);
	
	reg signed [data_width - 1 : 0] sample_latched;
	
	reg [15:0] state;
	
	reg invalid = 0;
	assign error = invalid;
	
	reg [63 : 0] sample_ctr = 0;
	
	always @(posedge clk) begin
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
					end
				end
			
				`PIPELINE_PROCESSING: begin
					ready <= &dones;
					if (ready) begin
						state <= `PIPELINE_READY;
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
	
	wire [data_width - 1 : 0] bus [0 : n_blocks][0 : n_channels - 1];
	wire [n_blocks - 1 : 0] dones;

	genvar c;
    generate
        for (c = 0; c < n_channels; c = c + 1) begin
            assign bus[0][c] = (c == 0) ? in_sample : '0;
        end
    endgenerate
    
    assign out_sample = bus[n_blocks][0];
    
    wire [data_width 		- 1 : 0] mul_bus_x [0 : n_blocks - 1];
    wire [data_width 		- 1 : 0] mul_bus_y [0 : n_blocks - 1];
    wire [2 * data_width 	- 1 : 0] mul_result;
    wire [n_blocks 			- 1 : 0] mul_done;
    
    distributed_multiplier #(.width(data_width), .n_blocks(n_blocks)) mul
		(
			.clk(clk),
			.reset(reset),
			
			.x(mul_bus_x),
			.y(mul_bus_y),
			
			.prod(mul_result),
			
			.done(mul_done)
		);
	
	genvar i;
	generate
		for (i = 0; i < n_blocks; i = i + 1) begin : blocks
			pipeline_block #(
				.n_registers(n_block_registers),
                .data_width(data_width),
                .n_channels(n_channels)
            ) block_inst (
                .clk   (clk),
                .tick(in_valid),
                .reset(reset),
                
                .ch_in (bus[i]),
                .ch_out(bus[i+1]),
                
                .mul_req_a(mul_bus_x[i]),
                .mul_req_b(mul_bus_y[i]),
                
                .mul_result(mul_result),
                .mul_done(mul_done[i]),
                
                .instr_val(instr_val),
                .instr_write(instr_write & (i == block_target)),
                
                .reg_target(reg_target),
                .reg_val(ctrl_data),
                .reg_write(reg_write & (i == block_target)),
                .reg_update(reg_update & (i == block_target)),
                
                .lut_req(block_lut_reqs[i]),
				.lut_handle(block_lut_req_handle[i]),
				.lut_arg(block_lut_req_arg[i]),
				.lut_data(lut_data),
				.lut_ready(lut_readies[i]),
                
                .delay_read_req(delay_read_reqs[i]),
				.delay_write_req(delay_write_reqs[i]),
				.delay_buf_handle(delay_req_handles[i]),
				.delay_buf_data_out(delay_req_args[i]),
				.delay_buf_data_in(delay_data),
				.delay_buf_read_ready(delay_read_readies[i]),
				.delay_buf_write_ready(delay_write_readies[i]),
				
				.delay_buf_invalid_read(invalid_delay_read),
				.delay_buf_invalid_write(invalid_delay_write),
                
                .done(dones[i])
            );
		end
	endgenerate
	
	wire [n_blocks - 1 : 0] block_lut_reqs;
	wire [`LUT_HANDLE_WIDTH - 1 : 0] block_lut_req_handle [n_blocks - 1 : 0];
	wire [data_width - 1 : 0] block_lut_req_arg  [n_blocks - 1 : 0];
	wire [data_width - 1 : 0] lut_data;
	wire [n_blocks - 1 : 0] lut_readies;
	
	wire lut_arbiter_req;
	wire [`LUT_HANDLE_WIDTH - 1 : 0] lut_arbiter_handle;
	wire [data_width - 1 : 0] lut_arbiter_arg;
	
	wire [data_width - 1 : 0] lut_arbiter_data;
	wire lut_ready;
	
	wire invalid_lut_request;
	
	rr_arbiter_handle #(.n_clients(n_blocks), .handle_width(`LUT_HANDLE_WIDTH), .req_data_width(data_width), .server_data_width(data_width))
		lut_arbiter
		(
			.clk(clk),
			.reset(reset),
			
			.req_data(block_lut_req_arg),
			.req_handles(block_lut_req_handle),
			.reqs(block_lut_reqs),
			
			.data_out(lut_data),
			.readies(lut_readies),
			
			.arbiter_req_data(lut_arbiter_arg),
			.arbiter_req_handle(lut_arbiter_handle),
			.arbiter_req(lut_arbiter_req),
			
			.server_data(lut_arbiter_data),
			.server_ready(lut_ready)
		);
	
	lut_master #(.data_width(data_width)) luts
		(
			.clk(clk),
			.reset(reset),
			
			.lut_handle(lut_arbiter_handle),
			.req_arg(lut_arbiter_arg),
			.req(lut_arbiter_req),
			
			.data_out(lut_arbiter_data),
			.ready(lut_ready),
			
			.invalid_request(invalid_lut_request)
		);
	
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
	
	wire [n_blocks   - 1 : 0] delay_read_reqs;
	wire [n_blocks   - 1 : 0] delay_write_reqs;
	wire [data_width - 1 : 0] delay_req_handles 	[n_blocks - 1 : 0];
	wire [data_width - 1 : 0] delay_req_args  		[n_blocks - 1 : 0];
	wire [data_width - 1 : 0] delay_data;
	wire [data_width - 1 : 0] delay_write_data;
	wire [n_blocks   - 1 : 0] delay_read_readies;
	wire [n_blocks   - 1 : 0] delay_write_readies;
	
	wire delay_read_arbiter_req;
	wire [data_width - 1 : 0] delay_read_arbiter_handle;
	wire [data_width - 1 : 0] delay_read_arbiter_arg;
	wire [data_width - 1 : 0] delay_read_arbiter_data;
	
	wire delay_write_arbiter_req;
	wire [data_width - 1 : 0] delay_write_arbiter_handle;
	wire [data_width - 1 : 0] delay_write_arbiter_arg;
	wire [data_width - 1 : 0] delay_write_arbiter_data;
	
	wire delay_read_ready;
	wire delay_write_ready;
	
	wire invalid_delay_read;
	wire invalid_delay_write;
	wire invalid_delay_alloc;
	
	wire [sram_addr_width - 1 : 0] ctrl_data_addr_width;
	
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
	
	delay_master #(
			.data_width(data_width), 
			.n_sram_buffers(8),
			.sram_addr_width(sram_addr_width),
			.sram_capacity(n_sram_banks * sram_bank_size)
		)
		delays (
			.clk(clk),
			.reset(reset),
			
			.read_req(delay_read_arbiter_req),
			.write_req(delay_write_arbiter_req),
			
			.alloc_sram_req(alloc_sram_delay),
			.alloc_size(ctrl_data_addr_width),
			
			.read_req_handle(delay_read_arbiter_handle),
			.read_req_arg(delay_read_arbiter_arg),
			
			.write_req_handle(delay_write_arbiter_handle),
			.write_req_arg(delay_write_arbiter_arg),
			
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
				
			.data_out(delay_read_arbiter_data),
			
			.read_ready(delay_read_ready),
			.write_ready(delay_write_ready),
			
			.invalid_read(invalid_delay_read),
			.invalid_write(invalid_delay_write),
			.invalid_alloc(invalid_delay_alloc)
		);
	
	rr_arbiter_handle #(.n_clients(n_blocks), .handle_width(data_width), .req_data_width(data_width), .server_data_width(data_width))
		delay_read_arbiter
		(
			.clk(clk),
			.reset(reset),
			
			.req_data(delay_req_args),
			.req_handles(delay_req_handles),
			.reqs(delay_read_reqs),
			
			.data_out(delay_data),
			.readies(delay_read_readies),
			
			.arbiter_req_data(delay_read_arbiter_arg),
			.arbiter_req_handle(delay_read_arbiter_handle),
			.arbiter_req(delay_read_arbiter_req),
			
			.server_data(delay_read_arbiter_data),
			.server_ready(delay_read_ready)
		);
	
	rr_arbiter_handle #(.n_clients(n_blocks), .handle_width(data_width), .req_data_width(data_width), .server_data_width(data_width))
		delay_write_arbiter
		(
			.clk(clk),
			.reset(reset),
			
			.req_data(delay_req_args),
			.req_handles(delay_req_handles),
			.reqs(delay_write_reqs),
			
			.data_out(delay_write_data),
			.readies(delay_write_readies),
			
			.arbiter_req_data(delay_write_arbiter_arg),
			.arbiter_req_handle(delay_write_arbiter_handle),
			.arbiter_req(delay_write_arbiter_req),
			
			.server_data(delay_write_arbiter_data),
			.server_ready(delay_write_ready)
		);
endmodule
