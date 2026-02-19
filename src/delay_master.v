module delay_buffer_controller #(parameter data_width = 16, parameter addr_width = 16, parameter handle)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		output reg [data_width - 1 : 0] data_out,
		output reg data_out_valid,
		
		output reg [data_width : 0] gain,
		
		input wire write,
		
		input wire signed [data_width - 1 : 0] data_in,
		input wire signed [data_width - 1 : 0] delay_inc,
		
		input wire init,
		
		input wire [	addr_width - 1 : 0] init_addr,
		input wire [	addr_width - 1 : 0] init_size,
		input wire [2 * data_width - 1 : 0] init_delay,
		
		output reg mem_read_req,
		output reg mem_write_req,
		
		output reg 		  [addr_width - 1 : 0] mem_read_addr,
		input wire signed [data_width - 1 : 0] mem_data_in,
		
		output reg [addr_width - 1 : 0] mem_write_addr,
		output reg signed [data_width - 1 : 0] mem_data_out,
		
		output wire ready,
		output reg write_done,
		
		input wire mem_read_valid,
		input wire mem_write_ack,
		
		output reg invalid
	);
	
	localparam DELAY_FORMAT = 8;
	
	localparam IDLE 	 	= 3'd0;
	localparam WRITE_REQ 	= 3'd1;
	localparam FETCH_1	 	= 3'd2;
	localparam FETCH_2	 	= 3'd2;
	localparam INTERP	 	= 3'd3;
	localparam INTERP_WAIT  = 3'd4;
	localparam DONE			= 3'd5;
	
	assign ready = (state == IDLE);
	
	reg initialised;
	
	reg [	addr_width - 1 : 0] addr;
	reg [	addr_width - 1 : 0] size;
	reg [2 * data_width - 1 : 0] delay;
	reg [	addr_width - 1 : 0] position;
	
	reg wrapped;
	
	reg [2:0] state;
	
	wire [2 * data_width - 1 : 0] delay_offset = delay >> DELAY_FORMAT;
	wire [	  addr_width - 1 : 0] delay_addr = (delay_offset > position) ? addr + position - delay_offset + size
																		 : addr + position - delay_offset;
	
	reg [2 * data_width - 1 : 0] delay_inc_clamped;
	
	wire 		[2 * data_width - 1 : 0] max_delay 	   = (size << DELAY_FORMAT);
	wire signed [2 * data_width - 1 : 0] max_delay_inc = max_delay - delay;
	wire signed [2 * data_width - 1 : 0] min_delay_inc = -delay;
	
	always @(posedge clk) begin
		if (reset) begin
			initialised <= 0;
			invalid 	<= 0;
			state 		<= IDLE;
			wrapped 	<= 0;
		end else if (init) begin
			state 	 	   <= IDLE;
			data_out 	   <= 0;
			data_out_valid <= 1;
			addr  	 	   <= init_addr;
			size  	 	   <= init_size;
			delay 	 	   <= init_delay;
			gain 	 	   <= 0;
			position 	   <= 0;
			invalid  	   <= 0;
			
			mem_write_req <= 0;
			mem_read_req  <= 0;
			
			initialised <= 1;
		end else if (enable) begin
			invalid <= 0;
			write_done <= 0;
			
			if (delay > max_delay) delay <= max_delay;
			
			if (max_delay[2 * data_width - 1]) delay <= -delay;
			
			case (state)
				IDLE: begin
					if (write) begin
						data_out_valid <= 0;
						
						if (!initialised) begin
							invalid <= 1;
						end else begin
							mem_write_addr <= addr + position;
							mem_data_out   <= data_in;
							mem_write_req  <= 1;
							
							delay_inc_clamped <= (delay_inc > max_delay_inc)
											  ? max_delay_inc
											  : ((delay_inc < min_delay_inc)
												  ? min_delay_inc
												  : delay_inc);
							
							state <= WRITE_REQ;
						end
					end
				end
				
				WRITE_REQ: begin
					if (mem_write_ack) begin
						mem_write_req <= 0;
						
						mem_read_addr <= delay_addr;
						mem_read_req  <= 1;
						
						delay <= delay + delay_inc_clamped;
						
						state <= FETCH_1;
					end
				end
				
				FETCH_1: begin
					if (mem_read_valid) begin
						data_out 		<= mem_data_in;
						data_out_valid 	<= 1;
						mem_read_req 	<= 0;
						
						if (position == size - 1) begin
							wrapped <= 1;
							position <= 0;
						end else begin
							position <= position + 1;
						end
						
						if (wrapped && gain < 16'b0100000000000000)
							gain <= gain + 16'b0000000001000000;
						
						write_done <= 1;
						state <= IDLE;
					end
				end
			endcase
		end
	end
	
endmodule

module delay_master
	#(
		parameter data_width  = 16,
		parameter n_buffers	  = 32,
		parameter memory_size = 12
	)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input wire alloc_req,
		input wire [2 * addr_width - 1 : 0] alloc_size,
		input wire [2 * addr_width - 1 : 0] alloc_delay,
		
		input wire read_req,
		input wire write_req,
		input wire signed [data_width - 1 : 0] write_data,
		input wire signed [data_width - 1 : 0] write_inc,
		
		input wire [data_width - 1 : 0] read_handle,
		input wire [data_width - 1 : 0] write_handle,
		
		output reg [data_width - 1 : 0] data_out,
		
		output reg read_valid,
		output reg write_ack,
		
		output reg invalid_alloc,
		output reg invalid_read,
		output reg invalid_write
	);
	
	localparam addr_width = $clog2(memory_size);
	
	(* ram_style = "block" *)
	reg [data_width - 1 : 0] memory [memory_size - 1 : 0];
	
	reg  [addr_width - 1 : 0] mem_write_addr;
	reg  [addr_width - 1 : 0] mem_read_addr;
	reg  [data_width - 1 : 0] mem_write_val;
	reg  [data_width - 1 : 0] mem_read_val;
	reg mem_write_enable;
	
	reg mem_read_req;
	reg mem_read_valid;
	reg mem_write_ack;
	
	always @(posedge clk) begin
		mem_write_ack <= 0;
		mem_read_valid <= 0;
		
		if (mem_read_req)
			mem_read_valid <= 1;
		
		mem_read_val <= memory[mem_read_addr];
		
		if (mem_write_enable) begin
			memory[mem_write_addr] <= mem_write_val;
			mem_write_ack <= 1;
		end
	end
	
	
	wire [data_width - 1 : 0] buf_data_out [n_buffers - 1 : 0];
	wire [n_buffers  - 1 : 0] buf_data_out_valid;
	
	reg  [n_buffers  - 1 : 0] buf_write;
	
	reg signed [data_width - 1 : 0] buf_data_in;
	reg signed [data_width - 1 : 0] buf_delay_inc;
	
	reg [data_width - 1 : 0] buf_gain [n_buffers - 1 : 0];
	
	reg  [n_buffers  - 1 : 0] buf_init;
	
	wire [n_buffers  - 1 : 0] buf_mem_read_req;
	wire [n_buffers  - 1 : 0] buf_mem_write_req;
	wire [addr_width - 1 : 0] buf_mem_read_addr [n_buffers - 1 : 0];
	reg  [data_width - 1 : 0] buf_mem_data_in 	[n_buffers - 1 : 0];
	
	wire [addr_width - 1 : 0] buf_mem_write_addr [n_buffers - 1 : 0];
	wire [data_width - 1 : 0] buf_mem_data_out 	 [n_buffers - 1 : 0];
	
	wire [n_buffers  - 1 : 0] buf_ready;
	wire [n_buffers  - 1 : 0] buf_write_done;
	
	wire [n_buffers  - 1 : 0] buf_invalid;

	genvar i;
	generate
		for (i = 0; i < n_buffers; i = i + 1) begin : DELAY_BUFFERS
			delay_buffer_controller #(.data_width(data_width), .addr_width($clog2(memory_size)), .handle(i))
				delay_controller(
					.clk(clk),
					.reset(reset),
					
					.enable(enable),
					
					.data_out(buf_data_out[i]),
					.data_out_valid(buf_data_out_valid[i]),
					
					.gain(buf_gain[i]),
					
					.write(buf_write[i]),
					
					.data_in  (write_data),
					.delay_inc(write_inc),
					
					.init(buf_init[i]),
					
					.init_addr (buf_init_addr),
					.init_size (buf_init_size),
					.init_delay(buf_init_delay),

					.mem_read_req (buf_mem_read_req [i]),
					.mem_write_req(buf_mem_write_req[i]),
					.mem_read_addr(buf_mem_read_addr[i]),
					.mem_data_in  (mem_read_val),

					.mem_write_addr(buf_mem_write_addr[i]),
					.mem_data_out  (buf_mem_data_out  [i]),

					.ready(buf_ready[i]),
					.write_done(buf_write_done[i]),

					.mem_read_valid(mem_read_valid),
					.mem_write_ack (mem_write_ack));
		end
	endgenerate
	
	localparam handle_width = $clog2(n_buffers);
	
	localparam IDLE  = 3'd0;
	localparam ALLOC = 3'd1;
	localparam READ  = 3'd2;
	localparam WRITE = 3'd3;
	
	reg [2:0] state;
	
	reg [addr_width - 1 : 0] buf_init_addr;
	reg [handle_width   : 0] n_buffers_initd;
	
	reg inc_buf_addr;
	reg [addr_width - 1 : 0] buf_addr_inc;
	
	wire alloc_valid	 = (n_buffers_initd < n_buffers && buf_init_addr + alloc_size < memory_size);
	wire read_req_valid  = read_handle  < n_buffers_initd;
	wire write_req_valid = write_handle < n_buffers_initd;
	
	reg [	addr_width - 1 : 0] buf_init_size;
	reg [2 * data_width - 1 : 0] buf_init_delay;

	reg [handle_width - 1 : 0] read_handle_latched;
	reg reading;
	reg [handle_width - 1 : 0] write_handle_latched;
	reg writing;
	reg write_dispatched;
	reg signed [data_width - 1 : 0] write_data_latched;
	reg signed [data_width - 1 : 0]  write_inc_latched;
	
	reg read_data_obtained;
	
	reg signed [data_width - 1 : 0] read_data;
	reg signed [data_width - 1 : 0] read_gain;
	
	wire signed [2 * data_width - 1 : 0] read_product = read_data * read_gain;

	always @(posedge clk) begin
		invalid_alloc <= 0;
		invalid_read  <= 0;
		invalid_write <= 0;
		
		mem_read_req <= 0;
		mem_write_enable <= 0;
		
		read_valid <= 0;
		write_ack  <= 0;
		
		buf_init <= 0;
		
		inc_buf_addr <= 0;
		
		read_data_obtained <= 0;
		
		if (reset) begin
			buf_init_addr <= 0;
			n_buffers_initd <= 0;
			reading <= 0;
			writing <= 0;
			write_dispatched <= 0;
			read_data_obtained <= 0;
		end else if (enable) begin
			
			if (inc_buf_addr)
				buf_init_addr <= buf_init_addr + buf_addr_inc;
			
			if (alloc_req) begin
				if (alloc_valid) begin
					buf_init_size <= alloc_size;
					buf_init_delay <= alloc_delay;
					buf_init[n_buffers_initd] <= 1;
					
					buf_addr_inc <= alloc_size;
					inc_buf_addr <= 1;
					
					n_buffers_initd <= n_buffers_initd + 1;
				end else begin
					invalid_alloc <= 1;
				end
			end
			
			if (reading) begin
				if (read_data_obtained) begin
					data_out <= read_product >>> 14;
					read_valid <= 1;
					reading <= 0;
				end else if (buf_data_out_valid[read_handle_latched]) begin
					read_data <= buf_data_out[read_handle_latched];
					read_gain <= buf_gain[read_handle_latched];
					read_data_obtained <= 1;
				end
			end else if (read_req) begin
				if (read_req_valid) begin
					reading <= 1;
					if (buf_data_out_valid[read_handle]) begin
						read_data <= buf_data_out[read_handle];
						read_gain <= buf_gain[read_handle];
						read_data_obtained <= 1;
					end else begin
						read_handle_latched <= read_handle;
					end
				end else begin
					invalid_read <= 1;
				end
			end
			
			if (writing) begin
				if (write_dispatched) begin
					buf_write[write_handle] <= 0;
					
					if (buf_write_done[write_handle_latched]) begin
						writing <= 0;
						write_dispatched <= 0;
					end else begin
						mem_write_addr   <= buf_mem_write_addr[write_handle];
						mem_write_val	<= buf_mem_data_out  [write_handle];
						mem_read_addr	<= buf_mem_read_addr [write_handle];
						mem_read_req	 <= buf_mem_read_req  [write_handle];
						mem_write_enable <= buf_mem_write_req [write_handle];
					end
				end else begin
					if (buf_ready[write_handle]) begin
						buf_write[write_handle] <= 1;
						write_dispatched <= 1;
					end
				end
			end else if (write_req) begin
				if (write_req_valid) begin
					write_handle_latched <= write_handle;
					buf_data_in[write_handle] <= write_data;
					buf_delay_inc[write_handle] <= write_inc;
					
					if (buf_ready[write_handle]) begin
						buf_write[write_handle] <= 1;
						write_dispatched <= 1;
					end
					
					writing <= 1;
					write_ack <= 1;
				end else begin
					invalid_write <= 1;
				end
			end
		end
	end

endmodule
