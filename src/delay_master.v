`default_nettype none

module delay_master #(parameter data_width  = 16,
					  parameter n_buffers   = 32,
					  parameter memory_size = 8192)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input wire read_req,
		input wire alloc_req,
		input wire write_req,
		
		output reg signed [data_width - 1 : 0] data_out,
		output reg read_valid,
		output reg write_ack,
		
		input wire [data_width - 1 : 0] read_handle,
		input wire [data_width - 1 : 0] write_handle,
		
		input wire signed [data_width - 1 : 0] write_data,
		input wire signed [data_width - 1 : 0] write_inc,
		
		input wire [	addr_width - 1 : 0] alloc_size,
		input wire [2 * data_width - 1 : 0] alloc_delay,
		
		output reg mem_read_req,
		output reg mem_write_req,
		
		output reg 		  [addr_width - 1 : 0] mem_read_addr,
		input wire signed [data_width - 1 : 0] mem_data_in,
		
		output reg [addr_width - 1 : 0] mem_write_addr,
		output reg signed [data_width - 1 : 0] mem_data_out,
		
		input wire mem_read_valid,
		input wire mem_write_ack,
		
		output reg invalid_read,
		output reg invalid_write,
		output reg invalid_alloc
	);
	
	localparam DELAY_FORMAT = 8;
	localparam addr_width = $clog2(memory_size);
	localparam delay_width  = addr_width + DELAY_FORMAT;
	localparam handle_width = $clog2(n_buffers);
	
	localparam IDLE 	 	= 3'd0;
	localparam WRITE_1		= 3'd1;
	localparam WRITE_2	 	= 3'd2;
	localparam WRITE_3	 	= 3'd3;
	localparam WRITE_4	 	= 3'd4;
	localparam WRITE_5	 	= 3'd5;
	localparam WRITE_6	 	= 3'd6;
	localparam WRITE_7	 	= 3'd7;
	
	localparam buf_info_width = addr_width + addr_width + delay_width + addr_width + data_width + 1 + 1;
	
	reg [buf_info_width - 1 : 0] buf_info [n_buffers - 1 : 0];
	
	reg [addr_width  - 1 : 0] addr;
	reg [addr_width  - 1 : 0] size;
	reg [delay_width - 1 : 0] delay;
	reg [addr_width  - 1 : 0] position;
	reg signed [data_width : 0] gain;
	reg wrapped;
	
	reg [buf_info_width - 1 : 0] buf_info_read;
	reg [buf_info_width - 1 : 0] buf_info_write_data;
	reg [buf_info_width - 1 : 0] buf_info_cuck_data;
	reg [handle_width   - 1 : 0] buf_info_write_handle;
	reg [handle_width   - 1 : 0] buf_info_read_handle;
	reg  buf_info_write_enable;
	
	always @(posedge clk) begin
		if (buf_info_write_enable)
			buf_info[buf_info_write_handle] <= buf_info_write_data;
		
		buf_info_read <= buf_info[buf_info_read_handle];
	end
	
	reg [n_buffers  - 1 : 0] buf_data_invalid;
	reg [n_buffers  - 1 : 0] buffer_initd;
	reg [data_width - 1 : 0] buf_data [n_buffers - 1 : 0];
	reg buf_data_write_enable;
	
	always @(posedge clk) begin
		if (buf_data_write_enable) begin
			buf_data[write_handle_r] <= buf_data_new;
			buf_data_invalid[write_handle_r] <= 0;
		end
	end
	
	
	reg [addr_width - 1 : 0] alloc_addr;
	
	reg [2:0] write_state;
	
	wire [addr_width - 1 : 0] delay_offset = delay >> DELAY_FORMAT;
	wire [addr_width - 1 : 0] delay_addr = (delay_offset > position) ? addr + position - delay_offset + size
																		 : addr + position - delay_offset;
	
	reg [addr_width + DELAY_FORMAT - 1 : 0] write_inc_clamped;
	
	wire 		[addr_width + DELAY_FORMAT - 1 : 0] max_delay 	   = (size << DELAY_FORMAT);
	wire signed [addr_width + DELAY_FORMAT - 1 : 0] max_delay_inc = max_delay - delay;
	wire signed [addr_width + DELAY_FORMAT - 1 : 0] min_delay_inc = -delay;
	
	reg [$clog2(n_buffers + 1) - 1 : 0] n_buffers_allocd;
	
	wire buffers_exhausted 	= (n_buffers_allocd == n_buffers);
	wire alloc_too_big		= alloc_addr + alloc_size >= memory_size;
	
	reg read_wait;
	reg [data_width - 1 : 0] read_wait_handle;
	
	reg signed [data_width - 1 : 0] buf_data_in;
	reg signed [data_width - 1 : 0] buf_data_new;
	
	reg signed [data_width - 1 : 0] write_data_r;
	reg signed [data_width - 1 : 0] write_inc_r;
	reg        [data_width - 1 : 0] write_handle_r;
	
	wire signed [2 * data_width - 1 : 0] product = $signed(buf_data_in) * $signed(gain);
	
	wire [addr_width  - 1 : 0] alloc_size_wm  = alloc_size [addr_width  - 1 : 0];
	wire [delay_width - 1 : 0] alloc_delay_wm = alloc_delay[delay_width - 1 : 0];
	
	reg read_wait_one;
	
	always @(posedge clk) begin
		write_ack <= 0;
		read_valid <= 0;
		
		buf_info_write_enable <= 0;
		buf_data_write_enable <= 0;
		
		invalid_alloc <= 0;
		invalid_write <= 0;
		invalid_read  <= 0;
		
		read_wait_one <= 0;
	
		if (reset) begin
			write_state <= IDLE;
			buf_data_invalid <= 0;
			n_buffers_allocd <= 0;
			buffer_initd <= 0;
			alloc_addr <= 0;
			read_wait <= 0;
		end else if (alloc_req) begin
			if (alloc_too_big || buffers_exhausted) begin
				invalid_alloc <= 1;
			end else begin
				alloc_addr <= alloc_addr + alloc_size;
				buffer_initd[n_buffers_allocd] <= 1;
				n_buffers_allocd <= n_buffers_allocd + 1;
				
				buf_info_write_data[buf_info_width                  - 1 : buf_info_width -     addr_width              ] <= alloc_addr;
				buf_info_write_data[buf_info_width - 1 * addr_width - 1 : buf_info_width - 2 * addr_width              ] <= alloc_size_wm;
				buf_info_write_data[buf_info_width - 2 * addr_width - 1 : buf_info_width - 2 * addr_width - delay_width] <= alloc_delay_wm;
				buf_info_write_data[addr_width + data_width + 1 : 0] <= 0;
				
				buf_info_write_handle <= n_buffers_allocd;
				buf_info_write_enable <= 1;
			end
		end else if (enable) begin
			if (read_wait) begin
				if (buf_data_write_enable || write_state == IDLE) begin
					data_out <= buf_data_new;
					read_valid <= 1;
					read_wait <= 0;
					read_wait_one <= 1;
				end else if (write_state == IDLE) begin
					data_out <= buf_data[read_handle];
					read_valid <= 1;
					read_wait <= 0;
					read_wait_one <= 1;
				end
			end else if (!read_wait_one && read_req) begin
				if (buf_data_invalid[read_handle]) begin
					read_wait <= 1;
					read_wait_handle <= read_handle;
				end else begin
					data_out <= buf_data[read_handle];
					read_valid <= 1;
					read_wait_one <= 1;
				end
			end
			
			case (write_state)
				IDLE: begin
					if (write_req) begin
						write_data_r <= write_data;
						write_inc_r <= write_inc;
						write_handle_r <= write_handle;
						buf_info_read_handle <= write_handle;
						
						invalid_write <= !buffer_initd[write_handle];
						
						write_inc_clamped <= (write_inc > max_delay_inc)
											  ? max_delay_inc
											  : ((write_inc < min_delay_inc)
												  ? min_delay_inc
												  : write_inc);
						
						write_state <= buffer_initd[write_handle] ? WRITE_1 : IDLE;
						write_ack <= 1;
					end
				end
				
				WRITE_1: begin
					write_state <= WRITE_2;
				end
				
				WRITE_2: begin
					{addr, size, delay, position, gain, wrapped} <= buf_info_read;
					write_state <= WRITE_3;
				end
				
				WRITE_3: begin
					buf_data_invalid[write_handle_r] <= 1;
					mem_data_out   <= write_data_r;
					mem_write_addr <= addr + position;
					mem_write_req  <= 1;
					
					write_state <= WRITE_4;
				end
				
				WRITE_4: begin
					if (mem_write_ack) begin
						mem_write_req <= 0;
						
						mem_read_addr <= delay_addr;
						mem_read_req  <= 1;
						
						delay <= delay + write_inc_clamped;
						
						write_state <= WRITE_5;
					end
				end
				
				WRITE_5: begin
					if (mem_read_valid) begin
						buf_data_in 	<= mem_data_in;
						mem_read_req 	<= 0;
						
						write_state <= WRITE_6;
					end
				end
				
				WRITE_6: begin
					buf_data_new <= product >>> 15;
					buf_data_write_enable <= 1;
					
					if (position == size - 1) begin
						wrapped <= 1;
						position <= 0;
					end else begin
						position <= position + 1;
					end
					
					if (wrapped && gain < 16'b0100000000000000)
						gain <= gain + 16'b0000000001000000;
					
					write_state <= WRITE_7;
				end
				
				WRITE_7: begin
					buf_info_write_data <= {addr, size, delay, position, gain, wrapped};
					buf_info_write_handle <= write_handle_r;
					buf_info_write_enable <= 1;
					write_state <= IDLE;
				end
			endcase
		end
	end
endmodule

`default_nettype wire
