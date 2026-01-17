`define DELAY_MASTER_STATE_READY 		0
`define DELAY_MASTER_STATE_READ_WAIT	1
`define DELAY_MASTER_STATE_WRITE_WAIT	2

module delay_master
	#(
		parameter data_width = 16,
		parameter n_sram_buffers  = 32,
		parameter sram_addr_width = 12,
		parameter sram_capacity	  = 8096
	)
	(
		input wire clk,
		input wire reset,
		
		input wire alloc_sram_req,
		input wire [sram_addr_width - 1 : 0] alloc_size,
		
		input wire read_req,
		input wire write_req,
		input wire [data_width - 1 : 0] read_req_handle,
		input wire [data_width - 1 : 0] read_req_arg,
		input wire [data_width - 1 : 0] write_req_handle,
		input wire [data_width - 1 : 0] write_req_arg,
		
		output reg req_sram_read,
		output reg req_sram_write,
		output reg [sram_addr_width - 1 : 0] req_sram_read_addr,
		output reg [sram_addr_width - 1 : 0] req_sram_write_addr,
		output reg [data_width 	  - 1 : 0] data_to_sram,
		
		input wire sram_read_ready,
		input wire sram_write_ready,
		input wire [data_width - 1 : 0] data_from_sram,
		
		input wire sram_read_invalid,
		input wire sram_write_invalid,
		
		output reg [data_width - 1 : 0] data_out,
		output reg read_ready,
		output reg write_ready,
		output reg invalid_read,
		output reg invalid_write,
		output reg invalid_alloc
	);

	reg [7:0] state;
	
	reg [sram_addr_width - 1 : 0] sram_buffer_addrs [n_sram_buffers - 1 : 0];
	reg [sram_addr_width - 1 : 0] sram_buffer_sizes [n_sram_buffers - 1 : 0];
	reg [sram_addr_width - 1 : 0] sram_buffer_posns [n_sram_buffers - 1 : 0];
	
	reg [n_sram_buffers - 1 : 0] sram_buffer_wrapped;

	localparam sram_buffer_handle_width = $clog2(n_sram_buffers);

	wire [sram_buffer_handle_width - 1 : 0] trunc_read_handle = read_req_handle[sram_buffer_handle_width - 1 : 0];
	reg  [sram_buffer_handle_width - 1 : 0] trunc_read_handle_latched;

	wire [sram_buffer_handle_width - 1 : 0] trunc_write_handle = write_req_handle[sram_buffer_handle_width - 1 : 0];
	reg  [sram_buffer_handle_width - 1 : 0] trunc_write_handle_latched;
	
	wire valid_read_handle  = ~(|read_req_handle [data_width - 1 : sram_buffer_handle_width]) & (trunc_read_handle  < sram_buffer_next_handle);
	wire valid_write_handle = ~(|write_req_handle[data_width - 1 : sram_buffer_handle_width]) & (trunc_write_handle < sram_buffer_next_handle);
	
	wire [sram_addr_width	- 1 : 0] read_req_arg_sram_addr;
	wire [sram_addr_width	- 1 : 0] write_req_arg_sram_addr;
	
	generate
		if (data_width > sram_addr_width) begin : SMALL_SRAM_ADDR
			assign read_req_arg_sram_addr  =  read_req_arg[sram_addr_width - 1 : 0];
			assign write_req_arg_sram_addr = write_req_arg[sram_addr_width - 1 : 0];
		end
		else begin : LARGE_SRAM_ADDR
			assign read_req_arg_sram_addr  = {{(sram_addr_width - data_width){1'b0}},  read_req_arg[data_width - 1 : 0]};
			assign write_req_arg_sram_addr = {{(sram_addr_width - data_width){1'b0}}, write_req_arg[data_width - 1 : 0]};
		end
	endgenerate

	wire [sram_addr_width   - 1 : 0] base_addr = sram_buffer_addrs[trunc_write_handle_latched];
	wire [sram_addr_width   - 1 : 0] mod_mask  = sram_buffer_sizes[trunc_read_handle_latched] - 1;
	wire [sram_addr_width	- 1 : 0] buffer_position 	= sram_buffer_posns[trunc_write_handle_latched];

	wire [sram_addr_width   - 1 : 0] read_sram_offset 	= (buffer_position - read_req_arg_sram_addr) & mod_mask;
	wire [sram_addr_width   - 1 : 0] read_sram_addr  	= base_addr + read_sram_offset;
	
	wire [sram_addr_width	- 1 : 0] next_buffer_offset 	= (buffer_position + 1) & mod_mask;
	wire [sram_addr_width   - 1 : 0] next_buffer_pos 	= base_addr + next_buffer_offset;

	reg [sram_buffer_handle_width 	- 1 : 0] sram_buffer_next_handle = 0;
	reg [sram_addr_width   			- 1 : 0] sram_alloc_addr = 0;

	reg  read_wait_one = 0;
	reg write_wait_one = 0;
	
	wire buffers_exhausted 		= (sram_buffer_next_handle >= (n_sram_buffers - 1));
	wire alloc_req_size_pow2 	= ~|(alloc_size & (alloc_size - 1));
	wire alloc_too_big			= ((sram_alloc_addr + alloc_size) >= sram_capacity);

	localparam data_sram_cmp_width = data_width > sram_addr_width ? data_width : sram_addr_width;
	
	wire [data_sram_cmp_width - 1 : 0] read_req_arg_ext 	= read_req_arg;
	wire [data_sram_cmp_width - 1 : 0] read_buffer_size_ext = sram_buffer_sizes[trunc_read_handle];


	always @(posedge clk) begin
		invalid_read 	<= 0;
		invalid_write 	<= 0;
		invalid_alloc 	<= 0;
		
		if (reset) begin
			state 			<= 0;
			read_ready 		<= 1;
			write_ready 	<= 1;
			read_wait_one  	<= 0;
			write_wait_one 	<= 0;
			
			sram_buffer_next_handle <= 0;
			sram_alloc_addr <= 0;
			
			sram_buffer_addrs[0] <= 0;
			sram_buffer_sizes[0] <= 0;
			sram_buffer_posns[0] <= 0;
			
			req_sram_read_addr <= 0;
			
			req_sram_read 	<= 0;
			req_sram_write 	<= 0;
			
			data_out 	<= 0;
			read_ready  <= 0;
			write_ready <= 0;
		end
		else begin
			if (alloc_sram_req) begin
				if (buffers_exhausted || ~alloc_req_size_pow2 || alloc_too_big) begin
					invalid_alloc <= 1;
				end
				else begin
					sram_buffer_addrs[sram_buffer_next_handle] <= sram_alloc_addr;
					sram_buffer_sizes[sram_buffer_next_handle] <= alloc_size;
					sram_buffer_posns[sram_buffer_next_handle] <= 0;
					
					sram_buffer_next_handle <= sram_buffer_next_handle + 1;
					sram_alloc_addr <= sram_alloc_addr + alloc_size;
				end
			end
				
			if (!state[0]) begin
				if (read_req) begin
					if (valid_read_handle) begin
						req_sram_read_addr 	<= read_sram_addr;
						req_sram_read  		<= 1;
						read_wait_one		<= 1;
						
						state[0] <= 1;
						read_ready <= 0;
					end
					else begin
						invalid_read <= 1;
					end
				end
			end
			else begin
				if (read_wait_one) begin
					read_wait_one <= 0;
				end
				else if (sram_read_invalid) begin
					invalid_read 	<= 1;
					state[0] 		<= 0;
					read_ready 		<= 1;
				end
				else if (sram_read_ready) begin
					data_out 	<= data_from_sram;
					
					req_sram_read 	<= 0;
					state[0] 		<= 0;
					read_ready 		<= 1;
				end
			end
			
			if (!state[1]) begin
				if (write_req) begin
					if (valid_write_handle) begin
						req_sram_write_addr <= sram_buffer_addrs[trunc_write_handle] + sram_buffer_posns[trunc_write_handle];
						data_to_sram 		<= write_req_arg;
						req_sram_write 		<= 1;
						
						trunc_write_handle_latched <= trunc_write_handle;
						
						write_wait_one 	<= 1;
						state[1] 		<= 1;
						write_ready 	<= 0;
					end
					else begin
						invalid_write <= 1;
					end
				end
			end
			else begin
				if (write_wait_one) begin
					write_wait_one <= 0;
				end
				else if (sram_write_ready || sram_write_invalid) begin
					req_sram_write 	<= 0;
					
					state[1] 		<= 0;
					write_ready 	<= 1;
					invalid_write 	<= sram_write_invalid;
					
					sram_buffer_posns[trunc_write_handle_latched] <= next_buffer_pos;
				end
			end
		end
	end
endmodule
