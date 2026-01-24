`define DELAY_MASTER_STATE_READY 		0
`define DELAY_MASTER_STATE_READ_WAIT	1
`define DELAY_MASTER_STATE_WRITE_WAIT	2

`define DELAY_CTRL_READ_STATE_READY  	0
`define DELAY_CTRL_READ_STATE_PAUSE1  	1
`define DELAY_CTRL_READ_STATE_DISPATCH 	2
`define DELAY_CTRL_READ_STATE_PAUSE2 	3
`define DELAY_CTRL_READ_STATE_WAIT 		4
`define DELAY_CTRL_READ_STATE_SETTLE	5


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
		output reg [data_width 	    - 1 : 0] data_to_sram,
		
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

	wire [sram_buffer_handle_width - 1 : 0] trunc_read_handle = read_req_handle_latched[sram_buffer_handle_width - 1 : 0];

	wire [sram_buffer_handle_width - 1 : 0] trunc_write_handle = write_req_handle_latched[sram_buffer_handle_width - 1 : 0];
	
	wire valid_read_handle  = ~(|read_req_handle_latched [data_width - 1 : sram_buffer_handle_width]) & (trunc_read_handle  < sram_buffer_next_handle);
	wire valid_write_handle = ~(|write_req_handle_latched[data_width - 1 : sram_buffer_handle_width]) & (trunc_write_handle < sram_buffer_next_handle);
	
	wire [sram_addr_width	- 1 : 0] read_req_arg_sram_addr;
	wire [sram_addr_width	- 1 : 0] write_req_arg_sram_addr;
	
	generate
		if (data_width > sram_addr_width) begin : SMALL_SRAM_ADDR
			assign read_req_arg_sram_addr  =  read_req_arg_latched[sram_addr_width - 1 : 0];
			assign write_req_arg_sram_addr = write_req_arg_latched[sram_addr_width - 1 : 0];
		end
		else begin : LARGE_SRAM_ADDR
			assign read_req_arg_sram_addr  = {{(sram_addr_width - data_width){1'b0}},  read_req_arg_latched[data_width - 1 : 0]};
			assign write_req_arg_sram_addr = {{(sram_addr_width - data_width){1'b0}}, write_req_arg_latched[data_width - 1 : 0]};
		end
	endgenerate

	reg [sram_addr_width   - 1 : 0] base_addr;
	reg [sram_addr_width   - 1 : 0] buffer_position;

	wire [sram_addr_width   - 1 : 0] read_sram_offset 	= (buffer_position - read_req_arg_sram_addr) & mod_mask;
	wire [sram_addr_width   - 1 : 0] read_sram_addr  	= base_addr + read_sram_offset;
	
	wire [sram_addr_width	- 1 : 0] next_buffer_offset 	= (buffer_position + 1) & mod_mask;
	wire [sram_addr_width   - 1 : 0] next_buffer_pos 	= base_addr + next_buffer_offset;

	reg [sram_buffer_handle_width 	- 1 : 0] sram_buffer_next_handle = 0;
	reg [sram_addr_width   			- 1 : 0] sram_alloc_addr = 0;

	reg  read_wait_one = 0;
	reg write_wait_one = 0;
	
	wire buffers_exhausted 		= (sram_buffer_next_handle >= (n_sram_buffers - 1));
	wire alloc_req_size_pow2 	= ~|(alloc_size_latched & (alloc_size_latched - 1));
	wire alloc_too_big			= ((sram_alloc_addr + alloc_size_latched) > sram_capacity);

	localparam data_sram_cmp_width = data_width > sram_addr_width ? data_width : sram_addr_width;
	
	wire [data_sram_cmp_width - 1 : 0] read_req_arg_ext 	= read_req_arg_latched;
	reg  [data_sram_cmp_width - 1 : 0] read_buffer_size_ext;
    wire [sram_addr_width     - 1 : 0] mod_mask = read_buffer_size_ext - 1;

    reg allocating = 0;
    reg [sram_addr_width - 1 : 0] alloc_size_latched;

    reg [2:0] write_state = 0;
    reg [data_width - 1 : 0] write_req_arg_latched;
    reg [data_width - 1 : 0] write_req_handle_latched;
    reg [2:0] read_state = 0;
    reg [data_width - 1 : 0] read_req_arg_latched;
    reg [data_width - 1 : 0] read_req_handle_latched;

    reg sram_buffer_addrs_we = 0;
    reg sram_buffer_sizes_we = 0;
    reg sram_buffer_posns_we = 0;

    reg [sram_buffer_handle_width - 1 : 0] sram_buffer_addrs_w_addr;
    reg [sram_buffer_handle_width - 1 : 0] sram_buffer_sizes_w_addr;
    reg [sram_buffer_handle_width - 1 : 0] sram_buffer_posns_w_addr;

    reg [sram_addr_width - 1 : 0] sram_buffer_addrs_din;
    reg [sram_addr_width - 1 : 0] sram_buffer_sizes_din;
    reg [sram_addr_width - 1 : 0] sram_buffer_posns_din;

	always @(posedge clk) begin
		invalid_read 	<= 0;
		invalid_write 	<= 0;
		invalid_alloc 	<= 0;
		
		read_ready  <= 0;
		write_ready <= 0;
		
		read_wait_one  	<= 0;
		write_wait_one 	<= 0;
		
        base_addr       <= sram_buffer_addrs[trunc_write_handle];
        buffer_position <= sram_buffer_posns[trunc_write_handle];

        read_buffer_size_ext <= sram_buffer_sizes[trunc_read_handle];

        if (sram_buffer_addrs_we)
            sram_buffer_addrs[sram_buffer_addrs_w_addr] <= sram_buffer_addrs_din;
        if (sram_buffer_sizes_we)
            sram_buffer_sizes[sram_buffer_sizes_w_addr] <= sram_buffer_sizes_din;
        if (sram_buffer_posns_we)
            sram_buffer_posns[sram_buffer_posns_w_addr] <= sram_buffer_posns_din;

        
        sram_buffer_addrs_we <= 0;
        sram_buffer_sizes_we <= 0;
        sram_buffer_posns_we <= 0;

		if (reset) begin
			state 			<= 0;
			read_ready 		<= 1;
			write_ready 	<= 1;
			read_wait_one  	<= 0;
			write_wait_one 	<= 0;
			
			sram_buffer_next_handle <= 0;
			sram_alloc_addr <= 0;
			
            sram_buffer_addrs_w_addr <= 0;
            sram_buffer_addrs_din <= 0;
            sram_buffer_addrs_we <= 1;
            sram_buffer_sizes_w_addr <= 0;
            sram_buffer_sizes_din <= 0;
            sram_buffer_sizes_we <= 1;
            sram_buffer_posns_w_addr <= 0;
            sram_buffer_posns_din <= 0;
            sram_buffer_posns_we <= 1;
			
			req_sram_read_addr <= 0;
			
			req_sram_read 	<= 0;
			req_sram_write 	<= 0;
			
			data_out 	<= 0;
		end
		else begin
			if (alloc_sram_req) begin
                alloc_size_latched <= alloc_size;
                allocating <= 1;
            end

            if (allocating) begin
				if (buffers_exhausted || ~alloc_req_size_pow2 || alloc_too_big) begin
					invalid_alloc <= 1;
                    allocating <= 0;
				end
				else begin
                    sram_buffer_addrs_w_addr <= sram_buffer_next_handle;
                    sram_buffer_addrs_din <= sram_alloc_addr;
                    sram_buffer_addrs_we <= 1;

					sram_buffer_sizes_w_addr <= sram_buffer_next_handle;
                    sram_buffer_sizes_din <= alloc_size_latched;
                    sram_buffer_sizes_we <= 1;

					sram_buffer_posns_w_addr <= sram_buffer_next_handle;
                    sram_buffer_posns_din <= 0;
                    sram_buffer_posns_we <= 1;
					
					sram_buffer_next_handle <= sram_buffer_next_handle + 1;
					sram_alloc_addr <= sram_alloc_addr + alloc_size_latched;
                    allocating <= 0;
				end
			end

            case (read_state)
                `DELAY_CTRL_READ_STATE_READY: begin
                    if (read_req) begin
                        read_req_arg_latched <= read_req_arg;
                        read_req_handle_latched <= read_req_handle;
                        read_state <= `DELAY_CTRL_READ_STATE_PAUSE1;
                    end
                end

                `DELAY_CTRL_READ_STATE_PAUSE1: begin
                    read_state <= `DELAY_CTRL_READ_STATE_DISPATCH;
                end

                `DELAY_CTRL_READ_STATE_DISPATCH: begin
                    if (valid_read_handle) begin
                        req_sram_read_addr 	<= read_sram_addr;
						req_sram_read  		<= 1;
						
						read_state <= `DELAY_CTRL_READ_STATE_PAUSE2;
                    end else begin
                        invalid_read <= 1;
                        read_state <= `DELAY_CTRL_READ_STATE_SETTLE;
                    end
                end

                `DELAY_CTRL_READ_STATE_PAUSE2: begin
                    read_state <= 4;
                end

                `DELAY_CTRL_READ_STATE_WAIT: begin
                    if (sram_read_invalid) begin
                        invalid_read 	<= 1;

                        req_sram_read 	<= 0;
                        read_state      <= `DELAY_CTRL_READ_STATE_SETTLE;
                    end else if (sram_read_ready) begin
                        data_out 	<= data_from_sram;
                        
                        req_sram_read 	<= 0;
                        read_ready 		<= 1;
                        read_state      <= `DELAY_CTRL_READ_STATE_SETTLE;
                    end
                end
    
                `DELAY_CTRL_READ_STATE_SETTLE: begin
                    read_state <= 0;
                end
            endcase
			
            case (write_state)
                0: begin
                    if (write_req) begin
                        write_req_arg_latched       <= write_req_arg;
                        write_req_handle_latched    <= write_req_handle;
                        write_state <= 1;
                    end
                end
                
                1: begin
                    write_state <= 2;
                end
                
                2: begin
                    if (valid_write_handle) begin
                        req_sram_write_addr <= base_addr + buffer_position;
						data_to_sram 		<= write_req_arg_latched;
						req_sram_write 		<= 1;
						
						write_state <= 3;
                    end else begin
                        invalid_write   <= 1;
                        write_state     <= 0;
                    end
                end

                3: begin
                    write_state <= 4;
                end

                4: begin
                    if (sram_write_ready || sram_write_invalid) begin
                        req_sram_write 	<= 0;
                        write_ready 	<= 1;
                        invalid_write 	<= sram_write_invalid;
                        
                        sram_buffer_posns_w_addr <= trunc_write_handle;
                        sram_buffer_posns_din <= next_buffer_pos;
                        sram_buffer_posns_we <= 1;

                        write_state <= 5;
                    end
                end

                5: begin
                    write_state <= 0;
                end

            endcase
		end
	end
endmodule
