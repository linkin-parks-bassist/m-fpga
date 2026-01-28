`define DELAY_CTRL_STATE_READY 		0
`define DELAY_CTRL_STATE_READ_WAIT	1
`define DELAY_CTRL_STATE_WRITE_WAIT	2

`define DELAY_CTRL_STATE_READ_PAUSE1  	3
`define DELAY_CTRL_STATE_READ_DISPATCH 	4
`define DELAY_CTRL_STATE_READ_PAUSE2 	5
`define DELAY_CTRL_STATE_READ_WAIT 		6
`define DELAY_CTRL_STATE_READ_VALID		7
`define DELAY_CTRL_STATE_READ_SETTLE	8

`define DELAY_CTRL_STATE_WRITE_PAUSE1	9
`define DELAY_CTRL_STATE_WRITE_DISPATCH	10
`define DELAY_CTRL_STATE_WRITE_PAUSE2	11
`define DELAY_CTRL_STATE_WRITE_WAIT		12
`define DELAY_CTRL_STATE_WRITE_SETTLE	13


module delay_master
	#(
		parameter data_width 	  = 16,
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
		input wire [data_width - 1 : 0] req_handle,
		input wire [data_width - 1 : 0] req_arg,
		
		output reg req_sram_read,
		output reg req_sram_write,
		output reg [sram_addr_width - 1 : 0] req_sram_read_addr,
		output reg [sram_addr_width - 1 : 0] req_sram_write_addr,
		output reg [data_width 	    - 1 : 0] data_to_sram,
		
		input wire sram_read_ready,
		input wire sram_write_ready,
		input wire signed [data_width - 1 : 0] data_from_sram,
		
		input wire sram_read_invalid,
		input wire sram_write_invalid,
		
		output reg [data_width - 1 : 0] data_out,
		output reg read_ready,
		output reg write_ready,
		output reg invalid_read,
		output reg invalid_write,
		output reg invalid_alloc
	);


	reg [sram_addr_width - 1 : 0] sram_buffer_addrs   [n_sram_buffers - 1 : 0];
	reg [sram_addr_width - 1 : 0] sram_buffer_sizes   [n_sram_buffers - 1 : 0];
	reg [sram_addr_width - 1 : 0] sram_buffer_posns   [n_sram_buffers - 1 : 0];
	reg signed [data_width   : 0] sram_buffer_gain	  [n_sram_buffers - 1 : 0];
	reg [n_sram_buffers  - 1 : 0] sram_buffer_wrapped;
	
	reg signed [data_width : 0] gain;
	reg signed [data_width : 0] sram_buffer_gain_din;
	reg sram_buffer_gain_we = 0;
	
	reg buffer_wrapped;
	
	reg sram_buffer_wrapped_we = 0;

	localparam sram_buffer_handle_width = $clog2(n_sram_buffers);

	wire [sram_buffer_handle_width - 1 : 0] trunc_handle = req_handle_latched[sram_buffer_handle_width - 1 : 0];
	
	wire valid_handle = ~(|req_handle_latched[data_width - 1 : sram_buffer_handle_width]) & (trunc_handle < sram_buffer_next_handle);
	
	wire [sram_addr_width	- 1 : 0] req_arg_sram_addr;
	
	generate
		if (data_width > sram_addr_width) begin : SMALL_SRAM_ADDR
			assign req_arg_sram_addr = req_arg_latched[sram_addr_width - 1 : 0];
		end
		else begin : LARGE_SRAM_ADDR
			assign req_arg_sram_addr = {{(sram_addr_width - data_width){1'b0}}, req_arg_latched[data_width - 1 : 0]};
		end
	endgenerate

	reg write_buffer_wrapped;
	reg [sram_addr_width   - 1 : 0] base_addr;
	reg [sram_addr_width   - 1 : 0] buffer_position;

	wire [sram_addr_width   - 1 : 0] read_sram_offset 	= (buffer_position - req_arg_sram_addr) & mod_mask;
	wire [sram_addr_width   - 1 : 0] read_sram_addr  	= base_addr + read_sram_offset;
	
	wire [sram_addr_width	- 1 : 0] next_buffer_offset = (buffer_position + 1) & mod_mask;
	wire [sram_addr_width   - 1 : 0] next_buffer_pos 	= base_addr + next_buffer_offset;

	reg [sram_buffer_handle_width 	- 1 : 0] sram_buffer_next_handle = 0;
	reg [sram_addr_width   			- 1 : 0] sram_alloc_addr = 0;
	
	wire buffers_exhausted 		= (sram_buffer_next_handle >= (n_sram_buffers - 1));
	wire alloc_req_size_pow2 	= ~|(alloc_size_latched & (alloc_size_latched - 1));
	wire alloc_too_big			= ((sram_alloc_addr + alloc_size_latched) > sram_capacity);

	localparam data_sram_cmp_width = data_width > sram_addr_width ? data_width : sram_addr_width;
	
	wire [data_sram_cmp_width - 1 : 0] req_arg_ext 	= req_arg_latched;
	reg  [data_sram_cmp_width - 1 : 0] buffer_size;
    wire [sram_addr_width     - 1 : 0] mod_mask = buffer_size - 1;

    reg allocating = 0;
    reg [sram_addr_width - 1 : 0] alloc_size_latched;

    reg [7:0] state = 0;
    reg [data_width - 1 : 0] req_arg_latched;
    reg [data_width - 1 : 0] req_handle_latched;

    reg sram_buffer_addrs_we = 0;
    reg sram_buffer_sizes_we = 0;
    reg sram_buffer_posns_we = 0;

    reg [sram_buffer_handle_width - 1 : 0] sram_buffer_addrs_w_addr;
    reg [sram_buffer_handle_width - 1 : 0] sram_buffer_sizes_w_addr;
    reg [sram_buffer_handle_width - 1 : 0] sram_buffer_posns_w_addr;
    reg [sram_buffer_handle_width - 1 : 0] sram_buffer_gain_w_addr;

    reg [sram_addr_width - 1 : 0] sram_buffer_addrs_din;
    reg [sram_addr_width - 1 : 0] sram_buffer_sizes_din;
    reg [sram_addr_width - 1 : 0] sram_buffer_posns_din;
    
    reg signed [2 * data_width : 0] read_val_att;

	integer i;
	always @(posedge clk) begin
		invalid_read 	<= 0;
		invalid_write 	<= 0;
		invalid_alloc 	<= 0;
		
		read_ready  <= 0;
		write_ready <= 0;
		
        base_addr       <= sram_buffer_addrs  [trunc_handle];
        buffer_position <= sram_buffer_posns  [trunc_handle];
        buffer_wrapped  <= sram_buffer_wrapped[trunc_handle];

        buffer_size <= sram_buffer_sizes[trunc_handle];
        gain		<= sram_buffer_gain [trunc_handle];

        if (sram_buffer_addrs_we)
            sram_buffer_addrs[sram_buffer_addrs_w_addr] <= sram_buffer_addrs_din;
        if (sram_buffer_sizes_we)
            sram_buffer_sizes[sram_buffer_sizes_w_addr] <= sram_buffer_sizes_din;
        if (sram_buffer_posns_we)
            sram_buffer_posns[sram_buffer_posns_w_addr] <= sram_buffer_posns_din;
        if (sram_buffer_gain_we)
			sram_buffer_gain[sram_buffer_gain_w_addr]   <= sram_buffer_gain_din;
		if (sram_buffer_wrapped_we)
			sram_buffer_wrapped[trunc_handle] <= 1;

        sram_buffer_addrs_we 	<= 0;
        sram_buffer_sizes_we 	<= 0;
        sram_buffer_posns_we 	<= 0;
        sram_buffer_gain_we  	<= 0;
        sram_buffer_wrapped_we 	<= 0;

		if (reset) begin
			state 			<= 0;
			read_ready 		<= 1;
			write_ready 	<= 1;
			
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
			
			for (i = 0; i < n_sram_buffers; i = i + 1) begin
				sram_buffer_wrapped[i] <= 0;
				sram_buffer_gain[i] <= 0;
			end
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

					sram_buffer_gain_w_addr <= sram_buffer_next_handle;
                    sram_buffer_gain_din <= 0;
                    sram_buffer_gain_we <= 1;
					
					sram_buffer_next_handle <= sram_buffer_next_handle + 1;
					sram_alloc_addr <= sram_alloc_addr + alloc_size_latched;
                    allocating <= 0;
				end
			end

            case (state)
                `DELAY_CTRL_STATE_READY: begin
					if (write_req) begin
                        req_arg_latched       <= req_arg;
                        req_handle_latched    <= req_handle;
                        state <= `DELAY_CTRL_STATE_WRITE_PAUSE1;
                    end else if (read_req) begin
                        req_arg_latched <= req_arg;
                        req_handle_latched <= req_handle;
                        state <= `DELAY_CTRL_STATE_READ_PAUSE1;
                    end
                end

                `DELAY_CTRL_STATE_READ_PAUSE1: begin
                    state <= `DELAY_CTRL_STATE_READ_DISPATCH;
                end

                `DELAY_CTRL_STATE_READ_DISPATCH: begin
                    if (valid_handle) begin
                        req_sram_read_addr 	<= read_sram_addr;
						req_sram_read  		<= 1;
						
						state <= `DELAY_CTRL_STATE_READ_PAUSE2;
                    end else begin
                        invalid_read <= 1;
                        state <= `DELAY_CTRL_STATE_READ_SETTLE;
                    end
                end

                `DELAY_CTRL_STATE_READ_PAUSE2: begin
                    state <= `DELAY_CTRL_STATE_READ_WAIT;
                end

                `DELAY_CTRL_STATE_READ_WAIT: begin
                    if (sram_read_invalid) begin
                        invalid_read 	<= 1;

                        req_sram_read 	<= 0;
                        state      <= `DELAY_CTRL_STATE_READ_SETTLE;
                    end else if (sram_read_ready) begin
                        read_val_att 	<= gain * data_from_sram;
                        
                        req_sram_read 	<= 0;
                        state      <= `DELAY_CTRL_STATE_READ_VALID;
                    end
                end
				
                `DELAY_CTRL_STATE_READ_VALID: begin
					data_out  <= read_val_att >> (data_width - 1);
					read_ready <= 1;
					state <= `DELAY_CTRL_STATE_READ_SETTLE;
				end
				
                `DELAY_CTRL_STATE_READ_SETTLE: begin
                    state <= `DELAY_CTRL_STATE_READY;
                end
                
                `DELAY_CTRL_STATE_WRITE_PAUSE1: begin
                    state <= `DELAY_CTRL_STATE_WRITE_DISPATCH;
                end
                
                `DELAY_CTRL_STATE_WRITE_DISPATCH: begin
                    if (valid_handle) begin
                        req_sram_write_addr <= base_addr + buffer_position;
						data_to_sram 		<= req_arg_latched;
						req_sram_write 		<= 1;
						
						if (buffer_position + 1 == buffer_size)
							sram_buffer_wrapped_we <= 1;
						
						if (buffer_wrapped) begin
							if (gain < 17'b01000000000000000) begin
								sram_buffer_gain_din <= gain + 17'b00000000100000000;
								sram_buffer_gain_we <= 1;
							end
						end
						
						state <= `DELAY_CTRL_STATE_WRITE_PAUSE2;
                    end else begin
                        invalid_write   <= 1;
                        state     <= `DELAY_CTRL_STATE_READY;
                    end
                end

                `DELAY_CTRL_STATE_WRITE_PAUSE2: begin
                    state <= `DELAY_CTRL_STATE_WRITE_WAIT;
                end

                `DELAY_CTRL_STATE_WRITE_WAIT: begin
                    if (sram_write_ready || sram_write_invalid) begin
                        req_sram_write 	<= 0;
                        write_ready 	<= 1;
                        invalid_write 	<= sram_write_invalid;
                        
                        sram_buffer_posns_w_addr <= trunc_handle;
                        sram_buffer_posns_din <= next_buffer_pos;
                        sram_buffer_posns_we <= 1;
						
                        state <= `DELAY_CTRL_STATE_WRITE_SETTLE;
                    end
                end

                `DELAY_CTRL_STATE_WRITE_SETTLE: begin
                    state <= `DELAY_CTRL_STATE_READY;
                end
            endcase
		end
	end
endmodule
