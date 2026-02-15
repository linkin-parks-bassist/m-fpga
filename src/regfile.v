module block_regfile #(parameter data_width = 16, parameter n_blocks = 256)
	(
		input wire clk,
		input wire reset,
		
		input wire [$clog2(n_blocks) - 1 : 0] n_active_blocks,
		
		input wire [$clog2(n_blocks) - 1 : 0] read_addr,
		output reg read_valid,
		
		input wire [$clog2(n_blocks) - 1 : 0] write_addr,
		input wire [data_width 		 - 1 : 0] write_value,
		input wire write_select,
		input wire write_enable,
		
		output reg [2 * data_width - 1 : 0] registers_packed_out,
		
		output wire [data_width - 1 : 0] register_0_out,
		output wire [data_width - 1 : 0] register_1_out,
		
		input wire sync,
		input wire [$clog2(n_blocks) - 1 : 0] sync_addr,
		input wire [2 * data_width   - 1 : 0] sync_value,
		output reg syncing
	);
	
	(* ram_style = "block" *)
	reg [2 * data_width - 1 : 0] registers [n_blocks - 1 : 0];
	
	reg [2 * data_width   - 1 : 0] read_back_val;
	reg write_select_latched;
	
	wire [$clog2(n_blocks) - 1 : 0] read_addr_int = write_enable ? write_addr : read_addr;
	reg  [$clog2(n_blocks) - 1 : 0] write_addr_int;
	reg  [2 * data_width   - 1 : 0] write_val_int;
	reg  [	data_width   - 1 : 0] write_val_latched;
	reg write_enable_int;
	
	reg [$clog2(n_blocks) - 1 : 0] sync_start_addr;
	reg [$clog2(n_blocks) - 1 : 0] sync_addr_prev;
	reg sync_addr_changed_ever;
	reg sync_addr_changed;
	reg sync_addr_wrapped;
	
	reg write_issued;
	
	assign register_0_out = registers_packed_out[	data_width - 1 :		  0];
	assign register_1_out = registers_packed_out[2 * data_width - 1 : data_width];
	
	
	always @(posedge clk) begin
		registers_packed_out <= registers[read_addr_int];
	
		if (write_enable_int)
			registers[write_addr_int] <= write_val_int;
	end
	
	always @(posedge clk) begin
		write_enable_int <= 0;
		write_issued <= 0;
	
		if (reset) begin
			read_valid <= 0;
			syncing <= 0;
			
			sync_addr_changed <= 0;
		end else if (syncing) begin
			read_valid <= 0;
			
			sync_addr_prev <= sync_addr;
			
			sync_addr_changed <= (sync_addr != sync_addr_prev);
			sync_addr_changed_ever <= sync_addr_changed_ever | sync_addr_changed;
			if (sync_addr_changed_ever && sync_addr == sync_start_addr)
				sync_addr_wrapped <= 1;
			
			write_addr_int <= sync_addr;
			write_val_int  <= sync_value;
			
			write_enable_int <= sync_addr_changed || (n_active_blocks < 2);
			
			syncing <= ~((n_active_blocks == 1) | sync_addr_wrapped);
		end else if (sync && |n_active_blocks) begin
			read_valid <= 0;
			syncing <= 1;
			sync_start_addr <= sync_addr;
			sync_addr_prev <= sync_addr;
			write_addr_int <= sync_addr;
			write_val_int <= sync_value;
			
			sync_addr_changed_ever <= 0;
			sync_addr_changed <= 0;
			sync_addr_wrapped <= 0;
		end else begin
			read_valid <= 1;
			syncing <= 0;
			
			if (write_enable) begin
				read_valid <= 0;
				write_issued <= 1;
				write_select_latched <= write_select;
				write_val_latched <= write_value;
				write_addr_int <= write_addr;
			end
			
			if (write_issued) begin
				if (write_select_latched)
					write_val_int <= {write_val_latched, register_0_out};
				else
					write_val_int <= {register_1_out, write_val_latched};
				
				write_enable_int <= 1;
			end
		end
	end
	
endmodule
