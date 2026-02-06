`include "instr_dec.vh"

`include "lut.vh"
`include "core.vh"


module commit_master #(parameter data_width = 16)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input wire sample_tick,
		input wire signed [data_width - 1 : 0] sample_in,
		
		input wire [`N_INSTR_BRANCHES - 1 : 0] in_valid,
		output reg [`N_INSTR_BRANCHES - 1 : 0] in_ready,
		
		input wire [2 * data_width 	  - 1 : 0] result		[`N_INSTR_BRANCHES - 1 : 0],
		input wire [3 					  : 0] dest			[`N_INSTR_BRANCHES - 1 : 0],
		input wire [8 					  : 0] commit_id	[`N_INSTR_BRANCHES - 1 : 0],
		input wire [`N_INSTR_BRANCHES - 1 : 0] commit_flag,
		
		output reg [3 : 0] 				channel_write_addr,
		output reg [data_width - 1 : 0] channel_write_val,
		output reg channel_write_enable,
		
		output reg [2 * data_width - 1 : 0] acc_write_val,
		output reg acc_write_enable,
		output reg acc_add_enable
	);
	
	reg [8:0] next_commit_id;
	
	bit found;
	
	integer i;
	always @(posedge clk) begin	
		
		in_ready <= 0;
		
		acc_add_enable <= 0;
		acc_write_enable <= 0;
		channel_write_enable <= 0;
		
		found = 0;
		
		if (reset) begin
			next_commit_id <= 0;
		end else if (enable && sample_tick) begin
			channel_write_addr 		<= 0;
			channel_write_val  		<= sample_in;
			channel_write_enable 	<= 1;
		end else if (enable) begin
			for (i = 0; i < `N_INSTR_BRANCHES && !found; i = i + 1) begin
				if (in_valid[i] && commit_id[i] == next_commit_id) begin
					if (i == `INSTR_BRANCH_MAC) begin
						acc_write_val <= result[i];
						acc_write_enable <= 1;
						acc_add_enable <= commit_flag[i];
					end else begin
						channel_write_addr <= dest[i];
						channel_write_val  <= result[i][data_width - 1 : 0];
						channel_write_enable <= 1;
					end
					
					next_commit_id <= next_commit_id + 1;
					in_ready[i] <= 1;
					
					found = 1;
				end
			end
		end
	end
endmodule
