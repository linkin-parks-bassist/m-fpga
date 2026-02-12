`include "instr_dec.vh"

`include "lut.vh"
`include "core.vh"


module commit_master #(parameter data_width = 16, parameter n_blocks = 256)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input wire sample_tick,
		input wire signed [data_width - 1 : 0] sample_in,
		
		
		input wire   [`N_INSTR_BRANCHES - 1 : 0] in_valid,
		output logic [`N_INSTR_BRANCHES - 1 : 0] in_ready,
		
		input wire [$clog2(n_blocks)  - 1 : 0] block_in		[`N_INSTR_BRANCHES - 1 : 0],
		input wire [2 * data_width 	  - 1 : 0] result		[`N_INSTR_BRANCHES - 1 : 0],
		input wire [3 					  : 0] dest			[`N_INSTR_BRANCHES - 1 : 0],
		input wire [8 					  : 0] commit_id	[`N_INSTR_BRANCHES - 1 : 0],
		input wire [`N_INSTR_BRANCHES - 1 : 0] commit_flag,
		
		output reg [3 : 0] channel_write_addr,
		output reg [data_width - 1 : 0] channel_write_val,
		output reg channel_write_enable,
		
		output reg [2 * data_width - 1 : 0] accumulator_write_val,
		output reg accumulator_write_enable,
		output reg accumulator_add_enable,
		
		output reg [8:0] next_commit_id
	);
	
    genvar i;
    generate
		for (i = 0; i < `N_INSTR_BRANCHES; i = i + 1) begin : one_hot
			assign in_ready[i] = (in_valid[i] && commit_id[i] == next_commit_id) & ~sample_tick;
		end
    endgenerate
    
    reg [`N_INSTR_BRANCHES - 1 : 0] in_ready_prev;
	reg acc_overwrite_prev;
	reg [2 * data_width    - 1 : 0] result_prev [`N_INSTR_BRANCHES - 1 : 0];
	reg [3 					   : 0] dest_prev	[`N_INSTR_BRANCHES - 1 : 0];

	integer j;
	integer k;
	always @(posedge clk) begin	
		
		accumulator_add_enable <= 0;
		accumulator_write_enable <= 0;
		channel_write_enable <= 0;
		
		acc_overwrite_prev <= commit_flag[`INSTR_BRANCH_MAC];
		in_ready_prev <= in_ready;
		
		for (k = 0; k < `N_INSTR_BRANCHES; k = k + 1) begin
			result_prev[k] <= result[k];
			dest_prev[k]   <= dest[k];
		end
		
		if (reset) begin
			next_commit_id <= 0;
		end else if (enable && sample_tick) begin
			channel_write_addr 		<= 0;
			channel_write_val  		<= sample_in;
			channel_write_enable 	<= 1;
			
			in_ready_prev <= in_ready_prev;
			acc_overwrite_prev <= acc_overwrite_prev;
			result_prev <= result_prev;
			dest_prev <= dest_prev;
		end else if (enable) begin
			if (|in_ready) next_commit_id <= next_commit_id + 1;
			
			for (j = 0; j < `N_INSTR_BRANCHES; j = j + 1) begin
				if (in_ready_prev[j]) begin
					if (j == `INSTR_BRANCH_MAC) begin
						accumulator_write_val <= result_prev[j];
						accumulator_write_enable <= 1;
						accumulator_add_enable <= ~acc_overwrite_prev;
					end else begin
						channel_write_val <= result_prev[j][data_width - 1 : 0];
						channel_write_addr <= dest_prev[j];
						channel_write_enable <= 1;
					end
				end
			end
		end
	end
endmodule
