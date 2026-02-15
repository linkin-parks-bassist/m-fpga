`include "instr_dec.vh"
`include "lut.vh"
`include "core.vh"

module resource_branch #(parameter data_width = 16, parameter handle_width = 8, parameter n_blocks = 256) (
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input  wire in_valid,
		output wire in_ready,
		
		output wire out_valid,
		input  wire out_ready,
		
		input wire [$clog2(n_blocks) - 1 : 0] block_in,
		output reg [$clog2(n_blocks) - 1 : 0] block_out,
		
		input wire write,
		
		input wire [handle_width - 1 : 0] handle_in,
		input wire [data_width   - 1 : 0] arg_a_in,
		input wire [data_width   - 1 : 0] arg_b_in,
		
		output wire read_req,
		output wire write_req,
		
		output reg 		[handle_width - 1 : 0] handle_out,
		output reg signed [data_width - 1 : 0] arg_a_out,
		output reg signed [data_width - 1 : 0] arg_b_out,
		input wire signed [data_width - 1 : 0] data_in,
		
		input wire read_valid,
		input wire write_ack,
		
		input wire [3:0] dest_in,
		output reg [3:0] dest_out,
		
		output reg signed [2 * data_width - 1 : 0] result_out,
		
		input wire [`COMMIT_ID_WIDTH - 1 : 0] commit_id_in,
		output reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id_out
	);
	
	localparam IDLE = 2'd0;
	localparam REQ  = 2'd1;
	localparam DONE = 2'd2;
	
	assign in_ready  = (state == IDLE);
	assign out_valid = (state == DONE);
	
	assign read_req  = (state == REQ) & ~write_latched;
	assign write_req = (state == REQ) &  write_latched;
	
	reg [$clog2(n_blocks) - 1 : 0] block_latched;
	reg [`COMMIT_ID_WIDTH - 1 : 0] commit_id_latched;
	reg [3:0] dest_latched;
	reg write_latched;
	reg [1:0] state;
	
	always @(posedge clk) begin
		if (reset) begin
			state <= 0;
			commit_id_out <= 0;
		end else if (enable) begin
			
			case (state)
				IDLE: begin
					if (in_valid && in_ready) begin
						handle_out <= handle_in;
						
						arg_a_out <= arg_a_in;
						arg_b_out <= arg_b_in;
						
						dest_latched <= dest_in;
						
						write_latched <= write;
						
						commit_id_latched <= commit_id_in;
						block_latched <= block_in;
						
						state <= REQ;
					end
				end
				
				REQ: begin
					if (write_latched && write_ack) begin
						state <= IDLE;
					end else if (!write_latched && read_valid) begin
						result_out <= {{(data_width){data_in[data_width-1]}}, data_in};
						
						commit_id_out <= commit_id_latched;
						block_out <= block_latched;
						dest_out <= dest_latched;
						
						state <= DONE;
					end
				end
				
				DONE: begin
					if (out_ready) begin
						state <= IDLE;
					end
				end
			endcase
		end
	end
endmodule
