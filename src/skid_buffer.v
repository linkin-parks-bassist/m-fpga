`default_nettype none

module skid_buffer #(parameter payload_width = 64)
	(
		input wire clk,
		input wire reset,
		
		input wire enable,
		
		input wire  in_valid,
		output wire in_ready,
		
		output reg out_valid,
		input wire out_ready,
		
		input wire [payload_width - 1 : 0] payload_in,
		output reg [payload_width - 1 : 0] payload_out
	);

	reg [payload_width - 1 : 0] payload_skid;
	reg skid;
	
	assign in_ready = ~(out_valid & skid);
	
	wire take_in = in_ready & in_valid;
	wire take_out = out_valid & out_ready;
	
	always @(posedge clk) begin
		if (reset) begin
			skid <= 0;
			out_valid <= 0;
			payload_out <= 0;
		end else if (enable) begin
			case ({take_in, take_out})
				2'b00: begin
				
				end
				
				2'b01: begin
					if (skid) begin
						payload_out <= payload_skid;
						skid <= 0;
					end else begin
						out_valid <= 0;
					end
				end
				
				2'b10: begin
					if (out_valid) begin
						payload_skid <= payload_in;
						skid <= 1;
					end else begin
						payload_out <= payload_in;
						out_valid <= 1;
					end
				end
				
				2'b11: begin
					if (skid) begin
						payload_out <= payload_skid;
						payload_skid <= payload_in;
					end else begin
						payload_out <= payload_in;
					end
				end
			endcase
		end
	end
endmodule

`default_nettype wire
