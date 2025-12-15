module fifo_buffer #(parameter data_width = 8, parameter n = 16)
	(
		input wire clk,
		input wire reset,
		
		input wire write,
		input wire next,
		
		input  wire [data_width - 1 : 0] data_in,
		output wire [data_width - 1 : 0] data_out,
		
		output wire nonempty,
		output wire full,
		
		output reg [index_width : 0] count
	);

	// Force the bank size to be a power of 2; induces division
	// by 0 error at compile time if this is not the case.
	localparam integer _IS_POW2 = ((n & (n - 1)) == 0);
	localparam integer _FORCE_POW2 = 1 / _IS_POW2;

	localparam index_width = $clog2(n);

	reg [index_width - 1 : 0] index_in;
	reg [index_width - 1 : 0] index_out;
	
	assign full 		= count[index_width];
	assign nonempty 	= (count != 0);
	assign data_out		= regs[index_out];
	
	reg [data_width - 1 : 0] output_reg;
	reg [data_width - 1 : 0] regs [0 : (1 << index_width) - 1];
	
	wire write_en = write && !full;
	wire read_en  = next  && nonempty;

	always @(posedge clk or posedge reset) begin
		if (reset) begin
			index_in 	<= 0;
			index_out 	<= 0;
			count		<= 0;
		end
		else begin
			if (write_en) begin
				regs[index_in] 	<= data_in;
				index_in 		<= index_in + 1;
			end
			
			if (read_en) begin
				index_out <= index_out + 1;
			end
			
			case ({write_en, read_en})
				2'b10: count <= count + 1;
				2'b01: count <= count - 1;
				default: count <= count;
			endcase
		end
	end
endmodule
