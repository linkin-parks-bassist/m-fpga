module combinatorial_interp
	#(
		parameter data_width = 16,
		parameter interp_bits = 4
	)
	(
		input  wire signed [data_width  - 1 : 0] base,
		input  wire signed [data_width  - 1 : 0] target,
		input  wire 	   [interp_bits - 1 : 0] frac,
		output wire signed [data_width  - 1 : 0] interpolated
	);

    wire signed [data_width-1:0] diff = target - base;

    wire diff_sign = diff[data_width-1];
    wire [data_width-1:0] diff_mag =
        diff_sign ? -diff : diff;

    wire signed [data_width-1:0] interp_terms [0:interp_bits-1];
    wire signed [data_width-1:0] interp_sums  [0:interp_bits-1];

    genvar i;
    generate
        for (i = 0; i < interp_bits; i = i + 1) begin : gen_interp

            wire [data_width-1:0] mag_term = diff_mag >> (i+1);

            wire signed [data_width-1:0] signed_term =
                diff_sign ? -$signed(mag_term)
                          :  $signed(mag_term);

            assign interp_terms[i] =
                frac[interp_bits-1-i] ? signed_term : 0;

            if (i == 0) begin
                assign interp_sums[i] = base + interp_terms[i];
            end else begin
                assign interp_sums[i] =
                    interp_sums[i-1] + interp_terms[i];
            end
        end
    endgenerate

    assign interpolated = interp_sums[interp_bits-1];

endmodule


module sequential_interp #(parameter data_width = 16, parameter interp_bits = 3)
	(
		input wire clk,
		input wire reset,
		
		input  wire start,
		output reg ready,
		
		input wire signed [data_width - 1 : 0] base,
		input wire signed [data_width - 1 : 0] target,
		input wire signed [interp_bits - 1 : 0] frac,
		output reg signed [data_width - 1 : 0] interpolated
	);

	wire signed [data_width - 1 : 0] diff = target - base;

	reg signed [data_width - 1 : 0] interp_sum;

	localparam index_width = $clog2(interp_bits);
	
	reg [index_width - 1 : 0] index;
	
	reg signed  [data_width  - 1 : 0] diff_latched;
	reg 		[interp_bits - 1 : 0] frac_latched;
	
	integer i;
	always @(posedge clk) begin
		if (reset) begin
			index <= (interp_bits - 1);
			interpolated <= 0;
			interp_sum <= 0;
			ready <= 1;
		end
		else begin
			if (start & ready) begin
				ready 		<= 0;
				
				interp_sum  <= base + (frac[interp_bits - 1] ? (diff >>> 1) : 0);
				
				if (diff[data_width-1])begin
					diff_latched <= -(((-diff) >> 2));
				end else begin
					diff_latched <= diff >> 2;
				end
				
				frac_latched <= frac;
				
				index <= (interp_bits - 2);
			end
			else if (!ready) begin
				if (index == 0) begin
					interpolated <= interp_sum + (frac_latched[0] ? diff_latched : 0);
					ready <= 1;
				end
				else begin
					interp_sum <= interp_sum + (frac_latched[index] ? diff_latched : 0);
					
					if (diff_latched[data_width-1]) begin
						diff_latched <= -(((-diff_latched) >> 1));
					end else begin
						diff_latched <= diff_latched >> 1;
					end
					
					index <= index - 1;
				end
			end
		end
	end
endmodule

