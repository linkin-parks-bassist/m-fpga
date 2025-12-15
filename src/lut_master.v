`define LUT_MASTER_STATE_READY 		 	0
`define LUT_MASTER_STATE_PROCESSING  	1
`define LUT_MASTER_STATE_SIN_WAIT  	    2
`define LUT_MASTER_STATE_TANH_WAIT  	3
`define LUT_MASTER_STATE_PROCESSING  	4
`define LUT_MASTER_STATE_WAIT_INTERP1 	5
`define LUT_MASTER_STATE_WAIT_INTERP2 	6
`define LUT_MASTER_STATE_SEND		 	7
`define LUT_MASTER_STATE_ERROR		 	8

`define LUT_HANDLE_SIN	0
`define LUT_HANDLE_TANH	1

`define N_CORE_LUTS	1

module lut_master #(parameter data_width = 16)
	(
		input wire clk,
		input wire reset,
		
		input wire req,
		input wire [`LUT_HANDLE_WIDTH - 1 : 0] lut_handle,
		input wire [data_width - 1 : 0] req_arg,
		
		output reg [data_width - 1 : 0] data_out,
		output reg ready,
		
		output reg invalid_request
	);
	
	reg [7:0] state = `LUT_MASTER_STATE_READY;
	
	reg [`LUT_HANDLE_WIDTH - 1 : 0] lut_handle_latched;
	reg [data_width - 1 : 0] req_arg_latched;
	
	reg [data_width - 1 : 0] base_sample;
	reg [data_width - 1 : 0] next_sample;
	
	reg [`LUT_FRAC_WIDTH - 1 : 0] frac;
	
	wire signed [data_width - 1 : 0] interpolated;
	
	reg  interp_start = 0;
	wire interp_ready;
	
	wire [data_width - 1 : 0] tanh_base_sample;
	wire [data_width - 1 : 0] tanh_next_sample;
	wire [`LUT_FRAC_WIDTH - 1 : 0] tanh_frac;
    reg tanh_read = 0;
    wire tanh_ready;

	wire [data_width - 1 : 0] sin_base_sample;
	wire [data_width - 1 : 0] sin_next_sample;
	wire [`LUT_FRAC_WIDTH - 1 : 0] sin_frac;
    reg sin_read = 0;
    wire sin_ready;

	sequential_interp #(.data_width(data_width), .interp_bits(`LUT_FRAC_WIDTH)) interp
		(
			.clk(clk),
			.reset(reset),
			
			.base(base_sample),
			.target(next_sample),
			.frac(frac),
			.interpolated(interpolated),
			
			.start(interp_start),
			.ready(interp_ready)
		);
	
    reg wait_one = 0;

	always @(posedge clk) begin
        wait_one <= 0;
        tanh_read <= 0;
        sin_read  <= 0;

		if (reset) begin
			invalid_request <= 0;
			ready <= 1;
			state <= `LUT_MASTER_STATE_READY;
		end
		else begin
			case (state)
				`LUT_MASTER_STATE_READY: begin
					if (req) begin
						lut_handle_latched 	<= lut_handle;
						req_arg_latched 	<= req_arg;
						ready <= 0;
						state <= `LUT_MASTER_STATE_PROCESSING;
					end
				end
				
				`LUT_MASTER_STATE_PROCESSING: begin
					case (lut_handle_latched)
						// Handle all the built-in LUTs
						`LUT_HANDLE_SIN: begin
							sin_read <= 1;

                            state <= `LUT_MASTER_STATE_SIN_WAIT;
                            wait_one <= 1;
						end
						
						`LUT_HANDLE_TANH: begin
                            tanh_read <= 1;

                            state <= `LUT_MASTER_STATE_TANH_WAIT;
                            wait_one <= 1;
                        end
						
						// If it wasn't one of those, it must be an alloc'd LUT.
						// ... to implement later
						default: begin
							invalid_request <= 1;
						end
					endcase
				end
				
                `LUT_MASTER_STATE_SIN_WAIT: begin
                    sin_read <= 0;
                    if (!wait_one && sin_ready) begin
                        base_sample <= sin_base_sample;
                        next_sample <= sin_next_sample;
                        frac <= sin_frac;
                        
                        interp_start <= 1;
                        state <= `LUT_MASTER_STATE_WAIT_INTERP1;
                    end
                end

                `LUT_MASTER_STATE_TANH_WAIT: begin
                    tanh_read <= 0;
                    if (!wait_one && tanh_ready) begin
                        base_sample <= tanh_base_sample;
                        next_sample <= tanh_next_sample;
                        frac <= tanh_frac;
                        
                        interp_start <= 1;
                        state <= `LUT_MASTER_STATE_WAIT_INTERP1;
                    end
                end

				`LUT_MASTER_STATE_WAIT_INTERP1: begin
					interp_start <= 0;
					state <= `LUT_MASTER_STATE_WAIT_INTERP2;
				end
				
				`LUT_MASTER_STATE_WAIT_INTERP2: begin
					if (interp_ready) begin
						data_out <= interpolated;
						ready <= 1;
						state <= `LUT_MASTER_STATE_READY;
					end
				end
				
				`LUT_MASTER_STATE_SEND: begin
					data_out <= interpolated;
					ready <= 1;
					state <= `LUT_MASTER_STATE_READY;
				end
				
				`LUT_MASTER_STATE_ERROR: begin
				
				end
			endcase
		end	
	end
	
	sin_2pi_lut_16 sin_lut (
        .clk(clk),
        .reset(reset),

        .x(req_arg_latched),
        .base_sample(sin_base_sample),
        .next_sample(sin_next_sample),
        .frac(sin_frac),

        .read(sin_read),
        .ready(sin_ready)
    );
	
	tanh_4_lut_16 tanh_lut (
        .clk(clk),
        .reset(reset),

        .x(req_arg_latched),
        .base_sample(tanh_base_sample),
        .next_sample(tanh_next_sample),
        .frac(tanh_frac),
    
        .read(tanh_read),
        .ready(tanh_ready)
    );
	
endmodule

module combinatorial_interp
	#(
		parameter data_width = 16,
		parameter interp_bits = 4
	)
	(
		input  wire signed [data_width-1:0] base,
		input  wire signed [data_width-1:0] target,
		input  wire [interp_bits-1:0]        frac,
		output wire signed [data_width-1:0]  interpolated
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
