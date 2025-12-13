module i2s_clock_gen #(parameter data_width = 16, parameter mclk_div = 5, parameter bclk_div = 2) (
		input wire clk,
		input wire reset,
		
		output reg mclk,
		output reg bclk,
		output reg lrclk
	);

	localparam  mclk_count_width = $clog2(mclk_div);
	localparam  bclk_count_width = $clog2(bclk_div);
	localparam lrclk_count_width = $clog2(4 * data_width);

	reg [ mclk_count_width - 1 : 0]  mclk_ctr = 0;
	reg [ bclk_count_width - 1 : 0]  bclk_ctr = 0;
	reg [lrclk_count_width - 1 : 0] lrclk_ctr = 0;
	
	localparam [lrclk_count_width - 1 : 0] lrclk_count_max  = 4 * data_width - 1;
	localparam [lrclk_count_width - 1 : 0] lrclk_count_half = 2 * data_width - 1;
	
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			mclk_ctr  <= 0;
			bclk_ctr  <= 0;
			lrclk_ctr <= 0;
			
			mclk  <= 0;
			bclk  <= 0;
			lrclk <= 0;
		end
		else begin
			if (mclk_ctr == mclk_div - 1) begin
				mclk <= ~mclk;
				mclk_ctr <= 0;
				
				if (bclk_ctr == bclk_div - 1) begin
					bclk <= ~bclk;
					bclk_ctr <= 0;
					
					if (lrclk_ctr == lrclk_count_max) begin
						lrclk <= 0;
						lrclk_ctr <= 0;
					end
					else begin
						if (lrclk_ctr == lrclk_count_half)
							lrclk <= 1;
						
						lrclk_ctr <= lrclk_ctr + 1;
					end
				end
				else begin
					bclk_ctr <= bclk_ctr + 1;
				end
			end
			else begin
				mclk_ctr <= mclk_ctr + 1;
			end
		end
	end
endmodule
	

// Generates LRCLK (word select) from BCLK.
// 64 BCLK edges per LRCLK full period -> 32 BCLK per channel.
// I2S timing: LRCLK edge marks start of half-frame, data MSB appears 1 BCLK later.
module i2s_lrclk_gen #(parameter data_width = 16) (
    input  wire bclk,   // bit clock driving I2S
    input  wire rst,    // synchronous reset (active high, in bclk domain)
    output reg  lrclk   // word-select: 0 = left, 1 = right (convention)
);

	localparam count_width = $clog2(4 * data_width);
	localparam [count_width - 1 : 0] count_max  = 4 * data_width - 1;
	localparam [count_width - 1 : 0] count_half = 2 * data_width - 1;

    reg [count_width - 1 : 0] count = 0;  // counts 0..63

    always @(posedge bclk or posedge rst) begin
        if (rst) begin
            count   <= 0;
            lrclk 	<= 0;  // start on left channel
        end else begin
            if (count == count_max) begin
				lrclk <= 0;
                count <= 0;
            end
            else begin
				if (count == count_half)
					lrclk <= 1;
				
                count <= count + 1;
			end
        end
    end

endmodule

// I2S receiver (mono) for INMP441-style data.
// Captures 24 bits from either left or right channel.
// Assumes: 64 BCLK per frame, 32 per channel, I2S one-bit delay from LRCLK edge.
//
// CAPTURE_CHANNEL = 0 -> capture when lrclk == 0 (left)
// CAPTURE_CHANNEL = 1 -> capture when lrclk == 1 (right)
module i2s_rx_mono #(
    parameter integer WORD_WIDTH      = 24,
    parameter         CAPTURE_CHANNEL = 1'b0,
    parameter integer BIT_OFFSET      = 1    // first valid data bit index
)(
    input  wire bclk,
    input  wire rst,
    input  wire lrclk,
    input  wire sdata,
    output reg  signed [WORD_WIDTH-1:0] sample,
    output reg                   sample_valid
);

    // Derived constants (compile-time)
    localparam integer BIT_LAST = BIT_OFFSET + WORD_WIDTH - 1;

    reg lrclk_d = 1'b0;
    reg [5:0] bit_cnt = 6'd0;
    reg signed [WORD_WIDTH-1:0] shift_reg = {WORD_WIDTH{1'b0}};

    always @(posedge bclk or posedge rst) begin
        if (rst) begin
            lrclk_d      <= 1'b0;
            bit_cnt      <= 6'd0;
            shift_reg    <= {WORD_WIDTH{1'b0}};
            sample       <= {WORD_WIDTH{1'b0}};
            sample_valid <= 1'b0;
        end else begin
            lrclk_d      <= lrclk;
            sample_valid <= 1'b0;

            // Half-frame counter: 0..31
            if (lrclk_d != lrclk) begin
                bit_cnt <= 6'd0;
            end else if (bit_cnt < 6'd31) begin
                bit_cnt <= bit_cnt + 6'd1;
            end

            // Capture window: BIT_OFFSET .. BIT_LAST
            if (lrclk == CAPTURE_CHANNEL) begin
                if (bit_cnt >= BIT_OFFSET &&
                    bit_cnt <= BIT_LAST) begin

                    shift_reg <= {shift_reg[WORD_WIDTH-2:0], sdata};

                    if (bit_cnt == BIT_LAST) begin
                        sample       <= {shift_reg[WORD_WIDTH-2:0], sdata};
                        sample_valid <= 1'b1;
                    end
                end
            end
        end
    end

endmodule

// INMP441 I2S RX, mono, literal to datasheet timing.
//
// Assumptions:
// - BCLK (SCK) is the same clock the INMP441 sees on its SCK pin.
// - WS (lrclk) is the same as the INMP441 WS pin.
// - L/R pin on the mic is strapped:
//     L/R = 0 -> mic drives LEFT half-frame  (WS = "left")
//     L/R = 1 -> mic drives RIGHT half-frame (WS = "right")
// - We treat WS = 0 as "left" and WS = 1 as "right" (standard I2S).
// - We reset bit_cnt at each WS transition and then count SCK edges.
//
// According to the datasheet, for the active channel:
//   - SD is high-Z at SCK count 0 (just after WS edge).
//   - MSB is valid starting at SCK count 1.
//   - 24 bits total: counts 1..24.
//   - After that, SD goes high-Z again.

module inmp441_rx_mono #(
    parameter integer WORD_WIDTH      = 24,
    parameter         CAPTURE_CHANNEL = 1'b0  // 0 = capture when WS==0, 1 = WS==1
)(
    input  wire                        bclk,         // SCK
    input  wire                        rst,          // async reset, active high
    input  wire                        lrclk,        // WS
    input  wire                        sdata,        // SD from INMP441
    output reg  signed [WORD_WIDTH-1:0] sample,      // captured 24-bit sample
    output reg                         sample_valid  // 1-cycle pulse (bclk domain)
);

    reg       lrclk_d   = 1'b0;     // delayed WS for edge detect
    reg [5:0] bit_cnt   = 6'd0;     // 0..31 within half-frame
    reg signed [WORD_WIDTH-1:0] shift_reg = {WORD_WIDTH{1'b0}};

    always @(negedge bclk or posedge rst) begin
        if (rst) begin
            lrclk_d      <= 1'b0;
            bit_cnt      <= 6'd0;
            shift_reg    <= {WORD_WIDTH{1'b0}};
            sample       <= {WORD_WIDTH{1'b0}};
            sample_valid <= 1'b0;
        end else begin
            lrclk_d      <= lrclk;
            sample_valid <= 1'b0;

            // New half-frame starts at any WS edge
            if (lrclk_d != lrclk) begin
                bit_cnt   <= 6'd0;
                shift_reg <= {WORD_WIDTH{1'b0}};
            end else begin
                if (bit_cnt < 6'd31)
                    bit_cnt <= bit_cnt + 6'd1;
            end

            // Capture only on the chosen channel half
            if (lrclk == CAPTURE_CHANNEL) begin
                // Datasheet: MSB is valid at the first SCK after WS edge.
                // With bit_cnt reset at WS edge and incremented on each BCLK,
                // MSB is at bit_cnt == 1, LSB at bit_cnt == 24.
                if (bit_cnt >= 6'd1 && bit_cnt <= 6'd24) begin
                    shift_reg <= {shift_reg[WORD_WIDTH-2:0], sdata};

                    if (bit_cnt == 6'd24) begin
                        sample       <= {shift_reg[WORD_WIDTH-2:0], sdata};
                        sample_valid <= 1'b1;
                    end
                end
            end
        end
    end

endmodule



// I2S transmitter (mono -> stereo).
// Sends the same 24-bit sample on left and right channels.
// Assumes 64 BCLK per frame, 32 per channel, I2S one-bit delay.
//
// 'sample_in' is held stable in the BCLK domain.
// Typically you update 'sample_in' whenever you see sample_valid from i2s_rx_mono.
module i2s_tx_mono_stereo #(
    parameter integer WORD_WIDTH = 24
)(
    input  wire bclk,                 // bit clock
    input  wire rst,                  // reset in bclk domain (active high)
    input  wire lrclk,                // word-select (from i2s_lrclk_gen)
    input  wire signed [WORD_WIDTH-1:0] sample_in,  // mono sample to output
    output reg  sdata                 // serial data to PCM5102 (DIN)
);

    reg lrclk_d = 1'b0;
    reg [5:0] bit_cnt = 6'd0;         // 0..31 within each half-frame
    reg signed [WORD_WIDTH-1:0] shift_reg = {WORD_WIDTH{1'b0}};
    reg signed [WORD_WIDTH-1:0] latched_sample = {WORD_WIDTH{1'b0}};

    always @(posedge bclk or posedge rst) begin
        if (rst) begin
            lrclk_d        <= 1'b0;
            bit_cnt        <= 6'd0;
            shift_reg      <= {WORD_WIDTH{1'b0}};
            latched_sample <= {WORD_WIDTH{1'b0}};
            sdata          <= 1'b0;
        end else begin
            lrclk_d <= lrclk;

            // Save a copy of the input sample continuously.
            // You can also choose to latch only on sample_valid externally.
            latched_sample <= sample_in;

            // Detect LRCLK edge = start of new half-frame
            if (lrclk_d != lrclk) begin
                bit_cnt   <= 6'd0;
                shift_reg <= latched_sample;  // load new word at start of half-frame
                sdata     <= 1'b0;            // I2S: one-bit delay (no MSB yet)
            end else begin
                if (bit_cnt < 6'd31)
                    bit_cnt <= bit_cnt + 6'd1;

                // I2S: output MSB starting at bit_cnt == 1, for WORD_WIDTH bits.
                if (bit_cnt >= 6'd1 && bit_cnt <= WORD_WIDTH[5:0]) begin
                    sdata     <= shift_reg[WORD_WIDTH-1];
                    shift_reg <= {shift_reg[WORD_WIDTH-2:0], 1'b0};
                end else begin
                    // After WORD_WIDTH bits, pad with zeros until end of half-frame.
                    sdata <= 1'b0;
                end
            end
        end
    end

endmodule
