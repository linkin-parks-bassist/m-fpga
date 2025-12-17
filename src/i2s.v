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
    parameter integer data_width      = 24,
    parameter         CAPTURE_CHANNEL = 1'b0,
    parameter integer BIT_OFFSET      = 1    // first valid data bit index
)(
    input  wire bclk,
    input  wire rst,
    input  wire lrclk,
    input  wire sdata,
    output reg  signed [data_width-1:0] sample,
    output reg                   sample_valid
);

    // Derived constants (compile-time)
    localparam integer BIT_LAST = BIT_OFFSET + data_width - 1;

    reg lrclk_d = 1'b0;
    reg [5:0] bit_cnt = 6'd0;
    reg signed [data_width-1:0] shift_reg = {data_width{1'b0}};

    always @(posedge bclk or posedge rst) begin
        if (rst) begin
            lrclk_d      <= 1'b0;
            bit_cnt      <= 6'd0;
            shift_reg    <= {data_width{1'b0}};
            sample       <= {data_width{1'b0}};
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

                    shift_reg <= {shift_reg[data_width-2:0], sdata};

                    if (bit_cnt == BIT_LAST) begin
                        sample       <= {shift_reg[data_width-2:0], sdata};
                        sample_valid <= 1'b1;
                    end
                end
            end
        end
    end

endmodule

module i2s_24bit_tx (
    input  wire        clk_122m88,
    input  wire        reset,

    input  wire [23:0] sample_l,
    input  wire [23:0] sample_r,

    output reg         bclk,
    output reg         lrclk,
    output reg         dout
);

    // ------------------------------------------------------------
    // BCLK generation: 122.88 MHz / 40 = 3.072 MHz
    // ------------------------------------------------------------
    reg [4:0] bclk_div = 0;

    always @(posedge clk_122m88) begin
        if (reset) begin
            bclk_div <= 0;
            bclk     <= 0;
        end else begin
            if (bclk_div == 19) begin
                bclk_div <= 0;
                bclk     <= ~bclk;
            end else begin
                bclk_div <= bclk_div + 1;
            end
        end
    end

    // ------------------------------------------------------------
    // I²S state (driven ONLY by BCLK)
    // ------------------------------------------------------------
    reg [5:0]  bit_cnt = 0;     // 0..63
    reg [31:0] shift_reg = 0;

    always @(negedge bclk) begin
        if (reset) begin
            bit_cnt   <= 0;
            lrclk     <= 0;
            shift_reg <= 0;
            dout      <= 0;
        end else begin
            // Advance bit counter
            bit_cnt <= bit_cnt + 1;

            // LRCLK toggles one BCLK BEFORE new word
            if (bit_cnt == 31)
                lrclk <= 1;   // switch to right
            else if (bit_cnt == 63)
                lrclk <= 0;   // switch to left

            // Load new word one cycle AFTER LRCLK edge
            if (bit_cnt == 32) begin
                shift_reg <= {sample_r, 8'b0};
            end else if (bit_cnt == 0) begin
                shift_reg <= {sample_l, 8'b0};
            end else begin
                shift_reg <= {shift_reg[30:0], 1'b0};
            end

            // Drive data (MSB first)
            dout <= shift_reg[31];
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
    parameter integer data_width = 24
)(
    input  wire bclk,                 // bit clock
    input  wire rst,                  // reset in bclk domain (active high)
    input  wire lrclk,                // word-select (from i2s_lrclk_gen)
    input  wire signed [data_width-1:0] sample_in,  // mono sample to output
    output reg  sdata                 // serial data to PCM5102 (DIN)
);

    reg lrclk_d = 1'b0;
    reg [5:0] bit_cnt = 6'd0;         // 0..31 within each half-frame
    reg signed [data_width-1:0] shift_reg = {data_width{1'b0}};
    reg signed [data_width-1:0] latched_sample = {data_width{1'b0}};

    always @(posedge bclk or posedge rst) begin
        if (rst) begin
            lrclk_d        <= 1'b0;
            bit_cnt        <= 6'd0;
            shift_reg      <= {data_width{1'b0}};
            latched_sample <= {data_width{1'b0}};
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

                // I2S: output MSB starting at bit_cnt == 1, for data_width bits.
                if (bit_cnt >= 6'd1 && bit_cnt <= data_width[5:0]) begin
                    sdata     <= shift_reg[data_width-1];
                    shift_reg <= {shift_reg[data_width-2:0], 1'b0};
                end else begin
                    // After data_width bits, pad with zeros until end of half-frame.
                    sdata <= 1'b0;
                end
            end
        end
    end

endmodule
