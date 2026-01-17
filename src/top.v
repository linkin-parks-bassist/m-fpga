module top
	#(
		parameter n_blocks 			= 3,
		parameter n_block_registers = 4,
		parameter n_channels 		= 4,
		parameter data_width 		= 16,
		parameter n_sram_banks 		= 8,
		parameter sram_bank_size 	= 1024,
		parameter spi_fifo_length	= 32
	)
    (
        input wire crystal,

        input  wire cs,
        input  wire mosi,
        output wire miso,
        input  wire sck,

        output wire led0,
        output wire led1,
        output wire led2,
        output wire led3,
        output wire led4,
        output wire led5,

        output wire mclk_out,
        output wire bclk_out,
        output wire lrclk_out,
        
        input  wire i2s_din,
        output wire i2s_dout,

        output wire codec_en
    );
    
    wire clk_112m;
    wire clk_11m;
    wire pll_lock;

    reg reset = 1;
    
    // Your existing PLL
    Gowin_rPLL pll(
        .clkout(clk_112m),
        .clkoutd(clk_11m),
        .clkin(crystal),
        .lock(pll_lock)
    );

    assign codec_en = pll_lock;
    assign mclk_out = clk_11m;
    
    // Internal registers for clock dividers
    reg [3:0] bclk_counter = 4'd0;  // Divide by 4 from 11.25M to get ~2.8MHz (44.1kHz * 32)
    reg bclk = 1'b0;
    
    reg [5:0] lrclk_counter = 5'd0; // Divide by 64 from 2.8MHz to get 44.1kHz
    assign lrclk = lrclk_counter[5];
    
    // BCLK: clk_11m / 4  -> 2.8224 MHz
    always @(posedge clk_11m) begin
        if (pll_lock) begin
            reset <= 0;
            bclk_counter <= bclk_counter + 1'b1;
            if (bclk_counter == 1) begin
                bclk <= ~bclk;
                bclk_counter <= 0;

                if (bclk)
                    lrclk_counter <= lrclk_counter + 1;
            end
        end
    end

    
    // Assign outputs
    assign bclk_out = bclk;
    assign lrclk_out = lrclk;
    
    reg [31:0] ctr = 0;
    reg led_reg = 0;

    wire [7:0] spi_in;;

    reg [4:0] spi_byte_ctr = 0;

    always  @(posedge clk_112m) begin
        if (spi_in_valid)
            spi_capture <= spi_in;

        if (ctr == 56250000) begin
            led_reg <= ~led_reg;
            ctr <= 0;

            spi_byte_ctr <= spi_byte_ctr + 1;
        end else  begin
            ctr <= ctr + 1;
        end
    end

    reg [7:0] spi_capture;

    // LED assignments (example - you can modify these)
    assign led0 = ~spi_capture[0];
    assign led1 = ~spi_capture[1];
    assign led3 = ~spi_capture[2];
    assign led4 = ~spi_capture[3];
    assign led5 = ~0;

    localparam sample_size = 16;

    wire [sample_size-1:0] sample_out;
    wire [sample_size-1:0] sample_in;

    wire [sample_size-1:0] sample_in_abs = sample_in[15] ? -sample_in : sample_in;

    wire sample_valid;

    wire invalid_command;

    i2s_trx #(.sample_size(sample_size)) i2s_driver
    (
        .bclk(bclk), .lrclk(lrclk), .din(i2s_din), .dout(i2s_dout),
        .enable(1'b1), .reset(reset), .rx_valid(sample_valid),
        .tx_l(sample_out), .tx_r(sample_out),
        .rx_l(sample_in), .rx_r()
    );

    dsp_engine_seq #(
            .n_blocks(n_blocks), 
            .n_block_registers(n_block_registers),
            .data_width(data_width),
            .n_channels(n_channels),
            .n_sram_banks(n_sram_banks),
            .sram_bank_size(sram_bank_size),
            .spi_fifo_length(spi_fifo_length)
        ) engine (
            .clk(clk_112m),
            .reset(reset),

            .in_sample(sample_in),
            .out_sample(sample_out),
        
            .sample_ready(sample_valid),
        
            .command_in(spi_in),
            .command_in_ready(spi_in_valid),
            .invalid_command(invalid_command),
        
            .ready(engine_ready),
        
            .fifo_count(fifo_count)
        );

    sync_spi_slave spi
        (
            .clk(clk_112m),
            .reset(reset),

            .sck(sck),
            .cs(cs),
            .mosi(mosi),
            .miso(miso),
            .miso_byte(data_out),

            .enable(1),

            .mosi_byte(spi_in),
            .data_ready(spi_in_valid)
        );

endmodule
