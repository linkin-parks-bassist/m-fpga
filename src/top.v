module top
	#(
		parameter n_blocks 			= 255,
		parameter n_block_registers = 2,
		parameter n_channels 		= 16,
		parameter data_width 		= 16,
		parameter n_sram_banks 		= 8,
		parameter sram_bank_size 	= 1024,
		parameter spi_fifo_length	= 512
	)
    (
		`ifndef verilator
        input wire crystal,
        `else
        input wire sys_clk,
        `endif

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
    
    wire pll_lock;
    
    `ifndef verilator
    wire sys_clk;
    reg mclk;
    `else
    reg mclk = 0;
    assign pll_lock = 1;
    `endif

    reg reset = 1;
    
    `ifndef verilator
    // Your existing PLL
    Gowin_rPLL pll(
        .clkout(sys_clk),
        //.clkoutd(mclk),
        .clkin(crystal),
        .lock(pll_lock)
    );
    `else
    
    `endif

    reg [2:0] mclk_ctr = 0;
    
    always @(posedge sys_clk) begin
		if (mclk_ctr == 4) begin
			mclk <= ~mclk;
			mclk_ctr <= 0;

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
		end else begin
			mclk_ctr <= mclk_ctr + 1;
		end    
    end

    assign codec_en = pll_lock;
    assign mclk_out = mclk;
    
    // Internal registers for clock dividers
    reg [3:0] bclk_counter = 4'd0;  // Divide by 4 from 11.25M to get ~2.8MHz (44.1kHz * 32)
    reg bclk = 1'b0;
    
    reg [5:0] lrclk_counter = 5'd0; // Divide by 64 from 2.8MHz to get ~44.1kHz
    assign lrclk = lrclk_counter[5];

    // BCLK: mclk / 4  -> 2.8224 MHz
    /*always @(posedge mclk) begin
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
    end*/

    
    // Assign outputs
    assign bclk_out  = bclk;
    assign lrclk_out = lrclk;
    
    reg [31:0] ctr = 0;
    reg led_reg = 0;

    wire [7:0] spi_in;

    reg [4:0] spi_byte_ctr = 0;


    reg reg_write_blinker = 0;
    reg [31:0] reg_write_blink_ctr = 0;
    reg instr_write_blinker = 0;
    reg [31:0] instr_write_blink_ctr = 0;
    reg spi_valid_blinker = 0;
    reg [31:0] spi_valid_blink_ctr = 0;

    always  @(posedge sys_clk) begin
		tick_engine <= 0;
    
        if (spi_valid_blinker) begin
            if (|spi_valid_blink_ctr) begin
                spi_valid_blink_ctr <= spi_valid_blink_ctr - 1;
            end else begin
                spi_valid_blinker <= 0;
            end
        end

        if (reg_write_blinker) begin
            if (|reg_write_blink_ctr) begin
                reg_write_blink_ctr <= reg_write_blink_ctr - 1;
            end else begin
                reg_write_blinker <= 0;
            end
        end

        if (reg_write_ack) begin
            if (!reg_write_blinker) begin
                reg_write_blinker <= 1;
                reg_write_blink_ctr <= 50500000;
            end
        end

        if (instr_write_blinker) begin
            if (|instr_write_blink_ctr) begin
                instr_write_blink_ctr <= instr_write_blink_ctr - 1;
            end else begin
                instr_write_blinker <= 0;
            end
        end

        if (instr_write_ack) begin
            if (!instr_write_blinker) begin
                instr_write_blinker <= 1;
                instr_write_blink_ctr <= 50500000;
            end
        end

        if (spi_in_valid) begin
            spi_capture <= spi_in;
            if (!spi_valid_blinker) begin
                spi_valid_blinker <= 1;
                spi_valid_blink_ctr <= 50500000;
            end
        end

        if (ctr == 56250000) begin
            led_reg <= ~led_reg;
            ctr <= 0;

            spi_byte_ctr <= spi_byte_ctr + 1;
        end else  begin
            ctr <= ctr + 1;
        end
        
        if (sample_valid) begin
			if (!sample_ack) begin
				tick_engine <= 1;
				sample_ack <= 1;
			end
        end else begin
			sample_ack <= 0;
		end
    end

    reg [7:0] spi_capture;

    // LED assignments (example - you can modify these)
    /*assign led0 = ~(~cs);
    assign led1 = ~0;
    assign led3 = ~(|sample_in_abs[15:12]);
    assign led4 = ~(|sample_out_abs[15:12]);*/

    assign led0 = ~(|out);
    assign led1 = ~(engine_ready);
    assign led3 = ~(|control_state[3:0]);
    assign led4 = ~(|control_state[7:4]);

    wire [7:0] control_state;

    localparam sample_size = 16;

    wire [sample_size-1:0] sample_out;
    wire [sample_size-1:0] sample_in;

    wire [sample_size-1:0] sample_in_abs = sample_in[15] ? -sample_in : sample_in;
    wire [sample_size-1:0] sample_out_abs = sample_out[15] ? -sample_out : sample_out;

    wire sample_valid;

    wire invalid_command;
    
    reg tick_engine = 0;
    reg sample_ack = 0;

    wire reg_write_ack;
    wire instr_write_ack;

    wire current_pipeline;

    wire [7:0] out;

    i2s_trx #(.sample_size(sample_size)) i2s_driver
    (
        .sys_clk(sys_clk), .bclk(bclk), .lrclk(lrclk), .din(i2s_din), .dout(i2s_dout),
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
            .clk(sys_clk),
            .reset(reset),

            .in_sample(sample_in),
            .out_sample(sample_out),
        
            .sample_ready(tick_engine),
        
            .command_in(spi_in),
            .command_in_ready(spi_in_valid),
            .invalid_command(invalid_command),

            .ready(engine_ready),
        
            .fifo_count(fifo_count),

            .current_pipeline(current_pipeline),

            .out(out)
        );

    sync_spi_slave spi
        (
            .clk(sys_clk),
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
