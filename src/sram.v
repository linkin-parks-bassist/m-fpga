module sram_bank
#(
    parameter data_width  = 16,
    parameter size        = 1024,
    parameter addr_width  = $clog2(size)
)
(
    input  wire clk,
    input  wire reset,

    input  wire read,
    input  wire write,

    input  wire [addr_width-1:0] write_addr,
    input  wire [addr_width-1:0] read_addr,
    input  wire [data_width-1:0] data_in,
    output reg  [data_width-1:0] data_out,

    output reg invalid_read,
    output reg invalid_write
);
    reg [data_width-1:0] mem [0:size-1];

    always @(posedge clk) begin
        invalid_read  <= 0;
        invalid_write <= 0;

        data_out <= mem[read_addr];

        if (write)
            mem[write_addr] <= data_in;

        if (read && read_addr >= size)
            invalid_read <= 1;

        if (write && write_addr >= size)
            invalid_write <= 1;
    end
endmodule



module contiguous_sram
#(
    parameter data_width = 16,
    parameter bank_size  = 1024,         // must be power of 2
    parameter n_banks    = 8,
    parameter addr_width = $clog2(bank_size * n_banks)
)
(
    input wire clk,
    input wire reset,

    input wire read,
    input wire write,

    input  wire [addr_width-1:0] write_addr,
    input  wire [addr_width-1:0] read_addr,
    input  wire [data_width-1:0] data_in,
    output reg  [data_width-1:0] data_out,

    output reg  read_ready,
    output reg  write_ready,

    output reg  invalid_read,
    output reg  invalid_write
);

    localparam BANK_ADDR_W = $clog2(n_banks);
    localparam BANK_OFF_W  = $clog2(bank_size);

    wire [BANK_ADDR_W-1:0] bank_r = read_addr [BANK_OFF_W+BANK_ADDR_W-1 : BANK_OFF_W];
    wire [BANK_OFF_W-1:0]  off_r  = read_addr [BANK_OFF_W-1:0];

    wire [BANK_ADDR_W-1:0] bank_w = write_addr[BANK_OFF_W+BANK_ADDR_W-1 : BANK_OFF_W];
    wire [BANK_OFF_W-1:0]  off_w  = write_addr[BANK_OFF_W-1:0];

    wire out_of_range_r = read_addr  >= n_banks*bank_size;
    wire out_of_range_w = write_addr >= n_banks*bank_size;

    reg [data_width-1:0] data_in_latched;

    // synchronous read outputs of every bank
    wire [data_width-1:0] bank_out [0:n_banks-1];

    // Instantiate banks — BRAM will infer cleanly here
    genvar i;
    generate
        for (i = 0; i < n_banks; i = i+1) begin : BANKS
            sram_bank #(
                .data_width(data_width),
                .size(bank_size),
                .addr_width(BANK_OFF_W)
            ) bank_inst (
                .clk(clk),
                .reset(reset),

                .read(1'b1),              // ALWAYS read — BRAM requirement
                .write(write && (bank_w == i)),

                .write_addr(off_w),
                .read_addr(off_r),

                .data_in(data_in_latched),
                .data_out(bank_out[i]),

                .invalid_read(),          // handled at top-level
                .invalid_write()
            );
        end
    endgenerate


	reg read_wait  = 0;
	reg write_wait = 0;
    always @(posedge clk) begin
		read_wait <= 0;
		write_wait <= 0;
		
        if (reset) begin
            read_ready   <= 1;
            write_ready  <= 1;
            invalid_read <= 0;
            invalid_write<= 0;
            data_out     <= 0;
            data_in_latched <= 0;
        end
        else begin
            invalid_read  <= 0;
            invalid_write <= 0;

            if (read && read_ready && !read_wait) begin
                if (out_of_range_r) begin
                    invalid_read <= 1;
                end
                read_ready <= 0;
            end
            else if (!read_ready) begin
                data_out   <= bank_out[bank_r];
                read_ready <= 1;
				read_wait <= 1;
            end

            if (write && write_ready && !write_wait) begin
                if (out_of_range_w)
                    invalid_write <= 1;

                data_in_latched <= data_in;
                write_ready     <= 0;
            end
            else if (!write_ready) begin
                write_ready <= 1;
                write_wait <= 1;
            end
        end
    end
endmodule
