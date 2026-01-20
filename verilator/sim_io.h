#ifndef DSP_SIM_IO_H_
#define DSP_SIM_IO_H_

#define SPI_SEND_QUEUE_DEPTH 	1024
#define SCK_RATE				10

typedef struct {
	int sys_clk_prev;

	int cs;
	int mosi;
	int sck;
	
	int sck_counter;
	int spi_bit;
	uint8_t spi_byte;
	
	int i2s_din;
	int i2s_dou;
	
	int mclk_prev;
	int bclk_prev;
	int lrclk_prev;
	int i2s_skip;
	
	int16_t sample_in;
	int16_t sample_out;
	
	uint16_t sample_out_sr;
	
	int i2s_ready;
	int i2s_bit;
	
	int sample_bit_ctr;
	
	uint8_t spi_send_queue[SPI_SEND_QUEUE_DEPTH];
	int spi_read_head;
	int spi_write_head;
	
	int spi_sending;
	
	int spi_cooldown;
} sim_io_state;

void sim_io_init(sim_io_state *io);

int spi_enqueue(sim_io_state *io, uint8_t byte);

int io_update(sim_io_state *io);

int sim_io_update(sim_io_state *io);

int spi_send(uint8_t byte);

extern sim_io_state io;

#endif
