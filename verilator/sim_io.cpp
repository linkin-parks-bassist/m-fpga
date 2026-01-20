#include <verilated.h>
#include "Vtop.h"
#include "verilated_fst_c.h"
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>
#include "math.h"

#include "sim_main.h"

sim_io_state io;

int spi_enqueue(sim_io_state *io, uint8_t byte)
{
	if (!io) return 2;
	
	if (((io->spi_write_head + 1) % SPI_SEND_QUEUE_DEPTH) == io->spi_read_head)
	{
		printf("ERROR: spi queue full!\n");
		return 1;
	}
	
	io->spi_send_queue[io->spi_write_head] = byte;
	
	io->spi_write_head = (io->spi_write_head + 1) % SPI_SEND_QUEUE_DEPTH;
	
	return 0;
}

int spi_waiting(sim_io_state *io)
{
	if (!io)
		return 0;
	
	return (io->spi_read_head != io->spi_write_head);
}

uint8_t spi_get(sim_io_state *io)
{
	if (!io)
		return 0;
	
	uint8_t byte = io->spi_send_queue[io->spi_read_head];
	
	io->spi_read_head = (io->spi_read_head + 1) % SPI_SEND_QUEUE_DEPTH;
	
	return byte;
}

void sim_io_init(sim_io_state *io)
{
	io->spi_read_head  = 0;
	io->spi_write_head = 0;
	
	io->sample_bit_ctr = 0;
	
	io->sample_out_sr = 0;
	
	io->cs  = 1;
	io->sck = 1;
	
	io->spi_sending 	= 0;
	io->sys_clk_prev 	= 0;
	io->mclk_prev 		= 0;
	io->bclk_prev 		= 0;
	io->lrclk_prev 		= 0;
	
	io->i2s_ready = 1;
	io->i2s_skip = 1;
}

int sim_io_update(sim_io_state *io)
{
	if (!dut || !io)
		return 1;
	
	int mclk_edge  = dut->mclk_out  - io->mclk_prev;
	int bclk_edge  = dut->bclk_out  - io->bclk_prev;
	int lrclk_edge = dut->lrclk_out - io->lrclk_prev;
	
	if (dut->sys_clk)
	{
		if (io->spi_sending)
		{
			if (io->sck_counter == (SCK_RATE - 1) / 2)
			{
				if (!io->sck)
				{
					io->mosi = !!(io->spi_byte & (1 << (7 - io->spi_bit)));
					
					
					io->spi_bit++;
				}
			}
			else if (io->sck_counter == SCK_RATE - 1)
			{
				io->sck = !io->sck;
				
				if (!io->sck)
				{
					if (io->spi_bit == 8)
					{
						io->spi_sending = 0;
						io->spi_bit = 0;
						io->cs = 1;
						io->sck = 1;
					}
					
				}
			}
		}
		else
		{
			if (io->sck_counter == SCK_RATE - 1)
			{
				if (dut->cs && spi_waiting(io))
				{
					io->spi_sending = 1;
					io->cs  = 0;
					io->sck = 0;
					
					io->spi_byte = spi_get(io);
				}
			}
		}
		
		io->sck_counter = (io->sck_counter + 1) % SCK_RATE;
	}
	
	if (bclk_edge == 1)
	{
		if (io->i2s_skip)
		{io->
			i2s_skip = 0;
		}
		else if (io->i2s_bit < 16)
		{
			io->sample_out_sr = (io->sample_out_sr << 1) + !!dut->i2s_dout;
			io->i2s_bit++;
		}
	}
	else if (bclk_edge == -1)
	{
		if (lrclk_edge == 1)
		{
			io->i2s_ready = 1;
			io->sample_out = (int16_t)io->sample_out_sr;
			io->i2s_bit = 0;
			io->i2s_skip = 1;
		}
		else 
		{
			if (io->i2s_bit < 16)
			{
				io->i2s_din = !!(io->sample_in & (1 << (15 - io->i2s_bit)));
			}
			else
			{
				io->i2s_din = 0;
			}
		}
	}
	
	dut->cs 	= io->cs;
	dut->sck	= io->sck;
	dut->mosi 	= io->mosi;
	
	dut->i2s_din = io->i2s_din;
	
	io->sys_clk_prev 	= dut->sys_clk;
	io->bclk_prev 		= dut->bclk_out;
	io->lrclk_prev 		= dut->lrclk_out;
	
	return 0;
}


int spi_send(uint8_t byte)
{
	return spi_enqueue(&io, byte);
}
