#include <time.h>
#include "sim_main.h"

int samples_processed = 0;

Vtop* dut = new Vtop;

#pragma pack(push, 1)
struct WavHeader {
    char     riff[4];
    uint32_t chunk_size;
    char     wave[4];
    char     fmt[4];
    uint32_t subchunk1_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data[4];
    uint32_t data_size;
};
#pragma pack(pop)

static bool read_wav16_mono(const char* path,
                            WavHeader& header,
                            std::vector<int16_t>& samples)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    f.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!f) return false;

    if (std::strncmp(header.riff, "RIFF", 4) != 0 ||
        std::strncmp(header.wave, "WAVE", 4) != 0 ||
        header.audio_format != 1 ||
        header.bits_per_sample != 16 ||
        header.num_channels != 1) {
        std::cerr << "Unsupported WAV format\n";
        return false;
    }

    size_t n = header.data_size / sizeof(int16_t);
    samples.resize(n);
    f.read(reinterpret_cast<char*>(samples.data()), header.data_size);
    return true;
}

static bool write_wav16_mono(const char* path,
                             uint32_t sample_rate,
                             const std::vector<int16_t>& samples)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    WavHeader h{};
    std::memcpy(h.riff, "RIFF", 4);
    std::memcpy(h.wave, "WAVE", 4);
    std::memcpy(h.fmt,  "fmt ", 4);
    std::memcpy(h.data, "data", 4);

    h.subchunk1_size = 16;
    h.audio_format   = 1;   // PCM
    h.num_channels   = 1;
    h.sample_rate    = sample_rate;
    h.bits_per_sample = 16;
    h.block_align     = 2;
    h.byte_rate       = sample_rate * 2;

    h.data_size  = samples.size() * 2;
    h.chunk_size = 36 + h.data_size;

    f.write(reinterpret_cast<const char*>(&h), sizeof(h));

    for (int16_t s : samples) {
        uint8_t lo = s & 0xFF;
        uint8_t hi = (s >> 8) & 0xFF;
        f.put(lo);
        f.put(hi);
    }

    return true;
}


VerilatedFstC* tfp = NULL;
static uint64_t ticks = 0;

void print_state()
{
	printf("\nSystem state: tick %d\n", (int)ticks);
	
	printf("dut->sys_clk   = %d\n", 	(int)dut->sys_clk);
	printf("dut->miso      = %d\n", 		(int)dut->miso);
	printf("dut->mosi      = %d\n", 		(int)dut->mosi);
	printf("dut->cs        = %d\n", 		(int)dut->cs);
	printf("dut->sck       = %d\n", 		(int)dut->sck);
	printf("\n");
	printf("dut->bclk      = %d\n", 		(int)dut->bclk_out);
	printf("dut->lrclk     = %d\n", 		(int)dut->lrclk_out);
	printf("dut->i2s_din   = %d\n", 		(int)dut->i2s_din);
	printf("dut->i2s_dout  = %d\n", 		(int)dut->i2s_dout);
	
	printf("\n");
	
	printf("io.sck_counter = %d\n", (int)io.sck_counter);
	printf("io.spi_bit     = %d\n", (int)io.spi_bit);
	printf("io.spi_byte    = 0x%04x\n", (int)io.spi_byte);
	printf("io.sample_in   = %d\n", (int)io.sample_in);
	printf("io.sample_out  = %d\n", (int)io.sample_out);
	printf("io.i2s_bit     = %d\n", (int)io.i2s_bit);
}

typedef struct sim_spi_send {
	m_fpga_transfer_batch batch;
	int tick;
	int started;
	int position;
	
	struct sim_spi_send *next;
} sim_spi_send;

sim_spi_send *send_queue = NULL;

int append_send_queue(m_fpga_transfer_batch batch, int when)
{	
	sim_spi_send *new_send = (sim_spi_send*)malloc(sizeof(sim_spi_send));
	
	if (!new_send) return 1;
	
	new_send->batch 	= batch;
	new_send->tick 		= when;
	new_send->started 	= 0;
	new_send->position 	= 0;
	new_send->next 		= NULL;
	
	if (!send_queue)
	{
		send_queue = new_send;
	}
	else
	{
		sim_spi_send *current = send_queue;
		
		while (current->next)
			current = current->next;
		
		current->next = new_send;	
	}
	
	return 0;
}

void pop_send_queue()
{
	if (!send_queue)
		return;
	
	sim_spi_send *ol = send_queue;
	
	send_queue = send_queue->next;
	
	if (ol->batch.buf)
		free(ol->batch.buf);
	free(ol);
}

int tick()
{
	if (!dut)
		return 1;
	
	dut->sys_clk = 1;
	sim_io_update(&io);
	dut->eval();
	#ifdef DUMP_WAVEFORM
	if (tfp) tfp->dump(ticks++);
	#endif
	
	#ifdef PRINT_STATE
	print_state();
	#endif
	
	dut->sys_clk = 0;
	sim_io_update(&io);
	dut->eval();
	#ifdef DUMP_WAVEFORM
	if (tfp) tfp->dump(ticks++);
	#endif
	
	#ifdef PRINT_STATE
	print_state();
	#endif
	
	return 0;
}

int pow2_ceil(int x)
{
	int y = 1;
	
	while (y && y <= x)
		y = y << 1;
	
	return y;
}

static inline int32_t arshift32(int32_t x, unsigned s)
{
    if (s == 0) return x;

    uint32_t ux = (uint32_t)x;
    uint32_t shifted = ux >> s;

    if (x < 0)
    {
        uint32_t mask = ~((uint32_t)0 >> s);
        shifted |= mask;
    }

    return (int32_t)shifted;
}

#define SAT_MIN -32768
#define SAT_MAX  32767

int16_t mul_wide(int16_t x, int16_t y, int no_shift, int shift, int sat)
{
	int actual_shift = no_shift ? 0 : (16 - 1 - shift);
	
	int32_t z = (int32_t)x * (int32_t)y;
	
	if (actual_shift)
		z = z / (1 << actual_shift);
	
	if (sat)
	{
		if (z > SAT_MAX)
			z = SAT_MAX;
		if (z < SAT_MIN)
			z = SAT_MIN;
	}
	
	return z;
}

int16_t mul(int16_t x, int16_t y, int no_shift, int shift, int sat)
{
	return (int16_t)mul_wide(x, y, no_shift, shift, sat);
}

int32_t sum_wide(int16_t x, int16_t y, int sat)
{
	int32_t z = x + y;
	
	if (sat)
	{
		if (z > SAT_MAX)
			z = SAT_MAX;
		if (z < SAT_MIN)
			z = SAT_MIN;
	}
	
	return z;
}

int16_t sum(int16_t x, int16_t y, int sat)
{
	return (int16_t)sum_wide(x, y, sat);
}

#define INTERP_BITS 8

int16_t linterp(int16_t x, int16_t y, int16_t frac)
{
	frac = (frac >> (16 - INTERP_BITS - 1)) & ((1 << INTERP_BITS) - 1);
	
	float ff = (float)frac / (float)(1 << INTERP_BITS);
	
	return (int16_t)(x + ff * (y-x));
}

typedef struct 
{
	int16_t *buffer;
	uint16_t position;
	uint16_t size;
	int16_t gain;
	int wrapped;
} sim_ddelay_buffer;

int init_sim_ddelay_buffer(sim_ddelay_buffer *buf)
{
	if (!buf)
		return 1;
	
	buf->buffer = NULL;
	buf->position = 0;
	buf->size = 0;
	buf->gain = 0;
	buf->wrapped = 0;
	
	return 0;
}

int alloc_sim_ddelay_buffer(sim_ddelay_buffer *buf, int size)
{
	if (!buf)
		return 1;
	
	buf->buffer = malloc(sizeof(int16_t) * size);
	buf->position = 0;
	buf->size = size;
	buf->gain = 0;
	buf->wrapped = 0;
	
	return 0;
}

int16_t sim_ddelay_read(sim_ddelay_buffer *buf, int16_t delay)
{
	if (!buf)
		return 0;
	
	int read_pos = (buf->position - delay) % buf->size;
	
	if (!buf->buffer)
		return 0;
	
	return mul(buf->gain, buf->buffer[read_pos], 0, 1, 1);
}

int sim_ddelay_write(sim_ddelay_buffer *buf, int16_t sample)
{
	if (!buf)
		return 1;
	
	if (!buf->buffer)
		return 1;
	
	if (buf->wrapped)
	{
		if (buf->gain < 0b0100000000000000)
		{
			buf->gain += 0b0000000010000000;
		}
	}
	
	if (buf->position == buf->size - 1)
	{
		buf->wrapped = 1;
	}
	
	buf->buffer[buf->position] = sample;
	
	buf->position = (buf->position + 1) % buf->size;
	
	return 0;
}

typedef struct
{
	m_dsp_block_instr instrs[256];
	int16_t block_regs[256*2];
	int16_t sample_out;
	int16_t channels[16];
	int32_t accumulator;
	int16_t *mem;
	sim_ddelay_buffer ddelay_buffers[32];
	int n_ddelay_buffers;
	int last_block;
} sim_pipeline;

typedef struct
{
	sim_pipeline pipelines[2];
	int current_pipeline;
	int pipelines_swapping;
	int16_t output_gain_a;
	int16_t output_gain_b;
	int16_t output_gain;
	int16_t input_gain;
	int16_t sample_out;
	int16_t pipeline_enables[2];
} sim_engine;

int init_sim_pipeline(sim_pipeline *pipeline)
{
	if (!pipeline)
		return 1;
	
	for (int i = 0; i < N_BLOCKS; i++)
		pipeline->instrs[i] = m_dsp_block_instr_nop();
	
	pipeline->sample_out = 0;
	
	for (int i = 0; i < 16; i++)
		pipeline->channels[i] = 0;
	
	pipeline->accumulator = 0;
	pipeline->mem = malloc(sizeof(int16_t) * 256);
	
	for (int i = 0; i < 32; i++)
		init_sim_ddelay_buffer(&pipeline->ddelay_buffers[i]);
	
	pipeline->n_ddelay_buffers = 0;
	pipeline->last_block = 0;
	
	return 0;
}

int sim_pipeline_alloc_ddelay(sim_pipeline *pipeline, int size)
{
	if (!pipeline)
		return 1;
	
	alloc_sim_ddelay_buffer(&pipeline->ddelay_buffers[pipeline->n_ddelay_buffers], size);
	pipeline->n_ddelay_buffers++;
	
	return 0;
}

sim_engine *new_sim_engine()
{
	sim_engine *sim = malloc(sizeof(sim_engine));
	
	init_sim_pipeline(&sim->pipelines[0]);
	init_sim_pipeline(&sim->pipelines[1]);
	
	sim->input_gain 	= (1 << 14);
	sim->output_gain	= (1 << 14);
	sim->output_gain_a 	= (1 << 14);
	sim->output_gain_b 	= 0;
	
	sim->sample_out = 0;
	
	sim->pipeline_enables[0] = 1;
	sim->pipeline_enables[1] = 0;
	
	return sim;
}

#define CONTROLLER_STATE_READY 		0
#define CONTROLLER_STATE_BEGIN	 	1
#define CONTROLLER_STATE_EXECUTE 	2
#define CONTROLLER_STATE_SWAP_PAUSE	3
#define CONTROLLER_STATE_SWAP_WAIT 	4
#define CONTROLLER_STATE_RESET_WAIT 5
#define CONTROLLER_STATE_PAUSE		6


/*
#define COMMAND_WRITE_BLOCK_INSTR 	0b10010000
#define COMMAND_WRITE_BLOCK_REG 	0b11100000
#define COMMAND_UPDATE_BLOCK_REG 	0b11101000
#define COMMAND_ALLOC_SRAM_DELAY 	0b00100000
#define COMMAND_SWAP_PIPELINES 		0b00000001
#define COMMAND_RESET_PIPELINE 		0b00001001
#define COMMAND_SET_INPUT_GAIN 		0b00000010
#define COMMAND_SET_OUTPUT_GAIN 	0b00000011
*/

int sim_handle_transfer_batch(sim_engine *sim, m_fpga_transfer_batch batch)
{
	if (!sim || !batch.buf)
		return 1;
	
	int16_t  data;
	uint32_t instr;
	uint16_t block;
	uint16_t reg;
	
	for (int i = 0; i < batch.len; i++)
	{
		switch (batch.buf[i])
		{
			case COMMAND_WRITE_BLOCK_INSTR:
				block = batch.buf[++i];
				if (N_BLOCKS > 255) block = (block << 8) | batch.buf[++i];
				
				instr = 			   batch.buf[++i];
				instr = (instr << 8) | batch.buf[++i];
				instr = (instr << 8) | batch.buf[++i];
				instr = (instr << 8) | batch.buf[++i];
				
				sim->pipelines[1-sim->current_pipeline].instrs[block] = m_decode_dsp_block_instr(instr);
				
				if (block > sim->pipelines[1-sim->current_pipeline].last_block) 
					sim->pipelines[1-sim->current_pipeline].last_block = block;
				break;
			
			case COMMAND_WRITE_BLOCK_REG:
				block = batch.buf[++i];
				if (N_BLOCKS > 255) block = (block << 8) | batch.buf[++i];
				
				reg =  batch.buf[++i];
				
				data = 			   batch.buf[++i];
				data = data << 8 | batch.buf[++i];
				
				sim->pipelines[1-sim->current_pipeline].block_regs[block * 2 + reg] = data;
				break;
			
			case COMMAND_UPDATE_BLOCK_REG:
				
				block = batch.buf[++i];
				if (N_BLOCKS > 255) block = (block << 8) | batch.buf[++i];
				
				reg =  batch.buf[++i];
				
				data = 			   batch.buf[++i];
				data = data << 8 | batch.buf[++i];
				
				sim->pipelines[sim->current_pipeline].block_regs[block * 2 + reg] = data;
				
				break;
			
			case COMMAND_ALLOC_SRAM_DELAY:
				
				data = 			   batch.buf[++i];
				data = data << 8 | batch.buf[++i];
				
				sim_pipeline_alloc_ddelay(&sim->pipelines[1-sim->current_pipeline], data);
				
				break;
			
			case COMMAND_SWAP_PIPELINES:
				
				sim->pipelines_swapping = 1;
				sim->pipeline_enables[!sim->current_pipeline] = 1;
				
				break;
			
			case COMMAND_RESET_PIPELINE:
				
				break;
			
			case COMMAND_SET_INPUT_GAIN:
				
				data = 			   batch.buf[++i];
				data = data << 8 | batch.buf[++i];
				
				sim->input_gain = data;
				
				break;
			
			case COMMAND_SET_OUTPUT_GAIN:
				
				data = 			   batch.buf[++i];
				data = data << 8 | batch.buf[++i];
				
				sim->output_gain = data;
				
				break;
		}
	}
	
	return 0;
}
/*
int sim_pipeline_process_sample(sim_pipeline *pipeline, int16_t sample)
{
	if (!pipeline)
		return 0;
	
	pipeline->channels[0] = sample;
	
	m_dsp_block_instr instr;
	
	int16_t src_a;
	int16_t src_b;
	int16_t src_c;
	
	for (int i = 0; i <= pipeline->last_block && i < N_BLOCKS; i++)
	{
		instr = pipeline->instrs[i];
		
		instr.src_a &= 0xF;
		instr.src_b &= 0xF;
		instr.src_c &= 0xF;
		instr.dest  &= 0xF;
		instr.res_addr &= 31;
		
		src_a = instr.src_a_reg ? pipeline->block_regs[i * 2 + !!(instr.src_a)] : pipeline->channels[instr.src_a];
		src_b = instr.src_b_reg ? pipeline->block_regs[i * 2 + !!(instr.src_b)] : pipeline->channels[instr.src_b];
		src_c = instr.src_c_reg ? pipeline->block_regs[i * 2 + !!(instr.src_c)] : pipeline->channels[instr.src_c];
		
		switch (instr.opcode)
		{
			case BLOCK_INSTR_NOP:
				break;
			
			case BLOCK_INSTR_ADD:
				pipeline->channels[instr.dest] = sum(src_a, src_b, !instr.sat);
				break;
			
			case BLOCK_INSTR_SUB:
				pipeline->channels[instr.dest] = sum(src_a, -src_b, !instr.sat);
				break;
			
			case BLOCK_INSTR_LSH:
				pipeline->channels[instr.dest] = src_a << instr.src_b;
				break;
			
			case BLOCK_INSTR_RSH:
				pipeline->channels[instr.dest] = (int16_t)((uint16_t)src_a >> instr.src_b);
				break;
			
			case BLOCK_INSTR_ARSH:
				pipeline->channels[instr.dest] = src_a >> instr.src_b;
				break;
			
			case BLOCK_INSTR_MUL:
				pipeline->channels[instr.dest] = mul(src_a, src_b, instr.no_shift, instr.shift, !instr.sat);
				break;
			
			case BLOCK_INSTR_MADD:
				pipeline->channels[instr.dest] = sum(src_c, mul(src_a, src_b, instr.no_shift, instr.shift, 0), !instr.sat);
				break;
			
			case BLOCK_INSTR_ABS:
				pipeline->channels[instr.dest] = (src_a < 0) ? -src_a : src_a;
				break;
			
			case BLOCK_INSTR_LUT:
				switch (instr.res_addr)
				{
					case 0: pipeline->channels[instr.dest] = (int16_t)(sinf(6.283185307179586 * src_a)) * (1 << 15);
					case 1: pipeline->channels[instr.dest] = (int16_t)(tanhf(4.0 * src_a)) * (1 << 15);
				}
				break;
			
			
			case BLOCK_INSTR_DELAY_READ:
				pipeline->channels[instr.dest] = sim_ddelay_read(&pipeline->ddelay_buffers[instr.res_addr], src_a);
				break;
			
			
			case BLOCK_INSTR_DELAY_WRITE:
				sim_ddelay_write(&pipeline->ddelay_buffers[instr.res_addr], src_a);
				break;
			
			
			case BLOCK_INSTR_SAVE:
				pipeline->mem[instr.res_addr] = src_a;
				break;
			
			
			case BLOCK_INSTR_LOAD:
				pipeline->channels[instr.dest] = pipeline->mem[instr.res_addr];
				break;
			
			
			case BLOCK_INSTR_MOV:
				pipeline->channels[instr.dest] = src_a;
				break;
			
			
			case BLOCK_INSTR_CLAMP:
				pipeline->channels[instr.dest] = (src_a < src_b) ? src_b : ((src_a > src_c) ? src_c : src_a);
				break;
			
			
			case BLOCK_INSTR_MACZ:
				pipeline->accumulator = mul_wide(src_a, src_b, 0, instr.shift, !instr.sat);
				break;
			
			
			case BLOCK_INSTR_MAC:
				pipeline->accumulator = sum_wide(src_c, mul_wide(src_a, src_b, 0, instr.shift, !instr.sat), !instr.sat);
				break;
			
			
			case BLOCK_INSTR_MOV_ACC:
				pipeline->channels[instr.dest] = (instr.sat) ? (pipeline->accumulator < -32768) ?
					-32768 : ((pipeline->accumulator > 32768) ? 32768 : pipeline->accumulator) : pipeline->accumulator;
				break;
			
			case BLOCK_INSTR_LINTERP:
				pipeline->channels[instr.dest] = linterp(src_a, src_b, src_c);
				break;
			
			case BLOCK_INSTR_FRAC_DELAY:
				pipeline->channels[instr.dest] = linterp(sim_ddelay_read(&pipeline->ddelay_buffers[instr.res_addr],  (uint16_t)(pipeline->accumulator & 0xFFFF0000) >> 16     ),
														 sim_ddelay_read(&pipeline->ddelay_buffers[instr.res_addr], ((uint16_t)(pipeline->accumulator & 0xFFFF0000) >> 16) + 1),
														 pipeline->accumulator & 0xFFFF);
				break;
			
			case BLOCK_INSTR_LOAD_ACC:
				pipeline->accumulator = pipeline->mem[instr.res_addr] << 16 | pipeline->mem[instr.res_addr + 1];
				break;
			
			case BLOCK_INSTR_SAVE_ACC:
				pipeline->mem[instr.res_addr    ] = (uint16_t)(pipeline->accumulator & 0xFFFF0000) >> 16;
				pipeline->mem[instr.res_addr + 1] = (uint16_t)(pipeline->accumulator & 0x0000FFFF) >> 0;
				break;
			
			
			case BLOCK_INSTR_ACC:
				pipeline->accumulator = sum_wide(pipeline->accumulator, src_a, !instr.sat);
				break;
			
			
			case BLOCK_INSTR_CLEAR_ACC:
				pipeline->accumulator = 0;
				break;
			
			
			case BLOCK_INSTR_MOV_UACC:
				pipeline->channels[instr.dest] = (uint16_t)(pipeline->accumulator & 0xFFFF0000) >> 16;
				break;
		}
	}
	
	pipeline->sample_out = pipeline->channels[0];
	return pipeline->channels[0];
}

int16_t sim_process_sample(sim_engine *sim, int16_t sample)
{
	if (!sim)
		return 1;
	
	int16_t sample_in = mul(sim->input_gain, sample, 0, 1, 0);
	
	int16_t sample_out_a = 0;
	int16_t sample_out_b = 0;
	
	if (sim->pipeline_enables[0])
	{
		sample_out_a = sim_pipeline_process_sample(&sim->pipelines[0], sample);
	}
	if (sim->pipeline_enables[1])
	{
		sample_out_b = sim_pipeline_process_sample(&sim->pipelines[1], sample);
	}
	
	sample_out_a = mul(sim->pipelines[0].sample_out, sim->output_gain_a, 0, 1, 1);
	sample_out_b = mul(sim->pipelines[1].sample_out, sim->output_gain_b, 0, 1, 1);
	
	sim->sample_out = sum(sample_out_a, sample_out_b, 1);
	
	if (sim->pipelines_swapping)
	{
		if ((sim->current_pipeline && sim->output_gain_b) || (!sim->current_pipeline && sim->output_gain_a))
		{
			if (sim->current_pipeline)
			{
				sim->output_gain_b -= 0b0000000001000000;
				sim->output_gain_a += 0b0000000001000000;
			}
			else
			{
				sim->output_gain_a -= 0b0000000001000000;
				sim->output_gain_b += 0b0000000001000000;
			}
		}
		else
		{
			sim->pipeline_enables[sim->current_pipeline] = 0;
			sim->current_pipeline = !sim->current_pipeline;
			sim->pipelines_swapping = 0;
		}
	}
	
	return sim->sample_out;
}
*/

m_effect_desc *m_biquad_eff_desc(float a0, float a1, float a2, float b0, float b1)
{
	m_effect_desc *eff = new_m_effect_desc("Biquad");
	
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_read(1, 1)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_read(2, 2)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_read(3, 3)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_read(4, 4))); //3
	
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_macz(0, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 4, 0, float_to_q_nminus1(a0, 3));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mac(1, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 5, 0, float_to_q_nminus1(a1, 3));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mac(2, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 6, 0, float_to_q_nminus1(a2, 3));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mac(3, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 7, 0, float_to_q_nminus1(b0, 3));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mac(4, 0, 0, 1, 3))); //8
	m_effect_desc_add_register_val_literal(eff, 8, 0, float_to_q_nminus1(b1, 3));
	
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_write(0, 0, 1)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_write(1, 0, 2)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_write(3, 0, 4)));
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mov_acc(0))); //12
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_mem_write(0, 0, 3)));
	
	return eff;
}

int main(int argc, char** argv)
{
	srand(time(0));
    Verilated::commandArgs(argc, argv);

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " in.wav out.wav\n";
        return 1;
    }
    
    sim_io_init(&io);

    const char* in_path  = argv[1];
    const char* out_path = argv[2];
    char out_path_em[256];
    
    sprintf(out_path_em, "%s.em.wav", out_path);

    WavHeader header;
    std::vector<int16_t> in_samples;
    if (!read_wav16_mono(in_path, header, in_samples)) {
        std::cerr << "Failed to read WAV\n";
        return 1;
    }

    std::vector<int16_t> out_samples;
    std::vector<int16_t> out_samples_emulated;
    
    int n_samples = in_samples.size();
    
    out_samples.reserve(n_samples);

    // ---------------- Verilator DUT ----------------

	Verilated::traceEverOn(true);
	
	#ifdef DUMP_WAVEFORM
	tfp = new VerilatedFstC;
	dut->trace(tfp, 99);
	tfp->open("./verilator/waveform.fst");
	#endif

	for (int i = 0; i < 16; i++)
		tick();
	
	printf("Starting...\n");
	
	m_fpga_transfer_batch batch = m_new_fpga_transfer_batch();
	m_effect_desc *eff = m_biquad_eff_desc(0.9915240, -1.9829756, 0.9915240, 1.9829756, -0.9830480);
	
	/*m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_clamp(0, 0, 0, 1, 1, 1, 0)));
	m_effect_desc_add_register_val_literal(eff, 14, 0, -256);
	m_effect_desc_add_register_val_literal(eff, 14, 1, -32768);*/
	
	m_fpga_resource_report local = m_empty_fpga_resource_report();
	m_fpga_resource_report res = m_empty_fpga_resource_report();
	
	m_fpga_transfer_batch_append_effect_desc(eff, &res, &local, &batch);
	
	m_fpga_batch_append(&batch, COMMAND_SWAP_PIPELINES);
	
	append_send_queue(batch, 128);
	
	/*int16_t new_val = 1024;
	
	batch = m_new_fpga_transfer_batch();
	m_fpga_batch_append(&batch, COMMAND_UPDATE_BLOCK_REG);
	m_fpga_batch_append(&batch, 4);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, (new_val & 0xFF00) >> 8);
	m_fpga_batch_append(&batch, (new_val & 0x00FF) >> 0);
	m_fpga_batch_append(&batch, COMMAND_COMMIT_REG_UPDATES);
	
	append_send_queue(batch, 512);
	
	batch = m_new_fpga_transfer_batch();
	eff = new_m_effect_desc("");
	
	m_effect_desc_add_block(eff, new_m_dsp_block_with_instr(m_dsp_block_instr_madd(0, 0, 0, 1, 1, 1, 0, 2)));
	m_effect_desc_add_register_val(eff, 0, 0, 2, "1.0");
	m_effect_desc_add_register_val(eff, 0, 1, 0, "0.0");
	
	local = m_empty_fpga_resource_report();
	res = m_empty_fpga_resource_report();
	
	m_fpga_transfer_batch_append_effect_desc(eff, &res, &local, &batch);
	
	m_fpga_batch_append(&batch, COMMAND_SWAP_PIPELINES);
	
	append_send_queue(batch, 800);
	
	batch = m_new_fpga_transfer_batch();
	
	new_val = float_to_q_nminus1(1.1, 2);
	
	batch = m_new_fpga_transfer_batch();
	m_fpga_batch_append(&batch, COMMAND_UPDATE_BLOCK_REG);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, (new_val & 0xFF00) >> 8);
	m_fpga_batch_append(&batch, (new_val & 0x00FF) >> 0);
	m_fpga_batch_append(&batch, COMMAND_COMMIT_REG_UPDATES);
	
	append_send_queue(batch, 900);
	
	batch = m_new_fpga_transfer_batch();
	
	new_val = float_to_q_nminus1(1.2, 2);
	
	batch = m_new_fpga_transfer_batch();
	m_fpga_batch_append(&batch, COMMAND_UPDATE_BLOCK_REG);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, (new_val & 0xFF00) >> 8);
	m_fpga_batch_append(&batch, (new_val & 0x00FF) >> 0);
	m_fpga_batch_append(&batch, COMMAND_COMMIT_REG_UPDATES);
	
	append_send_queue(batch, 1200);
	
	batch = m_new_fpga_transfer_batch();
	
	new_val = float_to_q_nminus1(1.3, 2);
	
	batch = m_new_fpga_transfer_batch();
	m_fpga_batch_append(&batch, COMMAND_UPDATE_BLOCK_REG);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, (new_val & 0xFF00) >> 8);
	m_fpga_batch_append(&batch, (new_val & 0x00FF) >> 0);
	m_fpga_batch_append(&batch, COMMAND_COMMIT_REG_UPDATES);
	
	append_send_queue(batch, 1500);
	
	batch = m_new_fpga_transfer_batch();
	
	new_val = float_to_q_nminus1(1.4, 2);
	
	batch = m_new_fpga_transfer_batch();
	m_fpga_batch_append(&batch, COMMAND_UPDATE_BLOCK_REG);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, (new_val & 0xFF00) >> 8);
	m_fpga_batch_append(&batch, (new_val & 0x00FF) >> 0);
	m_fpga_batch_append(&batch, COMMAND_COMMIT_REG_UPDATES);
	
	append_send_queue(batch, 1800);
	
	batch = m_new_fpga_transfer_batch();
	
	new_val = float_to_q_nminus1(1.5, 2);
	
	batch = m_new_fpga_transfer_batch();
	m_fpga_batch_append(&batch, COMMAND_UPDATE_BLOCK_REG);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, 0);
	m_fpga_batch_append(&batch, (new_val & 0xFF00) >> 8);
	m_fpga_batch_append(&batch, (new_val & 0x00FF) >> 0);
	m_fpga_batch_append(&batch, COMMAND_COMMIT_REG_UPDATES);
	
	append_send_queue(batch, 2000);*/
	
	int samples_to_process = (n_samples < MAX_SAMPLES) ? n_samples : MAX_SAMPLES;
	
	#ifdef RUN_EMULATOR
	sim_engine *emulator = new_sim_engine();
	#endif
	
	int16_t y;
	int16_t emulated_y = 0;
	
	float t = 0;
	
	const float sample_duration = 1.0f / (44.1f * 1000.0f);
	
    while (samples_processed < samples_to_process)
	{
		tick();
		
		if (samples_processed % 128 == 0)
			printf("\rSamples processed: %d/%d (%.2f%%)  ", samples_processed, samples_to_process, 100.0 * (float)samples_processed/(float)samples_to_process);
		
		if (io.i2s_ready)
		{
			if (send_queue)
			{
				if (samples_processed >= send_queue->tick)
				{
					if (!send_queue->started)
					{
						send_queue->started = 1;
						send_queue->position = 0;
						
						printf("\nSending batch. ");
						m_fpga_batch_print(send_queue->batch);
						
						#ifdef RUN_EMULATOR
						sim_handle_transfer_batch(emulator, send_queue->batch);
						#endif
					}
					
					if (send_queue->position >= send_queue->batch.len)
					{
						pop_send_queue();
					}
					else
					{
						if (spi_send(send_queue->batch.buf[send_queue->position]) == 0)
							send_queue->position++;
					}
				}
			}
			
			samples_processed++;
			t += sample_duration;
			
			io.sample_in = (uint16_t)(roundf(sinf(6.28 * 100.0f * t * ((float)samples_processed / (float)samples_to_process)) * 32767.0 * 0.5f));
			//io.sample_in = (uint16_t)(roundf(sinf(6.28 * 440.0f * t) * 32767.0 * 0.5f));
			
			//static_cast<int16_t>(in_samples[samples_processed]);
			y = static_cast<int16_t>(io.sample_out);
			out_samples.push_back(y);
			io.i2s_ready = 0;
			
			#ifdef RUN_EMULATOR
			if (samples_processed > 4)
			{
				emulated_y = sim_process_sample(emulator, in_samples[samples_processed - 3]);
			}
			
			out_samples_emulated.push_back(emulated_y);
			
			if (/*y != emulated_y*/ abs(emulated_y - y) / 32768.0f > 0.01)
			{
				printf("\rSimulation mismatch of %.02f%% detected at sample %d. Simulated: %d. Emulated: %d\n", 100 * abs(emulated_y - y) / 32768.0f, samples_processed, y, emulated_y);
			}
			#endif
		}
    }

	printf("\rSamples processed: %d/%d (100%%)  \n", samples_to_process, samples_to_process);
	
    #ifdef DUMP_WAVEFORM
	tfp->close();
	delete tfp;
	#endif
	
	#ifdef RUN_EMULATOR
    write_wav16_mono(out_path_em, header.sample_rate, out_samples_emulated);
    #endif
    
    if (!write_wav16_mono(out_path, header.sample_rate, out_samples))
    {
        std::cerr << "Failed to write WAV\n";
        return 1;
    }

    delete dut;
    return 0;
}
