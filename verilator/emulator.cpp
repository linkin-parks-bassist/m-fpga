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
	m_block_instr instrs[256];
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
		pipeline->instrs[i] = m_block_instr_nop();
	
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
#define COMMAND_ALLOC_DELAY 	0b00100000
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
			
			case COMMAND_ALLOC_DELAY:
				
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
	
	m_block_instr instr;
	
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
