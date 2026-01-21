#include "sim_main.h"

Vtop* dut = new Vtop;

#define DUMP_WAVEFORM

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

    // Explicit little-endian write (belt + suspenders)
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

int tick()
{
	if (!dut)
		return 1;
	
	dut->sys_clk = 1;
	sim_io_update(&io);
	dut->eval();
	if (tfp) tfp->dump(ticks++);
	
	//print_state();
	
	dut->sys_clk = 0;
	sim_io_update(&io);
	dut->eval();
	if (tfp) tfp->dump(ticks++);
	
	//print_state();
	
	return 0;
}

int16_t float_to_q15(float x)
{
	if (x >= 0.999969482421875f) return  32767;
    if (x <= -1.0f)              return -32768;
    
    return (int16_t)lrintf(x * 32768.0f);
}

#define BLOCK_INSTR(opcode, src_a, src_b, src_c, dest, a_reg, b_reg, c_reg, dest_reg, shift) \
	BLOCK_INSTR_S(opcode, src_a, src_b, src_c, dest, a_reg, b_reg, c_reg, dest_reg, shift, sat)
	
#define BLOCK_INSTR_S(opcode, src_a, src_b, src_c, dest, a_reg, b_reg, c_reg, dest_reg, shift, sat) (\
		  ((uint32_t)opcode) \
		| ((uint32_t)src_a 	<< (0 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)src_b 	<< (1 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)src_c 	<< (2 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)dest  	<< (3 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)a_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 0)) \
		| ((uint32_t)b_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 1)) \
		| ((uint32_t)c_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 2)) \
		| ((uint32_t)dest_reg	<< (BLOCK_INSTR_OP_TYPE_START + 3)) \
		| ((uint32_t)shift << (BLOCK_INSTR_PMS_START)) \
		| ((uint32_t)sat   << (BLOCK_INSTR_OP_TYPE_START + 4)))

void write_block_instr(int block, uint32_t instr)
{
	printf("Set block %d instruction to %d = 0b%032b\n", block, instr, instr);
	spi_send(COMMAND_WRITE_BLOCK_INSTR);
	
	spi_send(block);
	
	spi_send((instr >> 24) & 0xFFFF);
	
	spi_send((instr >> 16) & 0xFFFF);
	
	spi_send((instr >> 8) & 0xFFFF);
	
	spi_send(instr & 0xFFFF);
}

void send_data_command(int command, uint16_t data)
{
	spi_send(command);
	spi_send((data >> 8) & 0xFFFF);
	spi_send(data & 0xFFFF);
}

void write_block_register(int block, int reg, uint16_t val)
{
	printf("Write block register: %d.%d <= %.06f\n", block, reg, (float)(val / (float)(1 << 15)));
	
	printf("Send command COMMAND_WRITE_BLOCK_REG\n");
	spi_send(COMMAND_WRITE_BLOCK_REG);
	
	printf("Send block number\n");
	spi_send(block);
	
	printf("Send register number\n");
	
	spi_send(reg & 0xFFFF);
	
	printf("Send value\n");
	spi_send((val >> 8) & 0xFFFF);
	
	spi_send(val & 0xFFFF);
}

void load_biquad(int base_block,
                 float b0, float b1, float b2,
                 float a1, float a2)
{
	// channels:
	// ch0 = x[n]
	// ch1 = x[n]
	// ch2 = x[n-1]
	// ch3 = x[n-2]
	// ch4 = y[n-1]
	// ch5 = y[n-2]
	
	// move previous sample's x[n-1] to ch3
    write_block_instr(base_block + 0,
        BLOCK_INSTR_S(BLOCK_INSTR_MOV, 2, 0, 0, 3, 0, 0, 0, 0, 0, 1));
    // move previous sample's x[n] to ch2
    write_block_instr(base_block + 1,
        BLOCK_INSTR_S(BLOCK_INSTR_MOV, 1, 0, 0, 2, 0, 0, 0, 0, 0, 1));
    // copy x[n] to ch1
    write_block_instr(base_block + 2,
        BLOCK_INSTR_S(BLOCK_INSTR_MOV, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1));

	// ch0 = b0*x[n] = b0*ch0
    write_block_register(base_block + 3, 0, float_to_q_nminus1(b0, 1));
    write_block_instr(base_block + 3,
        BLOCK_INSTR_S(BLOCK_INSTR_MUL, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1));

	// ch0 = b0*x[n] + b1*x[n-1] = b1*ch2 + ch0
    write_block_register(base_block + 4, 0, float_to_q_nminus1(b1, 1));
    write_block_instr(base_block + 4,
        BLOCK_INSTR_S(BLOCK_INSTR_MAD, 2, 0, 0, 0, 0, 1, 0, 0, 0, 1));

	// ch0 = b0*x[n] + b1*x[n-1] + b2*x[n-1] = b2*ch3 + ch0
    write_block_register(base_block + 5, 0, float_to_q_nminus1(b2, 1));
    write_block_instr(base_block + 5,
        BLOCK_INSTR_S(BLOCK_INSTR_MAD, 3, 0, 0, 0, 0, 1, 0, 0, 0, 1));

	// ch0 = b0*x[n] + b1*x[n-1] + b2*x[n-1] - a1y[n-1] = -a1*ch4 + ch0
    write_block_register(base_block + 6, 0, float_to_q_nminus1(-a1, 1));
    write_block_instr(base_block + 6,
        BLOCK_INSTR_S(BLOCK_INSTR_MAD, 4, 0, 0, 0, 0, 1, 0, 0, 0, 1));

	// ch0 = b0*x[n] + b1*x[n-1] + b2*x[n-1] - a1y[n-1] - a2y[n-2] = -a2*ch5 + ch0
    write_block_register(base_block + 7, 0, float_to_q_nminus1(-a2, 1));
    write_block_instr(base_block + 7,
        BLOCK_INSTR_S(BLOCK_INSTR_MAD, 5, 0, 0, 0, 0, 1, 0, 0, 0, 1));

	// clamp to [-1, 1)
	write_block_instr(base_block + 8,
        BLOCK_INSTR_S(BLOCK_INSTR_CLAMP, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0));
       
    // shift back to format
	write_block_instr(base_block + 9,
        BLOCK_INSTR_S(BLOCK_INSTR_LSH, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));

	// move previous sample's y[n-1] to ch5
    write_block_instr(base_block + 10,
        BLOCK_INSTR_S(BLOCK_INSTR_MOV, 4, 0, 0, 5, 0, 0, 0, 0, 0, 0));
    // move y[n] to ch4
    write_block_instr(base_block + 11,
        BLOCK_INSTR_S(BLOCK_INSTR_MOV, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0));
}


int pow2_ceil(int x)
{
	int y = 1;
	
	while (y && y <= x)
		y = y << 1;
	
	return y;
}

#define MS_TO_SAMPLES(x) ((int)roundf(((float)x * 48.0f)))

void alloc_sram_delay(int ms)
{
	printf("Allocating SRAM delay buffer of size %d\n", pow2_ceil(MS_TO_SAMPLES(ms)));
	send_data_command(COMMAND_ALLOC_SRAM_DELAY, pow2_ceil(MS_TO_SAMPLES(ms)));
}

void set_input_gain(float gain_db)
{
	float v = powf(10, gain_db / 20.0f);
	uint16_t s = float_to_q_nminus1(v, 5);
			
	printf("Telling FPGA to change input gain to %fdB = %f = 0b%d%d%d%d%d.%d%d%d%d%d%d%d%d%d%d%d\n", gain_db, v,
		!!(s & (1 << 15)),
		!!(s & (1 << 14)),
		!!(s & (1 << 13)),
		!!(s & (1 << 12)),
		!!(s & (1 << 11)),
		!!(s & (1 << 10)),
		!!(s & (1 << 9)),
		!!(s & (1 << 8)),
		!!(s & (1 << 7)),
		!!(s & (1 << 6)),
		!!(s & (1 << 5)),
		!!(s & (1 << 4)),
		!!(s & (1 << 3)),
		!!(s & (1 << 2)),
		!!(s & (1 << 1)),
		!!(s & (1 << 0)));
	
	spi_send(COMMAND_SET_INPUT_GAIN);
	spi_send((s & 0xFF00) >> 8);
	spi_send(s & 0x00FF);
}

int main(int argc, char** argv)
{
    Verilated::commandArgs(argc, argv);

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " in.wav out.wav\n";
        return 1;
    }
    
    sim_io_init(&io);

    const char* in_path  = argv[1];
    const char* out_path = argv[2];

    WavHeader header;
    std::vector<int16_t> in_samples;
    if (!read_wav16_mono(in_path, header, in_samples)) {
        std::cerr << "Failed to read WAV\n";
        return 1;
    }

    std::vector<int16_t> out_samples;
    
    int n_samples = in_samples.size();
    
    out_samples.reserve(n_samples);

    // ---------------- Verilator DUT ----------------

	Verilated::traceEverOn(true);
	
	#ifdef DUMP_WAVEFORM
	tfp = new VerilatedFstC;
	dut->trace(tfp, 99);
	tfp->open("./verilator/waveform.fst");
	#endif

	pipeline *pl;
	effect *eff;
	transfer_sequence tfseq;

	for (int i = 0; i < 16; i++)
		tick();

	int samples_processed = 0;
	
	printf("Starting...\n");
	
    for (int i = 0; i < (1 << 26); i++)
	{
		tick();
		
		if (i == (1 << 24))
		{
			pl = new_pipeline();
			
			eff = create_biquad_effect(
				 0.065323,
				 0.0,
				-0.065323,
				-1.748845,
				 0.869354
			);
			pipeline_add_effect(pl, eff);
			
			tfseq = pipeline_transfer_sequence(pl);
			
			send_transfer_sequence(tfseq);
			while (spi_send(COMMAND_SWAP_PIPELINES) == 1)
				tick();
		}
		
		if (samples_processed % 128 == 0)
			printf("\rSamples processed: %d/%d (%.2f%%)", samples_processed, n_samples, 100.0 * (float)samples_processed/(float)n_samples);
		
		if (io.i2s_ready)
		{
			if (samples_processed < n_samples)
			{
				samples_processed++;
				io.sample_in = static_cast<int16_t>(in_samples[samples_processed]);
				int16_t y = static_cast<int16_t>(io.sample_out);
				out_samples.push_back(y);
				io.i2s_ready = 0;
			}
			else
			{
				break;
			}
		}
    }

	printf("\r\n");
	
    #ifdef DUMP_WAVEFORM
	tfp->close();
	delete tfp;
	#endif
	
    if (!write_wav16_mono(out_path, header.sample_rate, out_samples))
    {
        std::cerr << "Failed to write WAV\n";
        return 1;
    }


    delete dut;
    return 0;
}
