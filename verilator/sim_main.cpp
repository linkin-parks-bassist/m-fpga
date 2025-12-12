#include <verilated.h>
#include "Vpipeline.h"
#include "verilated_fst_c.h"
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>
#include "math.h"

// ---------------- WAV I/O ----------------
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
                             WavHeader header,
                             const std::vector<int16_t>& samples)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    header.data_size  = samples.size() * sizeof(int16_t);
    header.chunk_size = 36 + header.data_size;
    header.byte_rate  = header.sample_rate * 2;

    f.write(reinterpret_cast<const char*>(&header), sizeof(header));
    f.write(reinterpret_cast<const char*>(samples.data()), header.data_size);
    return true;
}

VerilatedFstC* tfp = NULL;
static uint64_t ticks = 0;

void tick(Vpipeline* dut)
{
	assert(dut != NULL);
	
	dut->clk ^= 1; dut->eval();
	if (tfp) tfp->dump(ticks++);
	dut->clk ^= 1; dut->eval();
	if (tfp) tfp->dump(ticks++);
}

int16_t float_to_q_nminus1(float x, int shift)
{
    // shift = 15 - n  (your instruction encoding)
    int n = 15 - shift;   // actual fractional bits

    // Qm.n scale
    float scale = (float)(1 << n);

    // Compute representable real range for signed 16-bit Qm.n
    float max =  (float)((1 << 15) - 1) / scale;
    float min = -(float)(1 << 15)       / scale;

    if (x > max) x = max;
    if (x < min) x = min;

    return (int16_t)lrintf(x * scale);
}


int16_t float_to_q15(float x)
{
	if (x >= 0.999969482421875f) return  32767;
    if (x <= -1.0f)              return -32768;
    
    return (int16_t)lrintf(x * 32768.0f);
}

#define BLOCK_INSTR_NOP 	0
#define BLOCK_INSTR_ADD 	1
#define BLOCK_INSTR_SUB 	2
#define BLOCK_INSTR_LSH 	3
#define BLOCK_INSTR_RSH 	4
#define BLOCK_INSTR_ARSH 	5
#define BLOCK_INSTR_MUL 	6
#define BLOCK_INSTR_MAC		7
#define BLOCK_INSTR_ABS		8
#define BLOCK_INSTR_BIQ_DF1	9
#define BLOCK_INSTR_LUT		10
#define BLOCK_INSTR_ENVD 	11
#define BLOCK_INSTR_DELAY 	12

#define BLOCK_INSTR_OP_WIDTH 5
#define BLOCK_REG_ADDR_WIDTH 4

#define BLOCK_INSTR_OP_TYPE_START 	(4 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH)
#define BLOCK_INSTR_PMS_START		(BLOCK_INSTR_OP_TYPE_START + 4)

#define BLOCK_INSTR(opcode, src_a, src_b, src_c, dest, a_reg, b_reg, c_reg, dest_reg, shift) (\
		  ((uint32_t)opcode) \
		| ((uint32_t)src_a 	<< (0 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)src_b 	<< (1 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)src_c 	<< (2 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)dest  	<< (3 * BLOCK_REG_ADDR_WIDTH + BLOCK_INSTR_OP_WIDTH    )) \
		| ((uint32_t)a_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 0)) \
		| ((uint32_t)b_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 1)) \
		| ((uint32_t)c_reg 	<< (BLOCK_INSTR_OP_TYPE_START + 2)) \
		| ((uint32_t)dest_reg	<< (BLOCK_INSTR_OP_TYPE_START + 3)) \
		| ((uint32_t)shift << (BLOCK_INSTR_PMS_START)))

#define COMMAND_WRITE_BLOCK_INSTR 	0b10010000
#define COMMAND_WRITE_BLOCK_REG 	0b11100000
#define COMMAND_UPDATE_BLOCK_REG 	0b11100001
#define COMMAND_ALLOC_SRAM_DELAY 	0b00100000


void send_input_byte(Vpipeline *dut, uint8_t byte)
{
	printf("Sending input byte 0b%08b\n", byte);
	dut->inp_byte = byte;
	
	dut->inp_ready = 0;
	while (!dut->inp_fifo_read)
		tick(dut);
	
	dut->inp_ready = 1;
	
	tick(dut);
	dut->inp_ready = 0;
	
	tick(dut);
}

void wait_for_input_req(Vpipeline *dut)
{
	dut->inp_ready = 0;
	while (!dut->inp_fifo_read)
		tick(dut);
}

void write_block_instr(Vpipeline* dut, int block, uint32_t instr)
{
	printf("Set block %d instruction to %d\n", block, instr);
	send_input_byte(dut, COMMAND_WRITE_BLOCK_INSTR);
	
	send_input_byte(dut, block);
	
	send_input_byte(dut, (instr >> 24) & 0xFFFF);
	
	send_input_byte(dut, (instr >> 16) & 0xFFFF);
	
	send_input_byte(dut, (instr >> 8) & 0xFFFF);
	
	send_input_byte(dut, instr & 0xFFFF);
}

void send_data_command(Vpipeline *dut, int command, uint16_t data)
{
	send_input_byte(dut, command);
	send_input_byte(dut, (data >> 8) & 0xFFFF);
	send_input_byte(dut, data & 0xFFFF);
}

void write_block_register(Vpipeline* dut, int block, int reg, uint16_t val)
{
	printf("Write block register: %d.%d <= %.06f\n", block, reg, (float)(val / (float)(1 << 15)));
	
	printf("Send command COMMAND_WRITE_BLOCK_REG\n");
	send_input_byte(dut, COMMAND_WRITE_BLOCK_REG);
	
	printf("Send block number\n");
	send_input_byte(dut, block);
	
	printf("Send register number\n");
	
	send_input_byte(dut, reg & 0xFFFF);
	
	printf("Send value\n");
	send_input_byte(dut, (val >> 8) & 0xFFFF);
	
	send_input_byte(dut, val & 0xFFFF);
}

void load_biquad(Vpipeline* dut, int block, float b0, float b1, float b2, float a1, float a2)
{   
    write_block_instr(dut, block, BLOCK_INSTR(BLOCK_INSTR_BIQ_DF1, 0, 0, 0, 0, 0, 0, 0, 0, 0));
    
    write_block_register(dut, block, 0, float_to_q_nminus1(b0, 1));
    write_block_register(dut, block, 1, float_to_q_nminus1(b1, 1));
    write_block_register(dut, block, 2, float_to_q_nminus1(b2, 1));
    write_block_register(dut, block, 3, float_to_q_nminus1(a1, 1));
    write_block_register(dut, block, 4, float_to_q_nminus1(a2, 1));
}

int pow2_ceil(int x)
{
	int y = 1;
	
	while (y && y <= x)
		y = y << 1;
	
	return y;
}

// ---------------- Simulation ----------------
int main(int argc, char** argv)
{
    Verilated::commandArgs(argc, argv);

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " in.wav out.wav\n";
        return 1;
    }

    const char* in_path  = argv[1];
    const char* out_path = argv[2];

    WavHeader header;
    std::vector<int16_t> in_samples;
    if (!read_wav16_mono(in_path, header, in_samples)) {
        std::cerr << "Failed to read WAV\n";
        return 1;
    }

    std::vector<int16_t> out_samples;
    out_samples.reserve(in_samples.size());

    // ---------------- Verilator DUT ----------------
    Vpipeline* dut = new Vpipeline;

	Verilated::traceEverOn(true);
	
	#ifdef DUMP_TRACE
	tfp = new VerilatedFstC;
	dut->trace(tfp, 99);
	tfp->open("wave.fst");
	#endif

    const int TICKS_PER_SAMPLE = 2000;   // <-- change as needed

    // Reset
    dut->reset = 1;
    dut->clk = 0;
    dut->in_sample = 0;
    dut->in_valid  = 0;

    tick(dut);
    tick(dut);
    tick(dut);
    tick(dut);
    tick(dut);
    tick(dut);
    
    dut->reset = 0;

	// Trivial biquad
	//load_biquad(dut, 0, 1, 0, 0, 0, 0);
	
	// 1kHz LPF
	//load_biquad(dut, 0, (float)0.0039160767, (float)0.0078321534, (float)0.0039160767, (float)-1.8153179157, (float)0.8309822224);

	// 200Hz HPF
	//load_biquad(dut, 0, (float)0.9816555739f, (float)-1.9633111479f, (float)0.9816555739f, (float)-1.9629747014f, (float)0.9636475944f);
	
	// 2kHz band-pass
	//load_biquad(dut, 0, (float)0.1253540215, (float)0.0, (float)-0.1253540215, (float)-1.8713103092, (float)0.9373229892);

	/*write_block_instr(dut, 0, BLOCK_INSTR(BLOCK_INSTR_ENVD,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0));
	tick(dut);tick(dut);tick(dut);tick(dut);tick(dut);tick(dut);tick(dut);tick(dut);
	write_block_register(dut, 0, 0, float_to_q15(0.99f));
	write_block_register(dut, 0, 1, float_to_q15(0.01f));*/

	#define MS_TO_SAMPLES(x) ((int)roundf(((float)x * 48.0f)))

	int delay = 254;

	send_data_command(dut, COMMAND_ALLOC_SRAM_DELAY, pow2_ceil(MS_TO_SAMPLES(delay)));
	
	write_block_instr(dut, 0, BLOCK_INSTR(BLOCK_INSTR_DELAY, 0, 0, 0, 0, 0, 0, 0, 0, 0));
	write_block_register(dut, 0, 1, MS_TO_SAMPLES(delay));
	write_block_register(dut, 0, 2, float_to_q15(0.4f));

	delay = 127;

	send_data_command(dut, COMMAND_ALLOC_SRAM_DELAY, pow2_ceil(MS_TO_SAMPLES(delay)));
	
	write_block_instr(dut, 1, BLOCK_INSTR(BLOCK_INSTR_DELAY, 0, 0, 0, 0, 0, 0, 0, 0, 0));
	write_block_register(dut, 1, 0, 1);
	write_block_register(dut, 1, 1, MS_TO_SAMPLES(delay));
	write_block_register(dut, 1, 2, float_to_q15(0.5f));
	
	
	//write_block_instr(dut, 1, BLOCK_INSTR(BLOCK_INSTR_MUL, 0, 0, 0, 0, 0, 0, 0, 0, 0));
	//write_block_register(dut, 1, 0, float_to_q15(0.5));

    // ---------------- Main loop ----------------
    size_t last_percent = 0;

    for (size_t i = 0; i < in_samples.size(); ++i)
    {

        // Feed sample
        dut->in_sample = static_cast<int16_t>(in_samples[i]);
        dut->in_valid  = 1;
        
        tick(dut);
		dut->in_valid  = 0;

        // Tick pipeline for this sample
        for (int t = 0; t < TICKS_PER_SAMPLE; ++t)
        {
            tick(dut);
            
            if (dut->ready)
				break;
        }

        dut->in_valid = 0;

        // Capture output
        int16_t y = static_cast<int16_t>(dut->out_sample);
        out_samples.push_back(y);

        // Progress report
        if (i % 1024 == 0)
		{
			size_t percent = (100 * i) / in_samples.size();
			
			std::cout << "\rProgress: " << i << "/" << in_samples.size() << " samples processed ("<< percent << "% done)" << std::flush;
		}
    }

    std::cout << "\rProgress: " << in_samples.size() << "/" << in_samples.size() << " samples processed (100% done)" << std::endl;
    
    #ifdef DUMP_TRACE
	tfp->close();
	delete tfp;
	#endif
	
    if (!write_wav16_mono(out_path, header, out_samples)) {
        std::cerr << "Failed to write WAV\n";
        return 1;
    }


    delete dut;
    return 0;
}
