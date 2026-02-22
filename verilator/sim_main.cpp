#include <time.h>
#include "sim_main.h"

int samples_processed = 0;

m_effect_desc *m_read_eff_desc_from_file(char *fname);

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
/*
m_effect_desc *m_h_eff_desc()
{
	m_effect_desc *eff = new_m_effect_desc("h");
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_lsh(0, 0, 1, 0)));
	
	return eff;
}

m_effect_desc *m_biquad_eff_desc(float a0, float a1, float a2, float b0, float b1)
{
	m_effect_desc *eff = new_m_effect_desc("Biquad");
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(1, 1)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(2, 2)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(3, 3)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(4, 4))); //3
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_macz(0, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 4, 0, float_to_q_nminus1(a0, 3));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(1, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 5, 0, float_to_q_nminus1(a1, 3));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(2, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 6, 0, float_to_q_nminus1(a2, 3));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(3, 0, 0, 1, 3)));
	m_effect_desc_add_register_val_literal(eff, 7, 0, float_to_q_nminus1(b0, 3));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(4, 0, 0, 1, 3))); //8
	m_effect_desc_add_register_val_literal(eff, 8, 0, float_to_q_nminus1(b1, 3));
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(0, 0, 1)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(1, 0, 2)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(3, 0, 4)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mov_acc(0))); //12
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(0, 0, 3)));
	
	return eff;
}

m_effect_desc *create_lpf_eff_desc()
{
	m_effect_desc *eff = new_m_effect_desc("Low Pass Filter");
	m_parameter *cutoff = new_m_parameter_wni("Cutoff Frequency", "cutoff", 100.0,  1.0, 1000.0);
	
	m_effect_desc_add_param(eff, cutoff);
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(1, 1)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(2, 2)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(3, 3)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(4, 4))); //3
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_macz(0, 0, 0, 1, 14)));
	m_effect_desc_add_register_val(eff, 4, 0, 0, "/ / - 1 cos / * * 2 pi cutoff sample_rate 2 + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(1, 0, 0, 1, 14)));
	m_effect_desc_add_register_val(eff, 5, 0, 0, "/ - 1 cos / * * 2 pi cutoff sample_rate + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(2, 0, 0, 1, 14)));
	m_effect_desc_add_register_val(eff, 6, 0, 0, "/ / - 1 cos / * * 2 pi cutoff sample_rate 2 + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac_noshift(3, 0, 0, 1)));
	m_effect_desc_add_register_val(eff, 7, 0, 1, "/ * 2 cos / * * 2 pi cutoff sample_rate + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac_noshift(4, 0, 0, 1))); //8
	m_effect_desc_add_register_val(eff, 8, 0, 1, "- 0 / - 1 / sin / * * 2 pi cutoff sample_rate sqrt 2 + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(0, 0, 1)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(1, 0, 2)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(3, 0, 4)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mov_acc_sh(14, 0))); //12
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(0, 0, 3)));
	
	return eff;
}

m_effect_desc *create_hpf_eff_desc()
{
	m_effect_desc *eff = new_m_effect_desc("High Pass Filter");
	m_parameter *cutoff = new_m_parameter_wni("Cutoff Frequency", "cutoff", 1000.0,  1.0, 1000.0);
	
	m_effect_desc_add_param(eff, cutoff);
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(1, 1)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(2, 2)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(3, 3)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_read(4, 4))); //3
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_macz(0, 0, 0, 1, 2)));
	m_effect_desc_add_register_val(eff, 4, 0, 2, "/ / + 1 cos / * * 2 pi cutoff sample_rate 2 + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(1, 0, 0, 1, 2)));
	m_effect_desc_add_register_val(eff, 5, 0, 2, "- 0 / + 1 cos / * * 2 pi cutoff sample_rate + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(2, 0, 0, 1, 2)));
	m_effect_desc_add_register_val(eff, 6, 0, 2, "/ / + 1 cos / * * 2 pi cutoff sample_rate 2 + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(3, 0, 0, 1, 1)));
	m_effect_desc_add_register_val(eff, 7, 0, 1, "/ * 2 cos / * * 2 pi cutoff sample_rate + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mac(4, 0, 0, 1, 1))); //8
	m_effect_desc_add_register_val(eff, 8, 0, 1, "- 0 / - 1 / sin / * * 2 pi cutoff sample_rate sqrt 2 + 1 / sin / * * 2 pi cutoff sample_rate sqrt 2");
	
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(0, 0, 1)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(1, 0, 2)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(3, 0, 4)));
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mov_acc(0))); //12
	m_effect_desc_add_block(eff, new_m_block_with_instr(m_block_instr_mem_write(0, 0, 3)));
	
	return eff;
}*/

void queue_sends()
{
	m_fpga_transfer_batch batch = m_new_fpga_transfer_batch();
	
	m_fpga_batch_append(&batch, 0x90); 
	m_fpga_batch_append(&batch, 0x00); 
	m_fpga_batch_append(&batch, 0x0a); 
	m_fpga_batch_append(&batch, 0x12); 
	m_fpga_batch_append(&batch, 0x80); 
	m_fpga_batch_append(&batch, 0x01); 
	m_fpga_batch_append(&batch, 0xe0); 
	m_fpga_batch_append(&batch, 0x00); 
	m_fpga_batch_append(&batch, 0x00); 
	m_fpga_batch_append(&batch, 0x00); 
	m_fpga_batch_append(&batch, 0x00); 
	m_fpga_batch_append(&batch, 0x01); 
	
	append_send_queue(batch, 64);
	
	batch = m_new_fpga_transfer_batch();
	
	m_fpga_batch_append(&batch, 0x01);
	
	append_send_queue(batch, 256);
	
	batch = m_new_fpga_transfer_batch();
	
	m_fpga_batch_append(&batch, 0x90);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x0a);
	m_fpga_batch_append(&batch, 0x12);
	m_fpga_batch_append(&batch, 0x80);
	m_fpga_batch_append(&batch, 0x01);
	m_fpga_batch_append(&batch, 0xe0);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x01);
	
	append_send_queue(batch, 512);
	
}

int main(int argc, char** argv)
{
	srand(time(0));
    Verilated::commandArgs(argc, argv);
    Verilated::randReset(2);

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
	
	/*m_effect_desc *eff = create_hpf_eff_desc();
	
	m_eff_resource_report res = m_empty_fpga_resource_report();
	m_eff_resource_report local = m_empty_fpga_resource_report();
	
	m_fpga_transfer_batch batch = m_new_fpga_transfer_batch();
	
	m_fpga_transfer_batch_append_effect_desc(eff, &res, &local, &batch);
	
	m_eff_resource_report_integrate(&res, &local);
	
	local = m_empty_fpga_resource_report();
	
	eff = create_lpf_eff_desc();
	
	m_fpga_transfer_batch_append_effect_desc(eff, &res, &local, &batch);
	
	m_fpga_batch_append(&batch, COMMAND_SWAP_PIPELINES);
	append_send_queue(batch, 70);*/
	
	m_effect_desc *eff = m_read_eff_desc_from_file("verilator/bqd.eff");
	
	m_transformer trans;
	
	int ret_val = init_transformer_from_effect_desc(&trans, eff);
	
	m_fpga_transfer_batch batch = m_new_fpga_transfer_batch();
	
	m_eff_resource_report res;
	res.memory = 0;
	res.delays = 0;
	
	int pos = 0;
	
	m_fpga_batch_append_transformer(&batch, &trans, &res, &pos);
	
	m_fpga_batch_append(&batch, COMMAND_SWAP_PIPELINES);
	
	append_send_queue(batch, 70);
	
	batch = m_new_fpga_transfer_batch();
	
	m_fpga_batch_append(&batch, 0xE8);
	m_fpga_batch_append(&batch, 0x04);
	m_fpga_batch_append(&batch, 0x28);
	m_fpga_batch_append(&batch, 0xB4);
	m_fpga_batch_append(&batch, 0xE8);
	m_fpga_batch_append(&batch, 0x05);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0xE8);
	m_fpga_batch_append(&batch, 0x06);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0xE8);
	m_fpga_batch_append(&batch, 0x07);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0xE8);
	m_fpga_batch_append(&batch, 0x08);
	m_fpga_batch_append(&batch, 0x00);
	m_fpga_batch_append(&batch, 0x00);

	m_fpga_batch_append(&batch, COMMAND_COMMIT_REG_UPDATES);

	append_send_queue(batch, 1024);
	
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
			
			io.sample_in = (uint16_t)(roundf(sinf(6.28 * 1000.0f * t * ((float)samples_processed / (float)samples_to_process)) * 32767.0 * 0.5f));
			//io.sample_in = static_cast<int16_t>(in_samples[samples_processed]);
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
