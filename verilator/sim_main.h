#ifndef SIM_MAIN_H_
#define SIM_MAIN_H_

#include <verilated.h>
#include "Vtop.h"
#include "verilated_fst_c.h"
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>

#include <libM/m_lib.h>

#include "sim_main.h"
//#include "sim_ctrl.h"
#include "sim_io.h"
#include "math.h"

#ifndef VERILATOR
#define VERILATOR
#endif

#define MAX_SAMPLES		2048
//#define RUN_EMULATOR

#define DUMP_WAVEFORM

int tick();

extern Vtop* dut;

#endif
