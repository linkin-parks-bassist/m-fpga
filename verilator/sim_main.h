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
#include "sim_main.h"
#include "sim_ctrl.h"
#include "sim_io.h"
#include "math.h"


#define MAX_SAMPLES		1024
//#define PRINT_STATE
//#define RUN_EMULATOR

#define DUMP_WAVEFORM

int tick();

extern Vtop* dut;

#endif
