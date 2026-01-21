verilator  src/pipeline.v 			\
		   src/lut.v 				\
		   src/pipeline_seq.v 		\
		   src/rr_arbiter.v 		\
		   src/lut_master.v 		\
		   src/controller.v 		\
		   src/sram.v 				\
		   src/delay_master.v 		\
		   src/spi.v 				\
		   src/i2s.v 				\
		   src/fifo.v 				\
		   src/mixer.v 				\
		   src/top.v 				\
		   src/seq.v 				\
		   src/engine_seq.v 		\
		   src/instr_dec.v 			\
	--top-module top  -Wno-fatal -Isrc -Iinclude -cc -CFLAGS -fpermissive -CFLAGS -Wno-error --trace-fst -exe verilator/sim_io.cpp verilator/sim_ctrl.cpp verilator/sim_main.cpp \
	&& make -C obj_dir -j -f Vtop.mk Vtop
