verilator  src/*.v \
	--top-module top  --x-assign unique --x-initial unique -Wno-fatal -Isrc -Iinclude -cc -CFLAGS "-fpermissive -Wno-error"  -LDFLAGS "-lM" --trace-fst -exe verilator/sim_main.cpp verilator/sim_io.cpp \
	&& make -C obj_dir -j -f Vtop.mk Vtop
