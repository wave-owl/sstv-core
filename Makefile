# sstv-core -- plain make, C99, no dependencies besides libm and the pinned
# fldigi-dsp-core submodule dsp/ (its FFT only).
#   make            static + shared library into $(BUILD) (after the dsp/ pin check)
#   make test       build and run the C tests (modes, receiver, transmitter, goldens)
#   make sanitize   the C tests under AddressSanitizer + UndefinedBehaviorSanitizer
#   make bench      CPU and memory benchmark (per mode, receiver and transmitter)
#   make check-dsp  only the dsp/ pin check (dsp.lock, wave-owl R-920.3)
#   make ref        reference differential tests: slowrx (needs FFTW3), pySSTV (Python 3 +
#                   Pillow), robot36 (JDK 17+); see tools/README.md
#   make BUILD=/abs/dir shared   out-of-tree build (used by wave-owl)
BUILD ?= build
CC ?= cc
CFLAGS ?= -O2
DSP ?= dsp
# -ffp-contract=off: no fused multiply-add, the arithmetic follows slowrx's and pySSTV's
# expression order (slowrx was built with gcc's default on x86-64, which does not fuse).
CORE_FLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -ffp-contract=off -fPIC -Iinclude \
             -I$(DSP)/include
LDLIBS = -lm
TEST_FLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -ffp-contract=off -Iinclude -I$(DSP)/include

UNAME := $(shell uname)
ifeq ($(UNAME),Darwin)
  SHLIB = libsstv.dylib
  SHFLAGS = -dynamiclib -install_name @rpath/libsstv.dylib
else
  SHLIB = libsstv.so
  SHFLAGS = -shared -Wl,-soname,libsstv.so
endif

DSP_SRC = fdsp_gfft
HDRS = include/sstv.h src/sstv_internal.h $(DSP)/include/fldigi_dsp.h
OBJ = $(BUILD)/sstv_modes.o $(BUILD)/sstv_pil.o $(BUILD)/sstv_rx.o $(BUILD)/sstv_tx.o \
      $(patsubst %,$(BUILD)/dsp_%.o,$(DSP_SRC))

all: static shared
static: $(BUILD)/libsstv.a
shared: $(BUILD)/$(SHLIB)

$(BUILD):
	mkdir -p $(BUILD)

# The pin gate runs before anything is compiled and again whenever dsp/ or dsp.lock change.
$(BUILD)/dsp.ok: dsp.lock tools/check_dsp.sh $(wildcard $(DSP)/include/*.h $(DSP)/src/*.c) | $(BUILD)
	sh tools/check_dsp.sh dsp.lock $(DSP)
	touch $@

check-dsp:
	sh tools/check_dsp.sh dsp.lock $(DSP)

$(BUILD)/%.o: src/%.c $(HDRS) $(BUILD)/dsp.ok
	$(CC) $(CORE_FLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/dsp_%.o: $(DSP)/src/%.c $(HDRS) $(BUILD)/dsp.ok
	$(CC) $(CORE_FLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/libsstv.a: $(OBJ)
	ar rcs $@ $^

$(BUILD)/$(SHLIB): $(OBJ)
	$(CC) $(SHFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/test_sstv: tests/test_sstv.c tests/test_util.h $(BUILD)/libsstv.a
	$(CC) $(TEST_FLAGS) $(CFLAGS) $< $(BUILD)/libsstv.a -o $@ $(LDLIBS)

$(BUILD)/sstv_tool: tools/sstv_tool.c tests/test_util.h $(BUILD)/libsstv.a
	$(CC) $(TEST_FLAGS) $(CFLAGS) $< $(BUILD)/libsstv.a -o $@ $(LDLIBS)

test: $(BUILD)/test_sstv
	$(BUILD)/test_sstv tests/golden

SAN_FLAGS = -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all

sanitize:
	$(MAKE) --no-print-directory BUILD=$(BUILD)/san CFLAGS="$(SAN_FLAGS)" \
		LDLIBS="$(LDLIBS) $(SAN_FLAGS)" test

$(BUILD)/bench_sstv: tests/bench_sstv.c tests/test_util.h $(BUILD)/libsstv.a
	$(CC) $(TEST_FLAGS) $(CFLAGS) $< $(BUILD)/libsstv.a -o $@ $(LDLIBS)

bench: $(BUILD)/bench_sstv
	$(BUILD)/bench_sstv

tool: $(BUILD)/sstv_tool

ref: $(BUILD)/sstv_tool
	$(MAKE) -C tools/slowrx BUILD=$(abspath $(BUILD))/slowrx
	python3 tools/ref_check.py --build $(BUILD)

clean:
	rm -rf $(BUILD)

.PHONY: all static shared check-dsp test sanitize bench tool ref clean
