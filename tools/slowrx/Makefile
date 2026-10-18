# Headless build of slowrx's decoder for the reference differential tests (not part of
# libsstv). Needs FFTW3 (Debian/Ubuntu: libfftw3-dev, Homebrew: fftw).
#   make BUILD=/abs/dir   -> $(BUILD)/slowrx_ref
SLOWRX ?= ../../ref/slowrx
BUILD ?= build
CC ?= cc
FFTW_CFLAGS := $(shell pkg-config --cflags fftw3 2>/dev/null)
FFTW_LIBS := $(shell pkg-config --libs fftw3 2>/dev/null || echo -lfftw3)
SRC = vis.c video.c sync.c fsk.c modespec.c
# -ffp-contract=off: the same arithmetic as sstv-core (no fused multiply-add on arm64)
CFLAGS ?= -O2
FLAGS = -std=gnu99 -w -ffp-contract=off -Ishim -I$(BUILD)/src $(FFTW_CFLAGS)

$(BUILD)/slowrx_ref: driver.c slowrx-bench.patch shim/gtk/gtk.h $(addprefix $(SLOWRX)/,$(SRC))
	rm -rf $(BUILD)/src && mkdir -p $(BUILD)/src
	cp $(addprefix $(SLOWRX)/,$(SRC) common.h) $(BUILD)/src/
	cd $(BUILD)/src && patch -s -p1 < $(abspath slowrx-bench.patch)
	$(CC) $(FLAGS) $(CFLAGS) driver.c $(addprefix $(BUILD)/src/,$(SRC)) -o $@ $(FFTW_LIBS) -lm -lpthread
