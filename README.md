# sstv-core

SSTV (slow-scan television) receiver and transmitter in C99, for every mode of slowrx: Martin
M1-M4, Scottie S1/S2/DX, Robot 72/36/24, Robot 24/12/8 B/W, PD-50/90/120/160/180/240/290,
Pasokon P3/P5/P7 and Wraase SC-2 120/180 (25 modes), receive and transmit.

- **Receiver**: a port of [slowrx](https://github.com/wave-owl/slowrx) by Oona Raisanen OH2EIQ
  (ISC, commit `ca6d701`) — VIS detection (`vis.c`), FM demodulation with SNR-adaptive
  windows and the pixel grid (`video.c`), slant correction by a Hough transform (`sync.c`),
  FSK ID (`fsk.c`), the mode table (`modespec.c`) — turned into a streaming state machine.
  At 44100 Hz (slowrx's rate) it reproduces slowrx's pictures **byte for byte**; at any rate
  from 8000 to 48000 Hz every slowrx sample count is scaled (the station runs it at 12000 Hz).
- **Transmitter**: a port of [pySSTV](https://github.com/wave-owl/pySSTV) by Andras
  Veres-Szentkiralyi (MIT, commit `d998fad`) with Pillow's colour conversions: VOX tones,
  calibration header, VIS, scan lines, FSK ID. For pySSTV's modes it reproduces pySSTV's
  samples **bit for bit**; the other modes use the same segment model with the timing of the
  mode table.
- **Oracle**: [robot36](https://github.com/wave-owl/robot36) by Ahmet Inan (commit `2af2390`),
  an independent decoder, decodes the transmitter's output in every mode it implements.

Contract `sstv-c1`, ABI 1, capability `tx=1`. The FFT comes from
[fldigi-dsp-core](https://github.com/wave-owl/fldigi-dsp-core) `v0.7.0` (submodule `dsp/`,
pinned by commit and content checksum in `dsp.lock`). Used by
[Wave Owl](https://github.com/wave-owl) (`src/sstv_native.py`).

## Build and test

```sh
git clone --recurse-submodules https://github.com/wave-owl/sstv-core
make            # build/libsstv.a and build/libsstv.{so,dylib}
make test       # C tests: API, mode table, VIS, noise, slant, partial images, loopback goldens
make sanitize   # the tests under ASan + UBSan
make bench      # CPU and memory per mode
make ref        # differential tests against slowrx, pySSTV and robot36 (tools/README.md)
```

Plain make, C99, `-Wall -Wextra -Werror -pedantic`, no dependencies besides libm. The
reference tests need FFTW3 (slowrx), Python 3 with numpy and Pillow (pySSTV) and a JDK
(robot36); CI runs everything on Linux and macOS.

## Modes

| id | mode | VIS | picture | line ms | over s (12 kHz, FSK ID) | colour | TX reference |
|---:|---|---|---|---:|---:|---|---|
| 1 | Martin M1 | 0x2c | 320x256 | 446.446 | 116.3 | GBR | pySSTV, bit-identical |
| 2 | Martin M2 | 0x28 | 320x256 | 226.799 | 60.0 | GBR | pySSTV (160 px, columns doubled) |
| 3 | Martin M3 | 0x24 | 320x128 (x2) | 446.446 | 59.1 | GBR | M1 lines, 128 rows |
| 4 | Martin M4 | 0x20 | 320x128 (x2) | 226.799 | 31.0 | GBR | M2 lines, 128 rows |
| 5 | Scottie S1 | 0x3c | 320x256 | 428.38 | 111.6 | GBR | pySSTV, bit-identical |
| 6 | Scottie S2 | 0x38 | 320x256 | 277.692 | 73.1 | GBR | pySSTV (160 px, columns doubled) |
| 7 | Scottie DX | 0x4c | 320x256 | 1050.3 | 270.8 | GBR | pySSTV, bit-identical |
| 8 | Robot 72 | 0x0c | 320x240 | 300 | 74.0 | YUV | robot36/QSSTV layout |
| 9 | Robot 36 | 0x08 | 320x240 | 150 | 38.0 | YUV | pySSTV, bit-identical |
| 10 | Robot 24 | 0x04 | 160x120 | 200 | 26.0 | YUV | QSSTV layout |
| 11 | Robot 24 B/W | 0x0a | 320x240 | 100 | 26.0 | B/W | pySSTV, bit-identical |
| 12 | Robot 12 B/W | 0x06 | 320x120 (x2) | 100 | 14.0 | B/W | R24BW lines, odd VIS parity |
| 13 | Robot 8 B/W | 0x02 | 320x120 (x2) | 66.9 | 10.0 | B/W | pySSTV (160 px, 59.9 ms scan) |
| 14 | PD-50 | 0x5d | 320x256 | 388.16 | 51.7 | YUV | pySSTV PD layout |
| 15 | PD-90 | 0x63 | 320x256 | 703.04 | 92.0 | YUV | pySSTV, bit-identical |
| 16 | PD-120 | 0x5f | 640x496 | 508.48 | 128.1 | YUV | pySSTV, bit-identical |
| 17 | PD-160 | 0x62 | 512x400 | 804.416 | 162.8 | YUV | pySSTV, bit-identical |
| 18 | PD-180 | 0x60 | 640x496 | 754.24 | 189.0 | YUV | pySSTV, bit-identical |
| 19 | PD-240 | 0x61 | 640x496 | 1000 | 250.0 | YUV | pySSTV, bit-identical |
| 20 | PD-290 | 0x5e | 800x616 | 937.28 | 290.6 | YUV | pySSTV, bit-identical |
| 21 | Pasokon P3 | 0x71 | 640x496 | 409.375 | 205.0 | RGB | pySSTV, bit-identical |
| 22 | Pasokon P5 | 0x72 | 640x496 | 614.065 | 306.5 | RGB | pySSTV, bit-identical |
| 23 | Pasokon P7 | 0x73 | 640x496 | 818.747 | 408.1 | RGB | pySSTV, bit-identical |
| 24 | Wraase SC-2 120 | 0x3f | 320x256 | 475.53 | 123.7 | RGB | pySSTV, bit-identical |
| 25 | Wraase SC-2 180 | 0x37 | 320x256 | 711.023 | 184.0 | RGB | pySSTV, bit-identical |

"(x2)": each row is shown twice (slowrx's LineHeight 2). The transmitter takes a
`width x lines` RGB picture. PD modes send two rows per sync pulse.

### Corrections of slowrx's mode table

Three rows of slowrx's `modespec.c` contradict the mode they name; the core uses corrected
rows (each checked by `test_line_times`: the transmitted line equals the table's line time):

- **C1 Martin M3** (`modespec.c:65-76`): pixel time 0.2288 ms is Martin M2's; with its own
  line time of 446.446 ms (Martin M1's) the line only adds up with 0.4576 ms.
- **C2 Robot 72** (`modespec.c:130-141`): slowrx decodes three equal 92 ms channels; the mode
  sends Y 138 ms and V, U 69 ms each, every chroma channel after a 4.5 ms separator and a
  1.5 ms porch (robot36 `Robot_72_Color.java:24-31`, QSSTV `sstvparam.cpp:319`).
- **C3 Robot 24** (`modespec.c:156-167`): slowrx's row is Robot 36's. The mode is 160x120
  with 200 ms lines in Robot 72's layout: 9 + 3 + 88 + 6 + 44 + 6 + 44 ms (QSSTV
  `sstvparam.cpp:317`, `sstvtx.cpp:109-110`).

For these three modes the receiver does not match slowrx (whose decode is wrong); Robot 72 is
verified against robot36, Martin M3 and Robot 24 by the transmitter loopback.

### Defined behaviour where slowrx has none

slowrx reads outside an array in four places and never terminates in one; the core defines
each (`src/sstv_rx.c`, U1-U7; the reference harness applies the same definitions,
`tools/README.md`): `HedrBuf[-1]` (U1), `lines[0][-60]` (U2), `HasSync[]` past the stored
samples (U3), `clip(NaN)` (U4), `GetBin` of a negative frequency (U5), the pixel grid read past
its end (U6), and the FSK ID loop that hangs when slowrx's capture-buffer pointer is below
1455 samples while in sync (U7).

## Sample rate

At 44100 Hz every sample count, FFT length, window and expression is slowrx's. At another
rate `r` each slowrx sample count `c` becomes `lround(c * r / 44100)` and each FFT length the
smallest power of two that holds the scaled length (VIS/FSK: 2048 -> 1024 at 12 kHz, video:
1024 -> 512); all times in seconds (mode table, VIS bits, FSK bits) are unchanged. The
video FFTs are evaluated only at the bins slowrx reads (direct DFT, the same coefficients;
`window_dft`). At 12000 Hz the loopback region error of every mode stays within 5.5 levels.

## Station extensions (not in slowrx)

- `timeout_s` (default 0 = slowrx: wait for the whole image time): an image ends as partial
  when slowrx's SNR estimate, as a signal-to-noise power ratio limited to [0, 1] and averaged
  over 0.5 s, stays below 0.1 (-10 dB) for `timeout_s`; rows drawn after the signal was gone
  are blanked. Ten minutes of noise never start an image.
- `slant_partial` (default 1): slant correction also for a partial image of at least 16 rows
  (slowrx never corrects an aborted image).
- `sstv_rx_flush`: end of input ends an image in progress as partial.
- Two picture buffers: the arriving image (rows readable as they are drawn) and the last
  finished, slant-corrected image.

## API

See `include/sstv.h`. Receiver: `sstv_rx_create(cfg)`, `sstv_rx_process(rx, x, n, events,
max)` with events `VIS` (mode, header shift), `LINE` (row drawn) and `IMAGE` (finished:
complete or partial, rows, FSK ID), `sstv_rx_image` / `sstv_rx_get_image_line` for the finished
picture, `sstv_rx_current` / `sstv_rx_get_line` for the arriving one. Transmitter:
`sstv_tx_create(cfg)` (mode, rate, VOX, FSK ID), `sstv_tx_put_image`, `sstv_tx_render`,
`sstv_tx_total_samples`. `sstv_sizeof()` lets bindings check their structure mirrors. All
memory is allocated at create; no globals; one instance per thread.

## Verification

| Rung | Result |
|---|---|
| Receiver vs slowrx (44100 Hz, `make ref`) | 22 modes x {clean, 12 dB SNR, +0.15 % clock}: pictures byte-identical, rate, skip and FSK ID identical (66 of 66). M3, R72, R24: corrected rows, not compared. |
| Transmitter vs pySSTV (`make ref`) | 16 pySSTV modes at 48000, 44100, 12000, 11025 and 8000 Hz: same length, bit-identical samples. M2, S2, R8BW (doubled columns): same length, max difference 1.3e-6. |
| Cross-decode by robot36 (`make ref`) | robot36 detects the mode and decodes the picture in all 15 modes it implements (M1, M2, S1, S2, SDX, R36, R72, PD50-PD290, W2180). |
| TX -> RX loopback (`make test`) | all 25 modes at 12000 Hz: mode, 100 % rows, FSK ID, region error <= 5.5 levels; goldens. |
| VIS | correct VIS of every mode decoded; flipped parity, unmapped code, noise, silence: no image; +60 Hz header shift detected; R12BW's odd parity. |
| Slant | +0.15 % clock: rate corrected to within 2e-4, PSNR 19 dB vs 4 dB uncorrected. |
| Noise | 10 min white noise and 10 min silence: no VIS. Threshold curves: `tools/threshold.py` (below). |
| Sanitizers | `make sanitize`: ASan + UBSan clean. |

Threshold curves at 12000 Hz (white noise, SNR in 3 kHz; per cell: images decoded / runs,
mean region error in levels, mean PSNR):

| mode | 30 dB | 20 dB | 15 dB | 10 dB | 5 dB | 0 dB | -5 dB |
|---|---:|---:|---:|---:|---:|---:|---:|
| M1 | 2/2, 4 lv, 27.1 dB | 2/2, 3 lv, 25.6 dB | 2/2, 6 lv, 24.2 dB | 2/2, 6 lv, 21.8 dB | 2/2, 5 lv, 20.3 dB | 2/2, 8 lv, 16.1 dB | 2/2, 167 lv, 4.4 dB |
| S1 | 2/2, 4 lv, 17.2 dB | 2/2, 3 lv, 16.3 dB | 2/2, 6 lv, 13.7 dB | 2/2, 6 lv, 13.6 dB | 2/2, 5 lv, 14.8 dB | 2/2, 6 lv, 14.7 dB | 2/2, 58 lv, 8.0 dB |
| SDX | 2/2, 1 lv, 19.5 dB | 2/2, 2 lv, 19.8 dB | 2/2, 3 lv, 18.8 dB | 2/2, 3 lv, 19.3 dB | 2/2, 2 lv, 18.8 dB | 2/2, 2 lv, 17.3 dB | 2/2, 10 lv, 13.5 dB |
| R36 | 2/2, 6 lv, 20.2 dB | 2/2, 8 lv, 19.4 dB | 2/2, 48 lv, 10.4 dB | 2/2, 14 lv, 16.8 dB | 2/2, 71 lv, 12.3 dB | 2/2, 114 lv, 9.6 dB | 2/2, 93 lv, 7.7 dB |
| R72 | 2/2, 6 lv, 21.2 dB | 2/2, 8 lv, 20.2 dB | 2/2, 14 lv, 12.7 dB | 2/2, 32 lv, 11.8 dB | 2/2, 38 lv, 12.0 dB | 2/2, 48 lv, 10.9 dB | 2/2, 60 lv, 8.9 dB |
| PD120 | 2/2, 6 lv, 17.7 dB | 2/2, 9 lv, 17.3 dB | 2/2, 24 lv, 9.7 dB | 2/2, 41 lv, 12.6 dB | 2/2, 33 lv, 8.8 dB | 2/2, 16 lv, 12.1 dB | 2/2, 43 lv, 7.9 dB |
| R8BW | 2/2, 3 lv, 18.5 dB | 2/2, 3 lv, 18.2 dB | 2/2, 5 lv, 17.8 dB | 2/2, 5 lv, 16.4 dB | 2/2, 4 lv, 14.8 dB | 2/2, 7 lv, 12.5 dB | 2/2, 44 lv, 7.2 dB |
| P3 | 2/2, 3 lv, 20.2 dB | 2/2, 3 lv, 20.0 dB | 2/2, 5 lv, 19.6 dB | 2/2, 5 lv, 19.4 dB | 2/2, 5 lv, 18.5 dB | 2/2, 7 lv, 15.4 dB | 2/2, 135 lv, 6.9 dB |
| W2180 | 2/2, 3 lv, 21.6 dB | 2/2, 3 lv, 21.3 dB | 2/2, 6 lv, 19.8 dB | 2/2, 5 lv, 20.6 dB | 2/2, 5 lv, 19.0 dB | 2/2, 6 lv, 16.9 dB | 2/2, 134 lv, 7.5 dB |

False images on white noise (station settings, timeout 5 s):

| noise level (rms, of full scale) | minutes | images |
|---:|---:|---:|
| 0.01 | 30 | 0 |
| 0.10 | 30 | 0 |
| 0.30 | 30 | 0 |

(`python3 tools/threshold.py --build build`, 2 runs per cell.) Martin, Scottie, Pasokon,
Wraase and the B/W modes stay within 10 levels down to 0 dB. The YUV modes
(Robot, PD) are as fragile as slowrx itself from about 15 dB down: slowrx's line-start finder
(`sync.c:96-127`) sometimes locks the line start 1-8 ms off under noise, which shifts the
picture sideways (the same pictures as slowrx, byte for byte, at 12 dB).

Benchmark (`make bench`, Apple M-series, one core): receiver 0.87 % of real time at
12000 Hz (2.2 % at 44100 Hz), footprint 9.1 MB at 12000 Hz (21.9 MB at 44100 Hz, dominated by
slowrx's stored luminance of the longest mode, Pasokon P7); transmitter 0.01 % of real time.

## Licence

ISC (see `LICENSE` and `NOTICE`: slowrx ISC, pySSTV MIT, Pillow MIT-CMU). The built library
links fldigi-dsp-core's FFT, which is GPL-3.0-or-later; a binary containing it is distributed
under GPL-3.0-or-later as a whole.
