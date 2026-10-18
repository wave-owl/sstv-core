# Tools and reference harnesses

| Tool | What it does |
|---|---|
| `sstv_tool.c` (`make tool`) | Command line front end: mode table, test picture, encode to WAV/raw, decode WAV, loopback of every mode. Used by the Python scripts. |
| `slowrx/` | Headless build of slowrx's decoder (`ref/slowrx`, commit `ca6d701`): `vis.c`, `video.c`, `sync.c`, `fsk.c`, `modespec.c` compiled unmodified apart from `slowrx-bench.patch` against stand-ins for GTK and ALSA (`shim/`) and a driver that replaces `slowrx.c`'s listener thread and `pcm.c`'s capture with a WAV file (`driver.c`). Needs FFTW3. `make -C tools/slowrx BUILD=$PWD/build/slowrx`. |
| `robot36/` | Headless build of robot36's decoder (`ref/robot36`, commit `2af2390`): the decoder classes compiled with stand-ins for four `android.graphics` classes (`stub/`) and a driver that feeds a WAV file through `Decoder` as robot36's `MainActivity` does (`src/xdsopl/robot36/Robot36Ref.java`). Needs a JDK. `sh tools/robot36/build.sh`. |
| `ref_check.py` | Differential tests against slowrx (byte-identical pictures, clean / 12 dB SNR / +0.15 % clock), pySSTV (bit-identical samples at five rates) and robot36 (mode detected, region error <= 25 levels). `make ref`. |
| `threshold.py` | Decode quality against SNR and false images on noise (README, "Verification"). |
| `check_dsp.sh` | The `dsp/` pin gate (wave-owl R-920.3, INV-920). |

`slowrx-bench.patch` gives defined behaviour where slowrx reads outside an array or never
terminates (sstv-core's U1, U2, U6, U7, see `src/sstv_rx.c`):

- `vis.c:67` reads `HedrBuf[-1]` when `HedrPtr` is 0 -> the cyclically previous entry;
- `sync.c:61` compares against `lines[0][-60]` before the first accumulator hit -> 0;
- `video.c:50` the pixel grid is read one entry past its end after the last pixel -> a
  sentinel entry that never matches;
- `fsk.c:48-51` loops forever when the capture-buffer pointer is below 1455 samples while in
  FSK sync (the pointer does not move inside that branch) -> branch removed; the FSK window
  can then reach up to 485 samples before slowrx's 4096-sample buffer, so the driver backs
  `pcm.Buffer` by the whole file (`readPcm` advances a pointer instead of shifting a copy:
  the same `Buffer[0 .. 4095]` at every call, and the stream's samples before it), and the
  patch computes the window index signed (slowrx's `WindowPtr + i - 485` with the unsigned
  `i` would wrap to an index 4 GiB away).

The driver allocates slowrx's `HasSync` twice as large as slowrx does, so reads past the
stored sync samples (the slant-corrected rate at the end of an image) see zeros (U3); it
clamps `GetBin` of a negative frequency at 0 (U5) and maps `clip(NaN)` to 0 (U4), as the
core does.
