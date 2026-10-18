# Changelog

## 0.1.0 (2026-10-18)

- SSTV receiver ported from slowrx (commit ca6d701) as a streaming state machine: VIS
  detection, FM demodulation with SNR-adaptive windows, slant correction (Hough transform),
  FSK ID; all 25 slowrx modes; byte-identical to slowrx at 44100 Hz for the 22 modes whose
  slowrx row is consistent, rate-generic (8000-48000 Hz).
- Corrections of three slowrx mode rows (Martin M3, Robot 72, Robot 24) after robot36 and
  QSSTV; defined behaviour for slowrx's out-of-bounds reads and its FSK ID hang.
- SSTV transmitter ported from pySSTV (commit d998fad) for all 25 modes, with Pillow's colour
  conversions; bit-identical to pySSTV for its 16 modes at their native width.
- Contract sstv-c1, ABI 1; fldigi-dsp-core 0.7.0 (FFT) pinned by dsp.lock.
