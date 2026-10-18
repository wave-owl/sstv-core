#!/usr/bin/env python3
"""Threshold curves: decode quality against signal-to-noise ratio, and false images on noise.

  python3 tools/threshold.py --build build [--rate 12000] [--modes M1,S1,R36,PD120,R8BW,P3]
                             [--snr 30,20,15,10,5,0,-5] [--seeds 3] [--noise-minutes 30]

For each mode and SNR (white Gaussian noise, SNR = signal power / noise power in 3 kHz) the
frozen test picture is sent through sstv-core's TX, noise is added, and sstv-core's RX (station
settings: timeout 5 s) decodes it. Reported per cell over the seeds: VIS detected (k/n), mean
region error (test_util.h region_error, levels), mean PSNR. Then --noise-minutes of pure noise
at three levels are decoded: any image is a false image. Prints Markdown tables.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ref_check import modes, read_info, read_ppm, read_wav, region_error, run, test_pattern, \
    write_ppm, write_wav  # noqa: E402


def add_noise(x, snr_db, rate, seed):
    rng = np.random.default_rng(seed)
    sig = np.sqrt(np.mean(x[x != 0].astype(float) ** 2))
    sigma = sig / math.sqrt(10 ** (snr_db / 10)) * math.sqrt(rate / 2 / 3000.0)
    y = x.astype(float) + rng.normal(0, sigma, x.size)
    return np.clip(np.round(y), -32768, 32767).astype(np.int16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default="build")
    ap.add_argument("--rate", type=int, default=12000)
    ap.add_argument("--modes", default="M1,S1,SDX,R36,R72,PD120,R8BW,P3,W2180")
    ap.add_argument("--snr", default="30,20,15,10,5,0,-5")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--noise-minutes", type=float, default=30)
    args = ap.parse_args()
    tool = os.path.join(os.path.abspath(args.build), "sstv_tool")
    rows = {m["short"]: m for m in modes(tool)}
    snrs = [float(v) for v in args.snr.split(",")]
    rate = args.rate
    print("| mode | " + " | ".join("%g dB" % s for s in snrs) + " |")
    print("|---|" + "---:|" * len(snrs))
    with tempfile.TemporaryDirectory() as tmp:
        for name in args.modes.split(","):
            m = rows[name]
            pat = os.path.join(tmp, name + ".ppm")
            wav = os.path.join(tmp, name + ".wav")
            write_ppm(pat, test_pattern(int(m["tx_width"]), int(m["lines"])))
            run([tool, "encode", name, str(rate), pat, wav, "DG6FL"])
            over, _ = read_wav(wav)
            over = np.concatenate([np.zeros(rate, np.int16), over, np.zeros(3 * rate, np.int16)])
            cells = []
            for snr in snrs:
                found, errs, psnrs = 0, [], []
                for seed in range(args.seeds):
                    nw = os.path.join(tmp, "n.wav")
                    write_wav(nw, add_noise(over, snr, rate, 100 + seed), rate)
                    pre = os.path.join(tmp, "d_")
                    for f in os.listdir(tmp):
                        if f.startswith("d_"):
                            os.remove(os.path.join(tmp, f))
                    run([tool, "decode", nw, pre, "5"])
                    if os.path.exists(pre + "0.ppm") and read_info(pre + "0.txt")["mode"] == name:
                        found += 1
                        img = read_ppm(pre + "0.ppm")
                        errs.append(region_error(m, img))
                        src = test_pattern(int(m["tx_width"]), int(m["lines"]))
                        xs = np.arange(img.shape[1]) * src.shape[1] // img.shape[1]
                        ref = src[:, xs].astype(float)
                        if m["color"] == "3":       # B/W: Pillow's L, as transmitted
                            ref = np.repeat(((ref[..., 0] * 19595 + ref[..., 1] * 38470 +
                                              ref[..., 2] * 7471 + 0x8000) // 65536)[..., None],
                                            3, axis=2)
                        se = np.mean((img.astype(float) - ref) ** 2)
                        psnrs.append(99.0 if se == 0 else 10 * math.log10(255 ** 2 / se))
                if found:
                    cells.append("%d/%d, %.0f lv, %.1f dB" % (found, args.seeds, np.mean(errs),
                                                              np.mean(psnrs)))
                else:
                    cells.append("0/%d" % args.seeds)
            print("| %s | %s |" % (name, " | ".join(cells)), flush=True)
        # false images on noise
        n = int(args.noise_minutes * 60 * rate)
        print("\n| noise level (rms, of full scale) | minutes | images |")
        print("|---:|---:|---:|")
        for level in (0.01, 0.1, 0.3):
            rng = np.random.default_rng(7)
            x = np.clip(np.round(rng.normal(0, level * 32768, n)), -32768, 32767).astype(np.int16)
            nw = os.path.join(tmp, "noise.wav")
            write_wav(nw, x, rate)
            out = run([tool, "decode", nw, os.path.join(tmp, "z_"), "5"])
            images = int(out.strip().split("\n")[-1].split()[-1])
            print("| %.2f | %g | %d |" % (level, args.noise_minutes, images), flush=True)


if __name__ == "__main__":
    main()
