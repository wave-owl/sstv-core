#!/usr/bin/env python3
"""Reference differential tests of sstv-core against slowrx, pySSTV and robot36.

  python3 tools/ref_check.py --build build [--only slowrx,pysstv,robot36] [--modes M1,PD120]
                             [--quick] [--json out.json]

slowrx  (ref/slowrx, built headless by tools/slowrx/Makefile into BUILD/slowrx/slowrx_ref):
        sstv-core's TX renders the frozen test picture at 44100 Hz; slowrx and sstv-core's
        RX decode the same int16 WAV, clean, with white noise (12 dB SNR in 3 kHz) and with a
        +0.15 % sample clock error (resampled). Expected: byte-identical pictures, identical
        rate/skip/FSK ID, for every mode whose row sstv-core does not correct (M3, R72, R24
        follow the corrected table; their slowrx decodes are reported, not compared).
pySSTV  (ref/pysstv, needs Pillow): gen_values() of every pySSTV mode against sstv-core's TX at
        48000, 44100, 12000, 11025 and 8000 Hz, with an FSK ID. Expected: the same number of
        samples and max |difference| <= 1e-9 (pySSTV's own test tolerance, test_sstv.py:52);
        in practice bit-identical.
        Martin M2, Scottie S2 and Robot 8 B/W: pySSTV sends 160 pixels (and a 60 ms scan for
        Robot 8 B/W); compared with a 160-pixel picture whose columns sstv-core gets doubled
        (and pySSTV's Robot8BW.SCAN set to 59.9); the split segments add the phase offset
        twice as often, tolerance 1e-5.
robot36 (ref/robot36, needs a JDK): robot36's decoder, built headless by
        tools/robot36/build.sh, decodes sstv-core's TX for every mode robot36 implements
        (robot36 has no encoder). Expected: robot36 detects the mode and the region error of
        its picture (bars and colour blocks of the test picture) is at most 25 levels.
"""
import argparse
import json
import math
import os
import random
import struct
import subprocess
import sys
import tempfile
import wave

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORRECTED = {"M3", "R72", "R24"}
ROBOT36_NAMES = {   # robot36 Decoder.java:104-127
    "M1": "Martin 1", "M2": "Martin 2", "S1": "Scottie 1", "S2": "Scottie 2",
    "SDX": "Scottie DX", "R36": "Robot 36 Color", "R72": "Robot 72 Color", "PD50": "PD 50",
    "PD90": "PD 90", "PD120": "PD 120", "PD160": "PD 160", "PD180": "PD 180",
    "PD240": "PD 240", "PD290": "PD 290", "W2180": "Wraase SC2\u2013180"}
ROBOT36_MODES = set(ROBOT36_NAMES)
PYSSTV_CLASSES = {
    "M1": ("color", "MartinM1"), "M2": ("color", "MartinM2"), "S1": ("color", "ScottieS1"),
    "S2": ("color", "ScottieS2"), "SDX": ("color", "ScottieDX"), "R36": ("color", "Robot36"),
    "P3": ("color", "PasokonP3"), "P5": ("color", "PasokonP5"), "P7": ("color", "PasokonP7"),
    "PD90": ("color", "PD90"), "PD120": ("color", "PD120"), "PD160": ("color", "PD160"),
    "PD180": ("color", "PD180"), "PD240": ("color", "PD240"), "PD290": ("color", "PD290"),
    "W2120": ("color", "WraaseSC2120"), "W2180": ("color", "WraaseSC2180"),
    "R8BW": ("grayscale", "Robot8BW"), "R24BW": ("grayscale", "Robot24BW"),
}
HALF_WIDTH = {"M2", "S2", "R8BW"}


def test_pattern(w, h):
    """tests/test_util.h test_pattern()."""
    bars = [(255, 255, 255), (255, 255, 0), (0, 255, 255), (0, 255, 0), (255, 0, 255),
            (255, 0, 0), (0, 0, 255), (0, 0, 0)]
    block = w // 20 if w // 20 > 2 else 2
    img = np.zeros((h, w, 3), dtype=np.uint8)
    for y in range(h):
        for x in range(w):
            if y < h // 3:
                img[y, x] = bars[x * 8 // w]
            elif y < 2 * h // 3:
                img[y, x] = 255 if ((x // block) + (y // block)) % 2 else 0
            elif x < w // 2:
                img[y, x] = x * 255 // (w // 2 - 1)
            else:
                k = (x - w // 2) * 3 // (w - w // 2)
                img[y, x] = [220 if k == c else 30 for c in range(3)]
    return img


def expected(mode_row, rgb):
    """test_util.h expected_rgb(): Pillow's L for the B/W modes."""
    if mode_row["color"] == "3":
        v = (int(rgb[0]) * 19595 + int(rgb[1]) * 38470 + int(rgb[2]) * 7471 + 0x8000) >> 16
        return np.array([v, v, v], dtype=float)
    return rgb.astype(float)


def region_error(mode_row, dec):
    """test_util.h region_error(): worst |mean - expected| over the eight bars and three
    colour blocks of the test picture, each inset by 20 %."""
    h, w = dec.shape[:2]
    tw = int(mode_row["tx_width"])
    src = test_pattern(tw, h)
    worst = 0.0
    for r in range(11):
        if r < 8:
            x0, x1, y0, y1 = (r + 0.2) / 8 * w, (r + 0.8) / 8 * w, 0.1 * h, 0.23 * h
        else:
            x0 = (0.5 + (r - 8 + 0.2) / 6) * w
            x1 = (0.5 + (r - 8 + 0.8) / 6) * w
            y0, y1 = 0.75 * h, 0.92 * h
        region = dec[int(y0):int(y1), int(x0):int(x1)].reshape(-1, 3).astype(float)
        cx, cy = int((x0 + x1) / 2 * tw / w), int((y0 + y1) / 2)
        e = np.max(np.abs(region.mean(axis=0) - expected(mode_row, src[cy, cx])))
        worst = max(worst, float(e))
    return worst


def write_ppm(path, img):
    h, w, _ = img.shape
    with open(path, "wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (w, h))
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


def read_ppm(path):
    with open(path, "rb") as f:
        data = f.read()
    parts = data.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    return np.frombuffer(parts[3], dtype=np.uint8).reshape(h, w, 3)


def read_info(path):
    info = {}
    with open(path) as f:
        for line in f:
            k, _, v = line.rstrip("\n").partition("=")
            info[k] = v
    return info


def read_wav(path):
    with wave.open(path) as w:
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16), w.getframerate()


def write_wav(path, samples, rate):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(np.asarray(samples, dtype=np.int16).tobytes())


def modes(tool):
    out = subprocess.run([tool, "modes"], check=True, capture_output=True, text=True).stdout
    lines = out.strip().split("\n")
    keys = lines[0].split("\t")
    return [dict(zip(keys, line.split("\t"))) for line in lines[1:]]


def run(cmd):
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def variants(clean, rate, seed):
    """clean / 12 dB SNR (white noise in 3 kHz) / +0.15 % clock error."""
    x = clean.astype(np.float64)
    rng = np.random.default_rng(seed)
    sig = np.sqrt(np.mean(x[x != 0] ** 2))
    noise_bw = 3000.0
    sigma = sig / math.sqrt(10 ** (12 / 10)) * math.sqrt(rate / 2 / noise_bw)
    noisy = np.clip(np.round(x + rng.normal(0, sigma, x.size)), -32768, 32767)
    n_new = int(round(x.size * 1.0015))
    t = np.arange(n_new) / 1.0015
    slant = np.clip(np.round(np.interp(t, np.arange(x.size), x)), -32768, 32767)
    return {"clean": clean, "noise12": noisy.astype(np.int16), "clock+0.15%": slant.astype(np.int16)}


def check_slowrx(args, tool, mode_rows, tmp, report):
    ref = os.path.join(args.build, "slowrx", "slowrx_ref")
    if not os.path.exists(ref):
        print("slowrx: %s missing (make ref)" % ref)
        report["slowrx"] = "missing"
        return False
    ok = True
    res = report.setdefault("slowrx", {})
    for m in mode_rows:
        name = m["short"]
        if args.modes and name not in args.modes:
            continue
        pat = os.path.join(tmp, name + ".ppm")
        wav = os.path.join(tmp, name + ".wav")
        run([tool, "pattern", name, pat])
        run([tool, "encode", name, "44100", pat, wav, "DG6FL"])
        over, rate = read_wav(wav)
        padded = np.concatenate([np.zeros(44100, np.int16), over, np.zeros(2 * 44100, np.int16)])
        for vname, audio in variants(padded, rate, 1234).items():
            if args.quick and vname != "clean":
                continue
            vw = os.path.join(tmp, "%s_%s.wav" % (name, vname.replace("%", "")))
            write_wav(vw, audio, rate)
            pr = os.path.join(tmp, "r_%s_%s_" % (name, vname.replace("%", "")))
            po = os.path.join(tmp, "o_%s_%s_" % (name, vname.replace("%", "")))
            run([ref, vw, pr])
            run([tool, "decode", vw, po])
            entry = {}
            if not os.path.exists(pr + "0.ppm") or not os.path.exists(po + "0.ppm"):
                entry["result"] = "no image (slowrx %s, sstv-core %s)" % (
                    os.path.exists(pr + "0.ppm"), os.path.exists(po + "0.ppm"))
                ok = ok and name in CORRECTED
            else:
                a, b = read_ppm(pr + "0.ppm"), read_ppm(po + "0.ppm")
                ia, ib = read_info(pr + "0.txt"), read_info(po + "0.txt")
                same_meta = (ia["mode"] == ib["mode"] and abs(float(ia["rate"]) -
                             float(ib["rate"])) < 1e-6 and ia["skip"] == ib["skip"] and
                             ia["fsk_id"] == ib["fsk_id"])
                if a.shape == b.shape:
                    diff = np.abs(a.astype(int) - b.astype(int))
                    entry["differing_bytes"] = int(np.count_nonzero(diff))
                    entry["max_diff"] = int(diff.max())
                else:
                    entry["differing_bytes"] = -1
                entry["identical"] = bool(entry["differing_bytes"] == 0 and same_meta)
                entry["slowrx"] = {k: ia[k] for k in ("mode", "rate", "skip", "fsk_id")}
                entry["sstv_core"] = {k: ib[k] for k in ("mode", "rate", "skip", "fsk_id")}
                if name in CORRECTED:
                    entry["result"] = "corrected row, not compared"
                else:
                    entry["result"] = "identical" if entry["identical"] else "DIFFERENT"
                    ok = ok and entry["identical"]
            res["%s/%s" % (name, vname)] = entry
            print("slowrx %-6s %-12s %s" % (name, vname, entry.get("result")), flush=True)
    return ok


def check_pysstv(args, tool, mode_rows, tmp, report):
    sys.dont_write_bytecode = True          # keep the pinned submodule clean
    sys.path.insert(0, os.path.join(ROOT, "ref", "pysstv"))
    try:
        from PIL import Image
        import pysstv.color as pcolor
        import pysstv.grayscale as pgray
    except ImportError as exc:
        print("pysstv: %s" % exc)
        report["pysstv"] = "missing"
        return False
    ok = True
    res = report.setdefault("pysstv", {})
    rates = [48000] if args.quick else [48000, 44100, 12000, 11025, 8000]
    for m in mode_rows:
        name = m["short"]
        if name not in PYSSTV_CLASSES or (args.modes and name not in args.modes):
            continue
        mod, cls_name = PYSSTV_CLASSES[name]
        cls = getattr(pcolor if mod == "color" else pgray, cls_name)
        if name == "R8BW":
            cls = type("Robot8BW599", (cls,), {"SCAN": 59.9})
        w, h = int(m["tx_width"]), int(m["lines"])
        full = test_pattern(w, h)
        if name in HALF_WIDTH:
            src = full[:, ::2, :]
            ours = np.repeat(src, 2, axis=1)
        else:
            src = full
            ours = full
        pat = os.path.join(tmp, "py_%s.ppm" % name)
        write_ppm(pat, ours)
        for rate in rates:
            enc = cls(Image.fromarray(np.ascontiguousarray(src), "RGB"), rate, 16)
            enc.add_fskid_text("DG6FL")
            ref = np.fromiter(enc.gen_values(), dtype=np.float64)
            raw = os.path.join(tmp, "py_%s_%d.f64" % (name, rate))
            run([tool, "raw", name, str(rate), pat, raw, "DG6FL"])
            got = np.fromfile(raw, dtype=np.float64)
            same_len = got.size == ref.size
            n = min(got.size, ref.size)
            maxdiff = float(np.max(np.abs(got[:n] - ref[:n]))) if n else 1.0
            exact = same_len and bool(np.array_equal(got, ref))
            # 1e-9: pySSTV's own tolerance (test_sstv.py:52). Doubled-column modes split each
            # pySSTV pixel into two segments: twice as many phase-offset additions.
            good = same_len and maxdiff <= (1e-5 if name in HALF_WIDTH else 1e-9)
            ok = ok and good
            res["%s/%d" % (name, rate)] = {"samples": int(ref.size), "same_length": same_len,
                                           "max_abs_diff": maxdiff, "bit_identical": exact}
            print("pysstv %-6s %5d Hz samples=%d same_len=%s max|d|=%.3g bit-identical=%s" % (
                name, rate, ref.size, same_len, maxdiff, exact), flush=True)
    return ok


def check_robot36(args, tool, mode_rows, tmp, report):
    jar = os.path.join(args.build, "robot36", "robot36-ref.jar")
    if not os.path.exists(jar):
        print("robot36: %s missing (tools/robot36/build.sh)" % jar)
        report["robot36"] = "missing"
        return False
    ok = True
    res = report.setdefault("robot36", {})
    for m in mode_rows:
        name = m["short"]
        if name not in ROBOT36_MODES or (args.modes and name not in args.modes):
            continue
        w, h = int(m["tx_width"]), int(m["lines"])
        src = test_pattern(w, h)
        pat = os.path.join(tmp, "r36_%s.ppm" % name)
        wav = os.path.join(tmp, "r36_%s.wav" % name)
        write_ppm(pat, src)
        run([tool, "encode", name, "44100", pat, wav])
        over, rate = read_wav(wav)
        padded = np.concatenate([np.zeros(rate, np.int16), over, np.zeros(2 * rate, np.int16)])
        write_wav(wav, padded, rate)
        out = os.path.join(tmp, "r36_%s_out" % name)
        java = os.environ.get("JAVA", "java")
        log = run([java, "-jar", jar, wav, out])
        got = out + "0.ppm"
        if not os.path.exists(got):
            res[name] = {"result": "no image", "log": log[-400:]}
            ok = False
            print("robot36 %-6s no image" % name, flush=True)
            continue
        img = read_ppm(got)
        mode_line = [line for line in log.split("\n") if line.startswith("image")]
        if img.shape[:2] != src.shape[:2]:
            ys = (np.arange(src.shape[0]) * img.shape[0] // src.shape[0])
            xs = (np.arange(src.shape[1]) * img.shape[1] // src.shape[1])
            img = img[ys][:, xs]
        se = np.mean((img.astype(float) - src.astype(float)) ** 2)
        val = 99.0 if se == 0 else 10 * math.log10(255 ** 2 / se)
        err = region_error(m, img)
        # robot36 converts YUV as studio range (ColorConverter.java:30-36: (Y - 16) * 298/256),
        # SSTV sends full range: up to ~20 levels on saturated colours.
        good = err <= 25.0 and bool(mode_line) and \
            mode_line[0].split(" ", 2)[2].rsplit(" ", 1)[0] == ROBOT36_NAMES[name]
        ok = ok and good
        res[name] = {"psnr_db": round(val, 2), "region_error": round(err, 2),
                     "decoder_mode": mode_line[:1], "result": "ok" if good else "BAD"}
        print("robot36 %-6s region error %.1f psnr=%.2f dB %s" % (name, err, val,
                                                                 mode_line[:1]), flush=True)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default=os.path.join(ROOT, "build"))
    ap.add_argument("--only", default="slowrx,pysstv,robot36")
    ap.add_argument("--modes", default="")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--json", default="")
    args = ap.parse_args()
    args.build = os.path.abspath(args.build)
    args.modes = set(filter(None, args.modes.split(",")))
    tool = os.path.join(args.build, "sstv_tool")
    mode_rows = modes(tool)
    report = {}
    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        for part in args.only.split(","):
            fn = {"slowrx": check_slowrx, "pysstv": check_pysstv, "robot36": check_robot36}[part]
            ok = fn(args, tool, mode_rows, tmp, report) and ok
    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=1, sort_keys=True)
    print("REFERENCE CHECK", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
