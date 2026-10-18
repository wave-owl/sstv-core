/*
 * SSTV transmitter: pySSTV (https://github.com/wave-owl/pySSTV, commit d998fad).
 *
 *   sstv.py:73-91    SSTV.gen_values        -> render (phase-continuous sine, fractional
 *                                             sample carry, the "sample" of the previous
 *                                             segment when a segment rounds to 0 samples)
 *   sstv.py:93-122   SSTV.gen_freq_bits     -> next_segment: VOX, header, VIS, image, FSK ID
 *   sstv.py:127-136  add_fskid_text, byte_to_freq
 *   color.py:15-250  ColorSSTV, MartinM1/M2, ScottieS1/S2/DX, Robot36, PasokonP3/P5/P7,
 *                    PD90/120/160/180/240/290, WraaseSC2180/SC2120 -> build_line
 *   grayscale.py:7-38 GrayscaleSSTV, Robot8BW, Robot24BW               -> build_line
 *
 * Modes pySSTV does not have are built from the same segment model with the timing of the
 * mode table (sstv_modes.c): Martin M3/M4 (Martin M1/M2 lines, 128 rows), Robot 72 and
 * Robot 24 (robot36 Robot_72_Color.java:24-31 layout, separators 1500/2300 Hz as pySSTV's
 * Robot 36), Robot 12 B/W (Robot 24 B/W lines, 120 rows, odd VIS parity as slowrx expects,
 * vis.c:116) and PD-50 (pySSTV's PD layout, pixel 0.286 ms).
 *
 * Width: every mode sends the receiver's width (slowrx ImgWidth). pySSTV sends 160 pixels
 * for Martin M2, Scottie S2 and Robot 8 B/W in the same scan time; a 160-pixel picture with
 * every column doubled gives pySSTV's samples (tests: diff_pysstv.py). Robot 8 B/W uses the
 * 59.9 ms scan of slowrx and QSSTV (modespec.c:195-206, sstvparam.cpp:323) instead of
 * pySSTV's 60 ms (grayscale.py:30).
 *
 * SPDX-License-Identifier: ISC
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "sstv_internal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* sstv.py:11-23 */
#define FREQ_VIS_BIT1 1100
#define FREQ_SYNC 1200
#define FREQ_VIS_BIT0 1300
#define FREQ_BLACK 1500
#define FREQ_VIS_START 1900
#define FREQ_WHITE 2300
#define FREQ_RANGE (FREQ_WHITE - FREQ_BLACK)
#define FREQ_FSKID_BIT1 1900
#define FREQ_FSKID_BIT0 2100
#define MSEC_VIS_START 300
#define MSEC_VIS_SYNC 10
#define MSEC_VIS_BIT 30
#define MSEC_FSKID_BIT 22

typedef struct seg {
    double freq;
    double msec;
} seg;

enum { PH_VOX = 0, PH_HEADER, PH_IMAGE, PH_FSK, PH_END };

struct sstv_tx {
    sstv_tx_config cfg;
    const sstv_mode_ext *m;
    char fsk[SSTV_FSK_ID_MAX + 4];   /* pySSTV fskid_payload: 0x20 0x2a text-0x20 0x01 */
    int fsk_len;
    unsigned char *rgb;              /* picture, tx_width x lines x 3 */
    int have_image;
    /* segment cursor */
    int phase, idx, line;
    seg *lsegs;                      /* segments of the current line */
    int lseg_n, lseg_i, lseg_cap;
    /* gen_values state (sstv.py:79-91) */
    double spms, offset, samples, factor;
    long sample;                     /* the loop variable "sample" of gen_values */
    double cur_ff;
    long cur_tx, cur_k;
    int in_seg;
    size_t rendered, total;
};

/* sstv.py:135-136 */
static double byte_to_freq(double value) { return FREQ_BLACK + FREQ_RANGE * value / 255; }

void sstv_tx_config_default(sstv_tx_config *cfg, int mode, int sample_rate)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->mode = mode;
    cfg->sample_rate = sample_rate;
}

/* ---------------------------------------------------------------- per-mode parameters */

typedef struct txp {
    double sync, scan, gap, porch, pixel;  /* milliseconds, pySSTV class attributes */
    double y_scan, c_scan;
    int height;
} txp;

static void tx_params(int id, txp *p)
{
    memset(p, 0, sizeof(*p));
    switch (id) {
    case SSTV_M1: case SSTV_M3:          /* color.py:36-50 */
        p->sync = 4.862; p->scan = 146.432; p->gap = 0.572; break;
    case SSTV_M2: case SSTV_M4:          /* color.py:53-56 */
        p->sync = 4.862; p->scan = 73.216; p->gap = 0.572; break;
    case SSTV_S1:                        /* color.py:59-63 */
        p->sync = 9; p->gap = 1.5; p->scan = 138.24 - 1.5; break;
    case SSTV_S2:                        /* color.py:74-77 */
        p->sync = 9; p->gap = 1.5; p->scan = 88.064 - 1.5; break;
    case SSTV_SDX:                       /* color.py:80-83 */
        p->sync = 9; p->gap = 1.5; p->scan = 345.6000 - 1.5; break;
    case SSTV_R36:                       /* color.py:86-96 */
        p->sync = 9; p->gap = 4.5; p->y_scan = 88; p->c_scan = 44; p->porch = 1.5; break;
    case SSTV_R72:                       /* Robot_72_Color.java:24-31 */
        p->sync = 9; p->gap = 4.5; p->y_scan = 138; p->c_scan = 69; p->porch = 1.5; break;
    case SSTV_R24:                       /* sstv_modes.c C3 */
        p->sync = 9; p->gap = 4.5; p->y_scan = 88; p->c_scan = 44; p->porch = 1.5; break;
    case SSTV_P3:                        /* color.py:114-133 */
        p->pixel = 1000 / 4800.; break;
    case SSTV_P5:                        /* color.py:143-148 */
        p->pixel = 1000 / 3200.; break;
    case SSTV_P7:                        /* color.py:150-155 */
        p->pixel = 1000 / 2400.; break;
    case SSTV_PD50: p->sync = 20; p->porch = 2.08; p->pixel = 0.286; break;
    case SSTV_PD90: p->sync = 20; p->porch = 2.08; p->pixel = 0.532; break;   /* 158-164 */
    case SSTV_PD120: p->sync = 20; p->porch = 2.08; p->pixel = 0.19; break;   /* 183-187 */
    case SSTV_PD160: p->sync = 20; p->porch = 2.08; p->pixel = 0.382; break;  /* 189-193 */
    case SSTV_PD180: p->sync = 20; p->porch = 2.08; p->pixel = 0.286; break;  /* 195-197 */
    case SSTV_PD240: p->sync = 20; p->porch = 2.08; p->pixel = 0.382; break;  /* 199-201 */
    case SSTV_PD290: p->sync = 20; p->porch = 2.08; p->pixel = 0.286; break;  /* 203-207 */
    case SSTV_W2180:                     /* color.py:210-218 */
        p->sync = 5.5225; p->porch = 0.5; p->scan = 235.0; break;
    case SSTV_W2120:                     /* color.py:230-240 */
        p->sync = 5.5225; p->porch = 0.5; p->scan = 156.0; break;
    case SSTV_R24BW: case SSTV_R12BW:    /* grayscale.py:33-38 */
        p->sync = 7; p->scan = 93; break;
    case SSTV_R8BW:                      /* grayscale.py:25-30, scan 59.9 (see top) */
        p->sync = 7; p->scan = 59.9; break;
    default: break;
    }
}

/* ---------------------------------------------------------------- scan lines */

static void push(sstv_tx *tx, double freq, double msec)
{
    if (tx->lseg_n < tx->lseg_cap) {
        tx->lsegs[tx->lseg_n].freq = freq;
        tx->lsegs[tx->lseg_n].msec = msec;
        tx->lseg_n++;
    }
}

static const unsigned char *px(const sstv_tx *tx, int col, int line)
{
    return tx->rgb + ((size_t)line * tx->m->pub.tx_width + (size_t)col) * 3;
}

/* Pixel value of one channel: RGB channel, Pillow YCbCr component or Pillow L. */
static int ycc(const sstv_tx *tx, int col, int line, int comp)
{
    unsigned char out[3];
    sstv_pil_rgb2ycbcr(px(tx, col, line), out);
    return out[comp];
}

/* Segments of picture line `line` (frame for PD) after the horizontal sync. */
static void build_line(sstv_tx *tx, int line)
{
    const sstv_mode *m = &tx->m->pub;
    int W = m->tx_width, col, c;
    txp p;
    tx_params(m->id, &p);
    tx->lseg_n = 0;
    switch (m->id) {
    case SSTV_M1: case SSTV_M2: case SSTV_M3: case SSTV_M4: {
        /* GrayscaleSSTV.gen_image_tuples + ColorSSTV.encode_line + MartinM1 hooks */
        static const int seq[3] = { 1, 2, 0 };      /* green, blue, red */
        double msec_pixel = p.scan / W;
        push(tx, FREQ_SYNC, p.sync);
        for (c = 0; c < 3; c++) {
            if (seq[c] == 1) push(tx, FREQ_BLACK, p.gap);
            for (col = 0; col < W; col++)
                push(tx, byte_to_freq(px(tx, col, line)[seq[c]]), msec_pixel);
            push(tx, FREQ_BLACK, p.gap);
        }
        break;
    }
    case SSTV_S1: case SSTV_S2: case SSTV_SDX: {
        /* ScottieS1: no horizontal sync, sync before red (color.py:59-71) */
        static const int seq[3] = { 1, 2, 0 };
        double msec_pixel = p.scan / W;
        for (c = 0; c < 3; c++) {
            if (seq[c] == 0) push(tx, FREQ_SYNC, p.sync);
            push(tx, FREQ_BLACK, p.gap);
            for (col = 0; col < W; col++)
                push(tx, byte_to_freq(px(tx, col, line)[seq[c]]), msec_pixel);
            push(tx, FREQ_BLACK, p.gap);
        }
        break;
    }
    case SSTV_R36: {
        /* Robot36.encode_line (color.py:101-111) */
        int channel = 2 - (line % 2);
        double y_pixel_time = p.y_scan / W, uv_pixel_time = p.c_scan / W;
        push(tx, FREQ_SYNC, p.sync);
        push(tx, FREQ_BLACK, 3);                          /* SYNC_PORCH */
        for (col = 0; col < W; col++) push(tx, byte_to_freq(ycc(tx, col, line, 0)), y_pixel_time);
        push(tx, channel == 1 ? FREQ_WHITE : FREQ_BLACK, p.gap);   /* INTER_CH_FREQS */
        push(tx, FREQ_VIS_START, p.porch);
        for (col = 0; col < W; col++)
            push(tx, byte_to_freq(ycc(tx, col, line, channel)), uv_pixel_time);
        break;
    }
    case SSTV_R72: case SSTV_R24: {
        double y_pixel_time = p.y_scan / W, uv_pixel_time = p.c_scan / W;
        push(tx, FREQ_SYNC, p.sync);
        push(tx, FREQ_BLACK, 3);
        for (col = 0; col < W; col++) push(tx, byte_to_freq(ycc(tx, col, line, 0)), y_pixel_time);
        push(tx, FREQ_BLACK, p.gap);
        push(tx, FREQ_VIS_START, p.porch);
        for (col = 0; col < W; col++)
            push(tx, byte_to_freq(ycc(tx, col, line, 2)), uv_pixel_time);    /* V = Cr */
        push(tx, FREQ_WHITE, p.gap);
        push(tx, FREQ_VIS_START, p.porch);
        for (col = 0; col < W; col++)
            push(tx, byte_to_freq(ycc(tx, col, line, 1)), uv_pixel_time);    /* U = Cb */
        break;
    }
    case SSTV_P3: case SSTV_P5: case SSTV_P7: {
        /* PasokonP3: SYNC 25 TU, gaps 5 TU, red green blue (color.py:114-141) */
        double sync = 25 * p.pixel, scan = W * p.pixel, gap = 5 * p.pixel;
        double msec_pixel = scan / W;
        push(tx, FREQ_SYNC, sync);
        for (c = 0; c < 3; c++) {
            if (c == 0) push(tx, FREQ_BLACK, gap);
            for (col = 0; col < W; col++)
                push(tx, byte_to_freq(px(tx, col, line)[c]), msec_pixel);
            push(tx, FREQ_BLACK, gap);
        }
        break;
    }
    case SSTV_PD50: case SSTV_PD90: case SSTV_PD120: case SSTV_PD160: case SSTV_PD180:
    case SSTV_PD240: case SSTV_PD290: {
        /* PD90.gen_image_tuples (color.py:166-180), line = first row of the pair */
        push(tx, FREQ_SYNC, p.sync);
        push(tx, FREQ_BLACK, p.porch);
        for (col = 0; col < W; col++) push(tx, byte_to_freq(ycc(tx, col, line, 0)), p.pixel);
        for (col = 0; col < W; col++)
            push(tx, byte_to_freq((ycc(tx, col, line, 2) + ycc(tx, col, line + 1, 2)) / 2.0),
                 p.pixel);
        for (col = 0; col < W; col++)
            push(tx, byte_to_freq((ycc(tx, col, line, 1) + ycc(tx, col, line + 1, 1)) / 2.0),
                 p.pixel);
        for (col = 0; col < W; col++)
            push(tx, byte_to_freq(ycc(tx, col, line + 1, 0)), p.pixel);
        break;
    }
    case SSTV_W2180: case SSTV_W2120: {
        /* WraaseSC2180 / SC2120 (color.py:210-250) */
        double msec_pixel = p.scan / W;
        push(tx, FREQ_SYNC, p.sync);
        for (c = 0; c < 3; c++) {
            if (m->id == SSTV_W2120) push(tx, FREQ_BLACK, p.porch);
            if (c == 0) push(tx, FREQ_BLACK, p.porch);
            for (col = 0; col < W; col++)
                push(tx, byte_to_freq(px(tx, col, line)[c]), msec_pixel);
        }
        break;
    }
    default: {
        /* GrayscaleSSTV (grayscale.py:11-22) */
        double msec_pixel = p.scan / W;
        push(tx, FREQ_SYNC, p.sync);
        for (col = 0; col < W; col++)
            push(tx, byte_to_freq(sstv_pil_rgb2l(px(tx, col, line))), msec_pixel);
        break;
    }
    }
}

static int line_step(const sstv_tx *tx)
{
    return tx->m->pub.channels == 4 ? 2 : 1;
}

/* gen_freq_bits (sstv.py:93-122), one segment at a time. Returns 0 at the end. */
static int next_segment(sstv_tx *tx, double *freq, double *msec)
{
    static const int vox[8] = { 1900, 1500, 1900, 1500, 2300, 1500, 2300, 1500 };
    for (;;) {
        switch (tx->phase) {
        case PH_VOX:
            if (tx->cfg.vox && tx->idx < 8) {
                *freq = vox[tx->idx++];
                *msec = 100;
                return 1;
            }
            tx->phase = PH_HEADER;
            tx->idx = 0;
            break;
        case PH_HEADER: {
            int i = tx->idx++, vis = tx->m->pub.vis, ones = 0, b;
            if (i == 0) { *freq = FREQ_VIS_START; *msec = MSEC_VIS_START; return 1; }
            if (i == 1) { *freq = FREQ_SYNC; *msec = MSEC_VIS_SYNC; return 1; }
            if (i == 2) { *freq = FREQ_VIS_START; *msec = MSEC_VIS_START; return 1; }
            if (i == 3) { *freq = FREQ_SYNC; *msec = MSEC_VIS_BIT; return 1; }  /* start bit */
            if (i >= 4 && i < 11) {
                *freq = ((vis >> (i - 4)) & 1) ? FREQ_VIS_BIT1 : FREQ_VIS_BIT0;
                *msec = MSEC_VIS_BIT;
                return 1;
            }
            if (i == 11) {
                for (b = 0; b < 7; b++) ones += (vis >> b) & 1;
                if (tx->m->vis_parity_odd) ones++;
                *freq = (ones % 2 == 1) ? FREQ_VIS_BIT1 : FREQ_VIS_BIT0;
                *msec = MSEC_VIS_BIT;
                return 1;
            }
            if (i == 12) { *freq = FREQ_SYNC; *msec = MSEC_VIS_BIT; return 1; }  /* stop bit */
            tx->phase = PH_IMAGE;
            tx->line = 0;
            tx->lseg_n = tx->lseg_i = 0;
            break;
        }
        case PH_IMAGE:
            if (tx->lseg_i < tx->lseg_n) {
                *freq = tx->lsegs[tx->lseg_i].freq;
                *msec = tx->lsegs[tx->lseg_i].msec;
                tx->lseg_i++;
                return 1;
            }
            if (tx->line >= tx->m->pub.lines) {
                tx->phase = PH_FSK;
                tx->idx = 0;
                break;
            }
            build_line(tx, tx->line);
            tx->line += line_step(tx);
            tx->lseg_i = 0;
            break;
        case PH_FSK:
            if (tx->idx < tx->fsk_len * 6) {
                int byte = (unsigned char)tx->fsk[tx->idx / 6], bit = (byte >> (tx->idx % 6)) & 1;
                tx->idx++;
                *freq = bit ? FREQ_FSKID_BIT1 : FREQ_FSKID_BIT0;
                *msec = MSEC_FSKID_BIT;
                return 1;
            }
            tx->phase = PH_END;
            break;
        default:
            return 0;
        }
    }
}

static void cursor_reset(sstv_tx *tx)
{
    tx->phase = PH_VOX;
    tx->idx = 0;
    tx->line = 0;
    tx->lseg_n = tx->lseg_i = 0;
    tx->spms = tx->cfg.sample_rate / 1000.0;
    tx->offset = 0;
    tx->samples = 0;
    tx->factor = 2 * M_PI / tx->cfg.sample_rate;
    tx->sample = 0;
    tx->in_seg = 0;
    tx->rendered = 0;
}

/* Exact sample count: gen_values' accumulator without the sine. The picture does not
 * change segment durations, so a black dummy picture is used for the dry run. */
static size_t count_samples(sstv_tx *tx)
{
    double f, ms, samples = 0;
    size_t total = 0;
    cursor_reset(tx);
    while (next_segment(tx, &f, &ms)) {
        long t;
        samples += tx->spms * ms;
        t = (long)samples;
        total += (size_t)t;
        samples -= t;
    }
    cursor_reset(tx);
    return total;
}

sstv_tx *sstv_tx_create(const sstv_tx_config *cfg)
{
    sstv_tx *tx;
    const sstv_mode_ext *m;
    size_t n;
    int i;
    if (!cfg || cfg->sample_rate < SSTV_MIN_RATE || cfg->sample_rate > SSTV_MAX_RATE) return NULL;
    m = sstv_mode_ext_by_id(cfg->mode);
    if (!m || !m->pub.tx) return NULL;
    if (cfg->fsk_id) {
        size_t len = strlen(cfg->fsk_id);
        if (len > SSTV_FSK_ID_MAX) return NULL;
        for (i = 0; i < (int)len; i++) {
            unsigned char c = (unsigned char)cfg->fsk_id[i];
            if (c < 0x20 || c > 0x5f) return NULL;
        }
    }
    tx = (sstv_tx *)calloc(1, sizeof(*tx));
    if (!tx) return NULL;
    tx->cfg = *cfg;
    tx->cfg.fsk_id = NULL;
    tx->m = m;
    if (cfg->fsk_id && cfg->fsk_id[0]) {
        /* add_fskid_text (sstv.py:127-129) */
        int len = (int)strlen(cfg->fsk_id);
        tx->fsk[0] = 0x20;
        tx->fsk[1] = 0x2a;
        for (i = 0; i < len; i++) tx->fsk[2 + i] = (char)(cfg->fsk_id[i] - 0x20);
        tx->fsk[2 + len] = 0x01;
        tx->fsk_len = len + 3;
    }
    n = (size_t)m->pub.tx_width * m->pub.lines * 3;
    tx->rgb = (unsigned char *)calloc(n, 1);
    tx->lseg_cap = 4 * SSTV_MAX_WIDTH + 16;
    tx->lsegs = (seg *)calloc((size_t)tx->lseg_cap, sizeof(seg));
    if (!tx->rgb || !tx->lsegs) {
        sstv_tx_destroy(tx);
        return NULL;
    }
    tx->total = count_samples(tx);
    return tx;
}

void sstv_tx_destroy(sstv_tx *tx)
{
    if (!tx) return;
    free(tx->rgb);
    free(tx->lsegs);
    free(tx);
}

int sstv_tx_put_image(sstv_tx *tx, const unsigned char *rgb, int width, int height)
{
    if (!tx || !rgb || width != tx->m->pub.tx_width || height != tx->m->pub.lines) return -1;
    memcpy(tx->rgb, rgb, (size_t)width * height * 3);
    tx->have_image = 1;
    cursor_reset(tx);
    return 0;
}

size_t sstv_tx_render(sstv_tx *tx, double *out, size_t max)
{
    size_t n = 0;
    if (!tx || !tx->have_image || !out) return 0;
    while (n < max) {
        if (!tx->in_seg) {
            double f, ms;
            if (!next_segment(tx, &f, &ms)) break;
            tx->samples += tx->spms * ms;
            tx->cur_tx = (long)tx->samples;
            tx->cur_ff = f * tx->factor;
            tx->cur_k = 0;
            tx->in_seg = 1;
        }
        while (n < max && tx->cur_k < tx->cur_tx) {
            out[n++] = sin(tx->cur_k * tx->cur_ff + tx->offset);
            tx->sample = tx->cur_k;
            tx->cur_k++;
        }
        if (tx->cur_k >= tx->cur_tx) {
            tx->offset += (tx->sample + 1) * tx->cur_ff;
            tx->samples -= tx->cur_tx;
            tx->in_seg = 0;
        }
    }
    tx->rendered += n;
    return n;
}

size_t sstv_tx_total_samples(const sstv_tx *tx) { return tx ? tx->total : 0; }

size_t sstv_tx_rendered(const sstv_tx *tx) { return tx ? tx->rendered : 0; }
