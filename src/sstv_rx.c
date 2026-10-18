/*
 * SSTV receiver: slowrx (https://github.com/wave-owl/slowrx, commit ca6d701) as a streaming
 * state machine.
 *
 *   slowrx.c:24-185 Listen   -> sstv_rx_process: HUNT -> VIDEO -> FSK -> finish -> HUNT
 *   vis.c:18-175    GetVIS   -> vis_step
 *   video.c:19-491  GetVideo -> video_start / video_step (first pass), redraw (second pass)
 *   fsk.c:20-110    GetFSK   -> fsk_step
 *   sync.c:18-133   FindSync -> find_sync
 *   pcm.c:19-64     readPcm  -> the input ring (slowrx's capture buffer, BUFLEN 4096)
 *
 * At SSTV_REFERENCE_RATE (44100 Hz, slowrx's only rate) every sample count, FFT length,
 * window and expression is slowrx's. At another rate each slowrx sample count c becomes
 * lround(c * rate / 44100) and each FFT length the smallest power of two that holds the
 * scaled length; every time constant in seconds is unchanged (README "Sample rate").
 *
 * Defined behaviour where slowrx reads outside an array (each marked "slowrx UB"):
 *   U1 vis.c:67   HedrBuf[(HedrPtr-1) % 45] reads HedrBuf[-1] when HedrPtr is 0: the
 *                 cyclically previous entry HedrBuf[44] is used.
 *   U2 sync.c:61  lines[dMost][qMost-MINSLANT*2] reads lines[0][-60] before the first
 *                 accumulator hit: compared as 0.
 *   U3 sync.c:40,101 HasSync[] past the samples GetVideo stored (slant-corrected rate at the
 *                 end of the image): read as no sync (calloc'd zero where slowrx's buffer
 *                 reaches that far).
 *   U4 video.c:406 clip() of a NaN frequency (all-zero spectrum): 0.
 *   U5 common.c:40 GetBin() of a negative frequency (header shift below -400 Hz): bin 0.
 *   U6 video.c:411,415 PixelGrid[PixelIdx] read one entry past the grid after the last
 *                 pixel: no further pixel.
 *   U7 fsk.c:48-51 never terminates when the capture-buffer pointer is below 1455 samples
 *                 while in FSK sync (the pointer does not move inside that branch); without
 *                 the branch the 22 ms window reaches up to 485 samples before slowrx's
 *                 buffer. The window is read from the input stream (the ring holds it).
 *
 * Not in slowrx (each marked "station extension"): timeout_s (an image ends as partial when
 * the video signal is gone), slant_partial (slant correction of a partial image), flush.
 *
 * SPDX-License-Identifier: ISC
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fldigi_dsp.h"
#include "sstv_internal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MINSLANT 30          /* common.h:4 */
#define MAXSLANT 150         /* common.h:5 */
#define NSLANT ((MAXSLANT - MINSLANT) * 2)
#define HOUGH_D 600          /* sync.c:24 lines[600][...] */
#define XACC 700             /* sync.c:23 */
#define NHANN 7              /* video.c:54 */
#define VIS_TONES 45         /* vis.c:72: 45 * 10 ms */
#define MIN_PARTIAL_SLANT_ROWS 16
/* Station extension (README "Signal loss"): the signal-to-noise power ratio of slowrx's SNR
 * estimate (video.c:336-341), limited to [0, 1] and averaged over about half a second, at or
 * above -10 dB counts as "signal present" for timeout_s; a partial image keeps the rows
 * completed while the signal was present. */
#define SIGNAL_RATIO 0.1
#define SIGNAL_AVG_S 0.5

enum { ST_HUNT = 0, ST_VIDEO, ST_FSK };

/* One entry of slowrx's PixelGrid (video.c:41-47), generated on the fly. */
typedef struct walk {
    int y, ch, x, sub;       /* generator position (sub: second entry of a PD chroma pair) */
    int done;
    int ex, ey, ech, eloop, elast;
    int et;                  /* Time in samples */
    double rate;
    int skip;
} walk;

struct sstv_rx {
    sstv_rx_config cfg;
    double r0;               /* nominal rate (slowrx: 44100) */
    /* slowrx sample counts at this rate */
    int buf_half;            /* BUFLEN/2 (pcm.c:54): first window centre */
    int vis_hop;             /* 441 (vis.c:45) */
    int vis_win;             /* 882 (vis.c:30) */
    int big_fft;             /* 2048 (vis.c:22, fsk.c:22) */
    int fsk_half;            /* 485 (fsk.c:46) */
    int fsk_win;             /* 970 (fsk.c:40) */
    int vid_fft;             /* 1024 (video.c:26) */
    int hann_len[NHANN];     /* 48 64 96 128 256 512 1024 (video.c:54) */
    int sync_hop;            /* 13 (video.c:295) */
    int snr_hop;             /* 256 (video.c:343) */
    int demod_hop;           /* 6 (video.c:350) */
    int look_ahead;          /* samples past the processed one that windows read */
    int post_vis;            /* 20e-3 * 44100 (vis.c:169-170) */
    double *hann_vis, *hann_fsk, *hann_vid[NHANN];
    fdsp_gfft fft_big;
    fdsp_cmpx *fbuf;
    double *power;
    double *xw;              /* windowed video segment (window_dft) */
    double *dft_cos, *dft_sin;   /* e^{-j 2 pi m / vid_fft}, m = 0 .. vid_fft-1 */
    double sin_q[NSLANT], cos_q[NSLANT];
    /* input ring (absolute sample index & mask) */
    double *ring;
    long long mask, count;
    /* state */
    int state;
    long long pos;           /* HUNT / FSK: window centre */
    double hedr[VIS_TONES];
    int hedr_ptr;
    /* image in progress */
    const sstv_mode_ext *m;
    short shift;
    long long base;          /* absolute index of SampleNum 0 */
    int length, s;
    double chan_start[4], chan_len[4];
    int sync_target, next_sync, sync_idx, next_snr;
    double snr, freq;
    long long last_signal;
    double signal_avg;       /* exponential average of psignal / pnoise */
    int rows_at_signal;      /* rows completed at last_signal */
    unsigned char *lum;      /* StoredLum */
    int lum_cap;
    unsigned char *has_sync; /* HasSync */
    int sync_cap, sync_count;
    walk w;
    unsigned char *img;      /* slowrx Image[x][y][3], four rows (y & 3) */
    int img_tag[4];
    unsigned char *fb;       /* slowrx pixbuf_rx */
    int rows;
    unsigned short *lines;   /* Hough accumulator lines[600][240] */
    sstv_image_info cur;
    /* FSK */
    int fsk_insync, fsk_bitptr, fsk_byteptr;
    unsigned fsk_byte;
    unsigned char fsk_testbits[24];
    unsigned fsk_testptr;    /* guint TestPtr: unsigned wrap is part of fsk.c:83 */
    char fsk_dest[SSTV_FSK_ID_MAX + 2];
    /* finished image */
    unsigned char *done;
    sstv_image_info done_info;
    int have_done;
    /* events */
    sstv_event *ev;
    size_t ev_max, ev_n, ev_dropped;
    size_t footprint;
};

/* ---------------------------------------------------------------- helpers (common.c) */

/* GetBin (common.c:39-41) with U5. */
static int getbin(const sstv_rx *rx, double freq, int fftlen)
{
    double b = freq / rx->r0 * fftlen;
    if (!(b > 0)) return 0;
    if (b >= fftlen - 1) return fftlen - 1;
    return (int)b;
}

/* power (common.c:44-46) */
static double pw(fdsp_cmpx c) { return pow(c.re, 2) + pow(c.im, 2); }

/* clip (common.c:49-53) with U4. */
static unsigned char clip(double a)
{
    if (a != a) return 0;
    if (a < 0) return 0;
    else if (a > 255) return 255;
    return (unsigned char)round(a);
}

static double deg2rad(double deg) { return (deg / 180) * M_PI; }   /* common.c:56-58 */

static double ring_at(const sstv_rx *rx, long long i)
{
    if (i < 0 || i >= rx->count || i < rx->count - 1 - rx->mask) return 0.0;
    return rx->ring[i & rx->mask];
}

static void emit(sstv_rx *rx, int kind, int line, int complete)
{
    sstv_event *e;
    if (rx->ev_n >= rx->ev_max) {
        rx->ev_dropped++;
        return;
    }
    e = &rx->ev[rx->ev_n++];
    e->kind = kind;
    e->mode = rx->m ? rx->m->pub.id : 0;
    e->line = line;
    e->complete = complete;
    e->shift_hz = rx->shift;
}

static int scaled(double c, double scale)
{
    long v = lround(c * scale);
    return v < 1 ? 1 : (int)v;
}

static int pow2_at_least(int n)
{
    int p = 16;
    while (p < n) p <<= 1;
    return p;
}

/* Hann-windowed segment of the input into fbuf, zero padded to n (vis.c:27,48,
 * video.c:275-278, 306-309, 369-375, fsk.c:38,55). */
static void window_fft(sstv_rx *rx, const fdsp_gfft *fft, long long first, const double *hann,
                       int len)
{
    int i, n = (int)fft->size;
    for (i = 0; i < n; i++) {
        rx->fbuf[i].re = 0.0;
        rx->fbuf[i].im = 0.0;
    }
    for (i = 0; i < len; i++) rx->fbuf[i].re = ring_at(rx, first + i) * hann[i];
    fdsp_gfft_forward(fft, rx->fbuf);
}

/* The video FFTs of slowrx (video.c:278-280, 309-311, 375-377: Hann window zero padded to
 * 1024 points) evaluated only at the bins slowrx then reads, by direct DFT: the same
 * coefficients X[k] = sum x[i] e^{-j 2 pi k i / N} as a full FFT, at a fraction of the cost
 * (the windows are 48..1024 samples, the bins about 20 of 512). */
static void window_seg(sstv_rx *rx, long long first, const double *hann, int len)
{
    int i;
    for (i = 0; i < len; i++) rx->xw[i] = ring_at(rx, first + i) * hann[i];
}

static void window_dft(sstv_rx *rx, int len, int lo, int hi)
{
    const int mask = rx->vid_fft - 1;
    int k, i;
    if (lo < 0) lo = 0;
    if (hi > mask) hi = mask;
    for (k = lo; k <= hi; k++) {
        double re = 0.0, im = 0.0;
        int m = 0;
        for (i = 0; i < len; i++) {
            re += rx->xw[i] * rx->dft_cos[m];
            im -= rx->xw[i] * rx->dft_sin[m];
            m = (m + k) & mask;
        }
        rx->fbuf[k].re = re;
        rx->fbuf[k].im = im;
    }
}

/* ---------------------------------------------------------------- configuration */

void sstv_rx_config_default(sstv_rx_config *cfg, int sample_rate)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->sample_rate = sample_rate;
    cfg->adaptive = 1;
    cfg->slant = 1;
    cfg->fsk_id = 1;
    cfg->slant_partial = 1;
    cfg->timeout_s = 0.0;
    cfg->vis_tolerance_hz = 25.0;
}

static void *alloc(sstv_rx *rx, size_t n, size_t size)
{
    void *p = calloc(n ? n : 1, size);
    if (p) rx->footprint += n * size;
    return p;
}

static int mode_length(const sstv_mode_ext *m, double r0)
{
    /* video.c:251-254 */
    if (m->pub.channels == 4)
        return (int)(m->pub.line_s * m->pub.lines / 2 * r0);
    return (int)(m->pub.line_s * m->pub.lines * r0);
}

static void hunt_reset(sstv_rx *rx);

sstv_rx *sstv_rx_create(const sstv_rx_config *cfg)
{
    static const int hann_lens[NHANN] = { 48, 64, 96, 128, 256, 512, 1024 };
    sstv_rx *rx;
    double scale;
    int i, j, max_len = 0, max_px = 0, cap, look;

    if (!cfg || cfg->sample_rate < SSTV_MIN_RATE || cfg->sample_rate > SSTV_MAX_RATE ||
        cfg->timeout_s < 0 || !(cfg->vis_tolerance_hz > 0) || cfg->vis_tolerance_hz > 100)
        return NULL;
    rx = (sstv_rx *)calloc(1, sizeof(*rx));
    if (!rx) return NULL;
    rx->footprint = sizeof(*rx);
    rx->cfg = *cfg;
    rx->r0 = cfg->sample_rate;
    scale = rx->r0 / SSTV_REFERENCE_RATE;
    rx->buf_half = scaled(2048, scale);
    rx->vis_hop = scaled(441, scale);
    rx->vis_win = 2 * rx->vis_hop;
    rx->fsk_half = scaled(485, scale);
    rx->fsk_win = 2 * rx->fsk_half;
    rx->big_fft = pow2_at_least(scaled(2048, scale));
    if (rx->big_fft < rx->fsk_win) rx->big_fft = pow2_at_least(rx->fsk_win);
    for (i = 0; i < NHANN; i++) rx->hann_len[i] = scaled(hann_lens[i], scale);
    rx->vid_fft = pow2_at_least(scaled(1024, scale));
    if (rx->vid_fft < rx->hann_len[NHANN - 1]) rx->vid_fft = pow2_at_least(rx->hann_len[6]);
    rx->sync_hop = scaled(13, scale);
    rx->snr_hop = scaled(256, scale);
    rx->demod_hop = scaled(6, scale);
    rx->post_vis = (int)(rx->buf_half + 20e-3 * rx->r0) - rx->buf_half;
    rx->look_ahead = rx->hann_len[6] - rx->hann_len[6] / 2;
    for (i = 0; i < NSLANT; i++) {
        rx->sin_q[i] = sin(deg2rad((i + MINSLANT * 2) / 2.0));
        rx->cos_q[i] = cos(deg2rad((i + MINSLANT * 2) / 2.0));
    }

    look = rx->big_fft;
    if (look < rx->vid_fft) look = rx->vid_fft;
    cap = pow2_at_least(16 * look);
    rx->mask = cap - 1;
    for (i = 1; i <= SSTV_MODE_LAST; i++) {
        const sstv_mode_ext *m = sstv_mode_ext_by_id(i);
        int len = mode_length(m, rx->r0), px = m->pub.width * m->pub.lines;
        if (len > max_len) max_len = len;
        if (px > max_px) max_px = px;
    }
    rx->lum_cap = max_len + 1;
    rx->sync_cap = max_len / rx->sync_hop + 2;

    rx->ring = (double *)alloc(rx, (size_t)cap, sizeof(double));
    rx->hann_vis = (double *)alloc(rx, (size_t)rx->vis_win, sizeof(double));
    rx->hann_fsk = (double *)alloc(rx, (size_t)rx->fsk_win, sizeof(double));
    for (i = 0; i < NHANN; i++)
        rx->hann_vid[i] = (double *)alloc(rx, (size_t)rx->hann_len[i], sizeof(double));
    rx->fbuf = (fdsp_cmpx *)alloc(rx, (size_t)rx->big_fft, sizeof(fdsp_cmpx));
    rx->power = (double *)alloc(rx, (size_t)rx->big_fft, sizeof(double));
    rx->xw = (double *)alloc(rx, (size_t)rx->vid_fft, sizeof(double));
    rx->dft_cos = (double *)alloc(rx, (size_t)rx->vid_fft, sizeof(double));
    rx->dft_sin = (double *)alloc(rx, (size_t)rx->vid_fft, sizeof(double));
    rx->lum = (unsigned char *)alloc(rx, (size_t)rx->lum_cap, 1);
    rx->has_sync = (unsigned char *)alloc(rx, (size_t)rx->sync_cap, 1);
    rx->img = (unsigned char *)alloc(rx, 4 * SSTV_MAX_WIDTH * 3, 1);
    rx->fb = (unsigned char *)alloc(rx, (size_t)max_px * 3, 1);
    rx->done = (unsigned char *)alloc(rx, (size_t)max_px * 3, 1);
    rx->lines = (unsigned short *)alloc(rx, HOUGH_D * NSLANT, sizeof(unsigned short));
    if (!rx->ring || !rx->hann_vis || !rx->hann_fsk || !rx->fbuf || !rx->power || !rx->lum ||
        !rx->xw || !rx->dft_cos || !rx->dft_sin ||
        !rx->has_sync || !rx->img || !rx->fb || !rx->done || !rx->lines ||
        fdsp_gfft_init(&rx->fft_big, (size_t)rx->big_fft) != 0) {
        sstv_rx_destroy(rx);
        return NULL;
    }
    for (i = 0; i < NHANN; i++)
        if (!rx->hann_vid[i]) {
            sstv_rx_destroy(rx);
            return NULL;
        }
    rx->footprint += ((size_t)rx->big_fft / 4 + 1) * sizeof(double) +
                     (size_t)rx->big_fft * sizeof(short);
    for (i = 0; i < rx->vid_fft; i++) {
        rx->dft_cos[i] = cos(2 * M_PI * i / rx->vid_fft);
        rx->dft_sin[i] = sin(2 * M_PI * i / rx->vid_fft);
    }
    /* 20 ms Hann window (vis.c:30) */
    for (i = 0; i < rx->vis_win; i++)
        rx->hann_vis[i] = 0.5 * (1 - cos((2 * M_PI * (double)i) / (rx->vis_win - 1)));
    /* 22 ms Hann window (fsk.c:40) */
    for (i = 0; i < rx->fsk_win; i++)
        rx->hann_fsk[i] = 0.5 * (1 - cos(2 * M_PI * i / (double)(rx->fsk_win - 1)));
    /* video.c:55-57 */
    for (j = 0; j < NHANN; j++)
        for (i = 0; i < rx->hann_len[j]; i++)
            rx->hann_vid[j][i] = 0.5 * (1 - cos((2 * M_PI * i) / (rx->hann_len[j] - 1)));
    sstv_rx_reset(rx);
    return rx;
}

void sstv_rx_destroy(sstv_rx *rx)
{
    int i;
    if (!rx) return;
    fdsp_gfft_free(&rx->fft_big);
    free(rx->ring);
    free(rx->hann_vis);
    free(rx->hann_fsk);
    for (i = 0; i < NHANN; i++) free(rx->hann_vid[i]);
    free(rx->fbuf);
    free(rx->power);
    free(rx->xw);
    free(rx->dft_cos);
    free(rx->dft_sin);
    free(rx->lum);
    free(rx->has_sync);
    free(rx->img);
    free(rx->fb);
    free(rx->done);
    free(rx->lines);
    free(rx);
}

static void hunt_reset(sstv_rx *rx)
{
    /* GetVIS starts with a zeroed HedrBuf (vis.c:23) */
    memset(rx->hedr, 0, sizeof(rx->hedr));
    rx->hedr_ptr = 0;
    rx->state = ST_HUNT;
}

void sstv_rx_reset(sstv_rx *rx)
{
    if (!rx) return;
    memset(rx->ring, 0, (size_t)(rx->mask + 1) * sizeof(double));
    rx->count = 0;
    rx->pos = rx->buf_half;
    rx->m = NULL;
    rx->have_done = 0;
    rx->ev_dropped = 0;
    memset(&rx->cur, 0, sizeof(rx->cur));
    memset(&rx->done_info, 0, sizeof(rx->done_info));
    hunt_reset(rx);
}

/* ---------------------------------------------------------------- pixel grid (video.c) */

static void chan_layout(sstv_rx *rx)
{
    const sstv_mode *p = &rx->m->pub;
    double *cs = rx->chan_start, *cl = rx->chan_len;
    memset(rx->chan_start, 0, sizeof(rx->chan_start));
    memset(rx->chan_len, 0, sizeof(rx->chan_len));
    switch (rx->m->layout) {
    case SSTV_LAYOUT_ROBOT2:          /* video.c:63-70 */
        cl[0] = p->pixel_s * p->width * 2;
        cl[1] = cl[2] = p->pixel_s * p->width;
        cs[0] = p->sync_s + p->porch_s;
        cs[1] = cs[0] + cl[0] + p->septr_s;
        cs[2] = cs[1];
        break;
    case SSTV_LAYOUT_SCOTTIE:         /* video.c:72-79 */
        cl[0] = cl[1] = cl[2] = p->pixel_s * p->width;
        cs[0] = p->septr_s;
        cs[1] = cs[0] + cl[0] + p->septr_s;
        cs[2] = cs[1] + cl[1] + p->sync_s + p->porch_s;
        break;
    case SSTV_LAYOUT_PD:              /* video.c:81-93 */
        cl[0] = cl[1] = cl[2] = cl[3] = p->pixel_s * p->width;
        cs[0] = p->sync_s + p->porch_s;
        cs[1] = cs[0] + cl[0] + p->septr_s;
        cs[2] = cs[1] + cl[1] + p->septr_s;
        cs[3] = cs[2] + cl[2] + p->septr_s;
        break;
    case SSTV_LAYOUT_ROBOT3:          /* correction C2/C3: chroma half the Y length */
        cl[0] = p->pixel_s * p->width;
        cl[1] = cl[2] = p->pixel_s * p->width / 2.0;
        cs[0] = p->sync_s + p->porch_s;
        cs[1] = cs[0] + cl[0] + p->septr_s;
        cs[2] = cs[1] + cl[1] + p->septr_s;
        break;
    default:                          /* video.c:95-100 */
        cl[0] = cl[1] = cl[2] = p->pixel_s * p->width;
        cs[0] = p->sync_s + p->porch_s;
        cs[1] = cs[0] + cl[0] + p->septr_s;
        cs[2] = cs[1] + cl[1] + p->septr_s;
        break;
    }
}

/* Compute the current entry from the generator position (video.c:135-209). */
static void walk_entry(sstv_rx *rx, walk *w)
{
    const sstv_mode *p = &rx->m->pub;
    int W = p->width, N = p->lines, nch = p->channels;
    if (nch == 4) {
        w->et = (int)round(w->rate * (w->y / 2 * p->line_s + rx->chan_start[w->ch] +
                                      p->pixel_s * 1.0 * (w->x + 0.5))) + w->skip;
        w->eloop = w->ch;
        w->ex = w->x;
        if (w->ch == 0) {
            w->ey = w->y;
            w->ech = 0;
        } else if (w->ch == 1 || w->ch == 2) {
            w->ey = w->y + w->sub;
            w->ech = w->ch;
        } else {
            w->ey = w->y + 1;
            w->ech = 0;
        }
        w->elast = (w->y == N - 2 && w->ch == 3 && w->x == W - 1);
    } else {
        int chan = w->ch;
        if (rx->m->layout == SSTV_LAYOUT_ROBOT2)
            chan = (w->ch == 1) ? ((w->y % 2 == 0) ? 1 : 2) : 0;
        w->ech = chan;
        w->eloop = w->ch;
        w->ex = w->x;
        w->ey = w->y;
        w->et = (int)round(w->rate * (w->y * p->line_s + rx->chan_start[w->ch] +
                                      (1.0 * (w->x - .5) / W * rx->chan_len[chan]))) + w->skip;
        w->elast = (w->y == N - 1 && w->ch == nch - 1 && w->x == W - 1);
    }
}

static void walk_advance(sstv_rx *rx, walk *w)
{
    const sstv_mode *p = &rx->m->pub;
    int W = p->width, N = p->lines, nch = p->channels;
    if (w->elast) {
        w->done = 1;
        return;
    }
    if (nch == 4 && (w->ch == 1 || w->ch == 2) && w->sub == 0) {
        w->sub = 1;
    } else {
        w->sub = 0;
        if (++w->x == W) {
            w->x = 0;
            if (++w->ch == nch) {
                w->ch = 0;
                w->y += (nch == 4) ? 2 : 1;
                if (w->y >= N) {
                    w->done = 1;
                    return;
                }
            }
        }
    }
    walk_entry(rx, w);
}

/* PixelGrid setup plus "first entry with Time >= 0" (video.c:211-216). */
static void walk_start(sstv_rx *rx, double rate, int skip)
{
    walk *w = &rx->w;
    memset(w, 0, sizeof(*w));
    w->rate = rate;
    w->skip = skip;
    walk_entry(rx, w);
    while (!w->done && w->et < 0) walk_advance(rx, w);
    rx->img_tag[0] = rx->img_tag[1] = rx->img_tag[2] = rx->img_tag[3] = -1;
}

static unsigned char *img_row(sstv_rx *rx, int y)
{
    int slot = y & 3;
    unsigned char *row = rx->img + (size_t)slot * SSTV_MAX_WIDTH * 3;
    if (rx->img_tag[slot] != y) {
        memset(row, 0, SSTV_MAX_WIDTH * 3);
        rx->img_tag[slot] = y;
    }
    return row;
}

/* Colour conversion of one row into the pixbuf (video.c:429-457). */
static void draw_row(sstv_rx *rx, int y)
{
    const sstv_mode *p = &rx->m->pub;
    unsigned char *im = img_row(rx, y), *out = rx->fb + (size_t)y * p->width * 3;
    int tx;
    for (tx = 0; tx < p->width; tx++) {
        const unsigned char *c = im + tx * 3;
        unsigned char *o = out + tx * 3;
        switch (p->color) {
        case SSTV_RGB:
            o[0] = c[0];
            o[1] = c[1];
            o[2] = c[2];
            break;
        case SSTV_GBR:
            o[0] = c[2];
            o[1] = c[0];
            o[2] = c[1];
            break;
        case SSTV_YUV:
            o[0] = clip((100 * c[0] + 140 * c[1] - 17850) / 100.0);
            o[1] = clip((100 * c[0] - 71 * c[1] - 33 * c[2] + 13260) / 100.0);
            o[2] = clip((100 * c[0] + 178 * c[2] - 22695) / 100.0);
            break;
        default:
            o[0] = o[1] = o[2] = c[0];
            break;
        }
    }
}

/* One PixelGrid entry (video.c:415-471). live: first pass (line events). */
static void walk_pixel(sstv_rx *rx, int live)
{
    walk *w = &rx->w;
    const sstv_mode *p = &rx->m->pub;
    unsigned char v = rx->lum[w->et];
    img_row(rx, w->ey)[w->ex * 3 + w->ech] = v;
    if (w->ech > 0 && rx->m->layout == SSTV_LAYOUT_ROBOT2)
        img_row(rx, w->ey + 1)[w->ex * 3 + w->ech] = v;
    if (w->ex == p->width - 1 || w->elast) {
        draw_row(rx, w->ey);
        if (live) {
            int final;
            if (p->channels == 4)
                final = (w->eloop == 2 && w->ey == w->y) || w->eloop == 3;
            else
                final = (w->eloop == p->channels - 1);
            if ((final && w->ex == p->width - 1) || w->elast) {
                if (w->ey + 1 > rx->rows) rx->rows = w->ey + 1;
                emit(rx, SSTV_EVENT_LINE, w->ey, 0);
            }
        }
    }
}

/* Second pass over StoredLum at the slant-corrected rate (GetVideo with Redraw = TRUE). */
static void redraw(sstv_rx *rx, double rate, int skip, int length)
{
    walk *w = &rx->w;
    int s_cur = 0;
    walk_start(rx, rate, skip);
    while (!w->done) {
        if (w->et >= length || w->et < s_cur) break;  /* never reached / stalled grid */
        s_cur = w->et;
        walk_pixel(rx, 0);
        walk_advance(rx, w);
    }
}

/* ---------------------------------------------------------------- FindSync (sync.c) */

static int has_sync_at(const sstv_rx *rx, double idx)
{
    int i;
    if (!(idx >= 0)) return 0;
    i = (int)idx;
    if (i >= rx->sync_count) return 0;                 /* slowrx UB U3 */
    return rx->has_sync[i];
}

static double find_sync(sstv_rx *rx, int *skip_out, double *slant_out)
{
    const sstv_mode *p = &rx->m->pub;
    int line_width = (int)(p->line_s / p->sync_s * 4);
    int x, y, q, d, q_most, d_most, retries = 0, xmax = 0, i;
    unsigned short x_acc[XACC];
    double rate = rx->r0, t, slant_angle = 0, s, convd, maxconvd = 0;
    static const double convo[8] = { 1, 1, 1, 1, -1, -1, -1, -1 };
    double hop = rx->sync_hop;

    if (line_width >= HOUGH_D) line_width = HOUGH_D - 1;
    while (1) {
        d_most = q_most = 0;
        memset(rx->lines, 0, sizeof(unsigned short) * HOUGH_D * NSLANT);
        for (y = 0; y < p->lines; y++) {
            for (x = 0; x < line_width; x++) {
                t = (y + 1.0 * x / line_width) * p->line_s;
                if (!has_sync_at(rx, t * rate / hop)) continue;
                for (q = MINSLANT * 2; q < MAXSLANT * 2; q++) {
                    d = line_width + (int)round(-x * rx->sin_q[q - MINSLANT * 2] +
                                                y * rx->cos_q[q - MINSLANT * 2]);
                    if (d > 0 && d < line_width) {
                        unsigned short *cell = &rx->lines[d * NSLANT + q - MINSLANT * 2];
                        unsigned short best = q_most == 0 ? 0 :     /* slowrx UB U2 */
                            rx->lines[d_most * NSLANT + q_most - MINSLANT * 2];
                        (*cell)++;
                        if (*cell > best) {
                            d_most = d;
                            q_most = q;
                        }
                    }
                }
            }
        }
        if (q_most == 0) break;                         /* no sync signal; giving up */
        slant_angle = q_most / 2.0;
        rate += tan(deg2rad(90 - slant_angle)) / line_width * rate;
        if (slant_angle > 89 && slant_angle < 91) {
            break;
        } else if (retries == 3) {
            rate = rx->r0;                              /* still slanted; giving up */
            break;
        }
        retries++;
    }
    memset(x_acc, 0, sizeof(x_acc));
    for (y = 0; y < p->lines; y++) {
        for (x = 0; x < XACC; x++) {
            t = y * p->line_s + x / 700.0 * p->line_s;
            x_acc[x] += (unsigned short)has_sync_at(rx, t / (hop / rx->r0) * rate / rx->r0);
        }
    }
    for (x = 0; x < XACC - 8; x++) {
        convd = 0;
        for (i = 0; i < 8; i++) convd += x_acc[x + i] * convo[i];
        if (convd > maxconvd) {
            maxconvd = convd;
            xmax = x + 4;
        }
    }
    if (xmax > 350) xmax -= 350;
    s = xmax / 700.0 * p->line_s - p->sync_s;
    if (rx->m->layout == SSTV_LAYOUT_SCOTTIE)
        s = s - p->pixel_s * p->width / 2.0 + p->porch_s * 2;
    *skip_out = (int)(s * rate);
    *slant_out = slant_angle;
    return rate;
}

/* ---------------------------------------------------------------- image start / finish */

static void cur_info(sstv_rx *rx)
{
    const sstv_mode *p = &rx->m->pub;
    rx->cur.mode = p->id;
    rx->cur.width = p->width;
    rx->cur.lines = p->lines;
    rx->cur.line_height = p->line_height;
    rx->cur.rows = rx->rows;
    rx->cur.shift_hz = rx->shift;
    rx->cur.snr_db = rx->snr;
}

static void video_start(sstv_rx *rx, const sstv_mode_ext *m, short shift)
{
    const sstv_mode *p = &m->pub;
    rx->m = m;
    rx->shift = shift;
    rx->base = rx->pos + rx->post_vis;
    rx->length = mode_length(m, rx->r0);
    rx->s = 0;
    chan_layout(rx);
    rx->sync_target = getbin(rx, 1200 + shift, rx->vid_fft);
    rx->next_sync = rx->sync_idx = rx->next_snr = 0;
    rx->sync_count = 0;
    rx->snr = 0;
    rx->freq = 0;
    rx->last_signal = 0;
    rx->signal_avg = 1.0;
    rx->rows_at_signal = 0;
    rx->rows = 0;
    memset(rx->fb, 0, (size_t)p->width * p->lines * 3);   /* gdk_pixbuf_fill(pixbuf_rx, 0) */
    memset(&rx->cur, 0, sizeof(rx->cur));
    rx->cur.rate = rx->r0;
    walk_start(rx, rx->r0, 0);
    cur_info(rx);
    rx->state = ST_VIDEO;
    emit(rx, SSTV_EVENT_VIS, 0, 0);
}

static void finish_image(sstv_rx *rx, int complete, int length)
{
    const sstv_mode *p = &rx->m->pub;
    int slant = rx->cfg.slant &&
                (complete || (rx->cfg.slant_partial && rx->rows >= MIN_PARTIAL_SLANT_ROWS));
    cur_info(rx);
    rx->cur.complete = complete;
    rx->cur.rate = rx->r0;
    rx->cur.skip = 0;
    if (complete && rx->cfg.fsk_id) {
        memcpy(rx->cur.fsk_id, rx->fsk_dest, sizeof(rx->cur.fsk_id));
        rx->cur.fsk_id[SSTV_FSK_ID_MAX] = '\0';
    }
    if (slant) {
        int skip = 0;
        double slant_deg = 0;
        double rate = find_sync(rx, &skip, &slant_deg);
        rx->cur.rate = rate;
        rx->cur.skip = skip;
        rx->cur.slant_deg = slant_deg;
        rx->cur.slant_corrected = 1;
        redraw(rx, rate, skip, length);
    }
    memcpy(rx->done, rx->fb, (size_t)p->width * p->lines * 3);
    rx->done_info = rx->cur;
    rx->have_done = 1;
    emit(rx, SSTV_EVENT_IMAGE, rx->rows, complete);
    hunt_reset(rx);
}

/* ---------------------------------------------------------------- GetVIS (vis.c) */

static void vis_step(sstv_rx *rx)
{
    const int n = rx->big_fft;
    const double tol = rx->cfg.vis_tolerance_hz;
    int i, j, k, maxbin = 0, vis = 0, parity, parity_bit, gotvis = 0;
    int b500 = getbin(rx, 500, n), b3300 = getbin(rx, 3300, n);
    double tone[VIS_TONES], *pwr = rx->power;
    short shift = 0;
    unsigned char bit[8] = { 0 };
    const sstv_mode_ext *sel = NULL;

    window_fft(rx, &rx->fft_big, rx->pos - rx->vis_hop, rx->hann_vis, rx->vis_win);
    for (i = 0; i <= b3300 + 1 && i < n; i++) {
        pwr[i] = pw(rx->fbuf[i]);
        if ((i >= b500 && i < b3300) && (maxbin == 0 || pwr[i] > pwr[maxbin])) maxbin = i;
    }
    /* Gaussian interpolation (vis.c:63-67) */
    if (maxbin > b500 && maxbin < b3300 && pwr[maxbin] > 0 && pwr[maxbin + 1] > 0 &&
        pwr[maxbin - 1] > 0)
        rx->hedr[rx->hedr_ptr] = maxbin + (log(pwr[maxbin + 1] / pwr[maxbin - 1])) /
                                 (2 * log(pow(pwr[maxbin], 2) /
                                          (pwr[maxbin + 1] * pwr[maxbin - 1])));
    else
        rx->hedr[rx->hedr_ptr] = rx->hedr[(rx->hedr_ptr + VIS_TONES - 1) % VIS_TONES]; /* U1 */
    rx->hedr[rx->hedr_ptr] = rx->hedr[rx->hedr_ptr] / n * rx->r0;     /* vis.c:70 */
    rx->hedr_ptr = (rx->hedr_ptr + 1) % VIS_TONES;
    for (i = 0; i < VIS_TONES; i++) tone[i] = rx->hedr[(rx->hedr_ptr + i) % VIS_TONES];

    /* vis.c:80-134 */
    for (i = 0; i < 3; i++) {
        if (shift != 0) break;
        for (j = 0; j < 3; j++) {
            if ((tone[1 * 3 + i] > tone[0 + j] - tol && tone[1 * 3 + i] < tone[0 + j] + tol) &&
                (tone[2 * 3 + i] > tone[0 + j] - tol && tone[2 * 3 + i] < tone[0 + j] + tol) &&
                (tone[3 * 3 + i] > tone[0 + j] - tol && tone[3 * 3 + i] < tone[0 + j] + tol) &&
                (tone[4 * 3 + i] > tone[0 + j] - tol && tone[4 * 3 + i] < tone[0 + j] + tol) &&
                (tone[5 * 3 + i] > tone[0 + j] - 700 - tol &&
                 tone[5 * 3 + i] < tone[0 + j] - 700 + tol) &&
                (tone[14 * 3 + i] > tone[0 + j] - 700 - tol &&
                 tone[14 * 3 + i] < tone[0 + j] - 700 + tol)) {
                gotvis = 1;
                for (k = 0; k < 8; k++) {
                    double t = tone[6 * 3 + i + 3 * k];
                    if (t > tone[0 + j] - 600 - tol && t < tone[0 + j] - 600 + tol)
                        bit[k] = 0;
                    else if (t > tone[0 + j] - 800 - tol && t < tone[0 + j] - 800 + tol)
                        bit[k] = 1;
                    else {
                        gotvis = 0;
                        break;
                    }
                }
                if (gotvis) {
                    double sh = tone[0 + j] - 1900;
                    shift = (sh > -32768 && sh < 32768) ? (short)sh : 0;
                    vis = bit[0] + (bit[1] << 1) + (bit[2] << 2) + (bit[3] << 3) +
                          (bit[4] << 4) + (bit[5] << 5) + (bit[6] << 6);
                    parity_bit = bit[7];
                    parity = bit[0] ^ bit[1] ^ bit[2] ^ bit[3] ^ bit[4] ^ bit[5] ^ bit[6];
                    sel = (const sstv_mode_ext *)NULL;
                    {
                        const sstv_mode *mp = sstv_mode_by_vis(vis);
                        if (mp) sel = sstv_mode_ext_by_id(mp->id);
                    }
                    if (sel && sel->vis_parity_odd) parity = !parity;
                    if (parity != parity_bit) {
                        gotvis = 0;                         /* parity fail */
                    } else if (!sel) {
                        gotvis = 0;                         /* unknown VIS */
                    } else {
                        break;
                    }
                }
            }
        }
    }
    if (gotvis && sel) {
        video_start(rx, sel, shift);
        return;
    }
    rx->pos += rx->vis_hop;
}

/* ---------------------------------------------------------------- GetVideo (video.c) */

static void video_step(sstv_rx *rx)
{
    const sstv_mode *p = &rx->m->pub;
    const int n = rx->vid_fft, s = rx->s;
    const long long at = rx->base + s;
    const int sh = rx->shift;
    int i;

    /* Store the sync band for later adjustments (video.c:271-298) */
    if (s == rx->next_sync) {
        double praw = 0, psync = 0;
        int b1500 = getbin(rx, 1500 + sh, n), b2300 = getbin(rx, 2300 + sh, n);
        window_seg(rx, at - rx->hann_len[1] / 2, rx->hann_vid[1], rx->hann_len[1]);
        window_dft(rx, rx->hann_len[1], b1500, b2300);
        window_dft(rx, rx->hann_len[1], rx->sync_target - 1, rx->sync_target + 1);
        for (i = b1500; i <= b2300; i++) praw += pw(rx->fbuf[i]);
        for (i = rx->sync_target - 1; i <= rx->sync_target + 1; i++)
            if (i >= 0) psync += pw(rx->fbuf[i]) * (i == rx->sync_target ? 1.0 : 0.5);
        praw /= (b2300 - b1500) > 0 ? (b2300 - b1500) : 1;
        psync /= 2.0;
        if (rx->sync_idx < rx->sync_cap) {
            rx->has_sync[rx->sync_idx] = (psync > 2 * praw);
            rx->sync_count = rx->sync_idx + 1;
        }
        rx->next_sync += rx->sync_hop;
        rx->sync_idx++;
    }

    /* Estimate SNR (video.c:304-344) */
    if (s == rx->next_snr) {
        double pvn = 0, pno = 0, pnoise, psignal;
        int vpn_bins, no_bins, rcv_bins;
        window_seg(rx, at - rx->hann_len[6] / 2, rx->hann_vid[6], rx->hann_len[6]);
        window_dft(rx, rx->hann_len[6], getbin(rx, 1500 + sh, n), getbin(rx, 2300 + sh, n));
        window_dft(rx, rx->hann_len[6], getbin(rx, 400 + sh, n), getbin(rx, 800 + sh, n));
        window_dft(rx, rx->hann_len[6], getbin(rx, 2700 + sh, n), getbin(rx, 3400 + sh, n));
        for (i = getbin(rx, 1500 + sh, n); i <= getbin(rx, 2300 + sh, n); i++)
            pvn += pw(rx->fbuf[i]);
        for (i = getbin(rx, 400 + sh, n); i <= getbin(rx, 800 + sh, n); i++)
            pno += pw(rx->fbuf[i]);
        for (i = getbin(rx, 2700 + sh, n); i <= getbin(rx, 3400 + sh, n); i++)
            pno += pw(rx->fbuf[i]);
        vpn_bins = getbin(rx, 2300, n) - getbin(rx, 1500, n) + 1;
        no_bins = getbin(rx, 800, n) - getbin(rx, 400, n) + 1 + getbin(rx, 3400, n) -
                  getbin(rx, 2700, n) + 1;
        rcv_bins = getbin(rx, 3400, n) - getbin(rx, 400, n);
        pnoise = pno * (1.0 * rcv_bins / no_bins);
        psignal = pvn - pno * (1.0 * vpn_bins / no_bins);
        rx->snr = ((psignal / pnoise < .01) ? -20 : 10 * log10(psignal / pnoise));
        {   /* station extension */
            double r = psignal / pnoise, a = rx->snr_hop / (SIGNAL_AVG_S * rx->r0);
            if (!(r > 0)) r = 0;
            if (r > 1) r = 1;
            rx->signal_avg += a * (r - rx->signal_avg);
            if (rx->signal_avg >= SIGNAL_RATIO) {
                rx->last_signal = s;
                rx->rows_at_signal = rx->rows;
            }
        }
        rx->next_snr += rx->snr_hop;
    }

    /* FM demodulation (video.c:350-400) */
    if (s % rx->demod_hop == 0) {
        int win, maxbin = 0, lo = getbin(rx, 1500 + sh, n) - 1, hi = getbin(rx, 2300 + sh, n) + 1;
        double *pwr = rx->power;
        if (lo < 1) lo = 1;
        if (!rx->cfg.adaptive) win = 0;
        else if (rx->snr >= 20) win = 0;
        else if (rx->snr >= 10) win = 1;
        else if (rx->snr >= 9) win = 2;
        else if (rx->snr >= 3) win = 3;
        else if (rx->snr >= -5) win = 4;
        else if (rx->snr >= -10) win = 5;
        else win = 6;
        if (p->id == SSTV_SDX && win < 6) win++;
        window_seg(rx, at - rx->hann_len[win] / 2, rx->hann_vid[win], rx->hann_len[win]);
        window_dft(rx, rx->hann_len[win], lo, hi);
        for (i = lo; i <= hi && i < n - 1; i++) {
            pwr[i] = pw(rx->fbuf[i]);
            if (maxbin == 0 || pwr[i] > pwr[maxbin]) maxbin = i;
        }
        if (maxbin > lo && maxbin < hi) {
            rx->freq = maxbin + (log(pwr[maxbin + 1] / pwr[maxbin - 1])) /
                       (2 * log(pow(pwr[maxbin], 2) / (pwr[maxbin + 1] * pwr[maxbin - 1])));
            rx->freq = rx->freq / n * rx->r0;
        } else {
            rx->freq = ((maxbin > getbin(rx, 1900 + sh, n)) ? 2300 : 1500) + sh;
        }
    }

    /* video.c:406 */
    rx->lum[s] = clip((rx->freq - (1500 + sh)) / 3.1372549);

    /* video.c:411-473 */
    while (!rx->w.done && rx->w.et == s) {
        walk_pixel(rx, 1);
        walk_advance(rx, &rx->w);
    }

    rx->s++;
    if (rx->s >= rx->length) {
        if (rx->cfg.fsk_id) {
            rx->pos = rx->base + rx->length;
            rx->fsk_insync = 0;
            rx->fsk_byte = 0;
            rx->fsk_bitptr = rx->fsk_byteptr = 0;
            rx->fsk_testptr = 0;
            memset(rx->fsk_testbits, 0, sizeof(rx->fsk_testbits));
            memset(rx->fsk_dest, 0, sizeof(rx->fsk_dest));
            rx->state = ST_FSK;
        } else {
            rx->pos = rx->base + rx->length;
            finish_image(rx, 1, rx->length);
        }
        return;
    }
    /* station extension: the video signal is gone -> partial image */
    if (rx->cfg.timeout_s > 0 && rx->s - rx->last_signal > rx->cfg.timeout_s * rx->r0) {
        const sstv_mode *p = &rx->m->pub;
        rx->pos = rx->base + rx->s;
        rx->rows = rx->rows_at_signal;
        /* rows drawn after the signal was gone are noise: blank them */
        memset(rx->fb + (size_t)rx->rows * p->width * 3, 0,
               (size_t)(p->lines - rx->rows) * p->width * 3);
        finish_image(rx, 0, (int)rx->last_signal);
    }
}

/* ---------------------------------------------------------------- GetFSK (fsk.c) */

static void fsk_end(sstv_rx *rx)
{
    rx->fsk_dest[rx->fsk_byteptr] = '\0';
    finish_image(rx, 1, rx->length);
}

static void fsk_step(sstv_rx *rx)
{
    static const unsigned char bitrev[64] = {
        0x00, 0x20, 0x10, 0x30, 0x08, 0x28, 0x18, 0x38, 0x04, 0x24, 0x14, 0x34, 0x0c, 0x2c, 0x1c,
        0x3c, 0x02, 0x22, 0x12, 0x32, 0x0a, 0x2a, 0x1a, 0x3a, 0x06, 0x26, 0x16, 0x36, 0x0e, 0x2e,
        0x1e, 0x3e, 0x01, 0x21, 0x11, 0x31, 0x09, 0x29, 0x19, 0x39, 0x05, 0x25, 0x15, 0x35, 0x0d,
        0x2d, 0x1d, 0x3d, 0x03, 0x23, 0x13, 0x33, 0x0b, 0x2b, 0x1b, 0x3b, 0x07, 0x27, 0x17, 0x37,
        0x0f, 0x2f, 0x1f, 0x3f };
    const int n = rx->big_fft, sh = rx->shift;
    int i, lo, mid, hi;
    double lopow = 0, hipow = 0;
    unsigned bit;

    window_fft(rx, &rx->fft_big, rx->pos - rx->fsk_half, rx->hann_fsk, rx->fsk_win);
    rx->pos += rx->fsk_insync ? rx->fsk_win : rx->fsk_half;
    lo = getbin(rx, 1900 + sh, n) - 1;
    mid = getbin(rx, 2000 + sh, n);
    hi = getbin(rx, 2100 + sh, n) + 1;
    if (lo < 0) lo = 0;
    for (i = lo; i <= hi; i++) {
        if (i < mid) lopow += pw(rx->fbuf[i]);
        else hipow += pw(rx->fbuf[i]);
    }
    bit = (lopow > hipow);
    if (!rx->fsk_insync) {
        unsigned testnum = 0;
        rx->fsk_testbits[rx->fsk_testptr % 24] = (unsigned char)bit;
        for (i = 0; i < 12; i++)
            testnum |= (unsigned)rx->fsk_testbits[(unsigned)(rx->fsk_testptr -
                                                             (unsigned)(23 - i * 2)) % 24]
                       << (11 - i);
        if (bitrev[(testnum >> 6) & 0x3f] == 0x20 && bitrev[testnum & 0x3f] == 0x2a) {
            rx->fsk_insync = 1;
            rx->fsk_byte = 0;
            rx->fsk_bitptr = 0;
            rx->fsk_byteptr = 0;
        }
        if (++rx->fsk_testptr > 200) fsk_end(rx);
    } else {
        rx->fsk_byte |= bit << rx->fsk_bitptr;
        if (++rx->fsk_bitptr == 6) {
            if (rx->fsk_byte < 0x0d || rx->fsk_byteptr > 9) {
                fsk_end(rx);
                return;
            }
            rx->fsk_dest[rx->fsk_byteptr] = (char)(rx->fsk_byte + 0x20);
            rx->fsk_bitptr = 0;
            rx->fsk_byte = 0;
            rx->fsk_byteptr++;
        }
    }
}

/* ---------------------------------------------------------------- driver */

static int step_ready(const sstv_rx *rx)
{
    long long newest = rx->count - 1;
    switch (rx->state) {
    case ST_HUNT:
        return rx->pos + rx->vis_hop - 1 <= newest;
    case ST_VIDEO:
        return rx->base + rx->s + rx->look_ahead <= newest;
    default:
        return rx->pos - rx->fsk_half + rx->fsk_win - 1 <= newest;
    }
}

static void run(sstv_rx *rx)
{
    while (step_ready(rx)) {
        switch (rx->state) {
        case ST_HUNT:
            vis_step(rx);
            break;
        case ST_VIDEO:
            video_step(rx);
            break;
        default:
            fsk_step(rx);
            break;
        }
    }
}

size_t sstv_rx_process(sstv_rx *rx, const double *x, size_t n, sstv_event *out, size_t max)
{
    size_t done = 0, block = (size_t)(rx->mask + 1) / 4;
    if (!rx) return 0;
    rx->ev = out;
    rx->ev_max = out ? max : 0;
    rx->ev_n = 0;
    while (done < n) {
        size_t k = n - done < block ? n - done : block, i;
        for (i = 0; i < k; i++) rx->ring[(rx->count + (long long)i) & rx->mask] = x[done + i];
        rx->count += (long long)k;
        done += k;
        run(rx);
    }
    if (rx->state == ST_VIDEO) cur_info(rx);
    return rx->ev_n;
}

size_t sstv_rx_flush(sstv_rx *rx, sstv_event *out, size_t max)
{
    if (!rx) return 0;
    rx->ev = out;
    rx->ev_max = out ? max : 0;
    rx->ev_n = 0;
    if (rx->state == ST_VIDEO) {
        rx->pos = rx->base + rx->s;
        finish_image(rx, 0, rx->s);
    } else if (rx->state == ST_FSK) {
        fsk_end(rx);
    }
    return rx->ev_n;
}

int sstv_rx_busy(const sstv_rx *rx) { return rx && rx->state != ST_HUNT; }

int sstv_rx_current(const sstv_rx *rx, sstv_image_info *info)
{
    if (!rx || rx->state == ST_HUNT || !rx->m) return -1;
    if (info) *info = rx->cur;
    return 0;
}

int sstv_rx_get_line(const sstv_rx *rx, int y, unsigned char *rgb)
{
    if (!rx || rx->state == ST_HUNT || !rx->m || y < 0 || y >= rx->m->pub.lines || !rgb)
        return -1;
    memcpy(rgb, rx->fb + (size_t)y * rx->m->pub.width * 3, (size_t)rx->m->pub.width * 3);
    return 0;
}

int sstv_rx_image(const sstv_rx *rx, sstv_image_info *info)
{
    if (!rx || !rx->have_done) return -1;
    if (info) *info = rx->done_info;
    return 0;
}

int sstv_rx_get_image_line(const sstv_rx *rx, int y, unsigned char *rgb)
{
    if (!rx || !rx->have_done || y < 0 || y >= rx->done_info.lines || !rgb) return -1;
    memcpy(rgb, rx->done + (size_t)y * rx->done_info.width * 3,
           (size_t)rx->done_info.width * 3);
    return 0;
}

size_t sstv_rx_events_dropped(const sstv_rx *rx) { return rx ? rx->ev_dropped : 0; }

size_t sstv_rx_footprint(const sstv_rx *rx) { return rx ? rx->footprint : 0; }
