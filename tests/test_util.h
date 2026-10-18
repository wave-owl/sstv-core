/* Test helpers shared by the tests, the bench and tools/sstv_tool.c.
 * SPDX-License-Identifier: ISC */
#ifndef SSTV_TEST_UTIL_H
#define SSTV_TEST_UTIL_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sstv.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* CHECK() counts into g_checks / g_failures, which the test program defines. */
#define CHECK(cond, ...)                                                                    \
    do {                                                                                    \
        g_checks++;                                                                         \
        if (!(cond)) {                                                                      \
            g_failures++;                                                                   \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);                            \
            fprintf(stderr, __VA_ARGS__);                                                   \
            fprintf(stderr, "\n");                                                          \
        }                                                                                   \
    } while (0)

/* The frozen test picture (tools/ref_check.py test_pattern() is the same function):
 * top third eight colour bars, middle third a checkerboard, bottom third a grey ramp over
 * the left half and red/green/blue blocks over the right half. */
static inline void test_pattern(int w, int h, unsigned char *rgb)
{
    static const unsigned char bars[8][3] = {
        { 255, 255, 255 }, { 255, 255, 0 }, { 0, 255, 255 }, { 0, 255, 0 },
        { 255, 0, 255 },   { 255, 0, 0 },   { 0, 0, 255 },   { 0, 0, 0 } };
    int x, y, block = w / 20 > 2 ? w / 20 : 2;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            unsigned char *p = rgb + ((size_t)y * w + x) * 3;
            if (y < h / 3) {
                const unsigned char *c = bars[x * 8 / w];
                p[0] = c[0];
                p[1] = c[1];
                p[2] = c[2];
            } else if (y < 2 * h / 3) {
                unsigned char v = (((x / block) + (y / block)) % 2) ? 255 : 0;
                p[0] = p[1] = p[2] = v;
            } else if (x < w / 2) {
                unsigned char v = (unsigned char)(x * 255 / (w / 2 - 1));
                p[0] = p[1] = p[2] = v;
            } else {
                int k = (x - w / 2) * 3 / (w - w / 2);
                p[0] = (unsigned char)(k == 0 ? 220 : 30);
                p[1] = (unsigned char)(k == 1 ? 220 : 30);
                p[2] = (unsigned char)(k == 2 ? 220 : 30);
            }
        }
    }
}

/* Deterministic quantisation of TX samples to int16 at -6 dBFS (the WAV files the
 * reference programs decode). */
static inline int16_t quantise(double v)
{
    long q = lround(v * 16384.0);
    if (q > 32767) q = 32767;
    if (q < -32768) q = -32768;
    return (int16_t)q;
}

/* Render a whole over into a freshly allocated int16 array. */
static inline int16_t *render_over(int mode, int rate, const unsigned char *rgb, const char *fsk,
                            size_t *n_out)
{
    sstv_tx_config cfg;
    sstv_tx *tx;
    const sstv_mode *m = sstv_mode_by_id(mode);
    size_t total, n = 0, k, i;
    double buf[4096];
    int16_t *out;
    sstv_tx_config_default(&cfg, mode, rate);
    cfg.fsk_id = fsk;
    tx = sstv_tx_create(&cfg);
    if (!tx || !m) return NULL;
    total = sstv_tx_total_samples(tx);
    out = (int16_t *)malloc((total + 1) * sizeof(int16_t));
    if (!out || sstv_tx_put_image(tx, rgb, m->tx_width, m->lines) != 0) {
        free(out);
        sstv_tx_destroy(tx);
        return NULL;
    }
    while ((k = sstv_tx_render(tx, buf, 4096)) > 0) {
        for (i = 0; i < k && n < total; i++) out[n++] = quantise(buf[i]);
    }
    sstv_tx_destroy(tx);
    *n_out = n;
    return out;
}

static inline int write_wav(const char *path, const int16_t *s, size_t n, int rate)
{
    FILE *f = fopen(path, "wb");
    uint32_t v32;
    uint16_t v16;
    if (!f) return -1;
#define W32(x) (v32 = (uint32_t)(x), fwrite(&v32, 4, 1, f))
#define W16(x) (v16 = (uint16_t)(x), fwrite(&v16, 2, 1, f))
    fwrite("RIFF", 1, 4, f);
    W32(36 + n * 2);
    fwrite("WAVEfmt ", 1, 8, f);
    W32(16);
    W16(1);
    W16(1);
    W32(rate);
    W32(rate * 2);
    W16(2);
    W16(16);
    fwrite("data", 1, 4, f);
    W32(n * 2);
    fwrite(s, 2, n, f);
#undef W32
#undef W16
    fclose(f);
    return 0;
}

/* Minimal reader for mono 16-bit PCM WAV files (the files written above and by Python's
 * wave module). Returns a malloc'd array, NULL on error. */
static inline int16_t *read_wav(const char *path, size_t *n_out, int *rate_out)
{
    FILE *f = fopen(path, "rb");
    unsigned char hdr[12], ck[8];
    int16_t *data = NULL;
    int rate = 0, fmt_ok = 0;
    if (!f) return NULL;
    if (fread(hdr, 1, 12, f) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) {
        fclose(f);
        return NULL;
    }
    while (fread(ck, 1, 8, f) == 8) {
        uint32_t len = ck[4] | (ck[5] << 8) | (ck[6] << 16) | ((uint32_t)ck[7] << 24);
        if (!memcmp(ck, "fmt ", 4)) {
            unsigned char fmt[16];
            if (len < 16 || fread(fmt, 1, 16, f) != 16) break;
            rate = fmt[4] | (fmt[5] << 8) | (fmt[6] << 16) | (fmt[7] << 24);
            fmt_ok = (fmt[0] == 1 && fmt[2] == 1 && fmt[14] == 16);
            if (len > 16) fseek(f, (long)(len - 16), SEEK_CUR);
        } else if (!memcmp(ck, "data", 4) && fmt_ok) {
            data = (int16_t *)malloc(len + 2);
            if (data && fread(data, 1, len, f) == len) {
                *n_out = len / 2;
                *rate_out = rate;
            } else {
                free(data);
                data = NULL;
            }
            break;
        } else {
            fseek(f, (long)(len + (len & 1)), SEEK_CUR);
        }
    }
    fclose(f);
    return data;
}

typedef struct decoded {
    int images;
    sstv_image_info info;
    unsigned char *rgb;            /* info.width x info.lines x 3 of the last image */
    int vis_events, line_events;
} decoded;

/* Decode int16 audio in chunks; keeps the last finished image. */
static inline void decode_audio(const int16_t *s, size_t n, const sstv_rx_config *cfg,
                                size_t chunk,
                         decoded *d)
{
    sstv_rx *rx = sstv_rx_create(cfg);
    double *buf = (double *)malloc(chunk * sizeof(double));
    sstv_event ev[256];
    size_t pos = 0, k, i, ne;
    memset(d, 0, sizeof(*d));
    if (!rx || !buf) {
        free(buf);
        sstv_rx_destroy(rx);
        return;
    }
    for (;;) {
        int flush = pos >= n;
        if (!flush) {
            k = n - pos < chunk ? n - pos : chunk;
            for (i = 0; i < k; i++) buf[i] = s[pos + i] / 32768.0;
            pos += k;
            ne = sstv_rx_process(rx, buf, k, ev, 256);
        } else {
            ne = sstv_rx_flush(rx, ev, 256);
        }
        for (i = 0; i < ne; i++) {
            if (ev[i].kind == SSTV_EVENT_VIS) d->vis_events++;
            if (ev[i].kind == SSTV_EVENT_LINE) d->line_events++;
            if (ev[i].kind == SSTV_EVENT_IMAGE) {
                int y;
                d->images++;
                sstv_rx_image(rx, &d->info);
                free(d->rgb);
                d->rgb = (unsigned char *)malloc((size_t)d->info.width * d->info.lines * 3);
                for (y = 0; y < d->info.lines; y++)
                    sstv_rx_get_image_line(rx, y, d->rgb + (size_t)y * d->info.width * 3);
            }
        }
        if (flush) break;
    }
    free(buf);
    sstv_rx_destroy(rx);
}

/* Peak signal-to-noise ratio of two RGB pictures, dB (99 for identical). */
static inline double psnr(const unsigned char *a, const unsigned char *b, size_t n)
{
    double se = 0;
    size_t i;
    for (i = 0; i < n; i++) {
        double d = (double)a[i] - b[i];
        se += d * d;
    }
    if (se == 0) return 99.0;
    return 10 * log10(255.0 * 255.0 / (se / n));
}

/* Resample a picture to w x h by nearest neighbour (TX picture -> RX geometry). */
static inline void nearest(const unsigned char *src, int sw, int sh, unsigned char *dst, int w,
                           int h)
{
    int x, y;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            memcpy(dst + ((size_t)y * w + x) * 3,
                   src + ((size_t)(y * sh / h) * sw + (size_t)(x * sw / w)) * 3, 3);
}

/* Expected colour of the source picture as the mode carries it: the RGB pixel, or Pillow's L
 * for the B/W modes (what the transmitter sends). */
static inline void expected_rgb(const sstv_mode *m, const unsigned char *src,
                                unsigned char *out)
{
    if (m->color == SSTV_BW) {
        unsigned long v = (unsigned long)src[0] * 19595UL + (unsigned long)src[1] * 38470UL +
                          (unsigned long)src[2] * 7471UL + 0x8000UL;
        out[0] = out[1] = out[2] = (unsigned char)(v >> 16);
    } else {
        memcpy(out, src, 3);
    }
}

/* Region metric: the eight colour bars and the three colour blocks of test_pattern(), each
 * inset by 20 % from its edges; returns the largest |mean decoded - expected| over regions
 * and channels and fills means[11][3] with the decoded region means. */
#define N_REGIONS 11
static inline double region_error(const sstv_mode *m, const unsigned char *dec,
                                  double means[][3])
{
    int r, w = m->width, h = m->lines, tw = m->tx_width;
    double worst = 0;
    unsigned char *src = (unsigned char *)malloc((size_t)tw * h * 3);
    test_pattern(tw, h, src);
    for (r = 0; r < N_REGIONS; r++) {
        double x0, x1, y0, y1, sum[3] = { 0, 0, 0 }, cnt = 0;
        int x, y, c, cx, cy;
        unsigned char exp[3];
        if (r < 8) {
            x0 = (r + 0.2) / 8 * w;
            x1 = (r + 0.8) / 8 * w;
            y0 = 0.1 * h;
            y1 = 0.23 * h;
        } else {
            x0 = (0.5 + (r - 8 + 0.2) / 6) * w;
            x1 = (0.5 + (r - 8 + 0.8) / 6) * w;
            y0 = 0.75 * h;
            y1 = 0.92 * h;
        }
        for (y = (int)y0; y < (int)y1; y++)
            for (x = (int)x0; x < (int)x1; x++) {
                for (c = 0; c < 3; c++) sum[c] += dec[((size_t)y * w + x) * 3 + c];
                cnt++;
            }
        cx = (int)((x0 + x1) / 2 * tw / w);
        cy = (int)((y0 + y1) / 2);
        expected_rgb(m, src + ((size_t)cy * tw + cx) * 3, exp);
        for (c = 0; c < 3; c++) {
            double mean = cnt > 0 ? sum[c] / cnt : 0, e = fabs(mean - exp[c]);
            if (means) means[r][c] = mean;
            if (e > worst) worst = e;
        }
    }
    free(src);
    return worst;
}

/* FNV-1a 64-bit digest. */
static inline uint64_t fnv1a(const void *data, size_t n, uint64_t h)
{
    const unsigned char *p = (const unsigned char *)data;
    size_t i;
    for (i = 0; i < n; i++) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}
#define FNV_INIT 14695981039346656037ULL

#endif
