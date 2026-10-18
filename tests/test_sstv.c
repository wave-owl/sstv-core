/*
 * sstv-core C tests.
 *
 *   test_sstv GOLDEN_DIR            run the tests (goldens in GOLDEN_DIR/loopback12000.txt)
 *   test_sstv --write-golden DIR    regenerate the goldens (reference build: macOS arm64,
 *                                   after tools/ref_check.py has shown the decoder
 *                                   byte-identical to slowrx and the encoder to pySSTV)
 *
 * Goldens: per mode the digest of the int16 TX samples at 12000 Hz, the digest of the decoded
 * picture with rate and skip, and the decoded region means. On macOS (the platform that wrote
 * them) the digests must match exactly; with another libm (glibc rounds some last bits of
 * sin/cos/log differently) the region means must agree within 1 level instead.
 *
 * SPDX-License-Identifier: ISC
 */
#include "test_util.h"

static int g_failures = 0;
static int g_checks = 0;

#define GOLDEN_RATE 12000

static int16_t *tone_buf;
static size_t tone_n, tone_cap;
static double tone_phase;

static void tone(double freq, double ms, int rate)
{
    size_t k, n = (size_t)(ms * rate / 1000.0 + 0.5);
    if (tone_n + n > tone_cap) {
        tone_cap = (tone_n + n) * 2;
        tone_buf = (int16_t *)realloc(tone_buf, tone_cap * sizeof(int16_t));
    }
    for (k = 0; k < n; k++) {
        tone_buf[tone_n++] = quantise(sin(tone_phase));
        tone_phase += 2 * M_PI * freq / rate;
    }
}

/* Calibration header + VIS with the given 7 data bits and parity bit, then 3 s of grey. */
static void header(int vis, int parity, double shift, int rate)
{
    int b;
    tone_n = 0;
    tone(0, 500, rate);
    tone(1900 + shift, 300, rate);
    tone(1200 + shift, 10, rate);
    tone(1900 + shift, 300, rate);
    tone(1200 + shift, 30, rate);
    for (b = 0; b < 7; b++) tone(((vis >> b) & 1 ? 1100 : 1300) + shift, 30, rate);
    tone((parity ? 1100 : 1300) + shift, 30, rate);
    tone(1200 + shift, 30, rate);
    tone(1900 + shift, 3000, rate);
}

static int parity_of(int vis)
{
    int b, p = 0;
    for (b = 0; b < 7; b++) p ^= (vis >> b) & 1;
    return p;
}

static void count_vis(const int16_t *s, size_t n, int rate, int *vis_events, int *mode,
                      double *shift)
{
    sstv_rx_config cfg;
    sstv_rx *rx;
    double buf[1024];
    sstv_event ev[64];
    size_t pos = 0, i, k, ne;
    sstv_rx_config_default(&cfg, rate);
    rx = sstv_rx_create(&cfg);
    *vis_events = 0;
    *mode = 0;
    *shift = 0;
    while (pos < n) {
        k = n - pos < 1024 ? n - pos : 1024;
        for (i = 0; i < k; i++) buf[i] = s[pos + i] / 32768.0;
        pos += k;
        ne = sstv_rx_process(rx, buf, k, ev, 64);
        for (i = 0; i < ne; i++)
            if (ev[i].kind == SSTV_EVENT_VIS) {
                (*vis_events)++;
                *mode = ev[i].mode;
                *shift = ev[i].shift_hz;
            }
    }
    sstv_rx_destroy(rx);
}

static void test_api(void)
{
    sstv_rx_config cfg;
    sstv_tx_config tcfg;
    sstv_rx *rx;
    sstv_tx *tx;
    unsigned char rgb[3] = { 0, 0, 0 };
    sstv_event ev[1];
    double zeros[4096];
    int i, ids[SSTV_MODE_LAST + 1] = { 0 };

    CHECK(strstr(sstv_version(), "contract=sstv-c1 abi=1 tx=1") != NULL, "version %s",
          sstv_version());
    CHECK(sstv_abi() == SSTV_ABI, "abi");
    CHECK(sstv_sizeof(SSTV_SIZEOF_MODE) == sizeof(sstv_mode) &&
          sstv_sizeof(SSTV_SIZEOF_RX_CONFIG) == sizeof(sstv_rx_config) &&
          sstv_sizeof(SSTV_SIZEOF_EVENT) == sizeof(sstv_event) &&
          sstv_sizeof(SSTV_SIZEOF_IMAGE_INFO) == sizeof(sstv_image_info) &&
          sstv_sizeof(SSTV_SIZEOF_TX_CONFIG) == sizeof(sstv_tx_config) && sstv_sizeof(99) == 0,
          "sstv_sizeof");
    CHECK(sstv_mode_count() == 25, "25 modes, got %d", sstv_mode_count());
    for (i = 0; i < sstv_mode_count(); i++) {
        const sstv_mode *m = sstv_mode_at(i);
        CHECK(m && m->id == i + 1, "mode %d id", i);
        CHECK(sstv_mode_by_vis(m->vis) == m, "VIS map %s", m->short_name);
        CHECK(sstv_mode_find(m->short_name) == m && sstv_mode_find(m->name) == m,
              "find %s", m->short_name);
        CHECK(m->tx == 1 && m->tx_width == m->width, "tx %s", m->short_name);
        ids[m->id]++;
    }
    for (i = 1; i <= SSTV_MODE_LAST; i++) CHECK(ids[i] == 1, "id %d once", i);
    CHECK(sstv_mode_find("pd120") == sstv_mode_by_id(SSTV_PD120), "case-insensitive find");
    CHECK(sstv_mode_find("nope") == NULL && sstv_mode_by_vis(0x01) == NULL &&
          sstv_mode_at(25) == NULL && sstv_mode_by_id(0) == NULL, "unknown lookups");

    sstv_rx_config_default(&cfg, 7999);
    CHECK(sstv_rx_create(&cfg) == NULL, "rate below range refused");
    sstv_rx_config_default(&cfg, 48001);
    CHECK(sstv_rx_create(&cfg) == NULL, "rate above range refused");
    sstv_rx_config_default(&cfg, 12000);
    cfg.timeout_s = -1;
    CHECK(sstv_rx_create(&cfg) == NULL, "negative timeout refused");
    sstv_rx_config_default(&cfg, 12000);
    rx = sstv_rx_create(&cfg);
    CHECK(rx != NULL, "rx create");
    CHECK(sstv_rx_footprint(rx) > 1000000 && sstv_rx_footprint(rx) < 20000000,
          "footprint %zu", sstv_rx_footprint(rx));
    CHECK(sstv_rx_image(rx, NULL) == -1 && sstv_rx_current(rx, NULL) == -1 &&
          sstv_rx_get_line(rx, 0, rgb) == -1 && !sstv_rx_busy(rx), "idle state");
    memset(zeros, 0, sizeof(zeros));
    CHECK(sstv_rx_process(rx, zeros, 4096, ev, 1) == 0, "silence: no events");
    CHECK(sstv_rx_flush(rx, ev, 1) == 0, "flush while idle: no event");
    sstv_rx_destroy(rx);
    sstv_rx_destroy(NULL);

    sstv_tx_config_default(&tcfg, SSTV_M1, 12000);
    tcfg.fsk_id = "dg6fl";                       /* lower case is outside ' ' .. '_' */
    CHECK(sstv_tx_create(&tcfg) == NULL, "FSK ID lower case refused");
    tcfg.fsk_id = "ABCDEFGHIJK";
    CHECK(sstv_tx_create(&tcfg) == NULL, "FSK ID longer than 10 refused");
    tcfg.fsk_id = NULL;
    tcfg.mode = 99;
    CHECK(sstv_tx_create(&tcfg) == NULL, "unknown mode refused");
    sstv_tx_config_default(&tcfg, SSTV_R36, 12000);
    tx = sstv_tx_create(&tcfg);
    CHECK(tx != NULL, "tx create");
    CHECK(sstv_tx_put_image(tx, rgb, 1, 1) == -1, "wrong picture size refused");
    CHECK(sstv_tx_render(tx, zeros, 16) == 0, "no render before the picture");
    sstv_tx_destroy(tx);
    sstv_tx_destroy(NULL);
}

/* TX line duration against the mode table (exposes rows that do not add up). */
static void test_line_times(void)
{
    int i;
    for (i = 0; i < sstv_mode_count(); i++) {
        const sstv_mode *m = sstv_mode_at(i);
        sstv_tx_config cfg;
        sstv_tx *tx;
        double header_s = 0.910, frames, line;
        sstv_tx_config_default(&cfg, m->id, 48000);
        tx = sstv_tx_create(&cfg);
        frames = m->channels == 4 ? m->lines / 2 : m->lines;
        line = ((double)sstv_tx_total_samples(tx) / 48000 - header_s) / frames;
        CHECK(fabs(line - m->line_s) / m->line_s < 1.5e-3, "%s TX line %.4f ms, table %.4f ms",
              m->short_name, line * 1e3, m->line_s * 1e3);
        sstv_tx_destroy(tx);
    }
}

static void test_vis(void)
{
    int rate, ev, mode;
    double shift;
    for (rate = 12000; rate <= 44100; rate += 32100) {
        header(0x2c, parity_of(0x2c), 0, rate);
        count_vis(tone_buf, tone_n, rate, &ev, &mode, &shift);
        CHECK(ev == 1 && mode == SSTV_M1, "VIS M1 at %d: %d events mode %d", rate, ev, mode);
        header(0x2c, !parity_of(0x2c), 0, rate);
        count_vis(tone_buf, tone_n, rate, &ev, &mode, &shift);
        CHECK(ev == 0, "flipped parity at %d: %d events", rate, ev);
        header(0x06, !parity_of(0x06), 0, rate);
        count_vis(tone_buf, tone_n, rate, &ev, &mode, &shift);
        CHECK(ev == 1 && mode == SSTV_R12BW, "R12BW odd parity at %d", rate);
        header(0x06, parity_of(0x06), 0, rate);
        count_vis(tone_buf, tone_n, rate, &ev, &mode, &shift);
        CHECK(ev == 0, "R12BW even parity refused at %d", rate);
        header(0x01, parity_of(0x01), 0, rate);
        count_vis(tone_buf, tone_n, rate, &ev, &mode, &shift);
        CHECK(ev == 0, "unmapped VIS 0x01 at %d: %d events", rate, ev);
        header(0x5f, parity_of(0x5f), 60, rate);
        count_vis(tone_buf, tone_n, rate, &ev, &mode, &shift);
        CHECK(ev == 1 && mode == SSTV_PD120 && fabs(shift - 60) <= 12,
              "shifted header at %d: mode %d shift %.1f", rate, mode, shift);
    }
}

/* Ten minutes of white noise and of silence: never a VIS (no false image). */
static void test_no_false_image(void)
{
    const int rate = 12000;
    size_t n = (size_t)rate * 600, i;
    int16_t *s = (int16_t *)malloc(n * sizeof(int16_t));
    uint32_t seed = 12345;
    int ev, mode;
    double shift;
    for (i = 0; i < n; i++) {
        double u1, u2;
        seed = seed * 1664525u + 1013904223u;
        u1 = (seed + 1.0) / 4294967297.0;
        seed = seed * 1664525u + 1013904223u;
        u2 = (seed + 1.0) / 4294967297.0;
        s[i] = quantise(0.3 * sqrt(-2 * log(u1)) * cos(2 * M_PI * u2));
    }
    count_vis(s, n, rate, &ev, &mode, &shift);
    CHECK(ev == 0, "white noise: %d VIS events", ev);
    memset(s, 0, n * sizeof(int16_t));
    count_vis(s, n, rate, &ev, &mode, &shift);
    CHECK(ev == 0, "silence: %d VIS events", ev);
    free(s);
}

typedef struct golden {
    char name[16];
    unsigned long long tx, rx;
    double means[N_REGIONS][3];
} golden;

static golden goldens[32];
static int n_goldens;

static void load_goldens(const char *dir)
{
    char path[1024];
    FILE *f;
    snprintf(path, sizeof(path), "%s/loopback%d.txt", dir, GOLDEN_RATE);
    f = fopen(path, "r");
    if (!f) return;
    while (n_goldens < 32) {
        golden *g = &goldens[n_goldens];
        int r, c, ok = fscanf(f, "%15s %llx %llx", g->name, &g->tx, &g->rx) == 3;
        for (r = 0; ok && r < N_REGIONS; r++)
            for (c = 0; c < 3; c++) ok = ok && fscanf(f, "%lf", &g->means[r][c]) == 1;
        if (!ok) break;
        n_goldens++;
    }
    fclose(f);
}

static const golden *find_golden(const char *name)
{
    int i;
    for (i = 0; i < n_goldens; i++)
        if (!strcmp(goldens[i].name, name)) return &goldens[i];
    return NULL;
}

/* TX -> RX of the frozen test picture for every mode at the station rate, against the
 * region bound and the goldens. */
static void test_loopback(const char *golden_dir, FILE *write)
{
    int i;
    if (!write) load_goldens(golden_dir);
    CHECK(write || n_goldens == 25, "goldens loaded: %d", n_goldens);
    for (i = 0; i < sstv_mode_count(); i++) {
        const sstv_mode *m = sstv_mode_at(i);
        size_t n;
        unsigned char *src = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
        int16_t *s;
        sstv_rx_config cfg;
        decoded d;
        double means[N_REGIONS][3], err;
        uint64_t htx, hrx;
        char meta[64];
        int r, c;
        test_pattern(m->tx_width, m->lines, src);
        s = render_over(m->id, GOLDEN_RATE, src, "DG6FL", &n);
        sstv_rx_config_default(&cfg, GOLDEN_RATE);
        decode_audio(s, n, &cfg, 4096, &d);
        CHECK(d.images == 1 && d.info.mode == m->id && d.vis_events == 1,
              "%s: %d images, mode %d", m->short_name, d.images, d.info.mode);
        if (d.images != 1 || d.info.mode != m->id) {
            free(src);
            free(s);
            free(d.rgb);
            continue;
        }
        CHECK(d.info.rows == m->lines && d.info.slant_corrected,
              "%s: rows %d complete %d", m->short_name, d.info.rows, d.info.complete);
        CHECK(!strcmp(d.info.fsk_id, "DG6FL"), "%s: FSK ID \"%s\"", m->short_name,
              d.info.fsk_id);
        CHECK(d.line_events == m->lines, "%s: %d line events", m->short_name, d.line_events);
        err = region_error(m, d.rgb, means);
        CHECK(err <= 12.0, "%s: region error %.1f", m->short_name, err);
        htx = fnv1a(s, n * sizeof(int16_t), FNV_INIT);
        snprintf(meta, sizeof(meta), "%.6f %d", d.info.rate, d.info.skip);
        hrx = fnv1a(d.rgb, (size_t)m->width * m->lines * 3, FNV_INIT);
        hrx = fnv1a(meta, strlen(meta), hrx);
        if (write) {
            fprintf(write, "%s %016llx %016llx", m->short_name, (unsigned long long)htx,
                    (unsigned long long)hrx);
            for (r = 0; r < N_REGIONS; r++)
                for (c = 0; c < 3; c++) fprintf(write, " %.3f", means[r][c]);
            fprintf(write, "\n");
        } else {
            const golden *g = find_golden(m->short_name);
            CHECK(g != NULL, "%s: golden present", m->short_name);
            if (g) {
#ifdef __APPLE__
                CHECK(g->tx == htx, "%s: TX digest %016llx, golden %016llx", m->short_name,
                      (unsigned long long)htx, g->tx);
                CHECK(g->rx == hrx, "%s: RX digest %016llx, golden %016llx", m->short_name,
                      (unsigned long long)hrx, g->rx);
#else
                double worst = 0;
                for (r = 0; r < N_REGIONS; r++)
                    for (c = 0; c < 3; c++)
                        if (fabs(means[r][c] - g->means[r][c]) > worst)
                            worst = fabs(means[r][c] - g->means[r][c]);
                CHECK(worst <= 1.0, "%s: region means differ from golden by %.2f",
                      m->short_name, worst);
                if (g->tx != htx || g->rx != hrx)
                    printf("note: %s digests differ from the macOS golden (libm)\n",
                           m->short_name);
#endif
            }
        }
        printf("loopback %-6s region error %5.2f  rate %.3f skip %d\n", m->short_name, err,
               d.info.rate, d.info.skip);
        free(src);
        free(s);
        free(d.rgb);
    }
}

/* Same audio fed in different chunk sizes gives the same picture. */
static void test_chunks(void)
{
    const sstv_mode *m = sstv_mode_by_id(SSTV_R36);
    size_t n, np = (size_t)m->width * m->lines * 3, chunks[3] = { 1, 37, 48000 };
    unsigned char *src = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
    int16_t *s;
    sstv_rx_config cfg;
    decoded d[3];
    int k;
    test_pattern(m->tx_width, m->lines, src);
    s = render_over(m->id, 12000, src, NULL, &n);
    sstv_rx_config_default(&cfg, 12000);
    for (k = 0; k < 3; k++) decode_audio(s, n, &cfg, chunks[k], &d[k]);
    for (k = 1; k < 3; k++)
        CHECK(d[k].images == 1 && d[0].images == 1 && !memcmp(d[0].rgb, d[k].rgb, np) &&
              d[0].info.skip == d[k].info.skip && d[0].info.rate == d[k].info.rate,
              "chunk size %zu differs from 1", chunks[k]);
    CHECK(d[0].info.fsk_id[0] == '\0', "no FSK ID sent -> none decoded");
    for (k = 0; k < 3; k++) free(d[k].rgb);
    free(src);
    free(s);
}

/* A +0.15 % clock error is slant-corrected (FindSync), and the picture is better than
 * without correction. */
static void test_slant(void)
{
    const sstv_mode *m = sstv_mode_by_id(SSTV_M1);
    size_t n, n2, i, np = (size_t)m->width * m->lines * 3;
    unsigned char *src = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
    unsigned char *ref = (unsigned char *)malloc(np);
    int16_t *s, *s2;
    sstv_rx_config cfg;
    decoded on, off;
    double p_on, p_off;
    test_pattern(m->tx_width, m->lines, src);
    nearest(src, m->tx_width, m->lines, ref, m->width, m->lines);
    s = render_over(m->id, 12000, src, NULL, &n);
    n2 = (size_t)(n * 1.0015);
    s2 = (int16_t *)malloc(n2 * sizeof(int16_t));
    for (i = 0; i < n2; i++) {
        double t = i / 1.0015;
        size_t j = (size_t)t;
        double f = t - j;
        s2[i] = (int16_t)lround(j + 1 < n ? s[j] * (1 - f) + s[j + 1] * f : 0);
    }
    sstv_rx_config_default(&cfg, 12000);
    decode_audio(s2, n2, &cfg, 4096, &on);
    cfg.slant = 0;
    decode_audio(s2, n2, &cfg, 4096, &off);
    CHECK(on.images == 1 && off.images == 1, "slant: images");
    if (on.images == 1 && off.images == 1) {
        p_on = psnr(on.rgb, ref, np);
        p_off = psnr(off.rgb, ref, np);
        CHECK(on.info.slant_corrected && fabs(on.info.rate / 12000.0 - 1.0015) < 2e-4,
              "slant: corrected rate %.3f (expected %.3f)", on.info.rate, 12000 * 1.0015);
        CHECK(on.info.slant_deg > 89 && on.info.slant_deg < 91, "slant angle %.1f",
              on.info.slant_deg);
        CHECK(p_on > p_off + 3, "slant: PSNR corrected %.2f, uncorrected %.2f", p_on, p_off);
        printf("slant: rate %.3f, PSNR %.2f dB corrected vs %.2f dB uncorrected\n",
               on.info.rate, p_on, p_off);
    }
    free(on.rgb);
    free(off.rgb);
    free(src);
    free(ref);
    free(s);
    free(s2);
}

/* Signal loss: with timeout_s the image ends as partial with the rows received; without it
 * (slowrx) the receiver waits for the full image time. flush() ends an image as partial. */
static void test_partial(void)
{
    const sstv_mode *m = sstv_mode_by_id(SSTV_M1);
    size_t n, cut, i;
    unsigned char *src = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
    int16_t *s, *t;
    sstv_rx_config cfg;
    decoded d;
    test_pattern(m->tx_width, m->lines, src);
    s = render_over(m->id, 12000, src, NULL, &n);
    cut = (size_t)(12000 * (0.91 + 100 * m->line_s));        /* VIS + 100 lines */
    t = (int16_t *)calloc(n + 12000 * 20, sizeof(int16_t));
    memcpy(t, s, cut * sizeof(int16_t));
    {
        uint32_t seed = 7;                                     /* weak noise after the cut */
        for (i = cut; i < n + 12000 * 20; i++) {
            seed = seed * 1664525u + 1013904223u;
            t[i] = (int16_t)((int)(seed >> 22) - 512);
        }
    }
    sstv_rx_config_default(&cfg, 12000);
    cfg.timeout_s = 3.0;
    decode_audio(t, n + 12000 * 20, &cfg, 4096, &d);
    CHECK(d.images == 1 && !d.info.complete && abs(d.info.rows - 100) <= 3,
          "timeout: %d images complete %d rows %d", d.images, d.info.complete, d.info.rows);
    free(d.rgb);
    cfg.timeout_s = 0;
    decode_audio(t, n + 12000 * 20, &cfg, 4096, &d);
    CHECK(d.images == 1 && d.info.complete && d.info.rows == 256,
          "no timeout (slowrx): complete %d rows %d", d.info.complete, d.info.rows);
    free(d.rgb);
    decode_audio(t, cut, &cfg, 4096, &d);             /* flush at the end of the input */
    CHECK(d.images == 1 && !d.info.complete && abs(d.info.rows - 100) <= 3,
          "flush: complete %d rows %d", d.info.complete, d.info.rows);
    free(d.rgb);
    free(src);
    free(s);
    free(t);
}

/* Event overflow is counted, not lost silently. */
static void test_event_overflow(void)
{
    const sstv_mode *m = sstv_mode_by_id(SSTV_R8BW);
    size_t n, pos = 0, k, i;
    unsigned char *src = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
    int16_t *s;
    sstv_rx_config cfg;
    sstv_rx *rx;
    double buf[12000];
    test_pattern(m->tx_width, m->lines, src);
    s = render_over(m->id, 12000, src, NULL, &n);
    sstv_rx_config_default(&cfg, 12000);
    rx = sstv_rx_create(&cfg);
    while (pos < n) {
        k = n - pos < 12000 ? n - pos : 12000;
        for (i = 0; i < k; i++) buf[i] = s[pos + i] / 32768.0;
        pos += k;
        sstv_rx_process(rx, buf, k, NULL, 0);
    }
    CHECK(sstv_rx_events_dropped(rx) > 100, "dropped events counted: %zu",
          sstv_rx_events_dropped(rx));
    sstv_rx_destroy(rx);
    free(src);
    free(s);
}

int main(int argc, char **argv)
{
    if (argc >= 3 && !strcmp(argv[1], "--write-golden")) {
        char path[1024];
        FILE *f;
        snprintf(path, sizeof(path), "%s/loopback%d.txt", argv[2], GOLDEN_RATE);
        f = fopen(path, "w");
        if (!f) return 1;
        test_loopback(NULL, f);
        fclose(f);
        printf("wrote %s (%d checks, %d failures)\n", path, g_checks, g_failures);
        return g_failures ? 1 : 0;
    }
    test_api();
    test_line_times();
    test_vis();
    test_no_false_image();
    test_chunks();
    test_slant();
    test_partial();
    test_event_overflow();
    test_loopback(argc > 1 ? argv[1] : "tests/golden", NULL);
    printf("%d checks, %d failures\n", g_checks, g_failures);
    free(tone_buf);
    return g_failures ? 1 : 0;
}
