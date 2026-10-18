/* sstv_tool -- command line front end used by tools/ref_check.py and for experiments.
 *
 *   sstv_tool modes                              mode table as TSV
 *   sstv_tool pattern MODE OUT.ppm               the frozen test picture of a mode (TX size)
 *   sstv_tool encode MODE RATE IN.ppm OUT.wav [FSKID]   TX -> int16 WAV (test_util quantise)
 *   sstv_tool raw MODE RATE IN.ppm OUT.f64 [FSKID]      TX -> raw float64 samples
 *   sstv_tool decode IN.wav OUT_PREFIX [timeout_s] [slant]  RX -> OUT_PREFIX<k>.ppm + .txt
 *   sstv_tool loopback RATE                      TX->RX of the test picture, every mode
 *
 * SPDX-License-Identifier: ISC */
#include "../tests/test_util.h"

static int write_ppm(const char *path, const unsigned char *rgb, int w, int h)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(rgb, 3, (size_t)w * h, f);
    fclose(f);
    return 0;
}

static unsigned char *read_ppm(const char *path, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    unsigned char *rgb;
    int maxv;
    if (!f) return NULL;
    if (fscanf(f, "P6 %d %d %d", w, h, &maxv) != 3 || maxv != 255) {
        fclose(f);
        return NULL;
    }
    fgetc(f);
    rgb = (unsigned char *)malloc((size_t)*w * *h * 3);
    if (rgb && fread(rgb, 3, (size_t)*w * *h, f) != (size_t)*w * *h) {
        free(rgb);
        rgb = NULL;
    }
    fclose(f);
    return rgb;
}

static const sstv_mode *mode_arg(const char *s)
{
    const sstv_mode *m = sstv_mode_find(s);
    if (!m) fprintf(stderr, "unknown mode %s\n", s);
    return m;
}

static int cmd_decode(int argc, char **argv)
{
    size_t n;
    int rate, k = 0;
    int16_t *s = read_wav(argv[2], &n, &rate);
    sstv_rx_config cfg;
    sstv_rx *rx;
    double buf[4096];
    sstv_event ev[256];
    size_t pos = 0;
    if (!s) {
        fprintf(stderr, "cannot read %s\n", argv[2]);
        return 1;
    }
    sstv_rx_config_default(&cfg, rate);
    if (argc > 4) cfg.timeout_s = atof(argv[4]);
    if (argc > 5) cfg.slant = atoi(argv[5]);
    rx = sstv_rx_create(&cfg);
    if (!rx) return 1;
    for (;;) {
        size_t c = 0, i, ne;
        int flush = pos >= n;
        if (!flush) {
            c = n - pos < 4096 ? n - pos : 4096;
            for (i = 0; i < c; i++) buf[i] = s[pos + i] / 32768.0;
            pos += c;
            ne = sstv_rx_process(rx, buf, c, ev, 256);
        } else {
            ne = sstv_rx_flush(rx, ev, 256);
        }
        for (i = 0; i < ne; i++) {
            if (ev[i].kind == SSTV_EVENT_IMAGE) {
                sstv_image_info info;
                char path[1024];
                unsigned char *rgb;
                int y;
                FILE *f;
                sstv_rx_image(rx, &info);
                rgb = (unsigned char *)malloc((size_t)info.width * info.lines * 3);
                for (y = 0; y < info.lines; y++)
                    sstv_rx_get_image_line(rx, y, rgb + (size_t)y * info.width * 3);
                snprintf(path, sizeof(path), "%s%d.ppm", argv[3], k);
                write_ppm(path, rgb, info.width, info.lines);
                snprintf(path, sizeof(path), "%s%d.txt", argv[3], k);
                f = fopen(path, "w");
                if (f) {
                    fprintf(f, "mode=%s\nrows=%d\ncomplete=%d\nshift=%g\nrate=%.6f\nskip=%d\n"
                               "slant=%.1f\nslant_corrected=%d\nsnr=%.2f\nfsk_id=%s\n",
                            sstv_mode_by_id(info.mode)->short_name, info.rows, info.complete,
                            info.shift_hz, info.rate, info.skip, info.slant_deg,
                            info.slant_corrected, info.snr_db, info.fsk_id);
                    fclose(f);
                }
                printf("image %d %s rows=%d complete=%d rate=%.3f skip=%d slant=%.1f "
                       "fsk=\"%s\"\n",
                       k, sstv_mode_by_id(info.mode)->short_name, info.rows, info.complete,
                       info.rate, info.skip, info.slant_deg, info.fsk_id);
                free(rgb);
                k++;
            }
        }
        if (flush) break;
    }
    sstv_rx_destroy(rx);
    free(s);
    printf("images %d\n", k);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "modes")) {
        int i;
        printf("id\tshort\tname\tvis\twidth\tlines\tline_height\tcolor\tchannels\ttx\t"
               "tx_width\tsync\tporch\tseptr\tpixel\tline\n");
        for (i = 0; i < sstv_mode_count(); i++) {
            const sstv_mode *m = sstv_mode_at(i);
            printf("%d\t%s\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.10g\t%.10g\t%.10g\t%.10g\t"
                   "%.10g\n",
                   m->id, m->short_name, m->name, m->vis, m->width, m->lines, m->line_height,
                   m->color, m->channels, m->tx, m->tx_width, m->sync_s, m->porch_s,
                   m->septr_s, m->pixel_s, m->line_s);
        }
        return 0;
    }
    if (argc >= 4 && !strcmp(argv[1], "pattern")) {
        const sstv_mode *m = mode_arg(argv[2]);
        unsigned char *rgb;
        if (!m) return 1;
        rgb = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
        test_pattern(m->tx_width, m->lines, rgb);
        return write_ppm(argv[3], rgb, m->tx_width, m->lines) ? 1 : 0;
    }
    if (argc >= 6 && (!strcmp(argv[1], "encode") || !strcmp(argv[1], "raw"))) {
        const sstv_mode *m = mode_arg(argv[2]);
        int rate = atoi(argv[3]), w, h;
        unsigned char *rgb;
        if (!m) return 1;
        rgb = read_ppm(argv[4], &w, &h);
        if (!rgb || w != m->tx_width || h != m->lines) {
            fprintf(stderr, "picture must be %dx%d PPM\n", m->tx_width, m->lines);
            return 1;
        }
        if (!strcmp(argv[1], "encode")) {
            size_t n;
            int16_t *s = render_over(m->id, rate, rgb, argc > 6 ? argv[6] : NULL, &n);
            if (!s) return 1;
            return write_wav(argv[5], s, n, rate) ? 1 : 0;
        } else {
            sstv_tx_config cfg;
            sstv_tx *tx;
            double buf[4096];
            size_t k;
            FILE *f = fopen(argv[5], "wb");
            sstv_tx_config_default(&cfg, m->id, rate);
            cfg.fsk_id = argc > 6 ? argv[6] : NULL;
            tx = sstv_tx_create(&cfg);
            if (!tx || !f || sstv_tx_put_image(tx, rgb, w, h)) return 1;
            while ((k = sstv_tx_render(tx, buf, 4096)) > 0) fwrite(buf, sizeof(double), k, f);
            fclose(f);
            sstv_tx_destroy(tx);
            return 0;
        }
    }
    if (argc >= 4 && !strcmp(argv[1], "decode")) return cmd_decode(argc, argv);
    if (argc >= 3 && !strcmp(argv[1], "loopback")) {
        int rate = atoi(argv[2]), i;
        for (i = 0; i < sstv_mode_count(); i++) {
            const sstv_mode *m = sstv_mode_at(i);
            size_t n, np = (size_t)m->width * m->lines * 3;
            unsigned char *src = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
            unsigned char *ref = (unsigned char *)malloc(np);
            int16_t *s;
            sstv_rx_config cfg;
            decoded d;
            test_pattern(m->tx_width, m->lines, src);
            nearest(src, m->tx_width, m->lines, ref, m->width, m->lines);
            s = render_over(m->id, rate, src, "DG6FL", &n);
            sstv_rx_config_default(&cfg, rate);
            decode_audio(s, n, &cfg, 4096, &d);
            printf("%-6s images=%d mode=%s rows=%d rate=%.3f skip=%d psnr=%.2f fsk=\"%s\"\n",
                   m->short_name, d.images,
                   d.images ? sstv_mode_by_id(d.info.mode)->short_name : "-", d.info.rows,
                   d.info.rate, d.info.skip,
                   d.images && d.info.mode == m->id ? psnr(d.rgb, ref, np) : 0.0,
                   d.info.fsk_id);
            free(d.rgb);
            free(src);
            free(ref);
            free(s);
        }
        return 0;
    }
    fprintf(stderr, "usage: see tools/sstv_tool.c\n");
    return 2;
}
