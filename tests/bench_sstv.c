/* CPU and memory benchmark: receiver and transmitter per mode at the station rate (12000 Hz)
 * and slowrx's rate (44100 Hz).
 *
 *   bench_sstv [rate ...]      default: 12000 44100
 *
 * Prints one Markdown row per mode: audio seconds, receiver CPU as % of real time, transmitter
 * render CPU as % of real time, receiver footprint. SPDX-License-Identifier: ISC */
#ifndef __APPLE__
#define _XOPEN_SOURCE 600
#endif
#include <sys/resource.h>
#include <time.h>

#include "test_util.h"

static double cpu_seconds(void)
{
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_utime.tv_sec + ru.ru_utime.tv_usec * 1e-6 + ru.ru_stime.tv_sec +
           ru.ru_stime.tv_usec * 1e-6;
}

int main(int argc, char **argv)
{
    int rates[8] = { 12000, 44100 }, nrates = 2, r, i;
    struct rusage ru;
    if (argc > 1) {
        nrates = 0;
        for (i = 1; i < argc && nrates < 8; i++) rates[nrates++] = atoi(argv[i]);
    }
    printf("| rate | mode | audio s | RX %% real time | TX %% real time | RX footprint MB |\n");
    printf("|---:|---|---:|---:|---:|---:|\n");
    for (r = 0; r < nrates; r++) {
        int rate = rates[r];
        double rx_total = 0, tx_total = 0, audio_total = 0;
        for (i = 0; i < sstv_mode_count(); i++) {
            const sstv_mode *m = sstv_mode_at(i);
            unsigned char *src = (unsigned char *)malloc((size_t)m->tx_width * m->lines * 3);
            size_t n;
            int16_t *s;
            sstv_rx_config cfg;
            sstv_rx *rx;
            decoded d;
            double t0, t_tx, t_rx, secs;
            test_pattern(m->tx_width, m->lines, src);
            t0 = cpu_seconds();
            s = render_over(m->id, rate, src, "DG6FL", &n);
            t_tx = cpu_seconds() - t0;
            sstv_rx_config_default(&cfg, rate);
            rx = sstv_rx_create(&cfg);
            t0 = cpu_seconds();
            decode_audio(s, n, &cfg, 4096, &d);
            t_rx = cpu_seconds() - t0;
            secs = (double)n / rate;
            printf("| %d | %s | %.1f | %.2f | %.2f | %.1f |\n", rate, m->short_name, secs,
                   100 * t_rx / secs, 100 * t_tx / secs, sstv_rx_footprint(rx) / 1048576.0);
            rx_total += t_rx;
            tx_total += t_tx;
            audio_total += secs;
            sstv_rx_destroy(rx);
            free(d.rgb);
            free(s);
            free(src);
        }
        printf("| %d | all | %.1f | %.2f | %.2f | |\n", rate, audio_total,
               100 * rx_total / audio_total, 100 * tx_total / audio_total);
    }
    getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
    printf("\npeak RSS %.1f MB\n", ru.ru_maxrss / 1048576.0);
#else
    printf("\npeak RSS %.1f MB\n", ru.ru_maxrss / 1024.0);
#endif
    printf("%s\n", sstv_version());
    return 0;
}
