/*
 * Headless driver for slowrx's decoder (bench harness, never part of libsstv): replaces
 * slowrx.c's Listen thread (slowrx.c:24-185), pcm.c's readPcm (pcm.c:19-64) and common.c's
 * helpers (common.c:14-63) so that vis.c, video.c, sync.c, fsk.c and modespec.c of slowrx
 * commit ca6d701 decode a mono 16-bit 44100 Hz WAV file, unmodified apart from
 * slowrx-bench.patch. Every finished picture is written as OUT_PREFIX<k>.ppm with its
 * metadata in OUT_PREFIX<k>.txt.
 *
 *   slowrx_ref IN.wav OUT_PREFIX
 *
 * SPDX-License-Identifier: ISC
 */
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gtk/gtk.h>
#include <alsa/asoundlib.h>
#include <fftw3.h>

#include "common.h"

/* common.c:14-36 */
gboolean Abort = FALSE;
gboolean Adaptive = TRUE;
gboolean *HasSync = NULL;
gboolean ManualActivated = FALSE;
gboolean ManualResync = FALSE;
guchar *StoredLum = NULL;
pthread_t thread1;
FFTStuff fft;
GuiObjs gui;
PicMeta CurrentPic;
PcmData pcm;
GdkPixbuf *pixbuf_rx = NULL;
GdkPixbuf *pixbuf_disp = NULL;
GdkPixbuf *pixbuf_PWR = NULL;
GdkPixbuf *pixbuf_SNR = NULL;
GtkListStore *savedstore = NULL;
GKeyFile *config = NULL;

/* common.c:39-63, GetBin clamped at 0 (sstv-core U5) */
guint GetBin(double Freq, guint FFTLen)
{
    double b = Freq / 44100 * FFTLen;
    return b > 0 ? (guint)b : 0;
}
double power(fftw_complex coeff) { return pow(coeff[0], 2) + pow(coeff[1], 2); }
guchar clip(double a)
{
    if (a != a) return 0;                         /* sstv-core U4 */
    if (a < 0) return 0;
    else if (a > 255) return 255;
    return (guchar)round(a);
}
double deg2rad(double Deg) { return (Deg / 180) * M_PI; }
void setVU(double *Power, int FFTLen, int WinIdx, gboolean ShowWin)
{
    (void)Power; (void)FFTLen; (void)WinIdx; (void)ShowWin;
}

/* The whole file, with BUFLEN samples of silence before it and enough after it. */
#define PAD_FRONT BUFLEN
/* longest image plus FSK ID (Pasokon P7, 406 s) and the capture buffer */
#define TAIL ((size_t)420 * 44100)
static int16_t *g_audio;     /* PAD_FRONT + file + tail padding */
static size_t g_n, g_pos;    /* g_pos: next unread sample, counted from the file start */
static int g_in_vis;
static jmp_buf g_eof;

/* pcm.c:19-64 with the file as sound card. slowrx copies into its BUFLEN capture buffer and
 * shifts it; here pcm.Buffer points into the file and advances, which gives the same
 * pcm.Buffer[0 .. BUFLEN-1] at every call and, where slowrx's fsk.c reads before the buffer
 * (U7, after slowrx-bench.patch removes the branch that never terminates), the stream's
 * samples there, as sstv-core reads them from its input ring. Past the end of the file it
 * delivers silence, and stops the program while waiting for a VIS. */
void readPcm(gint numsamples)
{
    int n = (pcm.WindowPtr == 0 ? BUFLEN : numsamples);
    if ((g_in_vis && g_pos >= g_n) || g_pos + BUFLEN >= g_n + TAIL) longjmp(g_eof, 1);
    if (pcm.WindowPtr == 0) {
        pcm.Buffer = g_audio + PAD_FRONT + g_pos;
        pcm.WindowPtr = BUFLEN / 2;
    } else {
        pcm.Buffer += numsamples;
        pcm.WindowPtr -= numsamples;
    }
    g_pos += (size_t)n;
}

static int16_t *read_wav(const char *path, size_t *n_out)
{
    FILE *f = fopen(path, "rb");
    unsigned char hdr[12], ck[8];
    int16_t *data = NULL;
    if (!f) return NULL;
    if (fread(hdr, 1, 12, f) != 12) { fclose(f); return NULL; }
    while (fread(ck, 1, 8, f) == 8) {
        unsigned len = ck[4] | (ck[5] << 8) | (ck[6] << 16) | ((unsigned)ck[7] << 24);
        if (!memcmp(ck, "data", 4)) {
            /* silence before (PAD_FRONT) and after the file (the decoder may run on for a
             * whole image time past the end) */
            size_t tail = TAIL;
            data = (int16_t *)calloc(PAD_FRONT + len / 2 + tail, sizeof(int16_t));
            if (data && fread(data + PAD_FRONT, 1, len, f) == len) *n_out = len / 2;
            break;
        }
        fseek(f, (long)(len + (len & 1)), SEEK_CUR);
    }
    fclose(f);
    return data;
}

static const char *g_prefix;
static int g_k;

static void *listen(void *arg)
{
    guchar Mode;
    gboolean Finished;
    char id[20];
    (void)arg;
    if (setjmp(g_eof)) return NULL;
    while (TRUE) {
        pcm.WindowPtr = 0;
        Abort = FALSE;
        do {
            g_in_vis = 1;
            Mode = GetVIS();
            g_in_vis = 0;
        } while (Mode == 0);
        CurrentPic.Rate = 44100;
        CurrentPic.Mode = Mode;
        CurrentPic.Skip = 0;
        free(StoredLum);
        StoredLum = calloc((int)((ModeSpec[Mode].LineTime * ModeSpec[Mode].NumLines + 1) * 44100),
                           sizeof(guchar));
        /* twice slowrx's size (slowrx.c:93): reads past the stored samples see zeros
         * (sstv-core U3) */
        HasSync = calloc(2 * (int)(ModeSpec[Mode].LineTime * ModeSpec[Mode].NumLines /
                                   (13.0 / 44100) + 1) + 4096, sizeof(gboolean));
        Finished = GetVideo(CurrentPic.Mode, 44100, 0, FALSE);
        id[0] = '\0';
        if (Finished) GetFSK(id);
        if (Finished) {
            CurrentPic.Rate = FindSync(CurrentPic.Mode, CurrentPic.Rate, &CurrentPic.Skip);
            GetVideo(CurrentPic.Mode, CurrentPic.Rate, CurrentPic.Skip, TRUE);
        }
        free(HasSync);
        HasSync = NULL;
        {
            char path[1024];
            FILE *f;
            snprintf(path, sizeof(path), "%s%d.ppm", g_prefix, g_k);
            f = fopen(path, "wb");
            if (f) {
                fprintf(f, "P6\n%d %d\n255\n", pixbuf_rx->width, pixbuf_rx->height);
                fwrite(pixbuf_rx->pixels, 3, (size_t)pixbuf_rx->width * pixbuf_rx->height, f);
                fclose(f);
            }
            snprintf(path, sizeof(path), "%s%d.txt", g_prefix, g_k);
            f = fopen(path, "w");
            if (f) {
                fprintf(f, "mode=%s\nshift=%d\nrate=%.6f\nskip=%d\nfsk_id=%s\n",
                        ModeSpec[CurrentPic.Mode].ShortName, CurrentPic.HedrShift,
                        CurrentPic.Rate, CurrentPic.Skip, id);
                fclose(f);
            }
            printf("image %d %s rate=%.3f skip=%d fsk=\"%s\"\n", g_k,
                   ModeSpec[CurrentPic.Mode].ShortName, CurrentPic.Rate, CurrentPic.Skip, id);
            g_k++;
        }
    }
    return NULL;
}

int main(int argc, char **argv)
{
    pthread_t th;
    pthread_attr_t attr;
    if (argc < 3) {
        fprintf(stderr, "usage: slowrx_ref IN.wav OUT_PREFIX\n");
        return 2;
    }
    g_audio = read_wav(argv[1], &g_n);
    if (!g_audio) {
        fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }
    g_prefix = argv[2];
    /* slowrx.c:217-232 */
    fft.in = fftw_alloc_real(2048);
    fft.out = fftw_alloc_complex(2048);
    memset(fft.in, 0, sizeof(double) * 2048);
    fft.Plan1024 = fftw_plan_dft_r2c_1d(1024, fft.in, fft.out, FFTW_ESTIMATE);
    fft.Plan2048 = fftw_plan_dft_r2c_1d(2048, fft.in, fft.out, FFTW_ESTIMATE);
    pixbuf_rx = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, 1, 1);
    pixbuf_disp = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, 1, 1);
    /* GetVideo keeps Image[800][616][3] and FindSync SyncImg[700][630] on the stack */
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 64 << 20);
    pthread_create(&th, &attr, listen, NULL);
    pthread_join(th, NULL);
    printf("images %d\n", g_k);
    return 0;
}
