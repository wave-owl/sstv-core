/*
 * sstv-core -- SSTV (slow-scan television) receiver and transmitter in C99.
 *
 * Copyright (c) 2026 Gerolf Ziegenhain
 *
 * The receiver is a port of slowrx by Oona Raisanen OH2EIQ (ISC licence,
 * https://github.com/wave-owl/slowrx, commit ca6d7012ae788b5057646170bd86590a7f68bd69):
 * vis.c (VIS detector), video.c (FM demodulator, pixel grid, colour conversion), sync.c
 * (slant correction), fsk.c (FSK ID), modespec.c (mode table, VIS map). The transmitter is a
 * port of pySSTV by Andras Veres-Szentkiralyi (MIT licence,
 * https://github.com/wave-owl/pySSTV, commit d998fad154d3e6ad2d73af5add49beec0d2ab59f):
 * pysstv/sstv.py, color.py, grayscale.py, with Pillow's RGB->YCbCr and RGB->L conversions
 * (libImaging/ConvertYCbCr.c, Convert.c, Pillow 12.3.0, MIT-CMU licence). robot36 by Ahmet
 * Inan (https://github.com/wave-owl/robot36, commit 2af2390) is the independent oracle used by
 * the tests. See NOTICE for the full upstream notices.
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose with or
 * without fee is hereby granted, provided that the above copyright notice and this
 * permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO
 * THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR
 * ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
 * CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE
 * OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef SSTV_H
#define SSTV_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SSTV_VERSION_STRING "0.1.0"
#define SSTV_ABI 1
/* slowrx's sound card rate (pcm.c:110). At this rate the receiver performs slowrx's
 * arithmetic sample for sample; at any other rate every slowrx sample count is scaled
 * by rate / 44100 (see sstv_rx_config.sample_rate). */
#define SSTV_REFERENCE_RATE 44100
#define SSTV_MIN_RATE 8000
#define SSTV_MAX_RATE 48000
/* Largest image of any mode (PD-290, modespec.c:312-323). */
#define SSTV_MAX_WIDTH 800
#define SSTV_MAX_LINES 616
/* Longest FSK ID slowrx reads (fsk.c:99: at most 10 characters). */
#define SSTV_FSK_ID_MAX 10

/* "sstv-core 0.1.0 contract=sstv-c1 abi=1 tx=1 dsp=0.7.0". Static string. The contract tag
 * changes only with an incompatible API change. */
const char *sstv_version(void);
int sstv_abi(void);                         /* SSTV_ABI */

/* sizeof the public structures, so a binding can check its mirrors of them (a ctypes
 * binding sizes its arrays from these). */
#define SSTV_SIZEOF_MODE 0
#define SSTV_SIZEOF_RX_CONFIG 1
#define SSTV_SIZEOF_EVENT 2
#define SSTV_SIZEOF_IMAGE_INFO 3
#define SSTV_SIZEOF_TX_CONFIG 4
size_t sstv_sizeof(int what);               /* 0 for an unknown structure */

/* ------------------------------------------------------------------------------------
 * Modes: every mode of slowrx's ModeSpec[] (modespec.c:37-364), in slowrx's enum order
 * (common.h:118-125, id 1 .. 25). Three rows differ from slowrx because slowrx's row
 * contradicts the published timing (see README, "Corrections"): Martin M3 (pixel time),
 * Robot 72 and Robot 24 (channel layout; Robot 24 also size and line time).
 * ------------------------------------------------------------------------------------ */
enum {
    SSTV_M1 = 1, SSTV_M2, SSTV_M3, SSTV_M4,
    SSTV_S1, SSTV_S2, SSTV_SDX,
    SSTV_R72, SSTV_R36, SSTV_R24, SSTV_R24BW, SSTV_R12BW, SSTV_R8BW,
    SSTV_PD50, SSTV_PD90, SSTV_PD120, SSTV_PD160, SSTV_PD180, SSTV_PD240, SSTV_PD290,
    SSTV_P3, SSTV_P5, SSTV_P7,
    SSTV_W2120, SSTV_W2180,
    SSTV_MODE_LAST = SSTV_W2180
};

/* Colour encodings (common.h:128-130). */
enum { SSTV_GBR = 0, SSTV_RGB, SSTV_YUV, SSTV_BW };

typedef struct sstv_mode {
    int id;                 /* SSTV_M1 .. SSTV_W2180 */
    const char *name;       /* "Martin M1" */
    const char *short_name; /* "M1", slowrx's file name tag */
    int vis;                /* 7-bit VIS code (modespec.c:376-383) */
    double sync_s;          /* SyncTime, seconds */
    double porch_s;         /* PorchTime */
    double septr_s;         /* SeptrTime */
    double pixel_s;         /* PixelTime (luminance pixel) */
    double line_s;          /* LineTime: sync start to next sync start */
    int width;              /* ImgWidth */
    int lines;              /* NumLines: rows the receiver delivers */
    int line_height;        /* LineHeight: 2 = each row is shown twice */
    int color;              /* SSTV_GBR .. SSTV_BW */
    int channels;           /* video channels per sync period (video.c:104-130) */
    int tx;                 /* 1: the transmitter supports this mode */
    int tx_width;           /* image the transmitter takes: tx_width x lines RGB */
} sstv_mode;

int sstv_mode_count(void);                      /* 25 */
const sstv_mode *sstv_mode_at(int index);       /* NULL outside 0 .. count-1 */
const sstv_mode *sstv_mode_by_id(int id);       /* NULL for an unknown id */
/* Case-insensitive lookup by short name ("PD120") or name ("PD-120"); NULL when unknown. */
const sstv_mode *sstv_mode_find(const char *name);
/* slowrx's VISmap (modespec.c:376-383); NULL for 0 or an unmapped code. */
const sstv_mode *sstv_mode_by_vis(int vis);

/* ------------------------------------------------------------------------------------
 * Receiver: slowrx's Listen loop (slowrx.c:24-185) as a streaming state machine:
 * GetVIS (vis.c:18-175) -> GetVideo (video.c:19-491) -> GetFSK (fsk.c:20-110) ->
 * FindSync (sync.c:18-133) -> GetVideo redraw -> back to GetVIS. The receiver owns the
 * image: rows can be read while the image arrives (sstv_rx_get_line) and the finished,
 * slant-corrected image stays readable until the next image is finished
 * (sstv_rx_get_image_line). Line events report rows of the first pass (slowrx's live
 * display at the nominal rate); the finished image is redrawn after slant correction.
 * ------------------------------------------------------------------------------------ */
typedef struct sstv_rx_config {
    int sample_rate;        /* SSTV_MIN_RATE .. SSTV_MAX_RATE; 44100 = slowrx exactly */
    int adaptive;           /* slowrx "noise adaptation" (Adaptive, video.c:356-364): 1 */
    int slant;              /* slowrx "slant correction" (tog_slant, slowrx.c:136): 1 */
    int fsk_id;             /* slowrx "FSK ID" (tog_fsk, slowrx.c:123): 1 */
    int slant_partial;      /* NOT slowrx: also slant-correct a partial image (>= 16 rows);
                             * slowrx never corrects an aborted image. Default 1. */
    double timeout_s;       /* NOT slowrx: end an image as partial when the video signal is
                             * gone for this long (sstv_rx_config_default: 0 = never, as
                             * slowrx). See README "Signal loss". */
    double vis_tolerance_hz;/* vis.c:85-99: 25 */
} sstv_rx_config;

/* slowrx's defaults at the given rate (adaptive, slant, FSK ID on; no timeout). */
void sstv_rx_config_default(sstv_rx_config *cfg, int sample_rate);

enum {
    SSTV_EVENT_VIS = 1,     /* VIS decoded, image started: mode, shift_hz */
    SSTV_EVENT_LINE,        /* row `line` of the arriving image is drawn */
    SSTV_EVENT_IMAGE        /* image finished (complete or partial): read it now */
};

typedef struct sstv_event {
    int kind;
    int mode;               /* mode id (all events) */
    int line;               /* SSTV_EVENT_LINE: row; SSTV_EVENT_IMAGE: rows received */
    int complete;           /* SSTV_EVENT_IMAGE: 1 when the full image was received */
    double shift_hz;        /* VIS header shift (CurrentPic.HedrShift, vis.c:106) */
} sstv_event;

typedef struct sstv_image_info {
    int mode;               /* mode id, 0 before the first image */
    int width, lines, line_height;
    int rows;               /* rows received (== lines when complete) */
    int complete;
    double shift_hz;        /* VIS header shift */
    double rate;            /* sample rate after slant correction (CurrentPic.Rate) */
    int skip;               /* line start skip in samples (CurrentPic.Skip) */
    double slant_deg;       /* last Hough slant angle, 90 = upright; 0 when not computed */
    int slant_corrected;    /* 1 when FindSync + redraw ran */
    double snr_db;          /* last SNR estimate of the video (video.c:304-344) */
    char fsk_id[SSTV_FSK_ID_MAX + 1];  /* decoded FSK ID, "" when none */
} sstv_image_info;

typedef struct sstv_rx sstv_rx;

/* NULL for a bad configuration or out of memory. All memory is allocated here. */
sstv_rx *sstv_rx_create(const sstv_rx_config *cfg);
void sstv_rx_destroy(sstv_rx *rx);
/* Back to the state after create: hunting for a VIS, no image. */
void sstv_rx_reset(sstv_rx *rx);
/* Feed n samples (sound card scale: int16 / 32768). Events go to out (at most max; more are
 * dropped and counted); returns their number. The finished image of an SSTV_EVENT_IMAGE is
 * readable after the call until the next image finishes; the shortest mode (Robot 8 B/W)
 * needs about 8.6 s from VIS to finish, so calls of at most SSTV_MAX_CALL_SECONDS of audio
 * can never finish two images. */
#define SSTV_MAX_CALL_SECONDS 5
size_t sstv_rx_process(sstv_rx *rx, const double *x, size_t n, sstv_event *out, size_t max);
/* End of input: an image in progress ends as partial (slant-corrected per slant_partial)
 * and is reported with SSTV_EVENT_IMAGE. Returns the number of events (0 or 1). */
size_t sstv_rx_flush(sstv_rx *rx, sstv_event *out, size_t max);
/* 1 while an image is arriving. */
int sstv_rx_busy(const sstv_rx *rx);
/* The arriving image: info and row y as RGB (3 * width bytes). Returns 0, or -1 when no
 * image is arriving or y is out of range. */
int sstv_rx_current(const sstv_rx *rx, sstv_image_info *info);
int sstv_rx_get_line(const sstv_rx *rx, int y, unsigned char *rgb);
/* The last finished image. Returns 0, or -1 before the first image or for a bad row. */
int sstv_rx_image(const sstv_rx *rx, sstv_image_info *info);
int sstv_rx_get_image_line(const sstv_rx *rx, int y, unsigned char *rgb);
size_t sstv_rx_events_dropped(const sstv_rx *rx);
/* Bytes the receiver allocated (all at create). */
size_t sstv_rx_footprint(const sstv_rx *rx);

/* ------------------------------------------------------------------------------------
 * Transmitter: pySSTV's gen_freq_bits / gen_values (sstv.py:73-122) for every mode with
 * tx = 1: optional VOX tones, calibration header, VIS, the mode's scan lines, optional FSK
 * ID. Output: pySSTV's gen_values samples (peak 1.0), before quantisation.
 * ------------------------------------------------------------------------------------ */
typedef struct sstv_tx_config {
    int mode;               /* mode id with tx = 1 */
    int sample_rate;        /* SSTV_MIN_RATE .. SSTV_MAX_RATE (pySSTV's samples_per_sec) */
    int vox;                /* pySSTV vox_enabled: eight 100 ms VOX tones first */
    const char *fsk_id;     /* pySSTV add_fskid_text: characters ' ' .. '_' (NULL: none) */
} sstv_tx_config;

void sstv_tx_config_default(sstv_tx_config *cfg, int mode, int sample_rate);

typedef struct sstv_tx sstv_tx;

/* NULL for a mode without TX, a bad rate, an FSK ID with characters outside ' ' .. '_' or
 * longer than SSTV_FSK_ID_MAX, or out of memory. */
sstv_tx *sstv_tx_create(const sstv_tx_config *cfg);
void sstv_tx_destroy(sstv_tx *tx);
/* The picture: tx_width x lines pixels, RGB, row after row. Must be given before the first
 * sstv_tx_render; returns 0, or -1 for a wrong size. */
int sstv_tx_put_image(sstv_tx *tx, const unsigned char *rgb, int width, int height);
/* Next samples of the over; returns their number, 0 when the over is complete (or no image
 * was given). */
size_t sstv_tx_render(sstv_tx *tx, double *out, size_t max);
/* Samples of the whole over (exact, independent of the picture). */
size_t sstv_tx_total_samples(const sstv_tx *tx);
size_t sstv_tx_rendered(const sstv_tx *tx);

#ifdef __cplusplus
}
#endif

#endif
