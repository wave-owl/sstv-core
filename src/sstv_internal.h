/* sstv-core internal declarations. SPDX-License-Identifier: ISC */
#ifndef SSTV_INTERNAL_H
#define SSTV_INTERNAL_H

#include "sstv.h"

/* Channel layouts of video.c:61-130 (ChanStart/ChanLen/NumChans by mode family). */
enum {
    SSTV_LAYOUT_DEFAULT = 0, /* video.c:95-100: Martin, Wraase, Pasokon (3 equal channels) */
    SSTV_LAYOUT_SCOTTIE,     /* video.c:72-79: sync before the third channel */
    SSTV_LAYOUT_ROBOT2,      /* video.c:63-70: Robot 36, Y + alternating half-height chroma */
    SSTV_LAYOUT_ROBOT3,      /* correction: Robot 72/24, Y + V + U, chroma half as long */
    SSTV_LAYOUT_PD,          /* video.c:81-93: four channels, two rows per sync */
    SSTV_LAYOUT_BW           /* video.c:105-108: one channel (default start times) */
};

typedef struct sstv_mode_ext {
    sstv_mode pub;
    int layout;
    int vis_parity_odd;      /* vis.c:116: Robot 12 B/W uses odd VIS parity */
} sstv_mode_ext;

const sstv_mode_ext *sstv_mode_ext_by_id(int id);

/* Pillow 12.3.0 conversions (libImaging/ConvertYCbCr.c:330-350, Convert.c:42,212-217). */
void sstv_pil_rgb2ycbcr(const unsigned char *rgb, unsigned char *ycc);
unsigned char sstv_pil_rgb2l(const unsigned char *rgb);

#endif
