/*
 * Mode table and VIS map: slowrx modespec.c:37-385 (ModeSpec[], VISmap[]), which cites
 * JL Barber N7CXI (2000) "Proposal for SSTV Mode Specifications" and Dave Jones KB4YZ
 * (1998, 1999) "List of SSTV Modes with VIS Codes", "SSTV modes - line timing".
 *
 * Corrections against slowrx (README "Corrections"; each row's self-consistency is checked
 * by tests/test_sstv.c):
 *   C1 Martin M3: slowrx's PixelTime 0.2288 ms (modespec.c:71) is Martin M2's; with its own
 *      LineTime 446.446 ms (= Martin M1's) the line only adds up with M1's 0.4576 ms.
 *   C2 Robot 72: slowrx gives three equal 92 ms channels (modespec.c:130-141 with the
 *      default layout, video.c:95-100); the mode sends Y 138 ms and V, U 69 ms each with
 *      4.5 ms separator + 1.5 ms porch (robot36 Robot_72_Color.java:24-31, QSSTV
 *      sstvparam.cpp:319 with modeRobot2).
 *   C3 Robot 24: slowrx's row is Robot 36's (modespec.c:156-167). The mode is 160 x 120 with
 *      200 ms lines in Robot 72's layout (QSSTV sstvparam.cpp:317, sstvtx.cpp:109-110):
 *      9 + 3 + 88 + 6 + 44 + 6 + 44 = 200 ms.
 *
 * SPDX-License-Identifier: ISC
 */
#include <ctype.h>
#include <stddef.h>

#include "sstv_internal.h"

#define ROW(id_, name_, short_, vis_, sync_, porch_, septr_, pixel_, line_, w_, n_, lh_, \
            color_, ch_, layout_, odd_) \
    { { id_, name_, short_, vis_, sync_, porch_, septr_, pixel_, line_, w_, n_, lh_, color_, \
        ch_, 1, w_ }, layout_, odd_ }

/* Ordered by slowrx's enum (common.h:118-125). */
static const sstv_mode_ext MODES[] = {
    /* modespec.c:39-50, VIS 0x2c */
    ROW(SSTV_M1, "Martin M1", "M1", 0x2c, 4.862e-3, 0.572e-3, 0.572e-3, 0.4576e-3,
        446.446e-3, 320, 256, 1, SSTV_GBR, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:52-63, VIS 0x28 */
    ROW(SSTV_M2, "Martin M2", "M2", 0x28, 4.862e-3, 0.572e-3, 0.572e-3, 0.2288e-3,
        226.7986e-3, 320, 256, 1, SSTV_GBR, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:65-76, VIS 0x24; C1: PixelTime 0.4576 ms */
    ROW(SSTV_M3, "Martin M3", "M3", 0x24, 4.862e-3, 0.572e-3, 0.572e-3, 0.4576e-3,
        446.446e-3, 320, 128, 2, SSTV_GBR, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:78-89, VIS 0x20 */
    ROW(SSTV_M4, "Martin M4", "M4", 0x20, 4.862e-3, 0.572e-3, 0.572e-3, 0.2288e-3,
        226.7986e-3, 320, 128, 2, SSTV_GBR, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:91-102, VIS 0x3c */
    ROW(SSTV_S1, "Scottie S1", "S1", 0x3c, 9e-3, 1.5e-3, 1.5e-3, 0.4320e-3, 428.38e-3,
        320, 256, 1, SSTV_GBR, 3, SSTV_LAYOUT_SCOTTIE, 0),
    /* modespec.c:104-115, VIS 0x38 */
    ROW(SSTV_S2, "Scottie S2", "S2", 0x38, 9e-3, 1.5e-3, 1.5e-3, 0.2752e-3, 277.692e-3,
        320, 256, 1, SSTV_GBR, 3, SSTV_LAYOUT_SCOTTIE, 0),
    /* modespec.c:117-128, VIS 0x4c */
    ROW(SSTV_SDX, "Scottie DX", "SDX", 0x4c, 9e-3, 1.5e-3, 1.5e-3, 1.08053e-3, 1050.3e-3,
        320, 256, 1, SSTV_GBR, 3, SSTV_LAYOUT_SCOTTIE, 0),
    /* modespec.c:130-141, VIS 0x0c; C2: Y 138 ms (0.43125 ms x 320), separator 4.5 + porch
     * 1.5 ms, chroma 69 ms */
    ROW(SSTV_R72, "Robot 72", "R72", 0x0c, 9e-3, 3e-3, 6e-3, 0.43125e-3, 300e-3,
        320, 240, 1, SSTV_YUV, 3, SSTV_LAYOUT_ROBOT3, 0),
    /* modespec.c:143-154, VIS 0x08 */
    ROW(SSTV_R36, "Robot 36", "R36", 0x08, 9e-3, 3e-3, 6e-3, 0.1375e-3, 150e-3,
        320, 240, 1, SSTV_YUV, 2, SSTV_LAYOUT_ROBOT2, 0),
    /* modespec.c:156-167, VIS 0x04; C3: 160 x 120, Y 88 ms (0.55 ms x 160), 200 ms lines */
    ROW(SSTV_R24, "Robot 24", "R24", 0x04, 9e-3, 3e-3, 6e-3, 0.55e-3, 200e-3,
        160, 120, 1, SSTV_YUV, 3, SSTV_LAYOUT_ROBOT3, 0),
    /* modespec.c:169-180, VIS 0x0a */
    ROW(SSTV_R24BW, "Robot 24 B/W", "R24BW", 0x0a, 7e-3, 0e-3, 0e-3, 0.291e-3, 100e-3,
        320, 240, 1, SSTV_BW, 1, SSTV_LAYOUT_BW, 0),
    /* modespec.c:182-193, VIS 0x06, odd parity (vis.c:116) */
    ROW(SSTV_R12BW, "Robot 12 B/W", "R12BW", 0x06, 7e-3, 0e-3, 0e-3, 0.291e-3, 100e-3,
        320, 120, 2, SSTV_BW, 1, SSTV_LAYOUT_BW, 1),
    /* modespec.c:195-206, VIS 0x02 */
    ROW(SSTV_R8BW, "Robot 8 B/W", "R8BW", 0x02, 7e-3, 0e-3, 0e-3, 0.1871875e-3, 66.9e-3,
        320, 120, 2, SSTV_BW, 1, SSTV_LAYOUT_BW, 0),
    /* modespec.c:234-245, VIS 0x5d */
    ROW(SSTV_PD50, "PD-50", "PD50", 0x5d, 20e-3, 2.08e-3, 0e-3, 0.286e-3, 388.16e-3,
        320, 256, 1, SSTV_YUV, 4, SSTV_LAYOUT_PD, 0),
    /* modespec.c:247-258, VIS 0x63 */
    ROW(SSTV_PD90, "PD-90", "PD90", 0x63, 20e-3, 2.08e-3, 0e-3, 0.532e-3, 703.04e-3,
        320, 256, 1, SSTV_YUV, 4, SSTV_LAYOUT_PD, 0),
    /* modespec.c:260-271, VIS 0x5f */
    ROW(SSTV_PD120, "PD-120", "PD120", 0x5f, 20e-3, 2.08e-3, 0e-3, 0.19e-3, 508.48e-3,
        640, 496, 1, SSTV_YUV, 4, SSTV_LAYOUT_PD, 0),
    /* modespec.c:273-284, VIS 0x62 */
    ROW(SSTV_PD160, "PD-160", "PD160", 0x62, 20e-3, 2.08e-3, 0e-3, 0.382e-3, 804.416e-3,
        512, 400, 1, SSTV_YUV, 4, SSTV_LAYOUT_PD, 0),
    /* modespec.c:286-297, VIS 0x60 */
    ROW(SSTV_PD180, "PD-180", "PD180", 0x60, 20e-3, 2.08e-3, 0e-3, 0.286e-3, 754.24e-3,
        640, 496, 1, SSTV_YUV, 4, SSTV_LAYOUT_PD, 0),
    /* modespec.c:299-310, VIS 0x61 */
    ROW(SSTV_PD240, "PD-240", "PD240", 0x61, 20e-3, 2.08e-3, 0e-3, 0.382e-3, 1000e-3,
        640, 496, 1, SSTV_YUV, 4, SSTV_LAYOUT_PD, 0),
    /* modespec.c:312-323, VIS 0x5e */
    ROW(SSTV_PD290, "PD-290", "PD290", 0x5e, 20e-3, 2.08e-3, 0e-3, 0.286e-3, 937.28e-3,
        800, 616, 1, SSTV_YUV, 4, SSTV_LAYOUT_PD, 0),
    /* modespec.c:325-336, VIS 0x71 */
    ROW(SSTV_P3, "Pasokon P3", "P3", 0x71, 5.208e-3, 1.042e-3, 1.042e-3, 0.2083e-3,
        409.375e-3, 640, 496, 1, SSTV_RGB, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:338-349, VIS 0x72 */
    ROW(SSTV_P5, "Pasokon P5", "P5", 0x72, 7.813e-3, 1.563e-3, 1.563e-3, 0.3125e-3,
        614.065e-3, 640, 496, 1, SSTV_RGB, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:351-362, VIS 0x73 */
    ROW(SSTV_P7, "Pasokon P7", "P7", 0x73, 10.417e-3, 2.083e-3, 2.083e-3, 0.4167e-3,
        818.747e-3, 640, 496, 1, SSTV_RGB, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:208-219, VIS 0x3f */
    ROW(SSTV_W2120, "Wraase SC-2 120", "W2120", 0x3f, 5.5225e-3, 0.5e-3, 0e-3,
        0.489039081e-3, 475.530018e-3, 320, 256, 1, SSTV_RGB, 3, SSTV_LAYOUT_DEFAULT, 0),
    /* modespec.c:221-232, VIS 0x37 */
    ROW(SSTV_W2180, "Wraase SC-2 180", "W2180", 0x37, 5.5225e-3, 0.5e-3, 0e-3, 0.734532e-3,
        711.0225e-3, 320, 256, 1, SSTV_RGB, 3, SSTV_LAYOUT_DEFAULT, 0),
};

#define NMODES ((int)(sizeof(MODES) / sizeof(MODES[0])))

int sstv_mode_count(void) { return NMODES; }

const sstv_mode *sstv_mode_at(int index)
{
    if (index < 0 || index >= NMODES) return NULL;
    return &MODES[index].pub;
}

const sstv_mode_ext *sstv_mode_ext_by_id(int id)
{
    int i;
    for (i = 0; i < NMODES; i++)
        if (MODES[i].pub.id == id) return &MODES[i];
    return NULL;
}

const sstv_mode *sstv_mode_by_id(int id)
{
    const sstv_mode_ext *m = sstv_mode_ext_by_id(id);
    return m ? &m->pub : NULL;
}

static int same_name(const char *a, const char *b)
{
    for (; *a && *b; a++, b++)
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
    return *a == *b;
}

const sstv_mode *sstv_mode_find(const char *name)
{
    int i;
    if (!name) return NULL;
    for (i = 0; i < NMODES; i++)
        if (same_name(name, MODES[i].pub.short_name) || same_name(name, MODES[i].pub.name))
            return &MODES[i].pub;
    return NULL;
}

const sstv_mode *sstv_mode_by_vis(int vis)
{
    int i;
    if (vis <= 0 || vis > 0x7f) return NULL;
    for (i = 0; i < NMODES; i++)
        if (MODES[i].pub.vis == vis) return &MODES[i].pub;
    return NULL;
}

#include "fldigi_dsp.h"

#define STR2(x) #x
#define STR(x) STR2(x)

const char *sstv_version(void)
{
    return "sstv-core " SSTV_VERSION_STRING " contract=sstv-c1 abi=" STR(SSTV_ABI)
           " tx=1 dsp=" FDSP_VERSION_STRING;
}

int sstv_abi(void) { return SSTV_ABI; }

size_t sstv_sizeof(int what)
{
    switch (what) {
    case SSTV_SIZEOF_MODE: return sizeof(sstv_mode);
    case SSTV_SIZEOF_RX_CONFIG: return sizeof(sstv_rx_config);
    case SSTV_SIZEOF_EVENT: return sizeof(sstv_event);
    case SSTV_SIZEOF_IMAGE_INFO: return sizeof(sstv_image_info);
    case SSTV_SIZEOF_TX_CONFIG: return sizeof(sstv_tx_config);
    default: return 0;
    }
}
