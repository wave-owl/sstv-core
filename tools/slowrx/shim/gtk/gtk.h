/* Headless stand-in for <gtk/gtk.h>: just the types and calls slowrx's vis.c, video.c,
 * sync.c, fsk.c and modespec.c use, so the decoder builds without GTK (bench harness only,
 * never part of libsstv). SPDX-License-Identifier: ISC */
#ifndef SLOWRX_SHIM_GTK_H
#define SLOWRX_SHIM_GTK_H
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef int gboolean;
typedef unsigned char guchar;
typedef short gshort;
typedef unsigned short gushort;
typedef unsigned int guint;
typedef int gint;
typedef int16_t gint16;
typedef int32_t gint32;
typedef char gchar;
typedef size_t gsize;
#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

typedef struct GtkWidget GtkWidget;
typedef struct GtkListStore GtkListStore;
typedef struct GKeyFile GKeyFile;
typedef struct GdkPixbuf {
    int width, height, rowstride;
    guchar *pixels;
} GdkPixbuf;

#define GDK_COLORSPACE_RGB 0
#define GDK_INTERP_BILINEAR 0
#define GDK_INTERP_HYPER 0
#define GTK_STATUSBAR(x) (x)
#define GTK_TOGGLE_BUTTON(x) (x)
#define GTK_COMBO_BOX(x) (x)
#define GTK_SPIN_BUTTON(x) (x)
#define GTK_IMAGE(x) (x)

static inline void gdk_threads_enter(void) {}
static inline void gdk_threads_leave(void) {}
static inline void gtk_statusbar_push(GtkWidget *w, int c, const char *s) { (void)w; (void)c; (void)s; }
static inline gboolean gtk_toggle_button_get_active(GtkWidget *w) { (void)w; return TRUE; }
static inline void gtk_widget_set_sensitive(GtkWidget *w, gboolean b) { (void)w; (void)b; }
static inline int gtk_combo_box_get_active(GtkWidget *w) { (void)w; return 0; }
static inline void gtk_combo_box_set_active(GtkWidget *w, int i) { (void)w; (void)i; }
static inline void gtk_spin_button_set_value(GtkWidget *w, double v) { (void)w; (void)v; }
static inline int gtk_spin_button_get_value_as_int(GtkWidget *w) { (void)w; return 0; }
static inline void gtk_image_set_from_pixbuf(GtkWidget *w, GdkPixbuf *p) { (void)w; (void)p; }

static inline GdkPixbuf *gdk_pixbuf_new(int cs, gboolean alpha, int bits, int w, int h)
{
    GdkPixbuf *p = (GdkPixbuf *)calloc(1, sizeof(GdkPixbuf));
    (void)cs; (void)alpha; (void)bits;
    p->width = w;
    p->height = h;
    p->rowstride = w * 3;
    p->pixels = (guchar *)calloc((size_t)w * h * 3 + 3, 1);
    return p;
}
static inline void g_object_unref(void *o)
{
    GdkPixbuf *p = (GdkPixbuf *)o;
    if (p) { free(p->pixels); free(p); }
}
static inline void gdk_pixbuf_fill(GdkPixbuf *p, unsigned v)
{
    size_t i;
    for (i = 0; i < (size_t)p->rowstride * p->height; i++) p->pixels[i] = (guchar)(v >> 24);
}
static inline int gdk_pixbuf_get_rowstride(GdkPixbuf *p) { return p->rowstride; }
static inline guchar *gdk_pixbuf_get_pixels(GdkPixbuf *p) { return p->pixels; }
static inline GdkPixbuf *gdk_pixbuf_scale_simple(GdkPixbuf *p, int w, int h, int interp)
{
    (void)p; (void)w; (void)h; (void)interp;
    return gdk_pixbuf_new(0, FALSE, 8, 1, 1);
}
#endif
