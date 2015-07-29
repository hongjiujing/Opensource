/*
 * Nokia OMAP3 development board
 *
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu-common.h"
#include "sysemu.h"
#include "console.h"
#include "boards.h"
#include "omap.h"
#include "i2c.h"
#include "flash.h"
#include "devices.h"
#include "net.h"
#include "sysbus.h"
#include "blockdev.h"

#ifdef CONFIG_GLES2
#include "gles2.h"
#endif

/* use a generic display panel instead of the real one */
#define N00_FAKE_DISPLAY

/* whether the display frame buffer is emulated */
#define N00_DSI_DOUBLEBUFFER

#define N00_SDRAM_SIZE      (512 * 1024 * 1024)
#define N00_ONENAND_CS      0
#define N00_ONENAND_BUFSIZE (0xc000 << 1)
#define N00_SMC_CS          1

#define N00_CAM_FOCUS_GPIO      38
#define N00_DAC33_IRQ_GPIO      53
#define N00_SMC_IRQ_GPIO        54
#define N00_DAC33_RESET_GPIO    60
#define N00_TS_IRQ_GPIO         61
#define N00_HIMALAYA_TE_GPIO    62
#define N00_AMI305_IRQ_GPIO     63
#define N00_AMI305_DRDY_GPIO    64
#define N00_ONENAND_GPIO        65
#define N00_CAM_CAPTURE_GPIO    67
#define N00_TS_RESET_GPIO       81
#define N00_HIMALAYA_RESET_GPIO 87 /* 163 */
#define N00_KEYPAD_SLIDE_GPIO   109
#define N00_SDCOVER_GPIO        160
#define N00_LIS302DL_INT1_GPIO  180
#define N00_LIS302DL_INT2_GPIO  181


#define N00_DISPLAY_WIDTH   854
#define N00_DISPLAY_HEIGHT  480

#define N00_TRACE(fmt, ...) \
    fprintf(stdout, "%s@%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#ifndef N00_FAKE_DISPLAY
#include "dsi.h"

//#define DEBUG_TAAL
#ifdef DEBUG_TAAL
#define TRACETAAL(...) N00_TRACE(__VA_ARGS__)
#else
#define TRACETAAL(...)
#endif

#define TAAL_TE_PERIODIC
#ifdef TAAL_TE_PERIODIC
#define TAAL_TE_CLOCK vm_clock
//#define TAAL_TE_CLOCK host_clock
#endif

typedef struct himalaya_s {
    DSICommonDevice dsi;
    struct omap_dss_s *dss;
    DisplayState *ds;
    qemu_irq te_irq;
#ifdef TAAL_TE_PERIODIC
    QEMUTimer *te_timer;
    int64_t te_timer_freq;
#endif
    int force_update;
#ifdef N00_DSI_DOUBLEBUFFER
    struct {
        void *data;
        int row_pitch, col_pitch;
        int bits_per_pixel;
    } fb;
#endif
} HimalayaState;

static uint32_t himalaya_read(DSICommonDevice *dev, uint32_t data, int len)
{
    switch (dev->cmd) {
    case 0xda: /* get id1 */
        return DSI_MAKERETURNBYTE(0);
    case 0xdb: /* get id2 */
        return DSI_MAKERETURNBYTE(0x84);
    case 0xdc: /* get id3 */
        return DSI_MAKERETURNBYTE(0);
    default:
        hw_error("%s: unknown command 0x%02x\n", __FUNCTION__, dev->cmd);
        break;
    }
    return 0;
}

static void himalaya_write(DSICommonDevice *dev, uint32_t data, int len)
{
    switch (dev->cmd) {
    case 0x51: /* set brightness */
        /* ignore */
        break;
    case 0x53: /* display control */
        /* ignore */
        break;
    case 0x55: /* write cabc */
        /* ignore */
        break;
    default:
        hw_error("%s: unknown command 0x%02x\n", __FUNCTION__, dev->cmd);
        break;
    }
}

static void himalaya_invalidate_display(void *opaque)
{
    HimalayaState *s = opaque;
    s->force_update = 1;
}

static void himalaya_reset(DSICommonDevice *dev)
{
    HimalayaState *s = DO_UPCAST(HimalayaState, dsi, dev);
    himalaya_invalidate_display(s);
#ifdef TAAL_TE_PERIODIC
    TRACETAAL("stop te generator");
    qemu_del_timer(s->te_timer);
    s->te_timer_freq = 0;
#endif
    if (s->te_irq) {
        qemu_irq_lower(s->te_irq);
    }
}

static void himalaya_reset_irq(void *opaque, int n, int level)
{
    HimalayaState *s = opaque;
    if (n) {
        hw_error("%s: unknown interrupt source\n", __FUNCTION__);
    } else {
        if (!level) {
            TRACETAAL("performing reset...");
            s->dsi.dsi.info->qdev.reset(&s->dsi.dsi.qdev);
        }
    }
}

static void himalaya_temode_changed(DSICommonDevice *dev)
{
#ifdef TAAL_TE_PERIODIC
    HimalayaState *s = FROM_DSI_DEVICE(HimalayaState, dev);
    switch (dev->te_mode) {
        case te_off:
            qemu_del_timer(s->te_timer);
            s->te_timer_freq = 0;
            break;
        case te_vsync:
            s->te_timer_freq = get_ticks_per_sec() / 60;
            break;
        case te_hvsync:
            s->te_timer_freq = get_ticks_per_sec() / (60 * N00_DISPLAY_HEIGHT);
            break;
        default:
            break;
    }
    TRACETAAL("new te freq = %" PRId64 "(mode %d)", s->te_timer_freq,
              dev->te_mode);
    if (s->te_timer_freq) {
        qemu_mod_timer(s->te_timer,
                       qemu_get_clock(TAAL_TE_CLOCK) + s->te_timer_freq);
    }
#endif
}

static void himalaya_powermode_changed(DSICommonDevice *dev)
{
    TRACETAAL("sleep=%s, display=%s",
              (dev->powermode & 0x10) ? "off" : "on",
              (dev->powermode & 0x04) ? "on" : "off");
    himalaya_invalidate_display(dev);
#ifdef TAAL_TE_PERIODIC
    if (dev->powermode == 0x14) {
        himalaya_temode_changed(dev);
    } else {
        HimalayaState *s = DO_UPCAST(HimalayaState, dsi, dev);
        qemu_del_timer(s->te_timer);
        s->te_timer_freq = 0;
    }
#endif
}

static void himalaya_te_generator(void *opaque)
{
    HimalayaState *s = opaque;
    if (s->dsi.te_mode != te_off && (s->dsi.powermode & 0x14) == 0x14) {
        TRACETAAL("te_trigger");
        if (s->te_irq) {
            qemu_irq_pulse(s->te_irq);
        } else {
            dsi_te_trigger(&s->dsi.dsi);
        }
    }
#ifdef TAAL_TE_PERIODIC
    qemu_mod_timer(s->te_timer,
                   qemu_get_clock(TAAL_TE_CLOCK) + s->te_timer_freq);
#endif
}

static int himalaya_blt(DSIDevice *dev, void *data, int width, int height,
                        int col_pitch, int row_pitch, int format)
{
    HimalayaState *s = DO_UPCAST(HimalayaState, dsi,
                                 DO_UPCAST(DSICommonDevice, dsi, dev));
    int host_display_enabled = (is_graphic_console() &&
                                (s->dsi.powermode & 0x04));
    TRACETAAL("update area is (%d,%d)-(%d,%d)",
              s->dsi.sc, s->dsi.sp, s->dsi.ec, s->dsi.ep);
    drawfn line_fn = dsi_get_drawfn(dev, format,
#ifdef N00_DSI_DOUBLEBUFFER
                                    s->fb.bits_per_pixel
#else
                                    host_display_enabled
                                    ? ds_get_bits_per_pixel(s->ds) : 32,
#endif
                                    );
    if (line_fn == NULL) {
        hw_error("unsupported omap3 dss color format: %d", format);
    }
    if (width != (s->dsi.ec - s->dsi.sc + 1) ||
        height != (s->dsi.ep - s->dsi.sp + 1)) {
        fprintf(stderr, "%s: target (%dx%d) doesn't match source (%dx%d)\n",
                __FUNCTION__, s->dsi.ec - s->dsi.sc + 1,
                s->dsi.ep - s->dsi.sp + 1, width, height);
    }
    uint8_t *src = data;
#ifdef N00_DSI_DOUBLEBUFFER
    const int dest_row_pitch = ds_get_linesize(s->ds);
    const int dest_col_pitch = ds_get_bytes_per_pixel(s->ds);
    const int fb_col_pitch = s->fb.col_pitch;
    const int fb_row_pitch = s->fb.row_pitch;
    uint8_t *fb = s->fb.data;
    fb += s->dsi.sp * fb_row_pitch + s->dsi.sc * fb_col_pitch;
#else
    const int dest_row_pitch = host_display_enabled
                               ? ds_get_linesize(s->ds) : 0;
    const int dest_col_pitch = host_display_enabled
                               ? ds_get_bytes_per_pixel(s->ds) : 0;
#endif
    uint8_t *dest = host_display_enabled ? ds_get_data(s->ds) : NULL;
    if (dest) {
        dest += s->dsi.sp * dest_row_pitch + s->dsi.sc * dest_col_pitch;
    }
    int i = height;
    if (host_display_enabled && dest) {
#ifdef N00_DSI_DOUBLEBUFFER
        if (fb_col_pitch != dest_col_pitch) {
            fprintf(stderr, "%s: double buffer pixel size mismatch!\n",
                    __FUNCTION__);
        } else {
            const int copy_width = width * dest_col_pitch;
            while (i--) {
                line_fn(NULL, fb, src, width, fb_col_pitch);
                memcpy(dest, fb, copy_width);
                src += row_pitch, dest += dest_row_pitch;
                fb += fb_row_pitch;
            }
        }
#else
        while (i--) {
            line_fn(NULL, dest, src, width, dest_col_pitch);
            src += row_pitch, dest += dest_row_pitch;
        }
#endif
        dpy_update(s->ds, s->dsi.sc, s->dsi.sp, width, height);
#ifdef N00_DSI_DOUBLEBUFFER
    } else {
        while (i--) {
            line_fn(NULL, fb, src, width, fb_col_pitch);
            src += row_pitch, fb += fb_row_pitch;
        }
#endif
    }
    return s->te_irq ? 1 : 0;
}

static void himalaya_update_display(void *opaque)
{
    HimalayaState *s = opaque;
    if (!s->ds) {
        return;
    }
    if (s->force_update) {
        if (ds_get_width(s->ds) != N00_DISPLAY_WIDTH ||
            ds_get_height(s->ds) != N00_DISPLAY_HEIGHT) {
            qemu_console_resize(s->ds, N00_DISPLAY_WIDTH,
                                N00_DISPLAY_HEIGHT);
        }
    }
    if (!(s->dsi.powermode & 0x04)) { /* display power off */
        if (s->force_update) {
            s->force_update = 0;
            void *p = ds_get_data(s->ds);
            if (p) {
                int height = ds_get_height(s->ds);
                int width = ds_get_width(s->ds) *
                            ds_get_bytes_per_pixel(s->ds);
                int row_pitch = ds_get_linesize(s->ds);
                while (height--) {
                    memset(p, 0, width);
                    p += row_pitch;
                }
                dpy_update(s->ds, 0, 0, ds_get_width(s->ds),
                           ds_get_height(s->ds));
            }
        }
    } else {
        if (s->force_update) {
            s->force_update = 0;
#ifdef N00_DSI_DOUBLEBUFFER
            if (s->fb.data) {
                void *p = ds_get_data(s->ds);
                if (p) {
                    if (s->fb.col_pitch != ds_get_bytes_per_pixel(s->ds)) {
                        fprintf(stderr,
                                "%s: double buffer pixel size mismatch!\n",
                                __FUNCTION__);
                    } else {
                        void *q = s->fb.data;
                        int height = ds_get_height(s->ds);
                        int width = ds_get_linesize(s->ds);
                        while (height--) {
                            memcpy(p, q, s->fb.row_pitch);
                            p += width;
                            q += s->fb.row_pitch;
                        }
                        dpy_update(s->ds, 0, 0, N00_DISPLAY_WIDTH,
                                   N00_DISPLAY_HEIGHT);
                    }
                }
            }
#endif     
        }
    }
}

static void himalaya_bltdone(DSIDevice *dev)
{
    HimalayaState *s = DO_UPCAST(HimalayaState, dsi,
                                 DO_UPCAST(DSICommonDevice, dsi, dev));
#ifdef TAAL_TE_PERIODIC
    qemu_mod_timer(s->te_timer,
                   qemu_get_clock(TAAL_TE_CLOCK) + get_ticks_per_sec() / 1000);
#else
    himalaya_te_generator(s);
#endif
}

#ifdef N00_DSI_DOUBLEBUFFER
static void himalaya_screen_dump(void *opaque, const char *filename)
{
    FILE *f;
    uint8_t *d, *d1;
    uint32_t v;
    int y, x;
    uint8_t r, g, b;
    HimalayaState *s = opaque;
    
    if (s->fb.data && (f = fopen(filename, "wb"))) {
        fprintf(f, "P6\n%d %d\n255\n", N00_DISPLAY_WIDTH, N00_DISPLAY_HEIGHT);
        d1 = s->fb.data;
        for (y = 0; y < N00_DISPLAY_HEIGHT; y++) {
            d = d1;
            for (x = 0; x < N00_DISPLAY_WIDTH; x++) {
                switch (s->fb.bits_per_pixel) {
                case 32:
                    v = *(uint32_t *)d;
                    r = (v >> 16) & 0xff;
                    g = (v >> 8) & 0xff;
                    b = v & 0xff;
                    break;
                case 24:
                    r = ((uint8_t *)d)[2];
                    g = ((uint8_t *)d)[1];
                    b = ((uint8_t *)d)[0];
                    break;
                case 16:
                    v = (uint32_t) (*(uint16_t *)d);
                    r = (v >> 8) & 0xf8;
                    g = (v >> 3) & 0xfc;
                    b = (v << 3) & 0xf8;
                    break;
                case 15:
                    v = (uint32_t) (*(uint16_t *)d);
                    r = (v >> 7) & 0xf8;
                    g = (v >> 2) & 0xf8;
                    b = (v << 3) & 0xf8;
                    break;
                default:
                    r = g = b = 0;
                    break;
                }
                fputc(r, f);
                fputc(g, f);
                fputc(b, f);
                d += s->fb.col_pitch;
            }
            d1 += s->fb.row_pitch;
        }
        fclose(f);
    }
}
#endif

static int himalaya_init(DSIDevice *dev)
{
    HimalayaState *s = DO_UPCAST(HimalayaState, dsi,
                                 DO_UPCAST(DSICommonDevice, dsi, dev));
    qdev_init_gpio_in(&dev->qdev, himalaya_reset_irq, 1);
    qdev_init_gpio_out(&dev->qdev, &s->te_irq, 1);
    s->ds = graphic_console_init(himalaya_update_display,
                                 himalaya_invalidate_display,
#ifdef N00_DSI_DOUBLEBUFFER
                                 himalaya_screen_dump,
#else
                                 NULL,
#endif
                                 NULL, s);
    qemu_console_resize(s->ds, N00_DISPLAY_WIDTH, N00_DISPLAY_HEIGHT);
#ifdef N00_DSI_DOUBLEBUFFER
    s->fb.col_pitch = ds_get_bytes_per_pixel(s->ds);
    s->fb.row_pitch = N00_DISPLAY_WIDTH * s->fb.col_pitch;
    s->fb.bits_per_pixel = ds_get_bits_per_pixel(s->ds);
    s->fb.data = qemu_malloc(N00_DISPLAY_HEIGHT * s->fb.row_pitch);
#endif
#ifdef TAAL_TE_PERIODIC
    s->te_timer = qemu_new_timer(TAAL_TE_CLOCK, himalaya_te_generator, s);
#endif
    return 0;
}

static DSICommonDeviceInfo himalaya_info = {
    .dsi.qdev.name = "himalaya",
    .dsi.qdev.size = sizeof(HimalayaState),
    .dsi.init = himalaya_init,
    .dsi.blt = himalaya_blt,
    .dsi.bltdone = himalaya_bltdone,
    .write = himalaya_write,
    .read = himalaya_read,
    .reset = himalaya_reset,
    .powermode_changed = himalaya_powermode_changed,
    .temode_changed = himalaya_temode_changed
};

#endif // !N00_FAKE_DISPLAY

//#define DEBUG_MXT

#ifdef DEBUG_MXT
#define TRACE_MXT(...) N00_TRACE(__VA_ARGS__)
#else
#define TRACE_MXT(...)
#endif

#define N00_MXT_NUM_FINGERS 2
#define N00_MXT_MSG_QUEUESIZE (40 * 8)

typedef struct mxtbl_s {
    i2c_slave i2c;
    qemu_irq *irq;
    int firstbyte;
    uint8_t cmd;
    uint8_t status;
} MXTBLState;

typedef struct mxt_s {
    i2c_slave i2c;
    qemu_irq irq;
    void (*invalidate_display)(void *);
    void *invalidate_display_opaque;
    int reset;
    int address;
    int reg;
    uint8_t check;
    uint8_t cached_tx;
    uint8_t powercfg[3];
    struct {
        int swapxy;
        int mirror_x;
        int mirror_y;
        int res_x;
        int res_y;
        struct {
            int x;
            int y;
            int contact;
        } finger[N00_MXT_NUM_FINGERS];
        uint8_t reg[31];
    } touch;
    uint8_t selftest[6];
    uint8_t comms[2];
    uint8_t otg[18];
    struct {
        uint8_t data[N00_MXT_MSG_QUEUESIZE];
        int used;
        int read_pos, write_pos;
        int end_counter;
    } msg_queue;
    MXTBLState *bl;
} MXTState;

static const uint8_t mxt_ib[] = {
    /* header */
    0x80, /* family id */
    0x00, /* variant id */
    0x00, /* version */
    0x00, /* build */
    0x40, /* matrix x */
    0x40, /* matrix y */
    0x07, /* num objects */
    /* #1: msg processor */
    0x05,       /* type */
    0x00, 0x01, /* address */
    0x08,       /* size - 1 */
    0x00,       /* inst - 1 */
    0x01,       /* rids/inst */
    /* #2: cmd processor */
    0x06,       /* type */
    0x00, 0x02, /* address */
    0xff,       /* size - 1 */
    0x00,       /* inst - 1 */
    0x01,       /* rids/inst */
    /* #3: powerconfig */
    0x07,       /* type */
    0x00, 0x03, /* address */
    0x02,       /* size - 1 */
    0x00,       /* inst - 1 */
    0x01,       /* rids/inst */
    /* #4: touch */
    0x09,                /* type */
    0x00, 0x04,          /* address */
    0x1e,                /* size - 1 */
    0x00,                /* inst - 1 */
    N00_MXT_NUM_FINGERS, /* rids/inst */
    /* #5: commsconfig */
    0x12,       /* type */
    0x00, 0x06, /* address */
    0x01,       /* size - 1 */
    0x00,       /* inst - 1 */
    0x01,       /* rids/inst */
    /* #6: onetouchgesture */
    0x18,       /* type */
    0x00, 0x07, /* address */
    0x12,       /* size - 1 */
    0x00,       /* inst - 1 */
    0x01,       /* rids/inst */
    /* #7: selftest */
    0x19,       /* type */
    0x00, 0x05, /* address */
    0x05,       /* size - 1 */
    0x00,       /* inst - 1 */
    0x01,       /* rids/inst */
    /* crc24 placeholder */
    0,0,0
};

static void mxt_reset(DeviceState *dev)
{
    MXTState *s = FROM_I2C_SLAVE(MXTState, I2C_SLAVE_FROM_QDEV(dev));
    s->address = 0;
    s->reg = 0;
    s->check = 0;
    //memset(s->powercfg, 0, sizeof(s->powercfg));
    memset(s->touch.finger, 0, sizeof(s->touch.finger));
    //memset(s->selftest, 0, sizeof(s->selftest));
    //memset(s->comms, 0, sizeof(s->comms));
    memset(s->otg, 0, sizeof(s->otg));
    s->msg_queue.used = 0;
    s->msg_queue.read_pos = 0;
    s->msg_queue.write_pos = 0;
    s->msg_queue.end_counter = 0;
    qemu_irq_raise(s->irq);
}

static uint8_t mxt_crc8(uint8_t crc, uint8_t c)
{
    uint8_t fb;
    int i;

    for (i = 0; i < 8; i++) {
        fb = (crc ^ c) & 0x01;
        c >>= 1;
        crc >>= 1;
        if (fb) {
            crc ^= 0x8c;
        }
    }

    return crc;
}

static uint32_t mxt_crc24_sub(uint32_t crc, uint8_t b0, uint8_t b1)
{
    uint32_t r = ((crc) << 1) ^ (uint32_t)((uint16_t)(b1 << 8) | b0);
    if (r & (1 << 24)) {
        r ^= 0x80001b;
    }
    return r;
}

static uint32_t mxt_crc24(uint32_t crc, const uint8_t *data, int len)
{
    const int swords = len >> 1;
    int i;

    for (i = 0; i < swords; i++) {
        crc = mxt_crc24_sub(crc, data[i * 2], data[i * 2 + 1]);
    }
    if (len & 1) {
        crc = mxt_crc24_sub(crc, data[len - 1], 0);
    }
    return crc & 0x00ffffff;
}

static uint8_t mxt_add_msg_queue_byte(MXTState *s, uint8_t d, uint8_t crc)
{
    if (s->msg_queue.used < N00_MXT_MSG_QUEUESIZE) {
        s->msg_queue.data[s->msg_queue.write_pos++] = d;
        if (s->msg_queue.write_pos >= N00_MXT_MSG_QUEUESIZE) {
            s->msg_queue.write_pos = 0;
        }
        s->msg_queue.used++;
        return mxt_crc8(crc, d);
    }
    hw_error("%s: message queue full\n", __FUNCTION__);
    return 0;
}

static void mxt_add_msg_queue(MXTState *s, uint8_t x0, uint8_t x1, uint8_t x2,
                              uint8_t x3, uint8_t x4, uint8_t x5, uint8_t x6,
                              uint8_t x7)
{
    if (s->msg_queue.used <= N00_MXT_MSG_QUEUESIZE - 8) {
        uint8_t crc = mxt_add_msg_queue_byte(s, x0, 0);
        crc = mxt_add_msg_queue_byte(s, x1, crc);
        crc = mxt_add_msg_queue_byte(s, x2, crc);
        crc = mxt_add_msg_queue_byte(s, x3, crc);
        crc = mxt_add_msg_queue_byte(s, x4, crc);
        crc = mxt_add_msg_queue_byte(s, x5, crc);
        crc = mxt_add_msg_queue_byte(s, x6, crc);
        crc = mxt_add_msg_queue_byte(s, x7, crc);
        mxt_add_msg_queue_byte(s, crc, crc);
        qemu_irq_lower(s->irq);
    } else {
        fprintf(stderr, "%s: message queue full, dropping "
                "%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n", __FUNCTION__,
                x0, x1, x2, x3, x4, x5, x6, x7);
    }
}

#define MXT_CRC24_CFG(data) \
    for (i = 0; i < sizeof(data); i++, count++) { \
        if ((count & 1)) { \
            b1 = data[i]; \
            crc = mxt_crc24_sub(crc, b0, b1); \
        } else { \
            b0 = data[i]; \
        } \
    }

static void mxt_add_cmd_msg(MXTState *s, uint8_t flags)
{
    uint8_t b0 = 0, b1 = 0, i;
    uint32_t crc = 0, count = 0;
    MXT_CRC24_CFG(s->powercfg);
    MXT_CRC24_CFG(s->touch.reg);
    MXT_CRC24_CFG(s->comms);
    MXT_CRC24_CFG(s->selftest);
    MXT_CRC24_CFG(s->otg);
    if ((count & 1)) {
        crc = mxt_crc24_sub(crc, b0, 0);
    }
    crc &= 0x00ffffff;
    mxt_add_msg_queue(s, 2,     /* cmd message */
                      flags,
                      crc & 0xff,
                      (crc >> 8) & 0xff,
                      (crc >> 16) & 0xff,
                      0, 0, 0); /* unknown */
}

static int mxt_rx_msg(MXTState *s, int addr)
{
    int result;

    s->reg--; /* freeze register ptr */
    if (!s->msg_queue.end_counter) {
        if (s->msg_queue.used) {
            result = s->msg_queue.data[s->msg_queue.read_pos++];
            if (s->msg_queue.read_pos >= N00_MXT_MSG_QUEUESIZE) {
                s->msg_queue.read_pos = 0;
            }
            s->msg_queue.used--;
        } else {
            s->msg_queue.end_counter = 1;
        }
    }

    if (s->msg_queue.end_counter) {
        if (s->msg_queue.end_counter++ < 9) {
            result = 0xff;
        } else {
            s->msg_queue.end_counter = 0;
            result = 0xc9;       /* crc for a message full of 0xff's */
            qemu_irq_raise(s->irq);
        }
    }
    
    TRACE_MXT("0x%02x", result);
    return result;
}

static void mxt_tx_cmd(MXTState *s, int addr, uint8_t data)
{
    switch (addr) {
    case 0:
        switch (data) {
        case 0x01:
        case 0xa5:
            TRACE_MXT("command: reset");
            mxt_reset(&s->i2c.qdev);
            mxt_add_cmd_msg(s, 0x80);
            mxt_add_cmd_msg(s, 0x00);
            if (data == 0xa5) {
                s->bl->status = 0xc0; /* flash mode, waiting command */
            }
            break;
        default:
            TRACE_MXT("unknown value (0x%02x) written to reset reg", data);
        }
        break;
    case 1:
        if (data == 0x55) {
            TRACE_MXT("command: backup");
        } else {
            TRACE_MXT("unknown value (0x%02x) written to backup reg", data);
        }
        break;
    case 2:
        if (data == 0x01) {
            TRACE_MXT("command: calibrate");
            mxt_add_cmd_msg(s, 0x10); /* device calibrating */
            mxt_add_cmd_msg(s, 0x00); 
        } else {
            TRACE_MXT("unknown value (0x%02x) written to calibration reg",
                      data);
        }
        break;
    default:
        hw_error("%s: unknown cmd reg %d (value 0x%02x)\n", __FUNCTION__,
                 addr, data);
        break;
    }
}

static int mxt_rx_powercfg(MXTState *s, int addr)
{
    int result = -1;
    switch (addr) {
    case 0x00: /* idleacqint */
    case 0x01: /* actvacqint */
    case 0x02: /* act2idleto */
        result = s->powercfg[addr];
        TRACE_MXT("reg 0x%02x = 0x%02x", addr, result);
        break;
    default:
        hw_error("%s: unknown power config reg %d\n", __FUNCTION__, addr);
        break;
    }
    return result;
}

static void mxt_tx_powercfg(MXTState *s, int addr, uint8_t data)
{
    switch (addr) {
    case 0x00: /* idleacqint */
    case 0x01: /* actvacqint */
    case 0x02: /* act2idleto */
        TRACE_MXT("reg 0x%02x = 0x%02x", addr, data);
        s->powercfg[addr] = data;
        break;
    default:
        hw_error("%s: unknown power config reg %d (value 0x%02x)\n", __FUNCTION__,
                 addr, data);
        break;
    }
}

static int mxt_rx_touch(MXTState *s, int addr)
{
    int result = -1;
    switch (addr) {
    case 0x00 ... 0x1e:
        result = s->touch.reg[addr];
        TRACE_MXT("reg[0x%02x] = 0x%02x", addr, result);
        break;
    default:
        hw_error("%s: unknown touch reg %d\n", __FUNCTION__, addr);
        break;
    }
    return result;
}

static void mxt_tx_touch(MXTState *s, int addr, uint8_t data)
{
    switch (addr) {
    case 0x00 ... 0x08:
    case 0x0a ... 0x11:
    case 0x16 ... 0x1f:
        TRACE_MXT("reg[0x%02x] = 0x%02x", addr, data);
        s->touch.reg[addr] = data;
        break;
    case 0x09:
        TRACE_MXT("orientation = 0x%02x", data);
        s->touch.reg[addr] = data;
        s->touch.swapxy = data & 1;
        /* TODO: invertx/y flags */
        break;
    case 0x12 ... 0x13:
        s->touch.reg[addr] = data;
        s->touch.res_x = (((int)s->touch.reg[0x13]) << 8) |
                         (s->touch.reg[0x12]);
        TRACE_MXT("x range = %d", s->touch.res_x);
        break;
    case 0x14 ... 0x15:
        s->touch.reg[addr] = data;
        s->touch.res_y = (((int)s->touch.reg[0x15]) << 8) |
                         (s->touch.reg[0x14]);
        TRACE_MXT("y range = %d", s->touch.res_y);
        break;
    default:
        printf("%s: unknown touch reg %d (value 0x%02x)\n", __FUNCTION__,
               addr, data);
        break;
    }
}

static int mxt_rx_selftest(MXTState *s, int addr)
{
    int result = 1;
    switch (addr) {
    case 0x00 ... 0x05:
        result = s->selftest[addr];
        TRACE_MXT("reg[0x%02x] = 0x%02x", addr, result);
        break;
    default:
        hw_error("%s: unknown selftest reg %d\n", __FUNCTION__, addr);
        break;
    }
    return result;
}

static void mxt_tx_selftest(MXTState *s, int addr, uint8_t data)
{
    switch (addr) {
    case 0x00:          /* control */
    case 0x02 ... 0x05: /* hisglim, losiglim */
        TRACE_MXT("reg[0x%02x] = 0x%02x", addr, data);
        s->selftest[addr] = data;
        break;
    case 0x01:          /* cmd */
        TRACE_MXT("cmd 0x%02x", data);
        s->selftest[1] = data;
        if ((s->selftest[0] & 3) == 3) { /* enable | rpten */
            mxt_add_msg_queue(s,
                              4 + N00_MXT_NUM_FINGERS + 2, /*selftest message*/
                              0xfe, /* test passed */
                              0, 0, 0, 0, 0, 0); /* info(??) */
        }
        break;
    default:
        hw_error("%s: unknown selftest reg %d (value 0x%02x)\n", __FUNCTION__,
                 addr, data);
        break;
    }
}

static int mxt_rx_comms(MXTState *s, int addr)
{
    int result = -1;
    switch (addr) {
    case 0x00: /* control */
    case 0x01: /* command */
        result = s->comms[addr];
        TRACE_MXT("reg[0x%02x] = 0x%02x", addr, result);
        break;
    default:
        hw_error("%s: unknown commsconfig reg %d\n", __FUNCTION__, addr);
        break;
    }
    return result;
}

static void mxt_tx_comms(MXTState *s, int addr, uint8_t data)
{
    switch (addr) {
    case 0x00: /* control */
    case 0x01: /* command */
        TRACE_MXT("reg[0x%02x] = 0x%02x", addr, data);
        s->comms[addr] = data;
        break;
    default:
        hw_error("%s: unknown commsconfig reg %d (value 0x%02x)\n",
                 __FUNCTION__, addr, data);
        break;
    }
}

static int mxt_rx_otg(MXTState *s, int addr)
{
    int result = -1;
    switch (addr) {
    case 0x00: /* control */
        result = s->otg[0];
        TRACE_MXT("ctrl = 0x%02x", result);
        break;
    case 0x01: /* numgest */
        result = s->otg[1];
        TRACE_MXT("numgest = 0x%02x", result);
        break;
    case 0x02 ... 0x03: /* gesten */
        result = s->otg[addr];
        TRACE_MXT("gesten%d = 0x%02x", addr - 0x02, result);
        break;
    case 0x04: /* process */
        result = s->otg[4];
        TRACE_MXT("process = 0x%02x", result);
        break;
    case 0x05: /* tapto */
        result = s->otg[5];
        TRACE_MXT("tapto = 0x%02x", result);
        break;
    case 0x06: /* flickto */
        result = s->otg[6];
        TRACE_MXT("flickto = 0x%02x", result);
        break;
    case 0x07: /* dragto */
        result = s->otg[7];
        TRACE_MXT("dragto = 0x%02x", result);
        break;
    case 0x08: /* spressto */
        result = s->otg[8];
        TRACE_MXT("spressto = 0x%02x", result);
        break;
    case 0x09: /* lpressto */
        result = s->otg[9];
        TRACE_MXT("lpressto = 0x%02x", result);
        break;
    case 0x0a: /* represstapto */
        result = s->otg[10];
        TRACE_MXT("represstapto = 0x%02x", result);
        break;
    case 0x0b ... 0x0c: /* flickthr */
        result = s->otg[addr];
        TRACE_MXT("flickthr%d = 0x%02x", addr - 0x0b, result);
        break;
    case 0x0d ... 0x0e: /* dragthr */
        result = s->otg[addr];
        TRACE_MXT("dragthr%d = 0x%02x", addr - 0x0d, result);
        break;
    case 0x0f ... 0x10: /* tapthr */
        result = s->otg[addr];
        TRACE_MXT("tapthr%d = 0x%02x", addr - 0x0f, result);
        break;
    case 0x11 ... 0x12: /* throwthr */
        result = s->otg[addr];
        TRACE_MXT("throwthr%d = 0x%02x", addr - 0x11, result);
        break;
    default:
        hw_error("%s: unknown otg reg %d\n", __FUNCTION__, addr);
        break;
    }
    return result;
}

static void mxt_tx_otg(MXTState *s, int addr, uint8_t data)
{
    switch (addr) {
    case 0x00: /* control */
        TRACE_MXT("ctrl = 0x%02x", data);
        s->otg[0] = data;
        break;
    case 0x01: /* numgest */
        TRACE_MXT("numgest = 0%02x", data);
        s->otg[1] = data;
        break;
    case 0x02 ... 0x03: /* gesten */
        s->otg[addr] = data;
        TRACE_MXT("gesten = 0x%02x%02x", s->otg[2], s->otg[3]);
        break;
    case 0x04: /* process */
        TRACE_MXT("process = 0x%02x", data);
        s->otg[4] = data;
        break;
    case 0x05: /* tapto */
        TRACE_MXT("tapto = 0x%02x", data);
        s->otg[5] = data;
        break;
    case 0x06: /* flickto */
        TRACE_MXT("flickto = 0x%02x", data);
        s->otg[6] = data;
        break;
    case 0x07: /* dragto */
        TRACE_MXT("dragto = 0x%02x", data);
        s->otg[7] = data;
        break;
    case 0x08: /* spressto */
        TRACE_MXT("spressto = 0x%02x", data);
        s->otg[8] = data;
        break;
    case 0x09: /* lpressto */
        TRACE_MXT("lpressto = 0x%02x", data);
        s->otg[9] = data;
        break;
    case 0x0a: /* represstapto */
        TRACE_MXT("represstapto = 0x%02x", data);
        s->otg[10] = data;
        break;
    case 0x0b ... 0x0c: /* flickthr */
        s->otg[addr] = data;
        TRACE_MXT("flickthr = 0x%02x%02x", s->otg[11], s->otg[12]);
        break;
    case 0x0d ... 0x0e: /* dragthr */
        s->otg[addr] = data;
        TRACE_MXT("dragthr = 0x%02x%02x", s->otg[13], s->otg[14]);
        break;
    case 0x0f ... 0x10: /* tapthr */
        s->otg[addr] = data;
        TRACE_MXT("tapthr = 0x%02x%02x", s->otg[15], s->otg[16]);
        break;
    case 0x11 ... 0x12: /* throwthr */
        s->otg[addr] = data;
        TRACE_MXT("throwthr = 0x%02x%02x", s->otg[17], s->otg[18]);
        break;
    default:
        hw_error("%s: unknown otg reg %d (value 0x%02x)\n",
                 __FUNCTION__, addr, data);
        break;
    }
}

static int mxt_rx(i2c_slave *i2c)
{
    MXTState *s = FROM_I2C_SLAVE(MXTState, i2c);
    int value = -1;
    if (!s->reset) {
        if (s->reg < sizeof(mxt_ib)) {
            if (s->reg < sizeof(mxt_ib) - 3) {
                value = mxt_ib[s->reg];
            } else {
                value = (int)mxt_crc24(0, mxt_ib, sizeof(mxt_ib) - 3);
                value >>= (3 - (sizeof(mxt_ib) - s->reg)) * 8;
                value &= 0xff;
            }
            TRACE_MXT("read info block @ 0x%02x returns 0x%02x", s->reg, value);
        } else switch (s->reg) {
        case 0x0100 ... 0x01ff:
            value = mxt_rx_msg(s, s->reg & 0xff);
            break;
        case 0x0300 ... 0x03ff:
            value = mxt_rx_powercfg(s, s->reg & 0xff);
            break;
        case 0x0400 ... 0x04ff:
            value = mxt_rx_touch(s, s->reg & 0xff);
            break;
        case 0x0500 ... 0x05ff:
            value = mxt_rx_selftest(s, s->reg & 0xff);
            break;
        case 0x0600 ... 0x06ff:
            value = mxt_rx_comms(s, s->reg & 0xff);
            break;
        case 0x0700 ... 0x07ff:
            value = mxt_rx_otg(s, s->reg & 0xff);
            break;
        default:
            hw_error("%s: unknown reg=0x%04x\n", __FUNCTION__, s->reg);
            break;
        }
        s->reg++;
    }
    return value;
}

static void mxt_tx_data(MXTState *s, uint8_t data)
{
    switch (s->reg) {
    case 0x0200 ... 0x02ff:
        mxt_tx_cmd(s, s->reg & 0xff, data);
        break;
    case 0x0300 ... 0x03ff:
        mxt_tx_powercfg(s, s->reg & 0xff, data);
        break;
    case 0x0400 ... 0x04ff:
        mxt_tx_touch(s, s->reg & 0xff, data);
        break;
    case 0x0500 ... 0x05ff:
        mxt_tx_selftest(s, s->reg & 0xff, data);
        break;
    case 0x0600 ... 0x06ff:
        mxt_tx_comms(s, s->reg & 0xff, data);
        break;
    case 0x0700 ... 0x07ff:
        mxt_tx_otg(s, s->reg & 0xff, data);
        break;
    default:
        hw_error("%s: unknown reg=0x%04x, data=0x%02x\n", __FUNCTION__,
                 s->reg, data);
        break;
    }
    s->reg++;
}

static int mxt_tx(i2c_slave *i2c, uint8_t data)
{
    MXTState *s = FROM_I2C_SLAVE(MXTState, i2c);
    if (s->reset) {
        return -1;
    }
    if (s->address < 2) {
        if (s->address) {
            s->check = data >> 7;
            s->reg |= (uint16_t)(data & 0x7f) << 8;
            TRACE_MXT("set address to 0x%04x", s->reg);
        } else {
            s->reg = data;
        }
        s->address++;
    } else {
        if (s->check) {
            if (s->check > 1) {
                mxt_tx_data(s, s->cached_tx);
            }
            s->check = 2;
            s->cached_tx = data;
        } else {
            mxt_tx_data(s, data);
        }
    }
    return 1;
}

static void mxt_event(i2c_slave *i2c, enum i2c_event event)
{
    MXTState *s = FROM_I2C_SLAVE(MXTState, i2c);
    if (!s->reset) {
        switch (event) {
            case I2C_START_SEND:
                s->address = 0;
                break;
            case I2C_FINISH:
                if (s->check) {
                    /* last received byte was crc, ignore */
                    s->check = 0;
                }
                if (s->reg <= sizeof(mxt_ib)) {
                    s->reg = 0;
                }
                break;
            default:
                break;
        }
    }
}

static void mxt_irq_handler(void *opaque, int n, int level)
{
    MXTState *s = opaque;
    if (n) {
        hw_error("%s: unknown input line %d\n", __FUNCTION__, n);
    } else {
        TRACE_MXT("reset line state = %d", level);
        if (!level) {
            s->reset = 1;
        } else {
            if (s->reset) {
                s->reset = 0;
                mxt_reset(&s->i2c.qdev);
                mxt_add_cmd_msg(s, 0x80); /* reset completed */
                mxt_add_cmd_msg(s, 0x00);
            }
        }
    }
}

static void mxt_report_touch(MXTState *s, int n)
{
    if ((s->otg[0] & 3) == 3) {
        /* should determine which gesture(s) is/are enabled etc. */
        mxt_add_msg_queue(s,
                          4 + N00_MXT_NUM_FINGERS + 1, /* otg message */
                          0x01, /* gesture */
                          0, 0, 0, 0, 0, 0);
    }
    if ((s->touch.reg[0] & 3) == 3) {
        int x = s->touch.swapxy ? s->touch.finger[n].y : s->touch.finger[n].x;
        int y = s->touch.swapxy ? s->touch.finger[n].x : s->touch.finger[n].y;
        mxt_add_msg_queue(s, 4 + n, /* touch finger n message */
                          s->touch.finger[n].contact ? 0x80 : 0x20,
                          (x >> 2) & 0xff,
                          (y >> 2) & 0xff,
                          ((x & 0x03) << 6) | ((y & 0x03) << 2),
                          0x01,   /* area? */
                          0x00,   /* amplitude? */
                          0x00);  /* vector? */
    }
}

static void mxt_mouse(void *opaque, int x, int y, int z, int bs)
{
    MXTState *s = (MXTState *)opaque;
    
    if ((bs & 0x09) || s->touch.finger[0].contact ||
        s->touch.finger[1].contact) {
        int x1 = ((x + 1) * s->touch.res_x) >> 15;
        int y1 = ((y + 1) * s->touch.res_y) >> 15;
        if (x1 < 0) {
            x1 = 0;
        }
        if (y1 < 0) {
            y1 = 0;
        }
        if (x1 > s->touch.res_x) {
            x1 = s->touch.res_x;
        }
        if (y1 > s->touch.res_y) {
            y1 = s->touch.res_y;
        }
        int x2 = s->touch.res_x - x1 - 1;
        int y2 = s->touch.res_y - y1 - 1;
        if (x2 < 0) {
            x2 = 0;
        }
        if (y2 < 0) {
            y2 = 0;
        }
        s->touch.finger[0].x = s->touch.mirror_x ? (s->touch.res_x - x1) : x1;
        s->touch.finger[0].y = s->touch.mirror_y ? (s->touch.res_y - y1) : y1;
        int f0 = s->touch.finger[0].contact || (bs & 0x01);
        s->touch.finger[0].contact = bs & 0x01;
        s->touch.finger[1].x = s->touch.mirror_x ? (s->touch.res_x - x2) : x2;
        s->touch.finger[1].y = s->touch.mirror_y ? (s->touch.res_y - y2) : y2;
        int f1 = s->touch.finger[1].contact || (bs & 0x08);
        s->touch.finger[1].contact = !!(bs & 0x08);
        if (f0) {
            mxt_report_touch(s, 0);
        }
        if (f1) {
            mxt_report_touch(s, 1);
            if (s->invalidate_display) {
                s->invalidate_display(s->invalidate_display_opaque);
            }
        }
        TRACE_MXT("1:(%d,%d,%d), 2:(%d,%d,%d)",
                  s->touch.swapxy ? s->touch.finger[0].y
                                  : s->touch.finger[0].x,
                  s->touch.swapxy ? s->touch.finger[0].x
                                  : s->touch.finger[0].y,
                  s->touch.finger[0].contact,
                  s->touch.swapxy ? s->touch.finger[1].y
                                  : s->touch.finger[1].x,
                  s->touch.swapxy ? s->touch.finger[1].x
                                  : s->touch.finger[1].y,
                  s->touch.finger[1].contact);
    }
}

static void mxt_kbd(void *opaque, int keycode)
{
	fprintf(stderr, "%s enter...\n", __func__);
    MXTState *s = opaque;
    if (keycode != 0xe0 && (keycode & 0x80) && (keycode & 0x7f) == 56 &&
        s->invalidate_display) {
        /* when left or right alt is released, force an update to
         * ensure the "second" mouse cursor is removed from screen
         */
        s->invalidate_display(s->invalidate_display_opaque);
    }
}

static int mxt_init(i2c_slave *i2c)
{
    MXTState *s = FROM_I2C_SLAVE(MXTState, i2c);
    qdev_init_gpio_in(&i2c->qdev, mxt_irq_handler, 1);
    qdev_init_gpio_out(&i2c->qdev, &s->irq, 1);
    multitouch_enabled = 1;
    qemu_add_mouse_event_handler(mxt_mouse, s, 1, "MXT Touchscreen");
    qemu_add_kbd_event_handler(mxt_kbd, s);
    s->bl->irq = &s->irq;
    return 0;
}

static int mxtbl_rx(i2c_slave *i2c)
{
    MXTBLState *s = FROM_I2C_SLAVE(MXTBLState, i2c);
    uint8_t value = s->status;
    switch (value) {
    case 0x02: /* crc check -> crc pass */
        s->status = 0x04;
        break;
    case 0x04: /* crc pass -> waiting frame data */
        s->status = 0x80;
        break;
    default:
        break;
    }
    TRACE_MXT("return 0x%02x", value);
    return value;
}

static int mxtbl_tx(i2c_slave *i2c, uint8_t data)
{
    MXTBLState *s = FROM_I2C_SLAVE(MXTBLState, i2c);
    if ((s->status & 0x87)) {
        if ((s->status & 0x40)) {
            if (s->firstbyte) {
                s->cmd = data;
            } else {
                if (!(s->cmd | data)) {
                    TRACE_MXT("flash mode reset");
                    s->status = 0;
                    qemu_irq_raise(*(s->irq));
                } else if (s->cmd == 0xdc && data == 0xaa) {
                    TRACE_MXT("flash unlock");
                    s->status = 0x80;
                    qemu_irq_lower(*(s->irq));
                } else {
                    hw_error("%s: unknown command sequence 0x%02x 0x%02x\n",
                             __FUNCTION__, s->cmd, data);
                }
            }
            s->firstbyte = !s->firstbyte;
        } else {
            if (s->status == 0x80) {
                /* waiting frame data -> crc check */
                s->status = 0x02;
            }
        }
    } else {
        TRACE_MXT("received 0x%02x, not in flash mode -> ignored", data);
    }
    return 1; 
}

static void mxtbl_event(i2c_slave *i2c, enum i2c_event event)
{
    // ignore
}

static int mxtbl_init(i2c_slave *i2c)
{
    MXTBLState *s = FROM_I2C_SLAVE(MXTBLState, i2c);
    s->firstbyte = 1;
    s->status = 0;
    return 0;
}

static I2CSlaveInfo mxt_info = {
    .qdev.name = "mxt",
    .qdev.size = sizeof(MXTState),
    .qdev.reset = mxt_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_INT32("res_x", MXTState, touch.res_x, 0),
        DEFINE_PROP_INT32("res_y", MXTState, touch.res_y, 0),
        DEFINE_PROP_INT32("swapxy", MXTState, touch.swapxy, 0),
        DEFINE_PROP_INT32("mirror_x", MXTState, touch.mirror_x, 0),
        DEFINE_PROP_INT32("mirror_y", MXTState, touch.mirror_y, 0),
        DEFINE_PROP_END_OF_LIST()
    },
    .init = mxt_init,
    .event = mxt_event,
    .recv = mxt_rx,
    .send = mxt_tx
};

static I2CSlaveInfo mxtbl_info = {
    .qdev.name = "mxtbl",
    .qdev.size = sizeof(MXTBLState),
    .init = mxtbl_init,
    .event = mxtbl_event,
    .recv = mxtbl_rx,
    .send = mxtbl_tx
};

static DeviceState *n00_mxt_init(struct omap_mpu_state_s *mpu,
                                 void (*invalidate_display)(void *),
                                 void *invd_opaque)
{
    i2c_bus *bus = omap_i2c_bus(mpu->i2c, 1);
    DeviceState *dev = i2c_create_slave_noinit(bus, "mxt", 0x4b);
    MXTState *s = FROM_I2C_SLAVE(MXTState, I2C_SLAVE_FROM_QDEV(dev));
    s->invalidate_display = invalidate_display;
    s->invalidate_display_opaque = invd_opaque;
    s->bl = FROM_I2C_SLAVE(MXTBLState,
                           I2C_SLAVE_FROM_QDEV(i2c_create_slave(bus, "mxtbl",
                                                                0x25)));
    qdev_prop_set_int32(dev, "swapxy", 1);
    qdev_prop_set_int32(dev, "res_x", N00_DISPLAY_WIDTH);
    qdev_prop_set_int32(dev, "res_y", N00_DISPLAY_HEIGHT);
    qdev_prop_set_int32(dev, "mirror_x", 0);
    qdev_prop_set_int32(dev, "mirror_y", 0);
    qdev_init_nofail(dev);
    qdev_connect_gpio_out(dev, 0,
                          qdev_get_gpio_in(mpu->gpio, N00_TS_IRQ_GPIO));
    qdev_connect_gpio_out(mpu->gpio, N00_TS_RESET_GPIO,
                          qdev_get_gpio_in(dev, 0));

    /* setup himalaya specific configuration values */
    s->powercfg[0] = 0x14;
    s->powercfg[1] = 0x0a;
    s->powercfg[2] = 0x32;
    s->touch.reg[0] = 0x8f;
    s->touch.reg[1] = 0x00;
    s->touch.reg[2] = 0x00;
    s->touch.reg[3] = 0x13;
    s->touch.reg[4] = 0x0b;
    s->touch.reg[5] = 0x00;
    s->touch.reg[6] = 0x30;
    s->touch.reg[7] = 0x46;
    s->touch.reg[8] = 0x02;
    s->touch.reg[9] = (s->touch.swapxy ? 0x01 : 0x00);
    s->touch.reg[10] = 0x00;
    s->touch.reg[11] = 0x0a;
    s->touch.reg[12] = 0x06;
    s->touch.reg[13] = 0x0e;
    s->touch.reg[14] = N00_MXT_NUM_FINGERS;
    s->touch.reg[15] = 0x05;
    s->touch.reg[16] = 0x0a;
    s->touch.reg[17] = 0x00;
    s->touch.reg[18] = (s->touch.swapxy ? s->touch.res_y
                        : s->touch.res_x) & 0xff;
    s->touch.reg[19] = ((s->touch.swapxy ? s->touch.res_y
                         : s->touch.res_x) >> 8) & 0xff;
    s->touch.reg[20] = (s->touch.swapxy ? s->touch.res_x
                        : s->touch.res_y) & 0xff;
    s->touch.reg[21] = ((s->touch.swapxy ? s->touch.res_x
                         : s->touch.res_y) >> 8) & 0xff;
    s->touch.reg[22] = 0x00;
    s->touch.reg[23] = 0x00;
    s->touch.reg[24] = 0x00;
    s->touch.reg[25] = 0x00;
    s->touch.reg[26] = 0xd7;
    s->touch.reg[27] = 0x2e;
    s->touch.reg[28] = 0x98;
    s->touch.reg[29] = 0x54;
    s->touch.reg[30] = 0x00;
    s->selftest[0] = 0x03;
    s->selftest[1] = 0x00;
    s->selftest[2] = 0x80;
    s->selftest[3] = 0x2f;
    s->selftest[4] = 0x06;
    s->selftest[5] = 0x18;
    s->comms[0] = 0x00;
    s->comms[1] = 0x00;
    
    return dev;
}

typedef struct dac33_s {
    i2c_slave i2c;
    int firstbyte;
    int reg;
    qemu_irq irq;
    uint8_t data[0x80];
} DAC33State;

static void dac33_interrupt_update(DAC33State *s)
{
    qemu_irq_raise(s->irq);
}

static void dac33_reset(DeviceState *dev)
{
    DAC33State *s = FROM_I2C_SLAVE(DAC33State, I2C_SLAVE_FROM_QDEV(dev));
    s->data[0x00] = 0x00;
    s->data[0x01] = 0x00;
    s->data[0x0e] = 0x01;
    s->data[0x7d] = 0xda;
    s->data[0x7e] = 0x33;
    s->data[0x7f] = 0x03;
    dac33_interrupt_update(s);
}

static void dac33_event(i2c_slave *i2c, enum i2c_event event)
{
    DAC33State *s = FROM_I2C_SLAVE(DAC33State, i2c);
    if (event == I2C_START_SEND)
        s->firstbyte = 1;
}

static int dac33_rx(i2c_slave *i2c)
{
    DAC33State *s = FROM_I2C_SLAVE(DAC33State, i2c);
    int value = 0;
    switch (s->reg) {
    case 0 ... 0x7f:
        value = s->data[s->reg];
        break;
    default:
        hw_error("%s: unknown register 0x%02x\n", __FUNCTION__, s->reg);
        break;
    }
    s->reg++;
    return value;
}

static int dac33_tx(i2c_slave *i2c, uint8_t data)
{
    DAC33State *s = FROM_I2C_SLAVE(DAC33State, i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
        case 0:
            if (data) {
                fprintf(stderr, "%s: non-zero register pages not supported\n",
                        __FUNCTION__);
            }
            break;
        case 1:
            if (data & 0x80) { /* RST */
                dac33_reset(&i2c->qdev);
            } else {
                s->data[s->reg] = data & 0x7f;
            }
            break;
        case 2 ... 0x7c:
            s->data[s->reg] = data;
            break;
        default:
            hw_error("%s: unknown register 0x%02x\n", __FUNCTION__, s->reg);
            break;
        }
        s->reg++;
    }
    return 1;
}

static void dac33_irq_handler(void *opaque, int n, int level)
{
    if (n) {
        hw_error("%s: unknown interrupt source %d\n", __FUNCTION__, n);
    } else {
        if (!level) {
            dac33_reset(opaque);
        }
    }
}

static int dac33_init(i2c_slave *i2c)
{
    DAC33State *s = FROM_I2C_SLAVE(DAC33State, i2c);
    qdev_init_gpio_in(&i2c->qdev, dac33_irq_handler, 1);
    qdev_init_gpio_out(&i2c->qdev, &s->irq, 1);
    return 0;
}

static I2CSlaveInfo dac33_info = {
    .qdev.name = "dac33",
    .qdev.size = sizeof(DAC33State),
    .qdev.reset = dac33_reset,
    .init = dac33_init,
    .event = dac33_event,
    .recv = dac33_rx,
    .send = dac33_tx
};

static DeviceState *n00_dac33_init(struct omap_mpu_state_s *cpu)
{
    DeviceState *dev = i2c_create_slave(omap_i2c_bus(cpu->i2c, 1),
                                        "dac33", 0x19);
    qdev_connect_gpio_out(dev, 0,
                          qdev_get_gpio_in(cpu->gpio, N00_DAC33_IRQ_GPIO));
    qdev_connect_gpio_out(cpu->gpio, N00_DAC33_RESET_GPIO,
                          qdev_get_gpio_in(dev, 0));
    return dev;
}

typedef struct bh1780gli_s {
    i2c_slave i2c;
    int firstbyte;
    int reg;
    uint8_t control;
    uint16_t data;
} BH1780GLIState;

static void bh1780gli_reset(DeviceState *dev)
{
    BH1780GLIState *s = FROM_I2C_SLAVE(BH1780GLIState,
                                       I2C_SLAVE_FROM_QDEV(dev));
    s->control = 0x00;
    s->data = 0xffff;
}

static int bh1780gli_change(DeviceState *dev, const char *target,
                            const char *arg)
{
    BH1780GLIState *s = (BH1780GLIState *)dev;
    uint16_t value = 0;
    if (strcmp(target, "data") ||
        sscanf(arg, "%" SCNu16, &value) != 1) {
        return -1;
    }
    s->data = value;
    return 0;
}

static void bh1780gli_event(i2c_slave *i2c, enum i2c_event event)
{
    BH1780GLIState *s = FROM_I2C_SLAVE(BH1780GLIState, i2c);
    if (event == I2C_START_SEND)
        s->firstbyte = 1;
}

static void bh1780gli_reg_advance(BH1780GLIState *s)
{
    switch (s->reg) {
    case 0x00: s->reg = 0x0a; break;
    case 0x0a: s->reg = 0x0b; break;
    case 0x0b: s->reg = 0x0c; break;
    case 0x0c: s->reg = 0x0d; break;
    case 0x0d: s->reg = 0x00; break;
    default: break;
    }
}

static int bh1780gli_rx(i2c_slave *i2c)
{
    BH1780GLIState *s = FROM_I2C_SLAVE(BH1780GLIState, i2c);
    int value = 0;
    switch (s->reg) {
    case 0x00: /* CONTROL */
        value = s->control;
        break;
    case 0x0a: /* PART ID */
        value = 0x81;
        break;
    case 0x0b: /* MANUFACTURE ID */
        value = 0x01;
        break;
    case 0x0c: /* DATALOW */
        value = s->data;
        break;
    case 0x0d: /* DATAHIGH */
        value = s->data >> 8;
        break;
    case 0xff: /* unrecognized command byte */
        break;
    default:
        hw_error("%s: unknown register 0x%02x\n", __FUNCTION__, s->reg);
        break;
    }
    bh1780gli_reg_advance(s);
    return value;
}

static int bh1780gli_tx(i2c_slave *i2c, uint8_t data)
{
    BH1780GLIState *s = FROM_I2C_SLAVE(BH1780GLIState, i2c);
    if (s->firstbyte) {
        s->reg = (data & 0x80) ? (data & 0x0f) : 0xff;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
        case 0: /* CONTROL */
            s->control = data;
            break;
        case 0x0a: /* PART ID */
        case 0x0b: /* MANUFACTURE ID */
            /* read-only */
            break;
        case 0x0c: /* DATALOW */
        case 0x0d: /* DATAHIGH */
            /* read-only? */
            break;
        case 0xff: /* unrecognized command byte */
            break;
        default:
            hw_error("%s: unknown register 0x%02x\n", __FUNCTION__, s->reg);
            break;
        }
        bh1780gli_reg_advance(s);
    }
    return 1;
}

static int bh1780gli_init(i2c_slave *i2c)
{
    return 0;
}

static I2CSlaveInfo bh1780gli_info = {
    .qdev.name = "bh1780gli",
    .qdev.size = sizeof(BH1780GLIState),
    .qdev.reset = bh1780gli_reset,
    .qdev.change = bh1780gli_change,
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT16("data", BH1780GLIState, data, 0xffff),
        DEFINE_PROP_END_OF_LIST()
    },
    .init = bh1780gli_init,
    .event = bh1780gli_event,
    .recv = bh1780gli_rx,
    .send = bh1780gli_tx
};

typedef struct ami305_s {
    i2c_slave i2c;
    int firstbyte;
    int reg;
    qemu_irq irq[2];
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t ctrl3;
    uint8_t int_ctrl;
    uint8_t int_thres;
    uint16_t x;
    uint16_t y;
    uint16_t z;
} AMI305State;

static void ami305_reset(DeviceState *dev)
{
    AMI305State *s = FROM_I2C_SLAVE(AMI305State,
                                       I2C_SLAVE_FROM_QDEV(dev));
    s->ctrl1 = 0x00;
    s->ctrl2 = 0x00;
    s->ctrl3 = 0x00;
    s->x = 0x00;
    s->y = 0x00;
    s->z = 0x00;
    s->int_ctrl = 0x00;
    s->int_thres = 0x00;
}

static int ami305_change(DeviceState *dev, const char *target,
                            const char *arg)
{
    AMI305State *s = (AMI305State *)dev;
    uint16_t value = 0;
    int axis;

    if (!strcmp(target, "x")) {
        axis = 0;
    } else if (!strcmp(target, "y")) {
        axis = 1;
    } else if (!strcmp(target, "z")) {
        axis = 2;
    } else {
        return -1;
    }
    if (sscanf(arg, "%" SCNu16, &value) != 1) {
        return -1;
    }
    switch (axis) {
    case 0:
        s->x = value;
        break;
    case 1:
        s->y = value;
        break;
    case 2:
        s->z = value;
        break;
    }
    return 0;
}

static void ami305_event(i2c_slave *i2c, enum i2c_event event)
{
    AMI305State *s = FROM_I2C_SLAVE(AMI305State, i2c);
    if (event == I2C_START_SEND)
        s->firstbyte = 1;
}

static int ami305_rx(i2c_slave *i2c)
{
    AMI305State *s = FROM_I2C_SLAVE(AMI305State, i2c);
    int value = 0;
    switch (s->reg) {
    case 0x0c: /* STB */
        value = (s->ctrl3 & 0x10) ? 0xaa : 0x55;
        s->ctrl3 &= ~0x10;
        break;
    case 0x0d: /* INFO */
    case 0x0e: /* INFO */
        value = 0;
        break;
    case 0x0f: /* WHOAMI */
        value = 0x47;
        break;
    case 0x18: /* STATUS */
        value = 0;
        break;
   case 0x1A: /* INT_CLEAR */
        /* lets pretend */
        break;
    case 0x1B: /* CTRL1 */
        value = s->ctrl1;
        break;
    case 0x1C: /* CTRL2 */
        value = s->ctrl2;
        break;
    case 0x1D: /* CTRL3 */
        value = s->ctrl3;
        break;
    case 0x1E: /* INT_CTRL */
        value = s->int_ctrl;
        break;
    case 0x26: /* INT_THRES */
        value = s->int_thres;
        break;
    case 0x10 ... 0x16: /*x-z*/
        break;
    default:
         hw_error("%s: unknown register 0x%02x\n", __FUNCTION__, s->reg);
         break;
    }
    s->reg++;
    return value;
}

static int ami305_tx(i2c_slave *i2c, uint8_t data)
{
    AMI305State *s = FROM_I2C_SLAVE(AMI305State, i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
        case 0x1B: /* CTRL1 */
            s->ctrl1 = data;
            break;
        case 0x1C: /* CTRL2 */
            s->ctrl2 = data;
            break;
        case 0x1D: /* CTRL3 */
            s->ctrl3 = data;
            break;
        case 0x1E: /* INT_CTRL */
            s->int_ctrl = data;
            break;
        case 0x26: /* INT_THRES */
            s->int_thres = data;
            break;
        case 0x30: /* PRESET */
            break;
        default:
            hw_error("%s: unknown register 0x%02x\n", __FUNCTION__, s->reg);
            break;
        }
        s->reg++;
    }
    return 1;
}

static int ami305_init(i2c_slave *i2c)
{
    AMI305State *s = FROM_I2C_SLAVE(AMI305State, i2c);
    qdev_init_gpio_out(&i2c->qdev, s->irq, 2);
    return 0;
}

static I2CSlaveInfo ami305_info = {
    .qdev.name = "ami305",
    .qdev.size = sizeof(AMI305State),
    .qdev.reset = ami305_reset,
    .qdev.change = ami305_change,
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT16("x", AMI305State, x, 0x0),
        DEFINE_PROP_UINT16("y", AMI305State, y, 0x0),
        DEFINE_PROP_UINT16("z", AMI305State, z, 0x0),
        DEFINE_PROP_END_OF_LIST()
    },
    .init = ami305_init,
    .event = ami305_event,
    .recv = ami305_rx,
    .send = ami305_tx
};

typedef struct BQ27521State_s {
    i2c_slave i2c;
    uint8_t firstbyte;
    uint8_t reg;
    
    uint16_t ctrl;
    uint16_t c1, c2;
    uint16_t st;
    uint16_t cl, ch;
    uint16_t cur;
} BQ27521State;

#define BQ27521_READ(r, v) \
    case r: value = s->v & 0xff; break; \
    case r + 1: value = s->v >> 8; break
#define BQ27521_WRITE_LSB(v) s->v = (s->v & 0xff00) | data
#define BQ27521_WRITE_MSB(v) s->v = (s->v & 0xff) | (((uint16_t)data) << 8)
#define BQ27521_WRITE(r, v) \
    case r: BQ27521_WRITE_LSB(v); break; \
    case r + 1: BQ27521_WRITE_MSB(v); break

static void bq27521_reset(DeviceState *qdev)
{
    BQ27521State *s = FROM_I2C_SLAVE(BQ27521State, I2C_SLAVE_FROM_QDEV(qdev));
    s->firstbyte = 0;
    s->ctrl = 0x0005;
    s->c1 = 0x37ff;
    s->c2 = 0x00c0;
    s->st = 0x0040;
    s->cl = s->ch = 0;
    s->cur = 0x0200; /* 512mA */
}

static void bq27521_event(i2c_slave *i2c, enum i2c_event event)
{
    BQ27521State *s = FROM_I2C_SLAVE(BQ27521State, i2c);
    if (event == I2C_START_SEND) {
        s->firstbyte = 1;
    }
}

static int bq27521_rx(i2c_slave *i2c)
{
    BQ27521State *s = FROM_I2C_SLAVE(BQ27521State, i2c);
    int value = -1;
    switch (s->reg) {
    BQ27521_READ(0x02, ctrl);
    BQ27521_READ(0x04, c1);
    BQ27521_READ(0x06, c2);
    BQ27521_READ(0x08, st);
    BQ27521_READ(0x0e, cur);
    BQ27521_READ(0x10, cur);
    case 0x12 ... 0x15:
        value = 0;
        break;
    BQ27521_READ(0x26, cl);
    BQ27521_READ(0x28, ch);
    case 0x32:
    case 0x33:
    case 0x34: 
        value = 0;
        break;
    case 0x35:
        value = 0x21;
        break;
    default:
        hw_error("%s: unknown register 0x%02x\n", __FUNCTION__, s->reg);
        break;
    }
    s->reg++;
    return value;
}

static int bq27521_tx(i2c_slave *i2c, uint8_t data)
{
    BQ27521State *s = FROM_I2C_SLAVE(BQ27521State, i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
        case 0x02:
            if (data & 0x80) {
                bq27521_reset(&i2c->qdev);
            }
            if (data & 0x40) {
                s->st &= ~(1 << 6);
            }
            data &= 0x3f;
            BQ27521_WRITE_LSB(ctrl);
            break;
        case 0x03:
            BQ27521_WRITE_MSB(ctrl);
            break;
        BQ27521_WRITE(0x04, c1);
        BQ27521_WRITE(0x06, c2);
        BQ27521_WRITE(0x26, cl);
        BQ27521_WRITE(0x28, ch);
        default:
            hw_error("%s: unknown register 0x%02x, value 0x%02x\n",
                     __FUNCTION__, s->reg, data);
            break;
        }
        s->reg++;
    }
    return 1;
}

static int bq27521_init(i2c_slave *i2c)
{
    return 0;
}

static I2CSlaveInfo bq27521_info = {
    .qdev.name = "bq27521",
    .qdev.size = sizeof(BQ27521State),
    .qdev.reset = bq27521_reset,
    .init = bq27521_init,
    .event = bq27521_event,
    .recv = bq27521_rx,
    .send = bq27521_tx
};

static uint32_t ssi_read(void *opaque, target_phys_addr_t addr)
{
    switch (addr) {
    case 0x00: /* REVISION */
        return 0x10;
    case 0x14: /* SYSSTATUS */
        return 1; /* RESETDONE */
    default:
        break;
    }
    //printf("%s: addr= " OMAP_FMT_plx "\n", __FUNCTION__, addr);
    return 0;
}

static void ssi_write(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    //printf("%s: addr=" OMAP_FMT_plx ", value=0x%08x\n", __FUNCTION__, addr, value);
}

static CPUReadMemoryFunc *ssi_read_func[] = {
    ssi_read,
    ssi_read,
    ssi_read,
};

static CPUWriteMemoryFunc *ssi_write_func[] = {
    ssi_write,
    ssi_write,
    ssi_write,
};

struct n00_s {
    struct omap_mpu_state_s *cpu;
    void *twl5031;
    DeviceState *nand;
    DeviceState *smc;
    DeviceState *bq24153, *bq24156, *bq27521;
    DeviceState *tpa6130;
    DeviceState *lis302dl;
    DeviceState *dac33;
    DeviceState *bh1780gli;
    DeviceState *ami305;
    DeviceState *mxt;
#ifndef N00_FAKE_DISPLAY
    DeviceState *lcd;
#endif
#ifdef CONFIG_GLES2
    void *gles2;
#endif
    int extended_key;
    int slide_open;
    QEMUTimer *shutdown_timer;
};

#ifdef CONFIG_SKINNING
#include "skin/skin_callbacks.h"
static int n00_switchstate_callback(void *opaque, const int keycode)
{
    struct n00_s *s = opaque;
    switch (keycode) {
        case 0x3b: return s->slide_open;
        default: break;
    }
    return -1;
}

static void n00_skinrotate_callback(void *opaque, int portrait)
{
    struct n00_s *s = opaque;
    s->lis302dl->info->change(s->lis302dl, "xyz",
                              portrait ? "0,58,0" : "58,0,0");
}
#endif

static void n00_reset(void *opaque)
{
    struct n00_s *s = opaque;
    qemu_irq_raise(qdev_get_gpio_in(s->cpu->gpio, N00_CAM_FOCUS_GPIO));
    qemu_irq_raise(qdev_get_gpio_in(s->cpu->gpio, N00_CAM_CAPTURE_GPIO));
    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, N00_KEYPAD_SLIDE_GPIO),
                 !s->slide_open);
    /* FIXME: hack - prevent reboot if shutdown was requested */
    if (s->shutdown_timer) {
        qemu_system_shutdown_request();
    }
}

static uint16_t n00_twl4030_madc_callback(twl4030_adc_type type, int ch)
{
    switch (type) {
        case TWL4030_ADC_GP:
            switch (ch) {
                case 0:                /* battery size indicator */
                case 4:  return 0x2b9; /* battery size indicator (old hw)*/
                case 1:  return 0x1ed; /* battery temperature */
                case 12: return 0x2b9; /* battery voltage */
                default: break;
            }
            break;
        default:
            break;
    }
    fprintf(stderr, "%s: unknown %s channel %d\n", __FUNCTION__,
            (type == TWL4030_ADC_RT) ? "RT" :
            (type == TWL4030_ADC_GP) ? "GP" : "BCI", ch);
    return 0;
}

static const TWL4030KeyMap n00_twl4030_keymap[] = {
    {0x36, 0, 0}, /* RIGHTSHIFT */
    {0x2a, 0, 1}, /* LEFTSHIFT */
    {0x1d, 0, 2}, /* LEFTCTRL */
    {0x38, 0, 3}, /* LEFTMETA --> LEFTALT */
    {0xb8, 0, 3}, /* LEFTMETA --> RIGHTALT */
    {0x9d, 0, 4}, /* RIGHTCTRL */
    {0x0e, 0, 5}, /* BACKSPACE */
    {0x42, 0, 6}, /* VOLUME UP --> F8 */
    {0x41, 0, 7}, /* VOLUME DOWN --> F7 */
    
    /* 1, 0 is reserved */
    /* 1, 1 is reserved */
    /* 1, 2 is reserved */
    {0x2c, 1, 3}, /* Z */
    {0x1e, 1, 4}, /* A */
    {0x10, 1, 5}, /* Q */
    {0x11, 1, 6}, /* W */
    {0x12, 1, 7}, /* E */
    
    /* 2, 0 is reserved */
    /* 2, 1 is reserved */
    /* 2, 2 is reserved */
    {0x2d, 2, 3}, /* X */
    {0x1f, 2, 4}, /* S */
    {0x20, 2, 5}, /* D */
    {0x2e, 2, 6}, /* C */
    {0x2f, 2, 7}, /* V */
    
    /* 3, 0 is reserved */
    /* 3, 1 is reserved */
    /* 3, 2 is reserved */
    {0x18, 3, 3}, /* O */
    {0x17, 3, 4}, /* I */
    {0x16, 3, 5}, /* U */
    {0x26, 3, 6}, /* L */
    {0x28, 3, 7}, /* APOSTROPHE (') */
    
    /* 4, 0 is reserved */
    /* 4, 1 is reserved */
    /* 4, 2 is reserved */
    {0x15, 4, 3}, /* Y */
    {0x25, 4, 4}, /* K */
    {0x24, 4, 5}, /* J */
    {0x23, 4, 6}, /* H */
    {0x22, 4, 7}, /* G */
    
    /* 5, 0 is reserved */
    /* 5, 1 is reserved */
    /* 5, 2 is reserved */
    {0x30, 5, 3}, /* B */
    {0x33, 5, 4}, /* COMMA (,) */
    {0x32, 5, 5}, /* M */
    {0x31, 5, 6}, /* N */
    {0x34, 5, 7}, /* DOT (.) */
    
    {0x39, 6, 0}, /* SPACE */
    /* 6, 1 is reserved */
    /* 6, 2 is reserved */
    {0x14, 6, 3}, /* T */
    {0xc8, 6, 4}, /* UP */
    {0xcb, 6, 5}, /* LEFT */
    {0xcd, 6, 6}, /* RIGHT */
    {0xd0, 6, 7}, /* DOWN */
    
    /* 7, 0 is reserved */
    /* 7, 1 is reserved */
    /* 7, 2 is reserved */
    {0x19, 7, 3}, /* P */
    {0x1c, 7, 4}, /* ENTER */
    {0x35, 7, 5}, /* SLASH (/) */
    {0x21, 7, 6}, /* F */
    {0x13, 7, 7}, /* R */
    
    {-1, -1, -1}
};

static void n00_key_handler(void *opaque, int keycode)
{
    struct n00_s *s = opaque;
    if (!s->extended_key && keycode == 0xe0) {
        s->extended_key = 0x80;
    } else {
        int release = keycode & 0x80;
        keycode = (keycode & 0x7f) | s->extended_key;
        s->extended_key = 0;
        switch (keycode) {
        case 0x01: /* escape */
            twl4030_set_powerbutton_state(s->twl5031, !release);
            break;
        case 0x3b: /* f1 */
            if (release) {
                s->slide_open = !s->slide_open;
                qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio,
                                              N00_KEYPAD_SLIDE_GPIO),
                             !s->slide_open);
            }
            break;
        case 0x3e: /* f4 */
            qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, N00_CAM_FOCUS_GPIO),
                         !!release);
            break;
        case 0x3f: /* f5 */
            qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, N00_CAM_CAPTURE_GPIO),
                         !!release);
            break;
        case 0x4f ... 0x50: /* kp1,2 */
            lis302dl_step(s->lis302dl, 0, keycode - 0x4f, !release);
            break;
        case 0x4b ... 0x4c: /* kp4,5 */
            lis302dl_step(s->lis302dl, 1, keycode - 0x4b, !release);
            break;
        case 0x47 ... 0x48: /* kp7,8 */
            lis302dl_step(s->lis302dl, 2, keycode - 0x47, !release);
            break;
        default:
            break;
        }
    }
}

static void n00_shutdown_timer_callback(void *opaque)
{
    static int power_state = 0;
    struct n00_s *s = opaque;
    power_state = !power_state;
    twl4030_set_powerbutton_state(s->twl5031, power_state);
    qemu_mod_timer(s->shutdown_timer,
                   qemu_get_clock(vm_clock)
                   + get_ticks_per_sec() * (power_state ? 10LL : 1LL));
}

static int n00_display_close_callback(void *opaque)
{
    struct n00_s *s = opaque;
    if (s->shutdown_timer == NULL) {
        s->shutdown_timer = qemu_new_timer(vm_clock,
                                           n00_shutdown_timer_callback,
                                           s);
        n00_shutdown_timer_callback(s);
    }
    return 0;
}

static void n00_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename,
                     const char *kernel_cmdline,
                     const char *initrd_filename,
                     const char *cpu_model)
{
    struct n00_s *s = (struct n00_s *)qemu_mallocz(sizeof(*s));
    DriveInfo *dmtd = drive_get(IF_MTD, 0, 0);
    DriveInfo *dsd  = drive_get(IF_SD, 0, 0);
    
    if (!dmtd && !dsd) {
        hw_error("%s: SD or NAND image required", __FUNCTION__);
    }
#if MAX_SERIAL_PORTS < 4
#error MAX_SERIAL_PORTS must be at least 4!
#endif
    s->cpu = omap3_mpu_init(omap3630, 1, N00_SDRAM_SIZE,
                            serial_hds[1], serial_hds[2],
                            serial_hds[0], serial_hds[3]);
    s->twl5031 = twl5031_init(omap_i2c_bus(s->cpu->i2c, 0),
                              s->cpu->irq[0][OMAP_INT_3XXX_SYS_NIRQ],
                              NULL, n00_twl4030_keymap);
    twl4030_madc_attach(s->twl5031, n00_twl4030_madc_callback);
#ifdef N00_FAKE_DISPLAY
    omap_lcd_panel_attach(s->cpu->dss);
#else
    s->lcd = dsi_create_common_device(omap_dsi_host(s->cpu->dss),
                                      "himalaya", 0);
    qdev_connect_gpio_out(s->lcd, 0,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N00_HIMALAYA_TE_GPIO));
    qdev_connect_gpio_out(s->cpu->gpio, N00_HIMALAYA_RESET_GPIO,
                          qdev_get_gpio_in(s->lcd, 0));
#endif
    s->nand = onenand_create(NAND_MFR_STMICRO, 0x58, 0x40, 1, 
                             qdev_get_gpio_in(s->cpu->gpio, N00_ONENAND_GPIO),
                             dmtd ? dmtd->bdrv : NULL);
    omap_gpmc_attach(s->cpu->gpmc, 0, s->nand, 0, 0);
    
    if (dsd) {
        omap3_mmc_attach(s->cpu->omap3_mmc[1], dsd->bdrv, 0, 1);
    }
    if ((dsd = drive_get(IF_SD, 0, 1)) != NULL) {
        omap3_mmc_attach(s->cpu->omap3_mmc[0], dsd->bdrv, 0, 0);
        qemu_irq_raise(qdev_get_gpio_in(s->cpu->gpio, N00_SDCOVER_GPIO));
    }
    
    cpu_register_physical_memory(0x48058000, 0x3c00,
                                 cpu_register_io_memory(ssi_read_func,
                                                        ssi_write_func, 0,
                                                        DEVICE_NATIVE_ENDIAN));
    s->mxt = n00_mxt_init(s->cpu,
#ifdef N00_FAKE_DISPLAY
                          omap_lcd_panel_invalidate_display, s->cpu->dss
#else
                          himalaya_invalidate_display, s->lcd
#endif
                          );
    cursor_hide = 0; // who wants to use touchscreen without a pointer?
    cursor_allow_grab = 0; // ...and please, don't stop the host cursor
    
    int i;
    for (i = 0; i < nb_nics; i++) {
        if (!nd_table[i].model || !strcmp(nd_table[i].model, "smc91c111")) {
            break;
        }
    }
    if (i < nb_nics) {
        s->smc = qdev_create(NULL, "smc91c111");
        qdev_set_nic_properties(s->smc, &nd_table[i]);
        qdev_init_nofail(s->smc);
        sysbus_connect_irq(sysbus_from_qdev(s->smc), 0,
                           qdev_get_gpio_in(s->cpu->gpio, N00_SMC_IRQ_GPIO));
        omap_gpmc_attach(s->cpu->gpmc, 1, s->smc, 0, 0);
    } else {
        hw_error("%s: no NIC for smc91c111\n", __FUNCTION__);
    }

    i2c_bus *i2c2 = omap_i2c_bus(s->cpu->i2c, 1);
    s->bq24153 = i2c_create_slave_noinit(i2c2, "bq2415x", 0x6b);
    qdev_prop_set_uint8(s->bq24153, "id", 0x51);
    s->bq24153->id = "bq24153";
    qdev_init_nofail(s->bq24153);
    s->bq24156 = i2c_create_slave_noinit(i2c2, "bq2415x", 0x6a);
    qdev_prop_set_uint8(s->bq24156, "id", 0x41);
    s->bq24156->id = "bq24156";
    qdev_init_nofail(s->bq24156);
    s->bq27521 = i2c_create_slave(i2c2, "bq27521", 0x55);
    s->tpa6130 = i2c_create_slave(i2c2, "tpa6130", 0x60);

    i2c_bus *i2c3 = omap_i2c_bus(s->cpu->i2c, 2);
    s->bh1780gli = i2c_create_slave(i2c3, "bh1780gli", 0x29);
    s->lis302dl = i2c_create_slave_noinit(i2c3, "lis302dl", 0x1d);
    qdev_prop_set_int32(s->lis302dl, "x", 58);
    qdev_prop_set_int32(s->lis302dl, "y", 0);
    qdev_prop_set_int32(s->lis302dl, "z", 0);
    qdev_prop_set_int32(s->lis302dl, "axis_max", 58);
    qdev_init_nofail(s->lis302dl);
    s->ami305 = i2c_create_slave(i2c3, "ami305", 0x0f);
    qdev_connect_gpio_out(s->lis302dl, 0,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N00_LIS302DL_INT1_GPIO));
    qdev_connect_gpio_out(s->lis302dl, 1,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N00_LIS302DL_INT2_GPIO));
    qdev_connect_gpio_out(s->ami305, 0,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N00_AMI305_IRQ_GPIO));
    qdev_connect_gpio_out(s->ami305, 1,
                          qdev_get_gpio_in(s->cpu->gpio,
                                            N00_AMI305_DRDY_GPIO));
    s->dac33 = n00_dac33_init(s->cpu);

#ifdef CONFIG_GLES2
    s->gles2 = gles2_init(s->cpu->env);
#endif

    s->slide_open = 0;

#ifdef CONFIG_SKINNING
    qemu_skin_add_switchstate_callback(n00_switchstate_callback, s);
    qemu_skin_add_rotate_callback(n00_skinrotate_callback, s);
    // the skin has already rotated at this point if requested,
    // so we need to follow that manually here...
    if (graphic_rotate) {
        n00_skinrotate_callback(s, 1);
    }
#endif
    qemu_add_kbd_event_handler(n00_key_handler, s);
    qemu_set_display_close_handler(n00_display_close_callback, s);
    qemu_register_reset(n00_reset, s);
}

static QEMUMachine n00_machine = {
    .name = "n00",
    .desc = "Nokia N00 (OMAP3)",
    .init = n00_init,
};

static void n00_register_devices(void)
{
    i2c_register_slave(&mxt_info);
    i2c_register_slave(&mxtbl_info);
    i2c_register_slave(&bq27521_info);
    i2c_register_slave(&dac33_info);
    i2c_register_slave(&bh1780gli_info);
    i2c_register_slave(&ami305_info);
#ifndef N00_FAKE_DISPLAY
    dsi_register_common_device(&himalaya_info);
#endif
}

static void n00_machine_init(void)
{
    qemu_register_machine(&n00_machine);
}

device_init(n00_register_devices);
machine_init(n00_machine_init);
