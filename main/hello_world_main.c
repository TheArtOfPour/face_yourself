// face_yourself: live face detection on an ILI9341 with a black / white / red
// cyberpunk HUD. XIAO ESP32S3 Sense camera -> detector task -> rotated, stylised
// preview.
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_camera.h"
#include "face_detection.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LCD_HOST SPI2_HOST
#define LCD_H_RES 240
#define LCD_V_RES 320
#define LCD_BAND_HEIGHT 40
#define LCD_BAND_COUNT (LCD_V_RES / LCD_BAND_HEIGHT)
#define LCD_BAND_BYTES (LCD_H_RES * LCD_BAND_HEIGHT * sizeof(uint16_t))

#define CAMERA_FRAME_WIDTH 320
#define CAMERA_FRAME_HEIGHT 240

// Rotating the 320x240 camera frame 90 degrees CCW gives exactly 240x320:
// a 1:1 pixel mapping with no cropping and no scaling.
_Static_assert(CAMERA_FRAME_HEIGHT == LCD_H_RES, "rotated frame must match panel width");
_Static_assert(CAMERA_FRAME_WIDTH == LCD_V_RES, "rotated frame must match panel height");

#define MAX_DETECTED_FACES 4

// The model input is square (224x224), so the detector gets a centered
// 240x240 crop of the camera frame instead of a squashed 320x240 one.
// (Faces in the outer 40 px on each side of the camera frame are not seen.)
#define DETECT_SIZE 240
#define DETECT_CROP_X ((CAMERA_FRAME_WIDTH - DETECT_SIZE) / 2)

// ---- Tracking / stability tuning ----
#define TRACK_HOLD_US      (600 * 1000)  // keep a box this long after the last detection
#define TRACK_NEW_WEIGHT   0.6f          // smoothing: weight of the newest detection
#define TRACK_IOU_MIN      0.20f         // min overlap to treat a detection as the same face

// ---- Misc ----
#define TOUCH_POLL_EVERY_N_FRAMES 10
#define DETECT_TASK_STACK 16384
#define DETECT_TASK_PRIORITY 5
#define DETECT_TASK_CORE 1

#define CAPTURE_TASK_STACK 4096
#define CAPTURE_TASK_PRIORITY 6
#define CAPTURE_TASK_CORE 0
#define RENDER_TASK_STACK 8192
#define RENDER_TASK_PRIORITY 4
#define RENDER_TASK_CORE 0

#define CAMERA_XCLK_HZ 20000000

// The esp32-camera driver runs the OV3660 at a very conservative pixel clock
// (10 MHz), which caps QVGA at about 3.7 fps. This lowers the sensor's system
// clock divider so the pixel clock, and with it the frame rate, goes up:
//   divider 4 (driver default) ~3.7 fps, 3 ~5 fps, 2 ~7.4 fps.
// Set to 0 to leave the driver's timing alone. If you see torn/garbled frames
// or "FB-OVF" messages, use 3 (or 0).
#define OV3660_SYS_DIV_TARGET 2

// ---- Orientation ----
// The ILI9341's default scan direction shows the image mirrored left-to-right,
// so the panel is mirrored in hardware to make the HUD text read correctly.
#define LCD_MIRROR_X 1
#define LCD_MIRROR_Y 0
// If the video comes out upside down, set this to 1 to turn it 180 degrees.
#define VIDEO_ROTATE_180 0

// ---- Look & feel ----
// Colors are written in "wire" format: RGB565 with the two bytes swapped, the
// same byte order the camera data already has.
#define SWAP16(c)  ((uint16_t)((((c) & 0xFF) << 8) | (((c) >> 8) & 0xFF)))
#define WIRE565(r, g, b) SWAP16((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define COL_BLACK WIRE565(0, 0, 0)
#define COL_WHITE WIRE565(255, 255, 255)
#define COL_RED   WIRE565(255, 0, 24)

#define HUD_TOP_H   22     // top bar occupies rows 0..21, red rule on row 22
#define HUD_BOT_Y   297    // red rule on row 297, bottom bar occupies 298..319
#define SCANLINE_DIM_PERCENT 40   // every 4th row is dimmed to this brightness
#define GLITCH_CHANCE        90   // ~1 glitch per N frames

#define PIN_NUM_LCD_SCLK 7
#define PIN_NUM_LCD_MISO 8
#define PIN_NUM_LCD_MOSI 9
#define PIN_NUM_LCD_CS 1
#define PIN_NUM_LCD_DC 2
#define PIN_NUM_TOUCH_CS 4

#define PIN_NUM_CAMERA_XCLK 10
#define PIN_NUM_CAMERA_SIOD 40
#define PIN_NUM_CAMERA_SIOC 39
#define PIN_NUM_CAMERA_Y9 48
#define PIN_NUM_CAMERA_Y8 11
#define PIN_NUM_CAMERA_Y7 12
#define PIN_NUM_CAMERA_Y6 14
#define PIN_NUM_CAMERA_Y5 16
#define PIN_NUM_CAMERA_Y4 18
#define PIN_NUM_CAMERA_Y3 17
#define PIN_NUM_CAMERA_Y2 15
#define PIN_NUM_CAMERA_VSYNC 38
#define PIN_NUM_CAMERA_HREF 47
#define PIN_NUM_CAMERA_PCLK 13

static const char *TAG = "camera_lcd";

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static uint16_t *band_buf[2];                // internal, DMA-capable, double buffered
static SemaphoreHandle_t lcd_transfer_done;  // COUNTING semaphore (2 transfers can be in flight)
static uint16_t lut_norm[256];               // luma -> black/red/white (wire format)
static uint16_t lut_dim[256];                // same, dimmed for scanlines

typedef struct {
    int left, top, right, bottom;   // display coordinates
    int id;
    char line1[24];
    char line2[16];
} face_overlay_box_t;

typedef struct {
    face_overlay_box_t faces[MAX_DETECTED_FACES];
    size_t face_count;
    uint32_t frame;
    int glitch_frames;   // frames left in the current glitch
    int glitch_y;
    int glitch_h;
    int glitch_dx;
} hud_t;

static hud_t hud;

// Timing counters (microseconds, summed over the logging window)
static uint32_t perf_cam_wait_us, perf_cam_frames;
static uint32_t perf_render_us, perf_spi_wait_us, perf_render_frames;

// Detection thread state
static uint16_t *detect_frame;           // PSRAM copy of the crop being analysed
static atomic_bool detect_busy;
static TaskHandle_t detect_task_handle;
static portMUX_TYPE result_mux = portMUX_INITIALIZER_UNLOCKED;
static face_detection_box_t result_boxes[MAX_DETECTED_FACES];
static size_t result_count;
static uint32_t result_seq;
static uint32_t result_infer_ms;

typedef struct {
    float x1, y1, x2, y2;     // camera-frame coordinates
    int64_t last_seen_us;
    bool active;
} track_t;

static track_t tracks[MAX_DETECTED_FACES];

// ---------------------------------------------------------------------------
// Tracking: smooth boxes and hold them briefly so flags don't flicker
// ---------------------------------------------------------------------------
static float box_iou(const track_t *t, const face_detection_box_t *b)
{
    const float bx1 = (float)b->x1, by1 = (float)b->y1;
    const float bx2 = (float)b->x2, by2 = (float)b->y2;
    const float ix1 = fmaxf(t->x1, bx1), iy1 = fmaxf(t->y1, by1);
    const float ix2 = fminf(t->x2, bx2), iy2 = fminf(t->y2, by2);
    const float iw = ix2 - ix1, ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f) {
        return 0.0f;
    }
    const float inter = iw * ih;
    const float area_t = (t->x2 - t->x1) * (t->y2 - t->y1);
    const float area_b = (bx2 - bx1) * (by2 - by1);
    const float uni = area_t + area_b - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

static void update_tracks(const face_detection_box_t *dets, size_t count, int64_t now_us)
{
    bool matched[MAX_DETECTED_FACES] = {false};

    for (size_t d = 0; d < count && d < MAX_DETECTED_FACES; d++) {
        int best = -1;
        float best_iou = TRACK_IOU_MIN;
        for (int t = 0; t < MAX_DETECTED_FACES; t++) {
            if (!tracks[t].active || matched[t]) continue;
            const float iou = box_iou(&tracks[t], &dets[d]);
            if (iou > best_iou) {
                best_iou = iou;
                best = t;
            }
        }

        if (best < 0) {
            // New face: take a free slot, otherwise evict the stalest track.
            int oldest = 0;
            for (int t = 0; t < MAX_DETECTED_FACES; t++) {
                if (!tracks[t].active) { oldest = t; break; }
                if (tracks[t].last_seen_us < tracks[oldest].last_seen_us) oldest = t;
            }
            best = oldest;
            tracks[best].x1 = (float)dets[d].x1;
            tracks[best].y1 = (float)dets[d].y1;
            tracks[best].x2 = (float)dets[d].x2;
            tracks[best].y2 = (float)dets[d].y2;
        } else {
            const float w = TRACK_NEW_WEIGHT;
            tracks[best].x1 += w * ((float)dets[d].x1 - tracks[best].x1);
            tracks[best].y1 += w * ((float)dets[d].y1 - tracks[best].y1);
            tracks[best].x2 += w * ((float)dets[d].x2 - tracks[best].x2);
            tracks[best].y2 += w * ((float)dets[d].y2 - tracks[best].y2);
        }
        tracks[best].last_seen_us = now_us;
        tracks[best].active = true;
        matched[best] = true;
    }
}

static void expire_tracks(int64_t now_us)
{
    for (int t = 0; t < MAX_DETECTED_FACES; t++) {
        if (tracks[t].active && now_us - tracks[t].last_seen_us > TRACK_HOLD_US) {
            tracks[t].active = false;
        }
    }
}

// Camera frame (x right, y down) -> display (the panel itself is mirrored in
// hardware, so on screen this reads as a 90 degree CCW turn of the picture):
//   display_x = camera_y,   display_y = camera_x
static size_t map_tracks(face_overlay_box_t *out, size_t capacity)
{
    size_t n = 0;
    for (int t = 0; t < MAX_DETECTED_FACES && n < capacity; t++) {
        if (!tracks[t].active) continue;

        const int x1 = (int)tracks[t].x1, y1 = (int)tracks[t].y1;
        const int x2 = (int)tracks[t].x2, y2 = (int)tracks[t].y2;

        face_overlay_box_t *o = &out[n];
#if VIDEO_ROTATE_180
        o->left = (CAMERA_FRAME_HEIGHT - 1) - y2;
        o->right = (CAMERA_FRAME_HEIGHT - 1) - y1;
        o->top = (CAMERA_FRAME_WIDTH - 1) - x2;
        o->bottom = (CAMERA_FRAME_WIDTH - 1) - x1;
#else
        o->left = y1;
        o->right = y2;
        o->top = x1;
        o->bottom = x2;
#endif

        if (o->left < 0) o->left = 0;
        if (o->top < 0) o->top = 0;
        if (o->right >= LCD_H_RES) o->right = LCD_H_RES - 1;
        if (o->bottom >= LCD_V_RES) o->bottom = LCD_V_RES - 1;
        if (o->left >= o->right || o->top >= o->bottom) continue;

        o->id = t;
        n++;
    }
    return n;
}

// ---------------------------------------------------------------------------
// Detection task (runs on the other core, at its own pace)
// ---------------------------------------------------------------------------
static void detect_task(void *arg)
{
    void *detector = arg;
    face_detection_box_t boxes[MAX_DETECTED_FACES];

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const int64_t start_us = esp_timer_get_time();
        const size_t count = face_detection_run(detector, detect_frame,
                                                DETECT_SIZE, DETECT_SIZE,
                                                boxes, MAX_DETECTED_FACES);
        const uint32_t infer_ms = (uint32_t)((esp_timer_get_time() - start_us) / 1000);

        portENTER_CRITICAL(&result_mux);
        memcpy(result_boxes, boxes, sizeof(boxes));
        result_count = count;
        result_infer_ms = infer_ms;
        result_seq++;
        portEXIT_CRITICAL(&result_mux);

        atomic_store(&detect_busy, false);
    }
}

static bool fetch_detection(face_detection_box_t *boxes, size_t *count,
                            uint32_t *infer_ms, uint32_t *last_seq)
{
    bool fresh = false;
    portENTER_CRITICAL(&result_mux);
    if (result_seq != *last_seq) {
        memcpy(boxes, result_boxes, sizeof(result_boxes));
        *count = result_count;
        *infer_ms = result_infer_ms;
        *last_seq = result_seq;
        fresh = true;
    }
    portEXIT_CRITICAL(&result_mux);
    return fresh;
}

// ---------------------------------------------------------------------------
// Palette: camera luma -> black / red / white
// ---------------------------------------------------------------------------
static void build_palette(void)
{
    for (int y = 0; y < 256; y++) {
        // Stretch contrast a little so faces pop.
        int t = (y - 20) * 255 / 200;
        if (t < 0) t = 0;
        if (t > 255) t = 255;

        int r, g, b;
        if (t < 150) {            // black -> red
            r = t * 255 / 150;
            g = 0;
            b = 0;
        } else {                  // red -> white highlights
            r = 255;
            g = b = (t - 150) * 255 / 105;
        }
        lut_norm[y] = WIRE565(r, g, b);
        lut_dim[y] = WIRE565(r * SCANLINE_DIM_PERCENT / 100,
                             g * SCANLINE_DIM_PERCENT / 100,
                             b * SCANLINE_DIM_PERCENT / 100);
    }
}

// Luma (0..254) of one camera pixel, which is stored as big-endian RGB565.
static inline int luma_of(uint16_t wire)
{
    const uint16_t v = __builtin_bswap16(wire);
    const int r = (v >> 11) & 31;
    const int g = (v >> 5) & 63;
    const int b = v & 31;
    return (157 * r + 152 * g + 60 * b) >> 6;
}

// ---------------------------------------------------------------------------
// Tiny drawing toolkit that draws straight into the current 40-row band
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t *buf;
    int y0;     // first display row of this band
} band_t;

static void band_fill(const band_t *b, int x0, int y0, int x1, int y1, uint16_t c)
{
    if (x0 < 0) x0 = 0;
    if (x1 >= LCD_H_RES) x1 = LCD_H_RES - 1;
    if (x0 > x1) return;
    const int ys = y0 > b->y0 ? y0 : b->y0;
    const int band_last = b->y0 + LCD_BAND_HEIGHT - 1;
    const int ye = y1 < band_last ? y1 : band_last;
    for (int y = ys; y <= ye; y++) {
        uint16_t *row = b->buf + (size_t)(y - b->y0) * LCD_H_RES;
        for (int x = x0; x <= x1; x++) row[x] = c;
    }
}

static inline void band_hline(const band_t *b, int x0, int x1, int y, uint16_t c)
{
    band_fill(b, x0, y, x1, y, c);
}

static inline void band_vline(const band_t *b, int x, int y0, int y1, uint16_t c)
{
    band_fill(b, x, y0, x, y1, c);
}

// Classic 5x7 font, ASCII 0x20..0x5A (uppercase only). Each byte is a column,
// bit 0 = top row.
static const uint8_t font5x7[59][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x56,0x20,0x50}, {0x00,0x08,0x07,0x03,0x00}, {0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00}, {0x2A,0x1C,0x7F,0x1C,0x2A}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x80,0x70,0x30,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x00,0x60,0x60,0x00},
    {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x72,0x49,0x49,0x49,0x46},
    {0x21,0x41,0x49,0x4D,0x33}, {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x31}, {0x41,0x21,0x11,0x09,0x07}, {0x36,0x49,0x49,0x49,0x36},
    {0x46,0x49,0x49,0x29,0x1E},
    {0x00,0x00,0x14,0x00,0x00}, {0x00,0x40,0x34,0x00,0x00}, {0x00,0x08,0x14,0x22,0x41},
    {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x59,0x09,0x06},
    {0x3E,0x41,0x5D,0x59,0x4E},
    {0x7C,0x12,0x11,0x12,0x7C}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x41,0x3E}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x41,0x51,0x73}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x1C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x26,0x49,0x49,0x49,0x32}, {0x03,0x01,0x7F,0x01,0x03}, {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x03,0x04,0x78,0x04,0x03}, {0x61,0x59,0x49,0x4D,0x43},
};

static void band_text(const band_t *b, int x, int y, const char *s, int scale, uint16_t color)
{
    if (y + 7 * scale <= b->y0 || y >= b->y0 + LCD_BAND_HEIGHT) return;

    for (; *s; s++, x += 6 * scale) {
        int c = (unsigned char)*s;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c < 0x20 || c > 0x5A) c = 0x20;
        const uint8_t *glyph = font5x7[c - 0x20];

        for (int gc = 0; gc < 5; gc++) {
            for (int gr = 0; gr < 7; gr++) {
                if (!(glyph[gc] & (1 << gr))) continue;
                const int px = x + gc * scale;
                const int py = y + gr * scale;
                band_fill(b, px, py, px + scale - 1, py + scale - 1, color);
            }
        }
    }
}

static void band_brackets(const band_t *b, int l, int t, int r, int bt, int len, int thick, uint16_t c)
{
    for (int k = 0; k < thick; k++) {
        band_hline(b, l, l + len, t + k, c);
        band_hline(b, r - len, r, t + k, c);
        band_hline(b, l, l + len, bt - k, c);
        band_hline(b, r - len, r, bt - k, c);
        band_vline(b, l + k, t, t + len, c);
        band_vline(b, l + k, bt - len, bt, c);
        band_vline(b, r - k, t, t + len, c);
        band_vline(b, r - k, bt - len, bt, c);
    }
}

static void glitch_row(uint16_t *row, int dx)
{
    uint16_t tmp[LCD_H_RES];
    memcpy(tmp, row, sizeof(tmp));
    for (int x = 0; x < LCD_H_RES; x++) {
        int s = x - dx;
        if (s < 0) s += LCD_H_RES;
        if (s >= LCD_H_RES) s -= LCD_H_RES;
        row[x] = tmp[s];
    }
}

// ---------------------------------------------------------------------------
// HUD pieces
// ---------------------------------------------------------------------------
static void draw_face(const band_t *b, const face_overlay_box_t *f)
{
    const int w = f->right - f->left;
    const int h = f->bottom - f->top;
    int len = (w < h ? w : h) / 4;
    if (len < 6) len = 6;
    if (len > 18) len = 18;

    band_brackets(b, f->left, f->top, f->right, f->bottom, len, 2, COL_WHITE);

    const int cx = (f->left + f->right) / 2;
    const int cy = (f->top + f->bottom) / 2;
    band_hline(b, cx - 6, cx + 6, cy, COL_RED);
    band_vline(b, cx, cy - 6, cy + 6, COL_RED);

    int lx = f->left;
    if (lx > LCD_H_RES - 62) lx = LCD_H_RES - 62;
    if (lx < 1) lx = 1;
    int ly = f->top - 19;
    if (ly < HUD_TOP_H + 2) ly = f->bottom + 4;
    if (ly + 17 > HUD_BOT_Y) ly = HUD_BOT_Y - 18;
    band_fill(b, lx - 1, ly - 1, lx + 61, ly + 16, COL_BLACK);
    band_text(b, lx, ly, f->line1, 1, COL_RED);
    band_text(b, lx, ly + 9, f->line2, 1, COL_WHITE);
}

static void draw_hud(const band_t *b, const hud_t *h)
{
    // Corner brackets framing the viewport
    band_brackets(b, 3, HUD_TOP_H + 4, LCD_H_RES - 4, HUD_BOT_Y - 4, 14, 1, COL_WHITE);

    // Top bar
    band_fill(b, 0, 0, LCD_H_RES - 1, HUD_TOP_H - 1, COL_BLACK);
    band_hline(b, 0, LCD_H_RES - 1, HUD_TOP_H, COL_RED);
    band_text(b, 6, 4, "FACE//YOURSELF", 2, COL_RED);
    if ((h->frame / 10) & 1) band_fill(b, 186, 8, 192, 14, COL_RED);   // blinking REC dot
    band_text(b, 198, 8, "REC", 1, COL_WHITE);

    // Bottom bar
    band_fill(b, 0, HUD_BOT_Y + 1, LCD_H_RES - 1, LCD_V_RES - 1, COL_BLACK);
    band_hline(b, 0, LCD_H_RES - 1, HUD_BOT_Y, COL_RED);
    if (h->face_count > 0) {
        if (((h->frame / 5) % 3) != 2) {
            band_text(b, 6, 302, "TARGET LOCKED", 2, COL_RED);
        }
    } else {
        static const char *scan[4] = {"SCANNING", "SCANNING.", "SCANNING..", "SCANNING..."};
        band_text(b, 6, 302, scan[(h->frame / 10) % 4], 2, COL_WHITE);
    }
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
// Turn the camera frame into one 40-row band of the portrait display, restyle
// it, then draw the glitch, scan line, face brackets and HUD on top.
//   display(x, y) = camera(x = y, y = x)
static void render_band(uint16_t *dst, int band_y, const uint16_t *src, const hud_t *h)
{
    const uint16_t *row_lut[LCD_BAND_HEIGHT];
    for (int row = 0; row < LCD_BAND_HEIGHT; row++) {
        row_lut[row] = ((band_y + row) & 3) == 3 ? lut_dim : lut_norm;   // scanlines
    }

    // Outer loop walks display columns (= camera rows) so PSRAM reads stay sequential.
    for (int dx = 0; dx < LCD_H_RES; dx++) {
#if VIDEO_ROTATE_180
        const uint16_t *src_px = src + (size_t)(CAMERA_FRAME_HEIGHT - 1 - dx) * CAMERA_FRAME_WIDTH
                                 + (CAMERA_FRAME_WIDTH - 1 - band_y);
        const int step = -1;
#else
        const uint16_t *src_px = src + (size_t)dx * CAMERA_FRAME_WIDTH + band_y;
        const int step = 1;
#endif
        uint16_t *dst_px = dst + dx;
        for (int row = 0; row < LCD_BAND_HEIGHT; row++) {
            dst_px[(size_t)row * LCD_H_RES] = row_lut[row][luma_of(src_px[row * step])];
        }
    }

    const band_t b = { .buf = dst, .y0 = band_y };

    if (h->glitch_frames > 0) {
        for (int row = 0; row < LCD_BAND_HEIGHT; row++) {
            const int y = band_y + row;
            if (y >= h->glitch_y && y < h->glitch_y + h->glitch_h) {
                glitch_row(dst + (size_t)row * LCD_H_RES, h->glitch_dx);
            }
        }
    }

    // Sweeping scan line
    const int span = HUD_BOT_Y - HUD_TOP_H - 4;
    const int sweep_y = HUD_TOP_H + 2 + (int)((h->frame * 4) % span);
    band_hline(&b, 0, LCD_H_RES - 1, sweep_y, COL_WHITE);
    band_hline(&b, 0, LCD_H_RES - 1, sweep_y - 1, COL_RED);
    band_hline(&b, 0, LCD_H_RES - 1, sweep_y - 2, COL_RED);

    for (size_t i = 0; i < h->face_count; i++) {
        draw_face(&b, &h->faces[i]);
    }
    draw_hud(&b, h);
}

static void update_hud(hud_t *h)
{
    h->frame++;

    h->face_count = map_tracks(h->faces, MAX_DETECTED_FACES);
    for (size_t i = 0; i < h->face_count; i++) {
        face_overlay_box_t *f = &h->faces[i];
        const int pct = 78 + (int)((h->frame / 5 + (uint32_t)f->id * 37) % 21);
        snprintf(f->line1, sizeof(f->line1), "SUBJECT %02d", f->id + 1);
        snprintf(f->line2, sizeof(f->line2), "HUMAN? %d%%", pct);
    }

    if (h->glitch_frames > 0) {
        h->glitch_frames--;
    } else if (esp_random() % GLITCH_CHANCE == 0) {
        h->glitch_frames = 3 + (int)(esp_random() % 4);
        h->glitch_y = HUD_TOP_H + 4 + (int)(esp_random() % 240);
        h->glitch_h = 6 + (int)(esp_random() % 24);
        h->glitch_dx = ((esp_random() & 1) ? 1 : -1) * (6 + (int)(esp_random() % 20));
    }
}

// ---------------------------------------------------------------------------
// LCD
// ---------------------------------------------------------------------------
static bool on_lcd_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *event_data,
                                    void *user_ctx)
{
    (void)panel_io;
    (void)event_data;
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static void wait_for_lcd_transfer(void)
{
    xSemaphoreTake(lcd_transfer_done, portMAX_DELAY);
}

static esp_lcd_panel_handle_t initialize_lcd(void)
{
    for (int i = 0; i < 2; i++) {
        band_buf[i] = heap_caps_malloc(LCD_BAND_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ESP_ERROR_CHECK(band_buf[i] != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    }

    const spi_bus_config_t bus_config = {
        .sclk_io_num = PIN_NUM_LCD_SCLK,
        .mosi_io_num = PIN_NUM_LCD_MOSI,
        .miso_io_num = PIN_NUM_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_BAND_BYTES,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    lcd_transfer_done = xSemaphoreCreateCounting(4, 0);
    ESP_ERROR_CHECK(lcd_transfer_done != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    const esp_lcd_panel_io_spi_config_t lcd_io_config =
        ILI9341_PANEL_IO_SPI_CONFIG(PIN_NUM_LCD_CS, PIN_NUM_LCD_DC,
                                    on_lcd_color_trans_done, lcd_transfer_done);
    esp_lcd_panel_io_handle_t lcd_io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &lcd_io_config, &lcd_io));

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(lcd_io, &panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, LCD_MIRROR_X, LCD_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    return panel;
}

static void clear_lcd(esp_lcd_panel_handle_t panel)
{
    memset(band_buf[0], 0, LCD_BAND_BYTES);
    for (int band = 0; band < LCD_BAND_COUNT; band++) {
        const int y = band * LCD_BAND_HEIGHT;
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_H_RES,
                                                  y + LCD_BAND_HEIGHT, band_buf[0]));
        wait_for_lcd_transfer();
    }
}

static void timed_wait_for_lcd_transfer(void)
{
    const int64_t t0 = esp_timer_get_time();
    wait_for_lcd_transfer();
    perf_spi_wait_us += (uint32_t)(esp_timer_get_time() - t0);
}

// Fill one band while the previous one is still being sent over SPI.
static void present_frame(esp_lcd_panel_handle_t panel, const uint16_t *src, const hud_t *h)
{
    for (int band = 0; band < LCD_BAND_COUNT; band++) {
        if (band >= 2) {
            timed_wait_for_lcd_transfer();  // frees band_buf[band & 1]
        }
        uint16_t *buf = band_buf[band & 1];
        const int y = band * LCD_BAND_HEIGHT;

        const int64_t t0 = esp_timer_get_time();
        render_band(buf, y, src, h);
        perf_render_us += (uint32_t)(esp_timer_get_time() - t0);

        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_H_RES,
                                                  y + LCD_BAND_HEIGHT, buf));
    }
    timed_wait_for_lcd_transfer();
    timed_wait_for_lcd_transfer();
}

// ---------------------------------------------------------------------------
// Touch
// ---------------------------------------------------------------------------
static esp_lcd_touch_handle_t initialize_touch(void)
{
    const esp_lcd_panel_io_spi_config_t touch_io_config =
        ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(PIN_NUM_TOUCH_CS);
    esp_lcd_panel_io_handle_t touch_io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &touch_io_config, &touch_io));

    const esp_lcd_touch_config_t touch_config = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_touch_handle_t touch = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_io, &touch_config, &touch));
    return touch;
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
// Dump the registers that decide the OV3660's pixel clock and frame length.
static void log_ov3660_timing(sensor_t *sensor, const char *label)
{
    if (sensor->id.PID != 0x3660 || sensor->get_reg == NULL) {
        return;
    }
    static const int regs[] = {0x3034, 0x3035, 0x3036, 0x3037, 0x3108, 0x3824,
                               0x380C, 0x380D, 0x380E, 0x380F, 0x3A00};
    char line[128];
    int n = 0;
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]) && n < (int)sizeof(line) - 10; i++) {
        n += snprintf(line + n, sizeof(line) - n, "%04X=%02X ", regs[i],
                      sensor->get_reg(sensor, regs[i], 0xff) & 0xff);
    }
    ESP_LOGI(TAG, "OV3660 timing %s: %s", label, line);
}

static void speed_up_ov3660(sensor_t *sensor)
{
    if (OV3660_SYS_DIV_TARGET == 0 || sensor->id.PID != 0x3660 ||
        sensor->get_reg == NULL || sensor->set_reg == NULL) {
        return;
    }

    const int before = sensor->get_reg(sensor, 0x3035, 0xff);   // SC_PLL_CTRL1
    if (before < 0) {
        ESP_LOGW(TAG, "Could not read OV3660 clock register");
        return;
    }
    const int current_div = (before >> 4) & 0x0f;
    ESP_LOGI(TAG, "OV3660 reg 0x3035=0x%02x (system clock divider %d)", before, current_div);

    if (current_div > OV3660_SYS_DIV_TARGET) {
        sensor->set_reg(sensor, 0x3035, 0xf0, OV3660_SYS_DIV_TARGET << 4);
        const int after = sensor->get_reg(sensor, 0x3035, 0xff);
        ESP_LOGI(TAG, "OV3660 reg 0x3035 now 0x%02x (system clock divider %d)",
                 after, (after >> 4) & 0x0f);
    }
}

static void initialize_camera(void)
{
    const camera_config_t camera_config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = PIN_NUM_CAMERA_XCLK,
        .pin_sccb_sda = PIN_NUM_CAMERA_SIOD,
        .pin_sccb_scl = PIN_NUM_CAMERA_SIOC,
        .pin_d7 = PIN_NUM_CAMERA_Y9,
        .pin_d6 = PIN_NUM_CAMERA_Y8,
        .pin_d5 = PIN_NUM_CAMERA_Y7,
        .pin_d4 = PIN_NUM_CAMERA_Y6,
        .pin_d3 = PIN_NUM_CAMERA_Y5,
        .pin_d2 = PIN_NUM_CAMERA_Y4,
        .pin_d1 = PIN_NUM_CAMERA_Y3,
        .pin_d0 = PIN_NUM_CAMERA_Y2,
        .pin_vsync = PIN_NUM_CAMERA_VSYNC,
        .pin_href = PIN_NUM_CAMERA_HREF,
        .pin_pclk = PIN_NUM_CAMERA_PCLK,
        .xclk_freq_hz = CAMERA_XCLK_HZ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 3,                     // camera keeps capturing while we process
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,   // always hand us the freshest frame
    };
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != NULL) {
        const int vertical_flip_result = sensor->set_vflip(sensor, 1);
        const int horizontal_mirror_result = sensor->set_hmirror(sensor, 1);
        ESP_LOGI(TAG, "Camera initialized, sensor PID=0x%04x", sensor->id.PID);
        if (vertical_flip_result != 0 || horizontal_mirror_result != 0) {
            ESP_LOGW(TAG, "Camera sensor did not accept both orientation settings");
        }

        log_ov3660_timing(sensor, "before");
        speed_up_ov3660(sensor);
        log_ov3660_timing(sensor, "after ");

        // Not every sensor driver implements every setter (an unset one is a
        // NULL pointer), so each call is guarded.
        if (sensor->set_whitebal) sensor->set_whitebal(sensor, 1);
        if (sensor->set_awb_gain) sensor->set_awb_gain(sensor, 1);
        if (sensor->set_exposure_ctrl) sensor->set_exposure_ctrl(sensor, 1);
        if (sensor->set_gain_ctrl) sensor->set_gain_ctrl(sensor, 1);
        if (sensor->set_aec2) sensor->set_aec2(sensor, 1);          // auto-exposure DSP
        if (sensor->set_ae_level) sensor->set_ae_level(sensor, 1);  // bias slightly bright (-2..2)
        if (sensor->set_lenc) sensor->set_lenc(sensor, 1);          // lens shading correction
    }
}

// ---------------------------------------------------------------------------
// Capture task and render task
//
// Capture only grabs frames and feeds the detector, so detection never waits
// for drawing. Rendering runs in its own task from a private copy of the frame.
// ---------------------------------------------------------------------------
#define FRAME_BYTES (CAMERA_FRAME_WIDTH * CAMERA_FRAME_HEIGHT * sizeof(uint16_t))

static esp_lcd_panel_handle_t g_panel;
static esp_lcd_touch_handle_t g_touch;
static void *g_detector;
static uint16_t *display_frame;       // PSRAM copy of the frame being drawn
static atomic_bool display_pending;
static TaskHandle_t render_task_handle;

static void capture_task(void *arg)
{
    (void)arg;

    while (true) {
        const int64_t wait_start_us = esp_timer_get_time();
        camera_fb_t *frame = esp_camera_fb_get();
        perf_cam_wait_us += (uint32_t)(esp_timer_get_time() - wait_start_us);

        if (frame == NULL) {
            ESP_LOGW(TAG, "Camera frame unavailable");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (frame->format == PIXFORMAT_RGB565 &&
            frame->width == CAMERA_FRAME_WIDTH && frame->height == CAMERA_FRAME_HEIGHT) {
            perf_cam_frames++;
            const uint16_t *source_pixels = (const uint16_t *)frame->buf;

            // Hand the detector a frame only when it is idle; never wait for it.
            if (g_detector != NULL && !atomic_load(&detect_busy)) {
                for (int row = 0; row < DETECT_SIZE; row++) {
                    memcpy(detect_frame + (size_t)row * DETECT_SIZE,
                           source_pixels + (size_t)row * CAMERA_FRAME_WIDTH + DETECT_CROP_X,
                           DETECT_SIZE * sizeof(uint16_t));
                }
                atomic_store(&detect_busy, true);
                xTaskNotifyGive(detect_task_handle);
            }

            // Same for the renderer: give it a frame only when it is idle.
            if (!atomic_load(&display_pending)) {
                memcpy(display_frame, source_pixels, FRAME_BYTES);
                atomic_store(&display_pending, true);
                xTaskNotifyGive(render_task_handle);
            }
        } else {
            ESP_LOGE(TAG, "Unexpected camera frame: %zux%zu format=%d",
                     frame->width, frame->height, frame->format);
        }

        esp_camera_fb_return(frame);
    }
}

static void render_task(void *arg)
{
    (void)arg;

    bool was_touched = false;
    uint32_t last_seq = 0;
    uint32_t last_infer_ms = 0;
    uint32_t frame_counter = 0;
    int64_t window_start_us = esp_timer_get_time();
    size_t last_logged_faces = SIZE_MAX;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t now_us = esp_timer_get_time();

        face_detection_box_t dets[MAX_DETECTED_FACES];
        size_t det_count = 0;
        if (fetch_detection(dets, &det_count, &last_infer_ms, &last_seq)) {
            for (size_t i = 0; i < det_count && i < MAX_DETECTED_FACES; i++) {
                dets[i].x1 += DETECT_CROP_X;   // crop space -> camera frame space
                dets[i].x2 += DETECT_CROP_X;
            }
            update_tracks(dets, det_count, now_us);
            if (det_count != last_logged_faces) {
                ESP_LOGI(TAG, "Faces detected: %zu", det_count);
                last_logged_faces = det_count;
            }
        }
        expire_tracks(now_us);

        update_hud(&hud);
        present_frame(g_panel, display_frame, &hud);
        atomic_store(&display_pending, false);
        perf_render_frames++;

        const int64_t elapsed_us = esp_timer_get_time() - window_start_us;
        if (elapsed_us >= 2000000) {
            const float secs = (float)elapsed_us / 1e6f;
            const uint32_t cam_n = perf_cam_frames ? perf_cam_frames : 1;
            const uint32_t draw_n = perf_render_frames ? perf_render_frames : 1;
            ESP_LOGI(TAG, "display=%.1f fps  camera=%.1f fps | cam_wait=%u ms  render=%u ms  spi_wait=%u ms (per frame) | detector=%u ms",
                     perf_render_frames / secs, perf_cam_frames / secs,
                     (unsigned)(perf_cam_wait_us / cam_n / 1000),
                     (unsigned)(perf_render_us / draw_n / 1000),
                     (unsigned)(perf_spi_wait_us / draw_n / 1000),
                     (unsigned)last_infer_ms);
            perf_cam_wait_us = perf_cam_frames = 0;
            perf_render_us = perf_spi_wait_us = perf_render_frames = 0;
            window_start_us = esp_timer_get_time();
        }

        // Touch shares the SPI bus with the LCD, so it is polled here, between frames.
        if (++frame_counter % TOUCH_POLL_EVERY_N_FRAMES == 0) {
            if (esp_lcd_touch_read_data(g_touch) == ESP_OK) {
                esp_lcd_touch_point_data_t touch_point = {0};
                uint8_t point_count = 0;
                if (esp_lcd_touch_get_data(g_touch, &touch_point, &point_count, 1) == ESP_OK) {
                    const bool is_touched = point_count > 0;
                    if (is_touched && !was_touched) {
                        ESP_LOGI(TAG, "Touch: x=%u y=%u strength=%u",
                                 touch_point.x, touch_point.y, touch_point.strength);
                    }
                    was_touched = is_touched;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
void app_main(void)
{
    build_palette();

    g_panel = initialize_lcd();
    clear_lcd(g_panel);
    g_touch = initialize_touch();
    initialize_camera();

    display_frame = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
    ESP_ERROR_CHECK(display_frame != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    atomic_init(&display_pending, false);

    g_detector = face_detection_create();
    if (g_detector == NULL) {
        ESP_LOGE(TAG, "Face detector initialization failed");
    } else {
        detect_frame = heap_caps_malloc(DETECT_SIZE * DETECT_SIZE * sizeof(uint16_t),
                                        MALLOC_CAP_SPIRAM);
        ESP_ERROR_CHECK(detect_frame != NULL ? ESP_OK : ESP_ERR_NO_MEM);
        atomic_init(&detect_busy, false);
        xTaskCreatePinnedToCore(detect_task, "face_detect", DETECT_TASK_STACK, g_detector,
                                DETECT_TASK_PRIORITY, &detect_task_handle, DETECT_TASK_CORE);
    }

    ESP_LOGI(TAG, "Live preview started; tap the display to log touch coordinates");

    // Render first so its handle exists before capture starts notifying it.
    xTaskCreatePinnedToCore(render_task, "render", RENDER_TASK_STACK, NULL,
                            RENDER_TASK_PRIORITY, &render_task_handle, RENDER_TASK_CORE);
    xTaskCreatePinnedToCore(capture_task, "capture", CAPTURE_TASK_STACK, NULL,
                            CAPTURE_TASK_PRIORITY, NULL, CAPTURE_TASK_CORE);
}