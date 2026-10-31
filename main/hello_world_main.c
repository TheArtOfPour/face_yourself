#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
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
#define CAMERA_CROP_WIDTH 180
#define CAMERA_CROP_X ((CAMERA_FRAME_WIDTH - CAMERA_CROP_WIDTH) / 2)

#define MAX_DETECTED_FACES 4

// The model input is square (224x224). Feeding the full 320x240 frame squashes
// faces horizontally, so we hand it a centered 240x240 crop instead. This
// contains the 180px-wide display crop with margin to spare.
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

// Camera RGB565 bytes are already in "wire order" for the LCD, so a color we
// draw ourselves must be byte-swapped. 0x07E0 (green) -> 0xE007.
#define BOX_COLOR 0xE007

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
// LCD state
// ---------------------------------------------------------------------------
static uint16_t *band_buf[2];            // internal, DMA-capable, double buffered
static SemaphoreHandle_t lcd_transfer_done;  // COUNTING semaphore (2 transfers can be in flight)
static uint16_t x_map[LCD_H_RES];        // dest column -> source column
static uint16_t y_map[LCD_V_RES];        // dest row    -> source row

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} face_overlay_box_t;

// ---------------------------------------------------------------------------
// Detection thread state
// ---------------------------------------------------------------------------
static uint16_t *detect_frame;           // PSRAM copy of the frame being analysed
static atomic_bool detect_busy;
static TaskHandle_t detect_task_handle;
static portMUX_TYPE result_mux = portMUX_INITIALIZER_UNLOCKED;
static face_detection_box_t result_boxes[MAX_DETECTED_FACES];
static size_t result_count;
static uint32_t result_seq;
static uint32_t result_infer_ms;

typedef struct {
    float x1, y1, x2, y2;
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

static size_t map_tracks(face_overlay_box_t *overlay_boxes, size_t capacity)
{
    size_t overlay_count = 0;
    const int crop_right = CAMERA_CROP_X + CAMERA_CROP_WIDTH;

    for (int t = 0; t < MAX_DETECTED_FACES && overlay_count < capacity; t++) {
        if (!tracks[t].active) continue;

        const int x1 = (int)tracks[t].x1, y1 = (int)tracks[t].y1;
        const int x2 = (int)tracks[t].x2, y2 = (int)tracks[t].y2;
        if (x2 < CAMERA_CROP_X || x1 >= crop_right || y2 < 0 || y1 >= CAMERA_FRAME_HEIGHT) {
            continue;
        }

        face_overlay_box_t *o = &overlay_boxes[overlay_count];
        o->left = (x1 - CAMERA_CROP_X) * LCD_H_RES / CAMERA_CROP_WIDTH;
        o->right = (x2 - CAMERA_CROP_X) * LCD_H_RES / CAMERA_CROP_WIDTH;
        o->top = y1 * LCD_V_RES / CAMERA_FRAME_HEIGHT;
        o->bottom = y2 * LCD_V_RES / CAMERA_FRAME_HEIGHT;

        if (o->left < 0) o->left = 0;
        if (o->top < 0) o->top = 0;
        if (o->right >= LCD_H_RES) o->right = LCD_H_RES - 1;
        if (o->bottom >= LCD_V_RES) o->bottom = LCD_V_RES - 1;

        if (o->left <= o->right && o->top <= o->bottom) {
            overlay_count++;
        }
    }
    return overlay_count;
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

static void build_scale_maps(void)
{
    for (int x = 0; x < LCD_H_RES; x++) {
        x_map[x] = (uint16_t)(CAMERA_CROP_X + x * CAMERA_CROP_WIDTH / LCD_H_RES);
    }
    for (int y = 0; y < LCD_V_RES; y++) {
        y_map[y] = (uint16_t)(y * CAMERA_FRAME_HEIGHT / LCD_V_RES);
    }
}

// Scale one band from the camera frame into dst, then draw box outlines on top.
static void render_band(uint16_t *dst, int band_y, const uint16_t *src,
                        const face_overlay_box_t *boxes, size_t box_count)
{
    for (int row = 0; row < LCD_BAND_HEIGHT; row++) {
        const uint16_t *src_row = src + (size_t)y_map[band_y + row] * CAMERA_FRAME_WIDTH;
        uint16_t *dst_row = dst + (size_t)row * LCD_H_RES;
        for (int x = 0; x < LCD_H_RES; x++) {
            dst_row[x] = src_row[x_map[x]];
        }
    }

    for (size_t b = 0; b < box_count; b++) {
        const face_overlay_box_t *box = &boxes[b];
        for (int row = 0; row < LCD_BAND_HEIGHT; row++) {
            const int y = band_y + row;
            if (y < box->top || y > box->bottom) continue;

            uint16_t *dst_row = dst + (size_t)row * LCD_H_RES;
            if (y - box->top < 2 || box->bottom - y < 2) {
                for (int x = box->left; x <= box->right; x++) dst_row[x] = BOX_COLOR;
            } else {
                dst_row[box->left] = BOX_COLOR;
                if (box->left + 1 <= box->right) dst_row[box->left + 1] = BOX_COLOR;
                dst_row[box->right] = BOX_COLOR;
                if (box->right - 1 >= box->left) dst_row[box->right - 1] = BOX_COLOR;
            }
        }
    }
}

// Fill one band while the previous one is still being sent over SPI.
static void present_frame(esp_lcd_panel_handle_t panel, const uint16_t *src,
                          const face_overlay_box_t *boxes, size_t box_count)
{
    for (int band = 0; band < LCD_BAND_COUNT; band++) {
        if (band >= 2) {
            wait_for_lcd_transfer();  // frees band_buf[band & 1]
        }
        uint16_t *buf = band_buf[band & 1];
        const int y = band * LCD_BAND_HEIGHT;
        render_band(buf, y, src, boxes, box_count);
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_H_RES,
                                                  y + LCD_BAND_HEIGHT, buf));
    }
    wait_for_lcd_transfer();
    wait_for_lcd_transfer();
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
        .xclk_freq_hz = 20000000,
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

        // Image quality knobs that help the detector in uneven light.
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
// Main
// ---------------------------------------------------------------------------
void app_main(void)
{
    build_scale_maps();

    esp_lcd_panel_handle_t panel = initialize_lcd();
    clear_lcd(panel);
    esp_lcd_touch_handle_t touch = initialize_touch();
    initialize_camera();

    void *face_detector = face_detection_create();
    if (face_detector == NULL) {
        ESP_LOGE(TAG, "Face detector initialization failed");
    } else {
        detect_frame = heap_caps_malloc(DETECT_SIZE * DETECT_SIZE * sizeof(uint16_t),
                                        MALLOC_CAP_SPIRAM);
        ESP_ERROR_CHECK(detect_frame != NULL ? ESP_OK : ESP_ERR_NO_MEM);
        atomic_init(&detect_busy, false);
        xTaskCreatePinnedToCore(detect_task, "face_detect", DETECT_TASK_STACK, face_detector,
                                DETECT_TASK_PRIORITY, &detect_task_handle, DETECT_TASK_CORE);
    }

    ESP_LOGI(TAG, "Live preview started; tap the display to log touch coordinates");

    bool was_touched = false;
    uint32_t last_seq = 0;
    uint32_t last_infer_ms = 0;
    uint32_t frame_counter = 0;
    uint32_t frames_in_window = 0;
    int64_t window_start_us = esp_timer_get_time();
    size_t last_logged_faces = SIZE_MAX;

    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            ESP_LOGW(TAG, "Camera frame unavailable");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (frame->format == PIXFORMAT_RGB565 &&
            frame->width == CAMERA_FRAME_WIDTH && frame->height == CAMERA_FRAME_HEIGHT) {
            const uint16_t *source_pixels = (const uint16_t *)frame->buf;
            const int64_t now_us = esp_timer_get_time();

            // Hand the detector a frame only when it is idle; never wait for it.
            if (face_detector != NULL && !atomic_load(&detect_busy)) {
                for (int row = 0; row < DETECT_SIZE; row++) {
                    memcpy(detect_frame + (size_t)row * DETECT_SIZE,
                           source_pixels + (size_t)row * CAMERA_FRAME_WIDTH + DETECT_CROP_X,
                           DETECT_SIZE * sizeof(uint16_t));
                }
                atomic_store(&detect_busy, true);
                xTaskNotifyGive(detect_task_handle);
            }

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

            face_overlay_box_t overlay_boxes[MAX_DETECTED_FACES];
            const size_t overlay_count = map_tracks(overlay_boxes, MAX_DETECTED_FACES);

            present_frame(panel, source_pixels, overlay_boxes, overlay_count);

            frames_in_window++;
            const int64_t elapsed_us = esp_timer_get_time() - window_start_us;
            if (elapsed_us >= 2000000) {
                ESP_LOGI(TAG, "display=%.1f fps  detector inference=%u ms",
                         frames_in_window * 1e6f / (float)elapsed_us, (unsigned)last_infer_ms);
                frames_in_window = 0;
                window_start_us = esp_timer_get_time();
            }
        } else {
            ESP_LOGE(TAG, "Unexpected camera frame: %zux%zu format=%d",
                     frame->width, frame->height, frame->format);
        }

        esp_camera_fb_return(frame);

        // Touch shares the SPI bus with the LCD, so poll it sparingly.
        if (++frame_counter % TOUCH_POLL_EVERY_N_FRAMES == 0) {
            if (esp_lcd_touch_read_data(touch) == ESP_OK) {
                esp_lcd_touch_point_data_t touch_point = {0};
                uint8_t point_count = 0;
                if (esp_lcd_touch_get_data(touch, &touch_point, &point_count, 1) == ESP_OK) {
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