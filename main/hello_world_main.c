#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_camera.h"
#include "face_detection.h"
#include "esp_err.h"
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
#define LCD_CLEAR_BAND_HEIGHT 40
#define CAMERA_FRAME_WIDTH 320
#define CAMERA_FRAME_HEIGHT 240
#define CAMERA_CROP_WIDTH 180
#define CAMERA_CROP_X ((CAMERA_FRAME_WIDTH - CAMERA_CROP_WIDTH) / 2)
#define MAX_DETECTED_FACES 4

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
static uint16_t clear_pixels[LCD_H_RES * LCD_CLEAR_BAND_HEIGHT];
static SemaphoreHandle_t lcd_transfer_done;

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} face_overlay_box_t;

static size_t map_face_boxes(const face_detection_box_t *face_boxes, size_t face_count,
                             face_overlay_box_t *overlay_boxes, size_t capacity)
{
    size_t overlay_count = 0;
    const int crop_right = CAMERA_CROP_X + CAMERA_CROP_WIDTH;
    for (size_t index = 0; index < face_count && overlay_count < capacity; index++) {
        const face_detection_box_t *face = &face_boxes[index];
        if (face->x2 < CAMERA_CROP_X || face->x1 >= crop_right ||
            face->y2 < 0 || face->y1 >= CAMERA_FRAME_HEIGHT) {
            continue;
        }

        face_overlay_box_t *overlay = &overlay_boxes[overlay_count];
        overlay->left = (face->x1 - CAMERA_CROP_X) * LCD_H_RES / CAMERA_CROP_WIDTH;
        overlay->right = (face->x2 - CAMERA_CROP_X) * LCD_H_RES / CAMERA_CROP_WIDTH;
        overlay->top = face->y1 * LCD_V_RES / CAMERA_FRAME_HEIGHT;
        overlay->bottom = face->y2 * LCD_V_RES / CAMERA_FRAME_HEIGHT;

        if (overlay->left < 0) overlay->left = 0;
        if (overlay->top < 0) overlay->top = 0;
        if (overlay->right >= LCD_H_RES) overlay->right = LCD_H_RES - 1;
        if (overlay->bottom >= LCD_V_RES) overlay->bottom = LCD_V_RES - 1;
        if (overlay->left <= overlay->right && overlay->top <= overlay->bottom) {
            overlay_count++;
        }
    }
    return overlay_count;
}

static bool is_face_edge_pixel(int x, int y, const face_overlay_box_t *boxes, size_t box_count)
{
    for (size_t index = 0; index < box_count; index++) {
        const face_overlay_box_t *box = &boxes[index];
        if (x >= box->left && x <= box->right && y >= box->top && y <= box->bottom &&
            (x - box->left < 2 || box->right - x < 2 ||
             y - box->top < 2 || box->bottom - y < 2)) {
            return true;
        }
    }
    return false;
}

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
    const spi_bus_config_t bus_config = {
        .sclk_io_num = PIN_NUM_LCD_SCLK,
        .mosi_io_num = PIN_NUM_LCD_MOSI,
        .miso_io_num = PIN_NUM_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(clear_pixels),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    lcd_transfer_done = xSemaphoreCreateBinary();
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
    for (size_t pixel = 0; pixel < sizeof(clear_pixels) / sizeof(clear_pixels[0]); pixel++) {
        clear_pixels[pixel] = 0x0000;
    }

    for (int y_start = 0; y_start < LCD_V_RES; y_start += LCD_CLEAR_BAND_HEIGHT) {
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y_start, LCD_H_RES,
                                                  y_start + LCD_CLEAR_BAND_HEIGHT,
                                                  clear_pixels));
                            wait_for_lcd_transfer();
    }
}

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
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
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
    }
}

void app_main(void)
{
    esp_lcd_panel_handle_t panel = initialize_lcd();
    clear_lcd(panel);
    esp_lcd_touch_handle_t touch = initialize_touch();
    initialize_camera();
    void *face_detector = face_detection_create();
    if (face_detector == NULL) {
        ESP_LOGE(TAG, "Face detector initialization failed");
    }
    ESP_LOGI(TAG, "Live preview started; tap the display to log touch coordinates");

    bool was_touched = false;
    size_t previous_face_count = SIZE_MAX;
    uint32_t frames_since_detector_status = 0;
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
            face_detection_box_t detected_faces[MAX_DETECTED_FACES];
            size_t face_count = 0;
            uint32_t inference_time_ms = 0;
            if (face_detector != NULL) {
                const int64_t inference_start_us = esp_timer_get_time();
                face_count = face_detection_run(face_detector, source_pixels,
                                                CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT,
                                                detected_faces, MAX_DETECTED_FACES);
                inference_time_ms = (uint32_t)((esp_timer_get_time() - inference_start_us) / 1000);
            }
            if (face_count != previous_face_count) {
                ESP_LOGI(TAG, "Faces detected: %zu", face_count);
                previous_face_count = face_count;
            }
            if (++frames_since_detector_status >= 20) {
                ESP_LOGI(TAG, "Detector active: faces=%zu inference=%ums",
                         face_count, (unsigned)inference_time_ms);
                frames_since_detector_status = 0;
            }
            face_overlay_box_t overlay_boxes[MAX_DETECTED_FACES];
            const size_t overlay_count = map_face_boxes(
                detected_faces, face_count, overlay_boxes, MAX_DETECTED_FACES);

            for (int band_y = 0; band_y < LCD_V_RES; band_y += LCD_CLEAR_BAND_HEIGHT) {
                for (int band_row = 0; band_row < LCD_CLEAR_BAND_HEIGHT; band_row++) {
                    const size_t destination_y = (size_t)(band_y + band_row);
                    const size_t source_y = destination_y * CAMERA_FRAME_HEIGHT / LCD_V_RES;
                    for (int destination_x = 0; destination_x < LCD_H_RES; destination_x++) {
                        const size_t source_x = CAMERA_CROP_X +
                            (size_t)destination_x * CAMERA_CROP_WIDTH / LCD_H_RES;
                        const size_t destination_index =
                            (size_t)band_row * LCD_H_RES + destination_x;
                        const size_t source_index = source_y * CAMERA_FRAME_WIDTH + source_x;
                        clear_pixels[destination_index] = is_face_edge_pixel(
                            destination_x, band_y + band_row, overlay_boxes, overlay_count)
                            ? 0x07E0 : source_pixels[source_index];
                    }
                }
                ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
                    panel, 0, band_y, LCD_H_RES, band_y + LCD_CLEAR_BAND_HEIGHT,
                    clear_pixels));
                wait_for_lcd_transfer();
            }
        } else {
            ESP_LOGE(TAG, "Unexpected camera frame: %zux%zu format=%d",
                     frame->width, frame->height, frame->format);
        }
        esp_camera_fb_return(frame);

        ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch));
        esp_lcd_touch_point_data_t touch_point = {0};
        uint8_t point_count = 0;
        ESP_ERROR_CHECK(esp_lcd_touch_get_data(touch, &touch_point, &point_count, 1));
        const bool is_touched = point_count > 0;
        if (is_touched && !was_touched) {
            ESP_LOGI(TAG, "Touch: x=%u y=%u strength=%u",
                     touch_point.x, touch_point.y, touch_point.strength);
        }
        was_touched = is_touched;
    }
}
