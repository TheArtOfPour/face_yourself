#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_camera.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LCD_HOST SPI2_HOST
#define LCD_H_RES 240
#define LCD_V_RES 320
#define LCD_CLEAR_BAND_HEIGHT 40
#define CAMERA_FRAME_WIDTH 240
#define CAMERA_FRAME_HEIGHT 176
#define CAMERA_PREVIEW_Y ((LCD_V_RES - CAMERA_FRAME_HEIGHT) / 2)

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

    const esp_lcd_panel_io_spi_config_t lcd_io_config =
        ILI9341_PANEL_IO_SPI_CONFIG(PIN_NUM_LCD_CS, PIN_NUM_LCD_DC, NULL, NULL);
    esp_lcd_panel_io_handle_t lcd_io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &lcd_io_config, &lcd_io));

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
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
        .frame_size = FRAMESIZE_HQVGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));

    const sensor_t *sensor = esp_camera_sensor_get();
    if (sensor != NULL) {
        ESP_LOGI(TAG, "Camera initialized, sensor PID=0x%04x", sensor->id.PID);
    }
}

void app_main(void)
{
    esp_lcd_panel_handle_t panel = initialize_lcd();
    clear_lcd(panel);
    esp_lcd_touch_handle_t touch = initialize_touch();
    initialize_camera();
    ESP_LOGI(TAG, "Live preview started; tap the display to log touch coordinates");

    bool was_touched = false;
    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            ESP_LOGW(TAG, "Camera frame unavailable");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (frame->format == PIXFORMAT_RGB565 &&
            frame->width == CAMERA_FRAME_WIDTH && frame->height == CAMERA_FRAME_HEIGHT) {
            ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
                panel, 0, CAMERA_PREVIEW_Y, LCD_H_RES,
                CAMERA_PREVIEW_Y + CAMERA_FRAME_HEIGHT, frame->buf));
        } else {
            ESP_LOGE(TAG, "Unexpected camera frame: %zux%zu format=%d",
                     frame->width, frame->height, frame->format);
        }
        esp_camera_fb_return(frame);

        ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch));
        uint16_t touch_x[1];
        uint16_t touch_y[1];
        uint16_t touch_strength[1];
        uint8_t point_count = 0;
        const bool is_touched = esp_lcd_touch_get_coordinates(
            touch, touch_x, touch_y, touch_strength, &point_count, 1);
        if (is_touched && !was_touched && point_count > 0) {
            ESP_LOGI(TAG, "Touch: x=%u y=%u strength=%u",
                     touch_x[0], touch_y[0], touch_strength[0]);
        }
        was_touched = is_touched;
    }
}
