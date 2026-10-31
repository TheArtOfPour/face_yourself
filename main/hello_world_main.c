#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
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
#define LCD_BAND_HEIGHT 40

#define PIN_NUM_LCD_SCLK 7
#define PIN_NUM_LCD_MISO 8
#define PIN_NUM_LCD_MOSI 9
#define PIN_NUM_LCD_CS 1
#define PIN_NUM_LCD_DC 2
#define PIN_NUM_TOUCH_CS 4

static const char *TAG = "lcd_test";
static uint16_t band_pixels[LCD_H_RES * LCD_BAND_HEIGHT];

void app_main(void)
{
    const spi_bus_config_t bus_config = {
        .sclk_io_num = PIN_NUM_LCD_SCLK,
        .mosi_io_num = PIN_NUM_LCD_MOSI,
        .miso_io_num = PIN_NUM_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(band_pixels),
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

    static const uint16_t colors[] = {
        0x0000, 0xFFFF, 0xF800, 0x07E0,
        0x001F, 0x07FF, 0xF81F, 0xFFE0,
    };
    for (size_t band = 0; band < sizeof(colors) / sizeof(colors[0]); band++) {
        for (size_t pixel = 0; pixel < sizeof(band_pixels) / sizeof(band_pixels[0]); pixel++) {
            band_pixels[pixel] = colors[band];
        }
        const int y_start = band * LCD_BAND_HEIGHT;
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y_start, LCD_H_RES,
                                                  y_start + LCD_BAND_HEIGHT, band_pixels));
    }

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
    ESP_LOGI(TAG, "LCD ready; tap the screen to print touch coordinates");

    bool was_touched = false;
    while (true) {
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
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}
