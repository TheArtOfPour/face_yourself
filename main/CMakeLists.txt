idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES driver esp_lcd esp_lcd_ili9341 esp_lcd_touch_xpt2046 esp32-camera
                       INCLUDE_DIRS "")
