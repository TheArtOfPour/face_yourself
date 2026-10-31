idf_component_register(SRCS "hello_world_main.c" "face_detection.cpp"
                       PRIV_REQUIRES driver esp_lcd esp_lcd_ili9341 esp_lcd_touch_xpt2046 esp32-camera human_face_detect
                       INCLUDE_DIRS "")
