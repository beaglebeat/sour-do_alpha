idf_component_register(
    SRCS
        "example_qspi_with_ram.c"
        "tof_bin_image.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_lcd
        esp_timer
        esp_wifi
        esp_http_server
        esp_netif
        esp_event
        nvs_flash
        lvgl
        touch_bsp
        read_lcd_id_bsp
)

set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE
)