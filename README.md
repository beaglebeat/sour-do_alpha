# Sour-Do QSPI LCD (with PSRAM) and Touch Panel Example

Compiles and runs with VSCode using ESP-IDF extension for Waveshare 1.43" AIO. CMakeLists pull all necessary libraries and firmware.

### Example Output

```bash
...
I (415) example: Turn off LCD backlight
I (420) gpio: GPIO[0]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (429) example: Initialize SPI bus
I (434) example: Install panel IO
I (438) example: Install SPD2010 panel driver
I (442) gpio: GPIO[17]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (452) spd2010: LCD panel create success, version: 0.0.1
I (741) example: Turn on LCD backlight
I (741) example: Initialize LVGL library
I (741) example: Register display driver to LVGL
I (746) example: Install LVGL tick timer
I (748) example: Starting LVGL task
I (795) example: Display LVGL demos
I (1038) main_task: Returned from app_main()
...
```