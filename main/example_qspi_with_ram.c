#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "tof_bin_image.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"

#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lvgl.h"
#include "esp_lcd_sh8601.h"
#include "touch_bsp.h"
#include "read_lcd_id_bsp.h"

static const char *TAG = "example";

/* ---------------- Board/display config ---------------- */

#define LCD_HOST SPI2_HOST

#define EXAMPLE_Rotate_90
#define SH8601_ID 0x86
#define CO5300_ID 0xff

static uint8_t READ_LCD_ID = 0x00;

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#else
#define LCD_BIT_PER_PIXEL (16)
#endif

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL

#define EXAMPLE_PIN_NUM_LCD_CS         (GPIO_NUM_9)
#define EXAMPLE_PIN_NUM_LCD_PCLK       (GPIO_NUM_10)
#define EXAMPLE_PIN_NUM_LCD_DATA0      (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_LCD_DATA1      (GPIO_NUM_12)
#define EXAMPLE_PIN_NUM_LCD_DATA2      (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_LCD_DATA3      (GPIO_NUM_14)
#define EXAMPLE_PIN_NUM_LCD_RST        (GPIO_NUM_21)
#define EXAMPLE_PIN_NUM_BK_LIGHT       (-1)

#define EXAMPLE_LCD_H_RES              466
#define EXAMPLE_LCD_V_RES              466

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 8)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (6 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

/* SHT41: Adafruit SHT41 default I2C address */
#define SHT41_I2C_PORT                 I2C_NUM_0
#define SHT41_I2C_ADDR                 0x44
#define SHT41_CMD_HIGH_PRECISION       0xFD

/* VEML7700: default I2C address, daisy-chained on same bus as SHT41 */
#define VEML7700_ENABLE                1
#define VEML7700_I2C_PORT              SHT41_I2C_PORT
#define VEML7700_I2C_ADDR              0x10
#define VEML7700_REG_ALS_CONF          0x00
#define VEML7700_REG_ALS               0x04
#define VEML7700_CONF_GAIN_1_IT_100MS  0x0000
#define VEML7700_LUX_PER_COUNT         0.0576f

/* TMF8821: SparkFun Qwiic dToF Imager, daisy-chained on same bus.
 * This lightweight code talks to the TMF8821 measurement application if it is
 * already running. If APPID reads 0x80, the sensor is in bootloader mode and
 * needs the full ams/SparkFun firmware-download driver before measurements.
 */
#define TMF8821_ENABLE                 1
#define TMF8821_I2C_PORT               SHT41_I2C_PORT
#define TMF8821_I2C_ADDR               0x41
#define TMF8821_REG_APPID              0x00
#define TMF8821_REG_CMD_STAT           0x08
#define TMF8821_REG_RESULT_BASE        0x20
#define TMF8821_REG_INT_STATUS         0xE1
#define TMF8821_REG_INT_ENABLE         0xE2
#define TMF8821_REG_ENABLE             0xE0
#define TMF8821_APPID_MEAS             0x03
#define TMF8821_APPID_BOOTLOADER       0x80
#define TMF8821_CMD_MEASURE            0x10
#define TMF8821_INT_RESULT             0x02
#define TMF8821_RESULT_BLOCK_LEN       132
#define TMF8821_MAX_ZONES              18
#define TMF8821_MIN_CONFIDENCE          20
#define TMF8821_MAX_VALID_DISTANCE_MM   5000
#define TMF8821_MAX_STEP_MM             600

#define TMF8821_BL_CMD_DOWNLOAD_INIT 0x14
#define TMF8821_BL_CMD_SET_ADDR      0x43
#define TMF8821_BL_CMD_WRITE_RAM     0x41
#define TMF8821_BL_CMD_RAM_REMAP     0x11
#define TMF8821_BL_DOWNLOAD_INIT_DATA 0x29
#define TMF8821_BL_CHUNK_SIZE        128

/* ---------------- Power / battery config ---------------- */
#define TOUCH_WAKE_GPIO                GPIO_NUM_NC
#define DIM_TIMEOUT_MS                 (5  * 1000)
#define DISPLAY_OFF_TIMEOUT_MS         (20 * 1000)
#define LIGHT_SLEEP_TIMEOUT_MS         (25 * 1000)
#define TOUCH_POLL_INTERVAL_US         (2  * 1000 * 1000)
#define LCD_BRIGHTNESS_FULL            0xFF
#define LCD_BRIGHTNESS_DIM             0x10

#define BATTERY_ADC_ENABLE             1
#define BATTERY_ADC_CHANNEL            ADC1_CHANNEL_3
#define BATTERY_ADC_ATTEN              ADC_ATTEN_DB_12
#define BATTERY_ADC_GPIO               GPIO_NUM_4
#define BATTERY_DIVIDER_RATIO          3.0f
#define BATTERY_EMPTY_MV               3300
#define BATTERY_FULL_MV                4200


/* MiCS-5524 gas sensor analog input.
 * On ESP32-S3, ADC1_CHANNEL_4 is GPIO5. Despite the common ESP32 mapping,
 * GPIO32 is not the correct ADC1_CH4 pin on ESP32-S3. Keep AO wired through
 * the 10k/20k divider to GPIO5.
 */
#define GAS_ADC_ENABLE                1
#define GAS_ADC_CHANNEL               ADC1_CHANNEL_4
#define GAS_ADC_GPIO                  GPIO_NUM_5
#define GAS_ADC_ATTEN                 ADC_ATTEN_DB_12
#define GAS_ADC_SAMPLES               16
#define GAS_DIVIDER_RATIO             1.5f   /* 10k top, 20k bottom: AO = ADC * 1.5 */
#define GAS_MAX_SENSOR_MV             5000

/* ---------------- Wi-Fi SoftAP / web monitor config ---------------- */
#define WEB_AP_SSID                    "SourDo Alpha"
#define WEB_AP_PASSWORD                "coolProfessors"
#define WEB_AP_CHANNEL                 1
#define WEB_AP_MAX_CONNECTIONS         4




static SemaphoreHandle_t lvgl_mux = NULL;
static esp_lcd_panel_io_handle_t g_panel_io_handle = NULL;
static esp_lcd_panel_handle_t g_panel_handle = NULL;
static volatile int64_t g_last_touch_time_us = 0;
static bool g_display_dimmed = false;
static bool g_display_off = false;
static bool g_web_server_running = true;
static lv_obj_t *battery_label = NULL;
static lv_obj_t *dim_overlay = NULL;

/* Forward declarations */
static bool example_lvgl_lock(int timeout_ms);
static void example_lvgl_unlock(void);


/* ---------------- LCD init tables ---------------- */

static const sh8601_lcd_init_cmd_t sh8601_lcd_init_cmds[] =
{
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0x44, (uint8_t []){0x01, 0xD1}, 2, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x53, (uint8_t []){0x20}, 1, 10},
    {0x51, (uint8_t []){0x00}, 1, 10},
    {0x29, (uint8_t []){0x00}, 0, 10},
    {0x51, (uint8_t []){0xFF}, 1, 0},
};

static const sh8601_lcd_init_cmd_t co5300_lcd_init_cmds[] =
{
    {0x11, (uint8_t []){0x00}, 0, 80},
    {0xC4, (uint8_t []){0x80}, 1, 0},
    {0x53, (uint8_t []){0x20}, 1, 1},
    {0x63, (uint8_t []){0xFF}, 1, 1},
    {0x51, (uint8_t []){0x00}, 1, 1},
    {0x29, (uint8_t []){0x00}, 0, 10},
    {0x51, (uint8_t []){0xFF}, 1, 0},
};

/* ---------------- SHT41 ---------------- */

static bool sht41_read(float *temp_c, float *humidity)
{
    uint8_t cmd = SHT41_CMD_HIGH_PRECISION;
    uint8_t data[6] = {0};

    esp_err_t err = i2c_master_write_to_device(
        SHT41_I2C_PORT,
        SHT41_I2C_ADDR,
        &cmd,
        1,
        pdMS_TO_TICKS(150)
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT41 command failed: %s", esp_err_to_name(err));
        return false;
    }

    /* High precision conversion time is typically under 10 ms. */
    vTaskDelay(pdMS_TO_TICKS(12));

    err = i2c_master_read_from_device(
        SHT41_I2C_PORT,
        SHT41_I2C_ADDR,
        data,
        6,
        pdMS_TO_TICKS(150)
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT41 read failed: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t raw_temp = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_hum  = ((uint16_t)data[3] << 8) | data[4];

    *temp_c = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    *humidity = -6.0f + 125.0f * ((float)raw_hum / 65535.0f);

    if (*humidity < 0.0f) {
        *humidity = 0.0f;
    } else if (*humidity > 100.0f) {
        *humidity = 100.0f;
    }

    return true;
}

/* ---------------- VEML7700 ---------------- */

static esp_err_t veml7700_write_reg16(uint8_t reg, uint16_t value)
{
#if !VEML7700_ENABLE
    LV_UNUSED(reg);
    LV_UNUSED(value);
    return ESP_ERR_NOT_SUPPORTED;
#else
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF)
    };

    return i2c_master_write_to_device(
        VEML7700_I2C_PORT,
        VEML7700_I2C_ADDR,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(150)
    );
#endif
}

static esp_err_t veml7700_read_reg16(uint8_t reg, uint16_t *value)
{
#if !VEML7700_ENABLE
    LV_UNUSED(reg);
    LV_UNUSED(value);
    return ESP_ERR_NOT_SUPPORTED;
#else
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_write_read_device(
        VEML7700_I2C_PORT,
        VEML7700_I2C_ADDR,
        &reg,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(150)
    );

    if (err != ESP_OK) {
        return err;
    }

    *value = ((uint16_t)data[1] << 8) | data[0];
    return ESP_OK;
#endif
}

static bool veml7700_init(void)
{
#if !VEML7700_ENABLE
    ESP_LOGI(TAG, "VEML7700 disabled");
    return false;
#else
    esp_err_t err = veml7700_write_reg16(VEML7700_REG_ALS_CONF,
                                         VEML7700_CONF_GAIN_1_IT_100MS);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "VEML7700 init failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "VEML7700 initialized at address 0x%02X", VEML7700_I2C_ADDR);
    return true;
#endif
}

static bool veml7700_read_lux(float *lux)
{
#if !VEML7700_ENABLE
    LV_UNUSED(lux);
    return false;
#else
    uint16_t raw_als = 0;
    esp_err_t err = veml7700_read_reg16(VEML7700_REG_ALS, &raw_als);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "VEML7700 read failed: %s", esp_err_to_name(err));
        return false;
    }

    *lux = (float)raw_als * VEML7700_LUX_PER_COUNT;
    return true;
#endif
}


/* ---------------- TMF8821 ---------------- */

typedef struct {
    bool present;
    bool app_running;
    uint8_t appid;
    uint16_t last_distance_mm;
    uint8_t last_confidence;
} tmf8821_state_t;

static tmf8821_state_t tmf8821_state = {0};

static esp_err_t tmf8821_write_reg8(uint8_t reg, uint8_t value)
{
#if !TMF8821_ENABLE
    LV_UNUSED(reg);
    LV_UNUSED(value);
    return ESP_ERR_NOT_SUPPORTED;
#else
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(TMF8821_I2C_PORT, TMF8821_I2C_ADDR,
                                      buf, sizeof(buf), pdMS_TO_TICKS(150));
#endif
}

static esp_err_t tmf8821_read_reg8(uint8_t reg, uint8_t *value)
{
#if !TMF8821_ENABLE
    LV_UNUSED(reg);
    LV_UNUSED(value);
    return ESP_ERR_NOT_SUPPORTED;
#else
    return i2c_master_write_read_device(TMF8821_I2C_PORT, TMF8821_I2C_ADDR,
                                        &reg, 1, value, 1, pdMS_TO_TICKS(150));
#endif
}

static esp_err_t tmf8821_read_block(uint8_t reg, uint8_t *buf, size_t len)
{
#if !TMF8821_ENABLE
    LV_UNUSED(reg);
    LV_UNUSED(buf);
    LV_UNUSED(len);
    return ESP_ERR_NOT_SUPPORTED;
#else
    return i2c_master_write_read_device(TMF8821_I2C_PORT, TMF8821_I2C_ADDR,
                                        &reg, 1, buf, len, pdMS_TO_TICKS(250));
#endif
}

static bool tmf8821_wait_ready(uint32_t timeout_ms)
{
#if !TMF8821_ENABLE
    LV_UNUSED(timeout_ms);
    return false;
#else
    int tries = (int)(timeout_ms / 10U);
    if (tries < 1) {
        tries = 1;
    }

    for (int i = 0; i < tries; i++) {
        uint8_t en = 0;
        if (tmf8821_read_reg8(TMF8821_REG_ENABLE, &en) != ESP_OK) {
            return false;
        }

        if (en & 0x40) {
            return true;
        }

        if ((en & 0x0F) == 0x02) {
            uint8_t wake = (en & 0x30) | 0x01;
            tmf8821_write_reg8(TMF8821_REG_ENABLE, wake);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return false;
#endif
}

static uint8_t tmf8821_bl_checksum(uint8_t cmd, uint8_t size, const uint8_t *data)
{
    uint16_t sum = (uint16_t)cmd + (uint16_t)size;
    for (uint8_t i = 0; i < size; i++) {
        sum += data[i];
    }
    return (uint8_t)(~(sum & 0xFFU));
}

static esp_err_t tmf8821_bl_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t size)
{
#if !TMF8821_ENABLE
    LV_UNUSED(cmd);
    LV_UNUSED(data);
    LV_UNUSED(size);
    return ESP_ERR_NOT_SUPPORTED;
#else
    /* Bootloader frames are written starting at CMD_STAT / register 0x08:
     * [0x08] [cmd] [size] [data0..dataN] [checksum]
     * checksum = one's-complement low byte of cmd + size + sum(data).
     */
    uint8_t buf[1 + 1 + 1 + TMF8821_BL_CHUNK_SIZE + 1];
    if (size > TMF8821_BL_CHUNK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    buf[0] = TMF8821_REG_CMD_STAT;
    buf[1] = cmd;
    buf[2] = size;
    if (size > 0 && data != NULL) {
        memcpy(&buf[3], data, size);
    }
    buf[3 + size] = tmf8821_bl_checksum(cmd, size, data);

    return i2c_master_write_to_device(TMF8821_I2C_PORT,
                                      TMF8821_I2C_ADDR,
                                      buf,
                                      (size_t)(4 + size),
                                      pdMS_TO_TICKS(250));
#endif
}

static esp_err_t tmf8821_wait_bl_ready(uint32_t timeout_ms)
{
#if !TMF8821_ENABLE
    LV_UNUSED(timeout_ms);
    return ESP_ERR_NOT_SUPPORTED;
#else
    int tries = (int)(timeout_ms / 5U);
    if (tries < 1) {
        tries = 1;
    }

    for (int i = 0; i < tries; i++) {
        uint8_t reg = TMF8821_REG_CMD_STAT;
        uint8_t status[3] = {0};
        esp_err_t err = i2c_master_write_read_device(TMF8821_I2C_PORT,
                                                     TMF8821_I2C_ADDR,
                                                     &reg,
                                                     1,
                                                     status,
                                                     sizeof(status),
                                                     pdMS_TO_TICKS(150));
        if (err != ESP_OK) {
            return err;
        }

        /* The bootloader-ready response is 0x00 0x00 0xFF when reading
         * three bytes from CMD_STAT. Non-zero CMD_STAT values are errors.
         */
        if (status[0] == 0x00 && status[2] == 0xFF) {
            return ESP_OK;
        }

        if (status[0] != 0x00 && status[0] < 0x10) {
            ESP_LOGW(TAG, "TMF8821 bootloader status/error: 0x%02X", status[0]);
            return ESP_FAIL;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGW(TAG, "TMF8821 bootloader ready timeout");
    return ESP_ERR_TIMEOUT;
#endif
}

static esp_err_t tmf8821_load_firmware(void)
{
#if !TMF8821_ENABLE
    return ESP_ERR_NOT_SUPPORTED;
#else
    const uint8_t *fw     = (const uint8_t *)tof_bin_image;
    const size_t   fw_len = (size_t)tof_bin_image_length;

    if (fw == NULL || fw_len == 0) {
        ESP_LOGW(TAG, "TMF8821 firmware image is empty");
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "TMF8821 loading firmware (%u bytes)...", (unsigned)fw_len);

    /* 1. Tell bootloader to accept a firmware image. */
    uint8_t init_data = TMF8821_BL_DOWNLOAD_INIT_DATA;
    esp_err_t err = tmf8821_bl_send_cmd(TMF8821_BL_CMD_DOWNLOAD_INIT, &init_data, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 DOWNLOAD_INIT failed: %s", esp_err_to_name(err));
        return err;
    }
    err = tmf8821_wait_bl_ready(500);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 not ready after DOWNLOAD_INIT: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. The SparkFun/ams app image used here is treated as one continuous
     * RAM block starting at address 0x0000. If your tof_bin_image is a custom
     * multi-block container, use the vendor parser instead of this raw loader.
     */
    uint8_t addr_data[2] = {0x00, 0x00};
    err = tmf8821_bl_send_cmd(TMF8821_BL_CMD_SET_ADDR, addr_data, sizeof(addr_data));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 SET_ADDR failed: %s", esp_err_to_name(err));
        return err;
    }
    err = tmf8821_wait_bl_ready(500);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 not ready after SET_ADDR: %s", esp_err_to_name(err));
        return err;
    }

    /* 3. Write firmware bytes in bootloader W_RAM frames. */
    size_t offset = 0;
    while (offset < fw_len) {
        size_t chunk = fw_len - offset;
        if (chunk > TMF8821_BL_CHUNK_SIZE) {
            chunk = TMF8821_BL_CHUNK_SIZE;
        }

        err = tmf8821_bl_send_cmd(TMF8821_BL_CMD_WRITE_RAM,
                                  &fw[offset],
                                  (uint8_t)chunk);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TMF8821 W_RAM failed at offset %u: %s",
                     (unsigned)offset, esp_err_to_name(err));
            return err;
        }

        err = tmf8821_wait_bl_ready(500);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TMF8821 not ready after W_RAM offset %u: %s",
                     (unsigned)offset, esp_err_to_name(err));
            return err;
        }

        offset += chunk;
    }

    /* 4. Jump from bootloader RAM to measurement application. */
    err = tmf8821_bl_send_cmd(TMF8821_BL_CMD_RAM_REMAP, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 RAM_REMAP failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < 50; i++) {
        uint8_t appid = 0;
        err = tmf8821_read_reg8(TMF8821_REG_APPID, &appid);
        if (err == ESP_OK && appid == TMF8821_APPID_MEAS) {
            ESP_LOGI(TAG, "TMF8821 firmware loaded OK, APPID=0x%02X", appid);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint8_t appid = 0;
    tmf8821_read_reg8(TMF8821_REG_APPID, &appid);
    ESP_LOGW(TAG, "TMF8821 app did not start after FW load, APPID=0x%02X", appid);
    return ESP_FAIL;
#endif
}

static bool tmf8821_start_measurements(void)
{
#if !TMF8821_ENABLE
    return false;
#else
    if (!tmf8821_state.app_running) {
        return false;
    }

    tmf8821_write_reg8(TMF8821_REG_INT_ENABLE, TMF8821_INT_RESULT);
    tmf8821_write_reg8(TMF8821_REG_INT_STATUS, 0xFF);

    esp_err_t err = tmf8821_write_reg8(TMF8821_REG_CMD_STAT, TMF8821_CMD_MEASURE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 measure command failed: %s", esp_err_to_name(err));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t stat = 0;
    if (tmf8821_read_reg8(TMF8821_REG_CMD_STAT, &stat) == ESP_OK) {
        ESP_LOGI(TAG, "TMF8821 CMD_STAT after MEASURE: 0x%02X", stat);
    }

    return true;
#endif
}

static bool tmf8821_init(void)
{
#if !TMF8821_ENABLE
    ESP_LOGI(TAG, "TMF8821 disabled");
    return false;
#else
    tmf8821_state.present = false;
    tmf8821_state.app_running = false;
    tmf8821_state.appid = 0;

    tmf8821_write_reg8(TMF8821_REG_ENABLE, 0x01);

    if (!tmf8821_wait_ready(250)) {
        ESP_LOGW(TAG, "TMF8821 not ready or not found at 0x%02X", TMF8821_I2C_ADDR);
        return false;
    }

    uint8_t appid = 0;
    esp_err_t err = tmf8821_read_reg8(TMF8821_REG_APPID, &appid);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 APPID read failed: %s", esp_err_to_name(err));
        return false;
    }

    tmf8821_state.present = true;
    tmf8821_state.appid = appid;
    tmf8821_state.app_running = (appid == TMF8821_APPID_MEAS);

    ESP_LOGI(TAG, "TMF8821 detected, APPID=0x%02X", appid);

if (appid == TMF8821_APPID_BOOTLOADER) {
        ESP_LOGI(TAG, "TMF8821 in bootloader mode, downloading firmware...");
        esp_err_t fw_err = tmf8821_load_firmware();
        if (fw_err != ESP_OK) {
            ESP_LOGW(TAG, "TMF8821 firmware load failed: %s", esp_err_to_name(fw_err));
            return true; /* present but not running — UI will show APPID error */
        }
        tmf8821_state.appid = TMF8821_APPID_MEAS;
        tmf8821_state.app_running = true;
        /* fall through to tmf8821_start_measurements() below */
    }

    if (!tmf8821_state.app_running) {
        ESP_LOGW(TAG, "TMF8821 unexpected APPID=0x%02X", appid);
        return true;
    }

    return tmf8821_start_measurements();
#endif
}

static bool tmf8821_read_distance(uint16_t *distance_mm, uint8_t *confidence)
{
#if !TMF8821_ENABLE
    LV_UNUSED(distance_mm);
    LV_UNUSED(confidence);
    return false;
#else
    if (!tmf8821_state.app_running) {
        return false;
    }

    uint8_t int_status = 0;
    esp_err_t err = tmf8821_read_reg8(TMF8821_REG_INT_STATUS, &int_status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 INT_STATUS read failed: %s", esp_err_to_name(err));
        return false;
    }

    if ((int_status & TMF8821_INT_RESULT) == 0) {
        return false;
    }

    /* Clear only the interrupt bits that were actually set, then read one full
     * 132-byte result frame starting at 0x20. The ams app note warns that
     * result data must be read as one block so the bytes all belong to the
     * same frame.
     */
    tmf8821_write_reg8(TMF8821_REG_INT_STATUS, int_status);

    uint8_t block[TMF8821_RESULT_BLOCK_LEN] = {0};
    err = tmf8821_read_block(TMF8821_REG_RESULT_BASE, block, sizeof(block));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TMF8821 result block read failed: %s", esp_err_to_name(err));
        return false;
    }

    /* Registers 0x20..0x23 are the result header. The first object results
     * begin at 0x24, i.e. block[4]. Each measurement is:
     *   distance LSB, distance MSB, confidence.
     *
     * The old code picked the zone with the highest confidence. That can make
     * a far wall/background win over the actual nearby target, which looks like
     * a steadily increasing/drifting distance. For a simple distance display,
     * pick the nearest valid primary-object result instead. Ignore the second
     * object set and reject low-confidence / out-of-range values.
     */
    uint16_t nearest_dist = 0;
    uint8_t nearest_conf = 0;

    for (int zone = 0; zone < TMF8821_MAX_ZONES; zone++) {
        int idx = 4 + zone * 3;
        if (idx + 2 >= TMF8821_RESULT_BLOCK_LEN) {
            break;
        }

        uint16_t d = ((uint16_t)block[idx + 1] << 8) | block[idx];
        uint8_t c = block[idx + 2];

        if (d == 0 || d > TMF8821_MAX_VALID_DISTANCE_MM || c < TMF8821_MIN_CONFIDENCE) {
            continue;
        }

        if (nearest_dist == 0 || d < nearest_dist) {
            nearest_dist = d;
            nearest_conf = c;
        }
    }

    if (nearest_dist == 0) {
        return false;
    }

    /* Optional sanity filter: reject a single-frame huge upward jump when we
     * already have a good previous reading. This prevents one bad background
     * frame from walking the graph upward.
     */
    if (tmf8821_state.last_distance_mm > 0 &&
        nearest_dist > tmf8821_state.last_distance_mm + TMF8821_MAX_STEP_MM) {
        ESP_LOGD(TAG, "TMF8821 rejected jump: prev=%u new=%u conf=%u",
                 tmf8821_state.last_distance_mm, nearest_dist, nearest_conf);
        return false;
    }

    tmf8821_state.last_distance_mm = nearest_dist;
    tmf8821_state.last_confidence = nearest_conf;

    *distance_mm = nearest_dist;
    *confidence = nearest_conf;
    return true;
#endif
}


/* ---------------- MiCS-5524 Gas Sensor ---------------- */

static bool gas_adc_initialized = false;

static void gas_adc_init(void)
{
#if GAS_ADC_ENABLE
    if (gas_adc_initialized) {
        return;
    }

    /* Shared with battery ADC setup. Width is global for ADC1; attenuation is per-channel. */
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(GAS_ADC_CHANNEL, GAS_ADC_ATTEN));
    gas_adc_initialized = true;
    ESP_LOGI(TAG, "MiCS-5524 ADC initialized on GPIO%d / ADC1_CH4", GAS_ADC_GPIO);
#endif
}

static bool gas_read_mics(float *adc_voltage_v,
                          float *sensor_ao_voltage_v,
                          int *raw_out)
{
#if !GAS_ADC_ENABLE
    LV_UNUSED(adc_voltage_v);
    LV_UNUSED(sensor_ao_voltage_v);
    LV_UNUSED(raw_out);
    return false;
#else
    if (adc_voltage_v == NULL || sensor_ao_voltage_v == NULL || raw_out == NULL) {
        return false;
    }

    if (!gas_adc_initialized) {
        gas_adc_init();
    }

    uint32_t raw_sum = 0;
    for (int i = 0; i < GAS_ADC_SAMPLES; i++) {
        int raw = adc1_get_raw(GAS_ADC_CHANNEL);
        if (raw < 0) {
            return false;
        }
        raw_sum += (uint32_t)raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    const float raw_avg = (float)raw_sum / (float)GAS_ADC_SAMPLES;
    const float adc_v = (raw_avg / 4095.0f) * 3.3f;
    const float sensor_v = adc_v * GAS_DIVIDER_RATIO;

    *raw_out = (int)(raw_avg + 0.5f);
    *adc_voltage_v = adc_v;
    *sensor_ao_voltage_v = sensor_v;
    return true;
#endif
}


/* ---------------- Deep sleep / battery helpers ---------------- */

static void set_lcd_brightness(uint8_t brightness)
{
    if (g_panel_io_handle == NULL) {
        return;
    }

    /*
     * This board is AMOLED, so EXAMPLE_PIN_NUM_BK_LIGHT is -1 and there is
     * no separate PWM/GPIO backlight to dim. Brightness must be controlled
     * with MIPI DCS commands through the panel driver.
     *
     * 0x53 = Write Control Display. Bit 5 enables brightness control. Some
     * SH8601/CO5300 firmwares ignore 0x51 changes unless this is refreshed.
     * 0x51 = Write Display Brightness. Your init table already uses one byte,
     * so keep the same width here.
     */
    uint8_t ctrl = 0x20;
    esp_err_t err = esp_lcd_panel_io_tx_param(g_panel_io_handle, 0x53, &ctrl, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD brightness control enable failed: %s", esp_err_to_name(err));
    }

    err = esp_lcd_panel_io_tx_param(g_panel_io_handle, 0x51, &brightness, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD brightness write 0x%02X failed: %s", brightness, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LCD brightness set to 0x%02X", brightness);
    }
}

static void set_visual_dim(bool dim)
{
    lv_obj_t *scr = lv_scr_act();
    if (scr == NULL) {
        return;
    }

    if (dim) {
        /*
         * Some SH8601/CO5300 AMOLED boards acknowledge the DCS brightness
         * commands but do not visibly change panel brightness. This overlay
         * gives reliable user-visible dimming through LVGL instead.
         */
        if (dim_overlay == NULL || lv_obj_get_parent(dim_overlay) != scr) {
            dim_overlay = lv_obj_create(scr);
            lv_obj_remove_style_all(dim_overlay);
            lv_obj_set_size(dim_overlay, LV_PCT(100), LV_PCT(100));
            lv_obj_set_pos(dim_overlay, 0, 0);
            lv_obj_set_style_bg_color(dim_overlay, lv_color_black(), 0);
            lv_obj_set_style_bg_opa(dim_overlay, LV_OPA_70, 0);
            lv_obj_clear_flag(dim_overlay, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(dim_overlay, LV_OBJ_FLAG_SCROLLABLE);
        }

        lv_obj_move_foreground(dim_overlay);
        lv_obj_clear_flag(dim_overlay, LV_OBJ_FLAG_HIDDEN);
    } else {
        if (dim_overlay != NULL) {
            lv_obj_add_flag(dim_overlay, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void display_on(void)
{
    if (g_display_off) {
        if (g_panel_handle != NULL) {
            esp_lcd_panel_disp_on_off(g_panel_handle, true);
        }
        g_display_off = false;
    }
    set_visual_dim(false);
    g_display_dimmed = false;
    set_lcd_brightness(LCD_BRIGHTNESS_FULL);
}

static void power_note_user_activity(void)
{
    g_last_touch_time_us = esp_timer_get_time();
    display_on();
}

static void enter_light_sleep(void)
{
    ESP_LOGI(TAG, "Entering light sleep");
    esp_sleep_enable_timer_wakeup(TOUCH_POLL_INTERVAL_US);
    esp_light_sleep_start();

    /* Resumes here after wakeup */
    uint16_t tx = 0, ty = 0;
    if (getTouch(&tx, &ty)) {
        ESP_LOGI(TAG, "Touch detected, waking display");
        power_note_user_activity();
    }
}

static void power_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (g_last_touch_time_us == 0) {
        power_note_user_activity();
        return;
    }

    const int64_t idle_ms = (esp_timer_get_time() - g_last_touch_time_us) / 1000LL;

    if (!g_display_dimmed && idle_ms >= DIM_TIMEOUT_MS) {
        /* Try hardware/panel dimming first, then guarantee visible dimming with LVGL. */
        set_lcd_brightness(LCD_BRIGHTNESS_DIM);
        set_visual_dim(true);
        g_display_dimmed = true;
        ESP_LOGI(TAG, "Display visually dimmed");
    }

    if (!g_display_off && idle_ms >= DISPLAY_OFF_TIMEOUT_MS) {
        set_visual_dim(false);
        set_lcd_brightness(0x00);
        if (g_panel_handle != NULL) {
            esp_lcd_panel_disp_on_off(g_panel_handle, false);
        }
        g_display_off = true;
        ESP_LOGI(TAG, "Display off");
    }

    if (g_display_off && idle_ms >= LIGHT_SLEEP_TIMEOUT_MS) {
        if (!g_web_server_running) {
            example_lvgl_unlock();
            enter_light_sleep();
            example_lvgl_lock(-1);
        }
    }
}

static uint32_t battery_read_mv(void)
{
#if !BATTERY_ADC_ENABLE
    return 0;
#else
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN));

    uint32_t raw_sum = 0;
    for (int i = 0; i < 16; i++) {
        int raw = adc1_get_raw(BATTERY_ADC_CHANNEL);
        if (raw < 0) return 0;
        raw_sum += (uint32_t)raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    const float adc_mv = ((float)raw_sum / 16.0f / 4095.0f) * 3300.0f;
    return (uint32_t)(adc_mv * BATTERY_DIVIDER_RATIO + 0.5f);
#endif
}

static int battery_mv_to_percent(uint32_t mv)
{
    if (mv <= BATTERY_EMPTY_MV) return 0;
    if (mv >= BATTERY_FULL_MV)  return 100;
    return (int)(((mv - BATTERY_EMPTY_MV) * 100U) / (BATTERY_FULL_MV - BATTERY_EMPTY_MV));
}

static void battery_update_label(void)
{
    if (battery_label == NULL) return;
#if !BATTERY_ADC_ENABLE
    lv_label_set_text(battery_label, "BAT ---.--V --%");
#else
    uint32_t mv = battery_read_mv();
    if (mv == 0) {
        lv_label_set_text(battery_label, "BAT ---.--V --%");
        return;
    }

    int pct = battery_mv_to_percent(mv);

    /* Small text-only battery indicator.
     * Example: BAT 4.08V 86%
     * This is an estimate from the Li-ion voltage curve, so it is best used
     * as a rough state-of-charge indicator rather than a precise fuel gauge.
     */
    char buf[40];
    snprintf(buf, sizeof(buf), "BAT %lu.%02luV %d%%",
             (unsigned long)(mv / 1000U),
             (unsigned long)((mv % 1000U) / 10U),
             pct);
    lv_label_set_text(battery_label, buf);
#endif
}

static void battery_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    battery_update_label();
}

/* ---------------- LVGL display/touch glue ---------------- */

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                            esp_lcd_panel_io_event_data_t *edata,
                                            void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv,
                                  const lv_area_t *area,
                                  lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    const int offsetx1 = (READ_LCD_ID == SH8601_ID) ? area->x1 : area->x1 + 0x06;
    const int offsetx2 = (READ_LCD_ID == SH8601_ID) ? area->x2 : area->x2 + 0x06;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if LCD_BIT_PER_PIXEL == 24
    uint8_t *to = (uint8_t *)color_map;
    uint8_t temp = 0;
    uint16_t pixel_num = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);

    temp = color_map[0].ch.blue;
    *to++ = color_map[0].ch.red;
    *to++ = color_map[0].ch.green;
    *to++ = temp;

    for (int i = 1; i < pixel_num; i++) {
        *to++ = color_map[i].ch.red;
        *to++ = color_map[i].ch.green;
        *to++ = color_map[i].ch.blue;
    }
#endif

    esp_lcd_panel_draw_bitmap(panel_handle,
                              offsetx1,
                              offsety1,
                              offsetx2 + 1,
                              offsety2 + 1,
                              color_map);
}

static void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;
    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    LV_UNUSED(drv);

    /*
     * FT3168/FT6x style touch controllers can occasionally report a single
     * noisy sample. LVGL polls this callback frequently, so require a touch to
     * be stable for a few consecutive polls before treating it as real.
     */
    enum { TOUCH_STABLE_SAMPLES = 3, TOUCH_RELEASE_SAMPLES = 3, TOUCH_MOVE_TOL_PX = 12 };

    static bool was_touched = false;
    static uint8_t press_count = 0;
    static uint8_t release_count = 0;
    static uint16_t cand_x = 0;
    static uint16_t cand_y = 0;
    static uint16_t last_x = 0;
    static uint16_t last_y = 0;

    uint16_t tp_x = 0;
    uint16_t tp_y = 0;
    uint8_t touched = getTouch(&tp_x, &tp_y);

    if (touched) {
        release_count = 0;

        int dx = (int)tp_x - (int)cand_x;
        int dy = (int)tp_y - (int)cand_y;
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;

        if (press_count == 0 || dx > TOUCH_MOVE_TOL_PX || dy > TOUCH_MOVE_TOL_PX) {
            cand_x = tp_x;
            cand_y = tp_y;
            press_count = 1;
        } else if (press_count < TOUCH_STABLE_SAMPLES) {
            press_count++;
        }

        if (press_count >= TOUCH_STABLE_SAMPLES) {
            last_x = tp_x;
            last_y = tp_y;

            if (!was_touched) {
                power_note_user_activity();
                ESP_LOGI(TAG, "Touch stable: x=%u y=%u", tp_x, tp_y);
            }

            was_touched = true;
            data->point.x = last_x;
            data->point.y = last_y;
            data->state = LV_INDEV_STATE_PRESSED;
            return;
        }

        /* Candidate touch has not been stable long enough. Keep LVGL released. */
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    press_count = 0;

    if (was_touched) {
        if (release_count < TOUCH_RELEASE_SAMPLES) {
            release_count++;
            data->point.x = last_x;
            data->point.y = last_y;
            data->state = LV_INDEV_STATE_PRESSED;
            return;
        }
    }

    was_touched = false;
    release_count = 0;
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_RELEASED;
}

static void example_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "LVGL mutex must be created first");

    const TickType_t timeout_ticks =
        (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "LVGL mutex must be created first");
    xSemaphoreGive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");

    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;

    while (1) {
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            example_lvgl_unlock();
        }

        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


/* ---------------- UI ---------------- */

typedef enum {
    ICON_TEMP,
    ICON_LIGHT,
    ICON_DISTANCE,
    ICON_GAS,
    ICON_RECIPE
} sensor_icon_t;

static lv_obj_t *screen_main = NULL;
static lv_obj_t *screen_temp = NULL;
static lv_obj_t *screen_light = NULL;
static lv_obj_t *screen_distance = NULL;
static lv_obj_t *screen_gas = NULL;
static lv_obj_t *screen_recipes = NULL;

static lv_style_t style_screen;
static lv_style_t style_card;
static lv_style_t style_back_btn;
static lv_style_t style_title;
static lv_style_t style_body;

static lv_obj_t *temp_chart = NULL;
static lv_obj_t *temp_value_label = NULL;
static lv_obj_t *hum_value_label = NULL;
static lv_chart_series_t *temp_series = NULL;
static lv_chart_series_t *hum_series = NULL;

static lv_obj_t *light_chart = NULL;
static lv_obj_t *light_value_label = NULL;
static lv_chart_series_t *light_series = NULL;

static lv_obj_t *distance_chart = NULL;
static lv_obj_t *distance_value_label = NULL;
static lv_obj_t *distance_status_label = NULL;
static lv_chart_series_t *distance_series = NULL;


static lv_obj_t *gas_chart = NULL;
static lv_obj_t *gas_value_label = NULL;
static lv_obj_t *gas_status_label = NULL;
static lv_chart_series_t *gas_series = NULL;

static lv_obj_t *recipe_value_label = NULL;
static lv_obj_t *recipe_status_label = NULL;
static float g_last_temp_c = 0.0f;
static float g_last_humidity = 0.0f;
static bool g_sht41_has_reading = false;

static float g_last_light_lux = 0.0f;
static bool g_light_has_reading = false;
static uint16_t g_last_distance_mm_web = 0;
static uint8_t g_last_distance_confidence_web = 0;
static bool g_distance_has_reading = false;
static float g_last_gas_adc_v = 0.0f;
static float g_last_gas_sensor_v = 0.0f;
static int g_last_gas_raw = 0;
static bool g_gas_has_reading = false;

/* ---------------- SoftAP web monitor ---------------- */

static const char index_html[] =
"<!doctype html><html><head>"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>Sensor Monitor</title>"
"<style>"
"body{margin:0;font-family:Arial,sans-serif;background:#10151d;color:#eef3fb;}"
"header{padding:16px;text-align:center;}h1{font-size:22px;margin:4px 0;}"
"p{margin:4px 0;color:#b8c5d6;font-size:14px;}"
".grid{display:grid;grid-template-columns:1fr;gap:12px;padding:0 12px 16px;}"
".card{background:#203047;border-radius:18px;padding:12px;box-shadow:0 6px 20px rgba(0,0,0,.25);}"
".top{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:8px;}"
".name{font-weight:700}.value{color:#8bc6ff;font-variant-numeric:tabular-nums;}"
"canvas{width:100%;height:150px;background:#121b28;border-radius:12px;}"
".hint{text-align:center;font-size:12px;color:#91a0b4;padding-bottom:14px;}"
"</style></head><body>"
"<header><h1>Starter Station</h1><p>ESP32-S3 live sensor graphs</p></header>"
"<div class=\"grid\">"
"<div class=\"card\"><div class=\"top\"><span class=\"name\">Temp / Humidity</span><span id=\"thv\" class=\"value\">--</span></div><canvas id=\"th\"></canvas></div>"
"<div class=\"card\"><div class=\"top\"><span class=\"name\">Light</span><span id=\"lv\" class=\"value\">--</span></div><canvas id=\"light\"></canvas></div>"
"<div class=\"card\"><div class=\"top\"><span class=\"name\">Distance</span><span id=\"dv\" class=\"value\">--</span></div><canvas id=\"dist\"></canvas></div>"
"<div class=\"card\"><div class=\"top\"><span class=\"name\">Gas AO</span><span id=\"gv\" class=\"value\">--</span></div><canvas id=\"gas\"></canvas></div>"
"</div><div class=\"hint\">Connect to Wi-Fi SensorMonitor, open http://192.168.4.1</div>"
"<script>"
"const N=60;const S={t:[],h:[],l:[],d:[],g:[]};"
"function push(a,v){if(v===null||isNaN(v))return;a.push(v);while(a.length>N)a.shift();}"
"function line(ctx,arr,min,max,w,h){if(arr.length<2)return;ctx.beginPath();arr.forEach((v,i)=>{let x=i*(w/(N-1));let y=h-((v-min)/(max-min))*h;y=Math.max(0,Math.min(h,y));if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();}"
"function draw(id,series,min,max){let c=document.getElementById(id),r=c.getBoundingClientRect(),dpr=window.devicePixelRatio||1;c.width=r.width*dpr;c.height=r.height*dpr;let ctx=c.getContext('2d');ctx.scale(dpr,dpr);let w=r.width,h=r.height;ctx.clearRect(0,0,w,h);ctx.strokeStyle='#2f435f';ctx.lineWidth=1;for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(0,h*i/4);ctx.lineTo(w,h*i/4);ctx.stroke();}ctx.lineWidth=2.5;series.forEach((obj)=>{ctx.strokeStyle=obj.color;line(ctx,obj.data,min,max,w,h);});}"
"async function tick(){try{let r=await fetch('/data');let j=await r.json();push(S.t,j.temp_c);push(S.h,j.humidity);push(S.l,j.light_lux);push(S.d,j.distance_mm);push(S.g,j.gas_ao_v);document.getElementById('thv').textContent=(j.temp_ok?j.temp_c.toFixed(1)+' C':'--')+' / '+(j.humidity_ok?j.humidity.toFixed(1)+' %':'--');document.getElementById('lv').textContent=j.light_ok?j.light_lux.toFixed(0)+' lux':'--';document.getElementById('dv').textContent=j.distance_ok?j.distance_mm+' mm':'--';document.getElementById('gv').textContent=j.gas_ok?j.gas_ao_v.toFixed(2)+' V':'--';draw('th',[{data:S.t,color:'#ff6b6b'},{data:S.h,color:'#4d96ff'}],0,100);draw('light',[{data:S.l,color:'#ffd166'}],0,1000);draw('dist',[{data:S.d,color:'#8bc6ff'}],0,5000);draw('gas',[{data:S.g,color:'#9dffb0'}],0,5);}catch(e){console.log(e);}}"
"setInterval(tick,1000);tick();"
"</script></body></html>";

static esp_err_t web_index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t web_data_handler(httpd_req_t *req)
{
    char json[512];
    snprintf(json, sizeof(json),
             "{"
             "\"temp_ok\":%s,\"temp_c\":%.2f,"
             "\"humidity_ok\":%s,\"humidity\":%.2f,"
             "\"light_ok\":%s,\"light_lux\":%.2f,"
             "\"distance_ok\":%s,\"distance_mm\":%u,\"distance_conf\":%u,"
             "\"gas_ok\":%s,\"gas_adc_v\":%.3f,\"gas_ao_v\":%.3f,\"gas_raw\":%d"
             "}",
             g_sht41_has_reading ? "true" : "false", g_last_temp_c,
             g_sht41_has_reading ? "true" : "false", g_last_humidity,
             g_light_has_reading ? "true" : "false", g_last_light_lux,
             g_distance_has_reading ? "true" : "false", g_last_distance_mm_web, g_last_distance_confidence_web,
             g_gas_has_reading ? "true" : "false", g_last_gas_adc_v, g_last_gas_sensor_v, g_last_gas_raw);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HTTP server start failed: %s", esp_err_to_name(err));
        return;
    }

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = web_index_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t data_uri = {
        .uri = "/data",
        .method = HTTP_GET,
        .handler = web_data_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &data_uri);

    ESP_LOGI(TAG, "HTTP server started: http://192.168.4.1");
}

static void softap_web_monitor_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.ap.ssid, WEB_AP_SSID, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(WEB_AP_SSID);
    strncpy((char *)wifi_config.ap.password, WEB_AP_PASSWORD, sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = WEB_AP_CHANNEL;
    wifi_config.ap.max_connection = WEB_AP_MAX_CONNECTIONS;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    if (strlen(WEB_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP started. SSID=%s password=%s URL=http://192.168.4.1",
             WEB_AP_SSID, WEB_AP_PASSWORD);

    web_server_start();
}


static void ui_go_home(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load_anim(screen_main, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 180, 0, false);
}

static void ui_open_temp(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load_anim(screen_temp, LV_SCR_LOAD_ANIM_MOVE_LEFT, 180, 0, false);
}

static void ui_open_light(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load_anim(screen_light, LV_SCR_LOAD_ANIM_MOVE_LEFT, 180, 0, false);
}

static void ui_open_distance(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load_anim(screen_distance, LV_SCR_LOAD_ANIM_MOVE_LEFT, 180, 0, false);
}

static void ui_open_gas(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load_anim(screen_gas, LV_SCR_LOAD_ANIM_MOVE_LEFT, 180, 0, false);
}

static void ui_open_recipes(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load_anim(screen_recipes, LV_SCR_LOAD_ANIM_MOVE_LEFT, 180, 0, false);
}

static void ui_init_styles(void)
{
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, lv_color_hex(0x10151D));
    lv_style_set_bg_opa(&style_screen, LV_OPA_COVER);
    lv_style_set_border_width(&style_screen, 0);
    lv_style_set_radius(&style_screen, 0);

    lv_style_init(&style_card);
    lv_style_set_radius(&style_card, LV_RADIUS_CIRCLE);
    lv_style_set_bg_color(&style_card, lv_color_hex(0x2B3A4F));
    lv_style_set_bg_opa(&style_card, LV_OPA_90);
    lv_style_set_border_width(&style_card, 0);
    lv_style_set_shadow_width(&style_card, 16);
    lv_style_set_shadow_opa(&style_card, LV_OPA_30);
    lv_style_set_shadow_spread(&style_card, 2);
    lv_style_set_shadow_ofs_y(&style_card, 5);
    lv_style_set_pad_all(&style_card, 6);

    lv_style_init(&style_back_btn);
    lv_style_set_radius(&style_back_btn, 18);
    lv_style_set_bg_color(&style_back_btn, lv_color_hex(0x40526D));
    lv_style_set_bg_opa(&style_back_btn, LV_OPA_COVER);
    lv_style_set_border_width(&style_back_btn, 0);

    lv_style_init(&style_title);
    lv_style_set_text_color(&style_title, lv_color_white());

    lv_style_init(&style_body);
    lv_style_set_text_color(&style_body, lv_color_hex(0xD5DCE6));
}

static lv_obj_t *ui_create_graphic_icon(lv_obj_t *parent, sensor_icon_t type)
{
    lv_color_t accent = lv_color_hex(0x8BC6FF);

    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, 72, 72);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    if (type == ICON_TEMP) {
        lv_obj_t *tube = lv_obj_create(box);
        lv_obj_remove_style_all(tube);
        lv_obj_set_size(tube, 18, 48);
        lv_obj_align(tube, LV_ALIGN_CENTER, 0, -6);
        lv_obj_set_style_radius(tube, 9, 0);
        lv_obj_set_style_bg_color(tube, accent, 0);
        lv_obj_set_style_bg_opa(tube, LV_OPA_COVER, 0);

        lv_obj_t *bulb = lv_obj_create(box);
        lv_obj_remove_style_all(bulb);
        lv_obj_set_size(bulb, 34, 34);
        lv_obj_align(bulb, LV_ALIGN_BOTTOM_MID, 0, -2);
        lv_obj_set_style_radius(bulb, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(bulb, accent, 0);
        lv_obj_set_style_bg_opa(bulb, LV_OPA_COVER, 0);
    } else if (type == ICON_LIGHT) {
        lv_obj_t *sun = lv_obj_create(box);
        lv_obj_remove_style_all(sun);
        lv_obj_set_size(sun, 34, 34);
        lv_obj_center(sun);
        lv_obj_set_style_radius(sun, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(sun, accent, 0);
        lv_obj_set_style_bg_opa(sun, LV_OPA_COVER, 0);

        static lv_point_t rays[][2] = {
            {{36, 0}, {36, 12}}, {{36, 60}, {36, 72}},
            {{0, 36}, {12, 36}}, {{60, 36}, {72, 36}},
            {{10, 10}, {18, 18}}, {{54, 54}, {62, 62}},
            {{62, 10}, {54, 18}}, {{18, 54}, {10, 62}},
        };

        for (int i = 0; i < 8; i++) {
            lv_obj_t *line = lv_line_create(box);
            lv_line_set_points(line, rays[i], 2);
            lv_obj_set_style_line_color(line, accent, 0);
            lv_obj_set_style_line_width(line, 4, 0);
            lv_obj_set_style_line_rounded(line, true, 0);
        }
    } else if (type == ICON_DISTANCE) {
        lv_obj_t *left = lv_obj_create(box);
        lv_obj_remove_style_all(left);
        lv_obj_set_size(left, 8, 48);
        lv_obj_align(left, LV_ALIGN_LEFT_MID, 8, 0);
        lv_obj_set_style_radius(left, 4, 0);
        lv_obj_set_style_bg_color(left, accent, 0);
        lv_obj_set_style_bg_opa(left, LV_OPA_COVER, 0);

        lv_obj_t *right = lv_obj_create(box);
        lv_obj_remove_style_all(right);
        lv_obj_set_size(right, 8, 48);
        lv_obj_align(right, LV_ALIGN_RIGHT_MID, -8, 0);
        lv_obj_set_style_radius(right, 4, 0);
        lv_obj_set_style_bg_color(right, accent, 0);
        lv_obj_set_style_bg_opa(right, LV_OPA_COVER, 0);

        static lv_point_t dist_line[] = {{20, 36}, {52, 36}};
        lv_obj_t *line = lv_line_create(box);
        lv_line_set_points(line, dist_line, 2);
        lv_obj_set_style_line_color(line, accent, 0);
        lv_obj_set_style_line_width(line, 5, 0);
        lv_obj_set_style_line_rounded(line, true, 0);

        lv_obj_t *dot = lv_obj_create(box);
        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, 10, 10);
        lv_obj_center(dot);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, accent, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    } else if (type == ICON_RECIPE) {
        lv_obj_t *book = lv_obj_create(box);
        lv_obj_remove_style_all(book);
        lv_obj_set_size(book, 46, 42);
        lv_obj_align(book, LV_ALIGN_CENTER, 0, 2);
        lv_obj_set_style_radius(book, 10, 0);
        lv_obj_set_style_bg_color(book, accent, 0);
        lv_obj_set_style_bg_opa(book, LV_OPA_COVER, 0);

        lv_obj_t *page = lv_obj_create(book);
        lv_obj_remove_style_all(page);
        lv_obj_set_size(page, 20, 34);
        lv_obj_align(page, LV_ALIGN_LEFT_MID, 4, 0);
        lv_obj_set_style_radius(page, 7, 0);
        lv_obj_set_style_bg_color(page, lv_color_hex(0x10151D), 0);
        lv_obj_set_style_bg_opa(page, LV_OPA_50, 0);

        static lv_point_t steam1[] = {{24, 8}, {20, 0}, {25, -8}};
        static lv_point_t steam2[] = {{36, 8}, {32, 0}, {37, -8}};
        lv_obj_t *line1 = lv_line_create(box);
        lv_line_set_points(line1, steam1, 3);
        lv_obj_set_style_line_color(line1, accent, 0);
        lv_obj_set_style_line_width(line1, 4, 0);
        lv_obj_set_style_line_rounded(line1, true, 0);

        lv_obj_t *line2 = lv_line_create(box);
        lv_line_set_points(line2, steam2, 3);
        lv_obj_set_style_line_color(line2, accent, 0);
        lv_obj_set_style_line_width(line2, 4, 0);
        lv_obj_set_style_line_rounded(line2, true, 0);
    } else {
        const lv_coord_t sizes[] = {34, 28, 22, 12};
        const lv_coord_t xs[]    = {5, 28, 42, 12};
        const lv_coord_t ys[]    = {28, 18, 34, 12};

        for (int i = 0; i < 4; i++) {
            lv_obj_t *bubble = lv_obj_create(box);
            lv_obj_remove_style_all(bubble);
            lv_obj_set_size(bubble, sizes[i], sizes[i]);
            lv_obj_set_pos(bubble, xs[i], ys[i]);
            lv_obj_set_style_radius(bubble, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_bg_color(bubble, accent, 0);
            lv_obj_set_style_bg_opa(bubble, LV_OPA_COVER, 0);
        }
    }

    return box;
}

static lv_obj_t *ui_create_back_button(lv_obj_t *parent)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_add_style(btn, &style_back_btn, 0);
    lv_obj_set_size(btn, 130, 44);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -28);
    lv_obj_add_event_cb(btn, ui_go_home, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Back");
    lv_obj_center(label);

    return btn;
}

static lv_obj_t *ui_create_basic_subscreen(const char *title_txt,
                                           const char *body_txt,
                                           sensor_icon_t icon_type)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_add_style(scr, &style_screen, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_add_style(title, &style_title, 0);
    lv_label_set_text(title, title_txt);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t *icon = ui_create_graphic_icon(scr, icon_type);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -35);

    lv_obj_t *body = lv_label_create(scr);
    lv_obj_add_style(body, &style_body, 0);
    lv_obj_set_width(body, 280);
    lv_obj_set_style_text_align(body, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
    lv_label_set_text(body, body_txt);
    lv_obj_align(body, LV_ALIGN_CENTER, 0, 45);

    ui_create_back_button(scr);
    return scr;
}

static void temp_chart_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    float temp_c = 0.0f;
    float hum = 0.0f;

    if (!sht41_read(&temp_c, &hum)) {
        ESP_LOGW(TAG, "SHT41 read failed in timer");
        lv_label_set_text(temp_value_label, "Temp: --.- C");
        lv_label_set_text(hum_value_label, "Humidity: --.- %");
        return;
    }

    g_last_temp_c = temp_c;
    g_last_humidity = hum;
    g_sht41_has_reading = true;

    ESP_LOGI(TAG, "SHT41: %.1f C, %.1f %%", temp_c, hum);

    char temp_buf[32];
    char hum_buf[32];

    snprintf(temp_buf, sizeof(temp_buf), "Temp: %.1f C", temp_c);
    snprintf(hum_buf, sizeof(hum_buf), "Humidity: %.1f %%", hum);

    lv_label_set_text(temp_value_label, temp_buf);
    lv_label_set_text(hum_value_label, hum_buf);

    lv_chart_set_next_value(temp_chart, temp_series, (lv_coord_t)temp_c);
    lv_chart_set_next_value(temp_chart, hum_series, (lv_coord_t)hum);
    lv_chart_refresh(temp_chart);
}

static lv_obj_t *ui_create_temp_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_add_style(scr, &style_screen, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_add_style(title, &style_title, 0);
    lv_label_set_text(title, "Temp / Humidity");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    temp_value_label = lv_label_create(scr);
    lv_obj_add_style(temp_value_label, &style_body, 0);
    lv_label_set_text(temp_value_label, "Temp: --.- C");
    lv_obj_align(temp_value_label, LV_ALIGN_TOP_MID, 0, 48);

    hum_value_label = lv_label_create(scr);
    lv_obj_add_style(hum_value_label, &style_body, 0);
    lv_label_set_text(hum_value_label, "Humidity: --.- %");
    lv_obj_align(hum_value_label, LV_ALIGN_TOP_MID, 0, 72);

    temp_chart = lv_chart_create(scr);
    lv_obj_set_size(temp_chart, 360, 210);
    lv_obj_align(temp_chart, LV_ALIGN_CENTER, 0, 20);
    lv_chart_set_type(temp_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(temp_chart, 24);
    lv_chart_set_range(temp_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);
    lv_chart_set_range(temp_chart, LV_CHART_AXIS_SECONDARY_Y, 0, 100);
    lv_chart_set_div_line_count(temp_chart, 6, 6);

    temp_series = lv_chart_add_series(temp_chart, lv_color_hex(0xFF6B6B), LV_CHART_AXIS_PRIMARY_Y);
    hum_series = lv_chart_add_series(temp_chart, lv_color_hex(0x4D96FF), LV_CHART_AXIS_SECONDARY_Y);

    lv_chart_set_all_value(temp_chart, temp_series, LV_CHART_POINT_NONE);
    lv_chart_set_all_value(temp_chart, hum_series, LV_CHART_POINT_NONE);

    lv_obj_t *legend = lv_label_create(scr);
    lv_obj_add_style(legend, &style_body, 0);
    lv_label_set_text(legend, "Red: C   Blue: RH%");
    lv_obj_align(legend, LV_ALIGN_BOTTOM_MID, 0, -82);

    ui_create_back_button(scr);

    /*
     * SHT41 graphing enabled.
     * This keeps the same I2C setup as the working touch version.
     */
    lv_timer_create(temp_chart_timer_cb, 5000, NULL);
    temp_chart_timer_cb(NULL);

    return scr;
}

static void light_chart_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    float lux = 0.0f;

    if (!veml7700_read_lux(&lux)) {
        ESP_LOGW(TAG, "VEML7700 read failed in timer");
        lv_label_set_text(light_value_label, "Light: --.- lux");
        return;
    }

    g_last_light_lux = lux;
    g_light_has_reading = true;

    ESP_LOGI(TAG, "VEML7700: %.1f lux", lux);

    char lux_buf[40];
    snprintf(lux_buf, sizeof(lux_buf), "Light: %.1f lux", lux);
    lv_label_set_text(light_value_label, lux_buf);

    lv_coord_t chart_value = (lv_coord_t)lux;
    if (chart_value < 0) {
        chart_value = 0;
    } else if (chart_value > 1000) {
        chart_value = 1000;
    }

    lv_chart_set_next_value(light_chart, light_series, chart_value);
    lv_chart_refresh(light_chart);
}

static lv_obj_t *ui_create_light_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_add_style(scr, &style_screen, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_add_style(title, &style_title, 0);
    lv_label_set_text(title, "Ambient Light");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    light_value_label = lv_label_create(scr);
    lv_obj_add_style(light_value_label, &style_body, 0);
    lv_label_set_text(light_value_label, "Light: --.- lux");
    lv_obj_align(light_value_label, LV_ALIGN_TOP_MID, 0, 52);

    lv_obj_t *icon = ui_create_graphic_icon(scr, ICON_LIGHT);
    lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 80);

    light_chart = lv_chart_create(scr);
    lv_obj_set_size(light_chart, 360, 180);
    lv_obj_align(light_chart, LV_ALIGN_CENTER, 0, 56);
    lv_chart_set_type(light_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(light_chart, 24);
    lv_chart_set_range(light_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_div_line_count(light_chart, 6, 6);

    light_series = lv_chart_add_series(light_chart,
                                       lv_color_hex(0xFFD166),
                                       LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(light_chart, light_series, LV_CHART_POINT_NONE);

    lv_obj_t *legend = lv_label_create(scr);
    lv_obj_add_style(legend, &style_body, 0);
    lv_label_set_text(legend, "VEML7700 ambient light, 0-1000 lux scale");
    lv_obj_align(legend, LV_ALIGN_BOTTOM_MID, 0, -82);

    ui_create_back_button(scr);

    lv_timer_create(light_chart_timer_cb, 2000, NULL);
    light_chart_timer_cb(NULL);

    return scr;
}


static void distance_chart_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    uint16_t distance_mm = 0;
    uint8_t confidence = 0;

    if (!tmf8821_state.present) {
        lv_label_set_text(distance_value_label, "Distance: sensor not found");
        lv_label_set_text(distance_status_label, "Check Qwiic/STEMMA QT chain and address 0x41");
        return;
    }

    if (!tmf8821_state.app_running) {
        char status_buf[96];
        snprintf(status_buf, sizeof(status_buf),
                 "APPID 0x%02X: firmware app not running", tmf8821_state.appid);
        lv_label_set_text(distance_value_label, "Distance: unavailable");
        lv_label_set_text(distance_status_label, status_buf);
        return;
    }

    if (!tmf8821_read_distance(&distance_mm, &confidence)) {
        lv_label_set_text(distance_status_label, "Waiting for TMF8821 result...");
        return;
    }

    g_last_distance_mm_web = distance_mm;
    g_last_distance_confidence_web = confidence;
    g_distance_has_reading = true;

    ESP_LOGI(TAG, "TMF8821: %u mm, conf=%u", distance_mm, confidence);

    char dist_buf[48];
    char status_buf[48];
    snprintf(dist_buf, sizeof(dist_buf), "Distance: %u mm", distance_mm);
    snprintf(status_buf, sizeof(status_buf), "Confidence: %u", confidence);
    lv_label_set_text(distance_value_label, dist_buf);
    lv_label_set_text(distance_status_label, status_buf);

    lv_coord_t chart_value = (lv_coord_t)distance_mm;
    if (chart_value < 0) {
        chart_value = 0;
    } else if (chart_value > 5000) {
        chart_value = 5000;
    }

    lv_chart_set_next_value(distance_chart, distance_series, chart_value);
    lv_chart_refresh(distance_chart);
}

static lv_obj_t *ui_create_distance_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_add_style(scr, &style_screen, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_add_style(title, &style_title, 0);
    lv_label_set_text(title, "Distance");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    distance_value_label = lv_label_create(scr);
    lv_obj_add_style(distance_value_label, &style_body, 0);
    lv_label_set_text(distance_value_label, "Distance: -- mm");
    lv_obj_align(distance_value_label, LV_ALIGN_TOP_MID, 0, 50);

    distance_status_label = lv_label_create(scr);
    lv_obj_add_style(distance_status_label, &style_body, 0);
    lv_obj_set_width(distance_status_label, 360);
    lv_obj_set_style_text_align(distance_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(distance_status_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(distance_status_label, "TMF8821 status: initializing");
    lv_obj_align(distance_status_label, LV_ALIGN_TOP_MID, 0, 76);

    lv_obj_t *icon = ui_create_graphic_icon(scr, ICON_DISTANCE);
    lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 104);

    distance_chart = lv_chart_create(scr);
    lv_obj_set_size(distance_chart, 360, 170);
    lv_obj_align(distance_chart, LV_ALIGN_CENTER, 0, 62);
    lv_chart_set_type(distance_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(distance_chart, 24);
    lv_chart_set_range(distance_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 5000);
    lv_chart_set_div_line_count(distance_chart, 6, 6);

    distance_series = lv_chart_add_series(distance_chart,
                                          lv_color_hex(0x8BC6FF),
                                          LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(distance_chart, distance_series, LV_CHART_POINT_NONE);

    lv_obj_t *legend = lv_label_create(scr);
    lv_obj_add_style(legend, &style_body, 0);
    lv_label_set_text(legend, "TMF8821 best zone distance, 0-5000 mm scale");
    lv_obj_align(legend, LV_ALIGN_BOTTOM_MID, 0, -82);

    ui_create_back_button(scr);

    lv_timer_create(distance_chart_timer_cb, 500, NULL);
    distance_chart_timer_cb(NULL);

    return scr;
}


static void gas_chart_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    float adc_v = 0.0f;
    float sensor_v = 0.0f;
    int raw = 0;

    if (!gas_read_mics(&adc_v, &sensor_v, &raw)) {
        ESP_LOGW(TAG, "MiCS-5524 ADC read failed");
        if (gas_value_label != NULL) {
            lv_label_set_text(gas_value_label, "Gas: --.- V");
        }
        if (gas_status_label != NULL) {
            lv_label_set_text(gas_status_label, "Check AO divider wiring to GPIO5");
        }
        return;
    }

    g_last_gas_adc_v = adc_v;
    g_last_gas_sensor_v = sensor_v;
    g_last_gas_raw = raw;
    g_gas_has_reading = true;

    ESP_LOGI(TAG, "MiCS-5524: raw=%d, ADC=%.2f V, AO≈%.2f V", raw, adc_v, sensor_v);

    char gas_buf[64];
    char status_buf[96];
    snprintf(gas_buf, sizeof(gas_buf), "Gas AO: %.2f V", sensor_v);
    snprintf(status_buf, sizeof(status_buf), "ADC: %.2f V   Raw: %d", adc_v, raw);

    lv_label_set_text(gas_value_label, gas_buf);
    lv_label_set_text(gas_status_label, status_buf);

    lv_coord_t chart_value = (lv_coord_t)(sensor_v * 1000.0f + 0.5f);
    if (chart_value < 0) {
        chart_value = 0;
    } else if (chart_value > GAS_MAX_SENSOR_MV) {
        chart_value = GAS_MAX_SENSOR_MV;
    }

    lv_chart_set_next_value(gas_chart, gas_series, chart_value);
    lv_chart_refresh(gas_chart);
}

static lv_obj_t *ui_create_gas_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_add_style(scr, &style_screen, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_add_style(title, &style_title, 0);
    lv_label_set_text(title, "Gas Sensor");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    gas_value_label = lv_label_create(scr);
    lv_obj_add_style(gas_value_label, &style_body, 0);
    lv_label_set_text(gas_value_label, "Gas AO: --.-- V");
    lv_obj_align(gas_value_label, LV_ALIGN_TOP_MID, 0, 50);

    gas_status_label = lv_label_create(scr);
    lv_obj_add_style(gas_status_label, &style_body, 0);
    lv_obj_set_width(gas_status_label, 360);
    lv_obj_set_style_text_align(gas_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(gas_status_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(gas_status_label, "MiCS-5524 warming up / reading GPIO5");
    lv_obj_align(gas_status_label, LV_ALIGN_TOP_MID, 0, 76);

    lv_obj_t *icon = ui_create_graphic_icon(scr, ICON_GAS);
    lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 104);

    gas_chart = lv_chart_create(scr);
    lv_obj_set_size(gas_chart, 360, 170);
    lv_obj_align(gas_chart, LV_ALIGN_CENTER, 0, 62);
    lv_chart_set_type(gas_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(gas_chart, 24);
    lv_chart_set_range(gas_chart, LV_CHART_AXIS_PRIMARY_Y, 0, GAS_MAX_SENSOR_MV);
    lv_chart_set_div_line_count(gas_chart, 6, 6);

    gas_series = lv_chart_add_series(gas_chart,
                                     lv_color_hex(0x7CFFB2),
                                     LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(gas_chart, gas_series, LV_CHART_POINT_NONE);

    lv_obj_t *legend = lv_label_create(scr);
    lv_obj_add_style(legend, &style_body, 0);
    lv_obj_set_width(legend, 360);
    lv_obj_set_style_text_align(legend, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(legend, LV_LABEL_LONG_WRAP);
    lv_label_set_text(legend, "Graph shows estimated MiCS AO voltage, 0-5 V scale");
    lv_obj_align(legend, LV_ALIGN_BOTTOM_MID, 0, -82);

    ui_create_back_button(scr);

    gas_adc_init();
    lv_timer_create(gas_chart_timer_cb, 1000, NULL);
    gas_chart_timer_cb(NULL);

    return scr;
}

#define MENU_CARD_SIZE 118
#define MENU_CARD_ICON_Y (-18)
#define MENU_CARD_LABEL_Y 36

static const char *recipe_text_from_environment(float temp_c, float humidity)
{
    if (temp_c > 27.0f && humidity > 70.0f) {
        return "RECIPE 1:\n40g flour\n40g water\n20g starter";
    } else if (temp_c <= 27.0f && humidity > 70.0f) {
        return "RECIPE 2:\n45g flour\n35g water\n20g starter";
    } else if (temp_c > 27.0f && humidity <= 70.0f) {
        return "RECIPE 3:\n35g flour\n45g water\n20g starter";
    } else {
        return "RECIPE 4:\n40g flour\n40g water\n20g starter";
    }
}

static void recipe_update_display(void)
{
    if (recipe_value_label == NULL || recipe_status_label == NULL) {
        return;
    }

    if (!g_sht41_has_reading) {
        lv_label_set_text(recipe_value_label, "Waiting for SHT41 reading...");
        lv_label_set_text(recipe_status_label, "Temp: --.- C   Humidity: --.- %");
        return;
    }

    char status_buf[64];
    snprintf(status_buf, sizeof(status_buf),
             "Temp: %.1f C   Humidity: %.1f%%",
             g_last_temp_c, g_last_humidity);

    lv_label_set_text(recipe_value_label,
                      recipe_text_from_environment(g_last_temp_c, g_last_humidity));
    lv_label_set_text(recipe_status_label, status_buf);
}

static void recipe_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    recipe_update_display();
}

static lv_obj_t *ui_create_recipes_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_add_style(scr, &style_screen, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_obj_add_style(title, &style_title, 0);
    lv_label_set_text(title, "Starter Recipe");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

    recipe_status_label = lv_label_create(scr);
    lv_obj_add_style(recipe_status_label, &style_body, 0);
    lv_obj_set_width(recipe_status_label, 360);
    lv_obj_set_style_text_align(recipe_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(recipe_status_label, "Temp: --.- C   Humidity: --.- %");
    lv_obj_align(recipe_status_label, LV_ALIGN_TOP_MID, 0, 58);

    lv_obj_t *icon = ui_create_graphic_icon(scr, ICON_RECIPE);
    lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 88);

    recipe_value_label = lv_label_create(scr);
    lv_obj_add_style(recipe_value_label, &style_body, 0);
    lv_obj_set_width(recipe_value_label, 340);
    lv_obj_set_style_text_align(recipe_value_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(recipe_value_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(recipe_value_label, "Waiting for SHT41 reading...");
    lv_obj_align(recipe_value_label, LV_ALIGN_CENTER, 0, 42);

    lv_obj_t *note = lv_label_create(scr);
    lv_obj_add_style(note, &style_body, 0);
    lv_obj_set_width(note, 340);
    lv_obj_set_style_text_align(note, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_label_set_text(note, "Rules: 27 C temperature split, 70% humidity split.");
    lv_obj_align(note, LV_ALIGN_BOTTOM_MID, 0, -82);

    ui_create_back_button(scr);
    recipe_update_display();
    return scr;
}

static lv_obj_t *ui_create_menu_card(lv_obj_t *parent,
                                     sensor_icon_t icon_type,
                                     const char *label_txt,
                                     lv_event_cb_t cb,
                                     lv_coord_t x,
                                     lv_coord_t y)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_add_style(btn, &style_card, 0);
    lv_obj_set_size(btn, MENU_CARD_SIZE, MENU_CARD_SIZE);
    lv_obj_set_pos(btn, x, y);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *icon = ui_create_graphic_icon(btn, icon_type);
    lv_obj_set_style_opa(icon, LV_OPA_100, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, MENU_CARD_ICON_Y);

    lv_obj_t *label = lv_label_create(btn);
    lv_obj_add_style(label, &style_body, 0);
    lv_obj_set_width(label, 96);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label, label_txt);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, MENU_CARD_LABEL_Y);

    return btn;
}

static void ui_create_main_screen(void)
{
    screen_main = lv_obj_create(NULL);
    lv_obj_add_style(screen_main, &style_screen, 0);
    lv_obj_clear_flag(screen_main, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(screen_main);
    lv_obj_add_style(title, &style_title, 0);
    lv_label_set_text(title, "Sour-Do");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *subtitle = lv_label_create(screen_main);
    lv_obj_add_style(subtitle, &style_body, 0);
    lv_label_set_text(subtitle, "Tap a card");
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 48);

    battery_label = lv_label_create(screen_main);
    lv_obj_add_style(battery_label, &style_body, 0);
    lv_label_set_text(battery_label, "BAT ---.--V --%");
    lv_obj_align(battery_label, LV_ALIGN_TOP_MID, 0, 72);

    /* Five-card circular layout for 466x466 screen. */
    ui_create_menu_card(screen_main, ICON_TEMP,     "Temp /\nHumidity", ui_open_temp,     44,  84);
    ui_create_menu_card(screen_main, ICON_LIGHT,    "Light",            ui_open_light,    304, 84);
    ui_create_menu_card(screen_main, ICON_RECIPE,   "Recipes",          ui_open_recipes,  174, 194);
    ui_create_menu_card(screen_main, ICON_DISTANCE, "Distance",         ui_open_distance, 44,  306);
    ui_create_menu_card(screen_main, ICON_GAS,      "Gas",              ui_open_gas,      304, 306);
}

static void ui_create(void)
{
    ui_init_styles();

    ui_create_main_screen();

    screen_temp = ui_create_temp_screen();

    screen_light = ui_create_light_screen();

    screen_distance = ui_create_distance_screen();

    screen_gas = ui_create_gas_screen();

    screen_recipes = ui_create_recipes_screen();

    lv_timer_create(recipe_timer_cb, 2000, NULL);

    lv_scr_load(screen_main);

    battery_update_label();
    lv_timer_create(battery_timer_cb, 10000, NULL);

    power_note_user_activity();
    lv_timer_create(power_timer_cb, 1000, NULL);
}

/* ---------------- app_main ---------------- */

void app_main(void)
{
    READ_LCD_ID = read_lcd_id();

    softap_web_monitor_start();

    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg =
        SH8601_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                     EXAMPLE_PIN_NUM_LCD_DATA0,
                                     EXAMPLE_PIN_NUM_LCD_DATA1,
                                     EXAMPLE_PIN_NUM_LCD_DATA2,
                                     EXAMPLE_PIN_NUM_LCD_DATA3,
                                     EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config =
        SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                    example_notify_lvgl_flush_ready,
                                    &disp_drv);

    sh8601_vendor_config_t vendor_config = {
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &io_config,
                                             &io_handle));
    g_panel_io_handle = io_handle;

    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    vendor_config.init_cmds =
        (READ_LCD_ID == SH8601_ID) ? sh8601_lcd_init_cmds : co5300_lcd_init_cmds;

    vendor_config.init_cmds_size =
        (READ_LCD_ID == SH8601_ID)
            ? sizeof(sh8601_lcd_init_cmds) / sizeof(sh8601_lcd_init_cmds[0])
            : sizeof(co5300_lcd_init_cmds) / sizeof(co5300_lcd_init_cmds[0]);

    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    g_panel_handle = panel_handle;
    set_lcd_brightness(LCD_BRIGHTNESS_FULL);

    ESP_LOGI(TAG, "Initialize touch");
    Touch_Init();

    /* VEML7700 and TMF8821 are daisy-chained on the same already-working I2C bus as SHT41. */
    veml7700_init();
    tmf8821_init();

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES *
                                         EXAMPLE_LVGL_BUF_HEIGHT *
                                         sizeof(lv_color_t),
                                         MALLOC_CAP_DMA);
    assert(buf1);

    lv_color_t *buf2 = NULL;

    lv_disp_draw_buf_init(&disp_buf, buf1, buf2,
                        EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;

#ifdef EXAMPLE_Rotate_90
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_270;
#endif

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer,
                                             EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);

    xTaskCreate(example_lvgl_port_task,
                "LVGL",
                EXAMPLE_LVGL_TASK_STACK_SIZE,
                NULL,
                EXAMPLE_LVGL_TASK_PRIORITY,
                NULL);

    ESP_LOGI(TAG, "Display custom UI");
    if (example_lvgl_lock(-1)) {
        ui_create();
        example_lvgl_unlock();
    }
}
