#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "touch_bsp.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define TOUCH_HOST  I2C_NUM_0
#define EXAMPLE_PIN_NUM_TOUCH_SCL         (GPIO_NUM_48)
#define EXAMPLE_PIN_NUM_TOUCH_SDA         (GPIO_NUM_47)
#define EXAMPLE_PIN_NUM_TOUCH_RST         (-1)
#define EXAMPLE_PIN_NUM_TOUCH_INT         (-1)
#define EXAMPLE_LCD_H_RES                 466
#define EXAMPLE_LCD_V_RES                 466
#define I2C_ADDR_FT3168                   0x38

#define FT3168_REG_MODE                   0x00
#define FT3168_REG_TD_STATUS              0x02
#define FT3168_REG_P1_XH                  0x03
#define FT3168_MAX_TOUCHES                2

static const char *TAG_TOUCH = "touch_bsp";

uint8_t I2C_writr_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t I2C_read_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

void Touch_Init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        /* 600 kHz is marginal on daisy-chained Qwiic/STEMMA wiring. */
        .master.clk_speed = 400 * 1000,
    };

    esp_err_t err = i2c_param_config(TOUCH_HOST, &i2c_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TOUCH, "i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG_TOUCH, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    uint8_t data = 0x00;
    I2C_writr_buff(I2C_ADDR_FT3168, FT3168_REG_MODE, &data, 1); // normal mode
}

uint8_t getTouch(uint16_t *x, uint16_t *y)
{
    uint8_t td_status = 0;

    if (I2C_read_buff(I2C_ADDR_FT3168, FT3168_REG_TD_STATUS, &td_status, 1) != ESP_OK) {
        return 0;
    }

    uint8_t touches = td_status & 0x0F;
    if (touches == 0 || touches > FT3168_MAX_TOUCHES) {
        return 0;
    }

    uint8_t buf[4] = {0};
    if (I2C_read_buff(I2C_ADDR_FT3168, FT3168_REG_P1_XH, buf, sizeof(buf)) != ESP_OK) {
        return 0;
    }

    /* FT3168/FT6x event flag is in bits 7:6. Accept down/contact only. */
    uint8_t event = (buf[0] >> 6) & 0x03;
    if (!(event == 0 || event == 2)) {
        return 0;
    }

    uint16_t rx = (((uint16_t)buf[0] & 0x0F) << 8) | (uint16_t)buf[1];
    uint16_t ry = (((uint16_t)buf[2] & 0x0F) << 8) | (uint16_t)buf[3];

    /* Reject impossible/out-of-panel coordinates instead of clamping noise. */
    if (rx >= EXAMPLE_LCD_H_RES || ry >= EXAMPLE_LCD_V_RES) {
        return 0;
    }

    *x = rx;
    *y = ry;
    return 1;
}

uint8_t I2C_writr_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t *pbuf = (uint8_t *)malloc((size_t)len + 1U);
    if (pbuf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    pbuf[0] = reg;
    for (uint8_t i = 0; i < len; i++) {
        pbuf[i + 1] = buf[i];
    }

    esp_err_t ret = i2c_master_write_to_device(TOUCH_HOST, addr, pbuf, len + 1, pdMS_TO_TICKS(100));
    free(pbuf);
    return ret;
}

uint8_t I2C_read_buff(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2c_master_write_read_device(TOUCH_HOST, addr, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

uint8_t I2C_master_write_read_device(uint8_t addr, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen)
{
    return i2c_master_write_read_device(TOUCH_HOST, addr, writeBuf, writeLen, readBuf, readLen, pdMS_TO_TICKS(100));
}
