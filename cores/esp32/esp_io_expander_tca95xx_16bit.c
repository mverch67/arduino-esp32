/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "driver/i2c.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp32-hal-log.h"

#include "esp_io_expander.h"
#include "esp_io_expander_tca95xx_16bit.h"

/* Timeout of each I2C communication */
#define I2C_TIMEOUT_MS          (50)

#define IO_COUNT                (16)

/* Register address */
#define INPUT_REG_ADDR          (0x00)
#define OUTPUT_REG_ADDR         (0x02)
#define DIRECTION_REG_ADDR      (0x06)

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL     (0xffff)
#define OUT_REG_DEFAULT_VAL     (0xffff)

/**
 * @brief Device Structure Type
 *
 */
typedef struct {
    esp_io_expander_t base;
    i2c_port_t i2c_num;
    uint32_t i2c_address;
    struct {
        uint16_t direction;
        uint16_t output;
    } regs;
} esp_io_expander_tca95xx_16bit_t;

static const char *TAG = "tca95xx_16";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

static TaskHandle_t *xTaskToNotify = NULL;

/**
 * @brief isr handler to notify a waiting task that some input bits have changed.
 *        The processing task will process the bits and run the related callbacks
 */
static void io_expander_isr_handler(void *arg)
{
    if (xTaskToNotify) {
        log_d("IRQ!");
        BaseType_t xHigherPriorityTaskWoken = pdTRUE;
        vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
        xTaskToNotify = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Processing routine that will wait for notification from ISR and read the io expander input bits
 */
static esp_err_t esp_io_expander_process_irq_tca95xx_16bit(esp_io_expander_handle_t handle)
{
    xTaskToNotify = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t value = 0;
    esp_err_t err = handle->read_input_reg(handle, &value);
    if (err != ESP_OK) {
        log_e("failed to read input pins: %d", err);
        return err;
    }

    if (handle->mask == 0) return ESP_OK;
    for (int i=0; i<IO_COUNT; i++) {
        if (handle->pinIOExpanderISRs[i].functional) {
            bool trigger = false;
            if (handle->pinIOExpanderISRs[i].mode == GPIO_INTR_POSEDGE || handle->pinIOExpanderISRs[i].mode == GPIO_INTR_HIGH_LEVEL) {
                if ((value & (1 << i)) != 0) {
                    log_d("IRQ pin rising/high %d", i);
                    trigger = true;
                }
            }
            else if (handle->pinIOExpanderISRs[i].mode == GPIO_INTR_NEGEDGE || handle->pinIOExpanderISRs[i].mode == GPIO_INTR_LOW_LEVEL) {
                if ((value & (1 << i)) == 0) {
                    log_d("IRQ pin falling/low %d", i);
                    trigger = true;
                }
            }
            else if (handle->pinIOExpanderISRs[i].mode == GPIO_INTR_ANYEDGE) {
                static int previous = 0;
                if ((value & (1 << i)) != previous) {
                    previous = value & (1 << i);
                    log_d("IRQ pin anyedge %d", i);
                    trigger = true;
                }
            }

            if (trigger) {
                (handle->pinIOExpanderISRs[i].fn)(handle->pinIOExpanderISRs[i].arg);
            }
        }
    }
    return ESP_OK;
}

esp_err_t esp_io_expander_new_i2c_tca95xx_16bit(i2c_port_t i2c_num, uint32_t i2c_address, esp_io_expander_handle_t *handle)
{
    log_d("io expander init i2c(%d:0x%02x)", i2c_num, i2c_address);
    ESP_RETURN_ON_FALSE(i2c_num < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "Invalid i2c num");
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)calloc(1, sizeof(esp_io_expander_tca95xx_16bit_t));
    ESP_RETURN_ON_FALSE(tca, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    tca->base.config.io_count = IO_COUNT;
    tca->base.config.flags.dir_out_bit_zero = 1;
    tca->i2c_num = i2c_num;
    tca->i2c_address = i2c_address;
    tca->base.read_input_reg = read_input_reg;
    tca->base.write_output_reg = write_output_reg;
    tca->base.read_output_reg = read_output_reg;
    tca->base.write_direction_reg = write_direction_reg;
    tca->base.read_direction_reg = read_direction_reg;
    tca->base.reset = reset;
    tca->base.del = del;
    tca->base.process = esp_io_expander_process_irq_tca95xx_16bit;
    tca->base.pinIOExpanderISRs = NULL;
    tca->base.mask = 0;

    esp_err_t ret = ESP_OK;
    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&tca->base), err, TAG, "Reset failed");

    *handle = &tca->base;

    esp_io_expander_setup_IRQ(*handle, io_expander_isr_handler, IO_EXPANDER_IRQ, GPIO_INTR_NEGEDGE);
    return ESP_OK;
err:
    free(tca);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);

    uint8_t temp[2] = {0, 0};
    esp_err_t err_rc = i2c_master_write_read_device(tca->i2c_num, tca->i2c_address, (uint8_t[]){INPUT_REG_ADDR}, 1, (uint8_t*)&temp, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err_rc != ESP_OK) {
        // do a retry if read fails
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(
            i2c_master_write_read_device(tca->i2c_num, tca->i2c_address, (uint8_t[]){INPUT_REG_ADDR}, 1, (uint8_t*)&temp, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
            TAG, "Read input reg failed");
    }
    *value = (((uint32_t)temp[1]) << 8) | (temp[0]);
    //log_d("i2c read values=%u %u", (uint16_t)temp[0], (uint16_t)temp[1]);
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {OUTPUT_REG_ADDR, value & 0xff, value >> 8};
    //log_d("i2c(%d:0x%02x) write values: %u %u %u", tca->i2c_num, tca->i2c_address, data[0], data[1], data[2]);
    esp_err_t err_rc = i2c_master_write_to_device(tca->i2c_num, tca->i2c_address, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err_rc != ESP_OK) {
        // do a retry if write fails
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(
            i2c_master_write_to_device(tca->i2c_num, tca->i2c_address, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
            TAG, "Write output reg failed");
    }
    tca->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);

    *value = tca->regs.output;
    //log_d("value=%d", *value);
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {DIRECTION_REG_ADDR, value & 0xff, value >> 8};
    //log_d("i2c(%d) write values: %u %u %u", tca->i2c_num, data[0], data[1], data[2]);
    ESP_RETURN_ON_ERROR(
        i2c_master_write_to_device(tca->i2c_num, tca->i2c_address, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Write direction reg failed");
    tca->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);

    *value = tca->regs.direction;
    //log_d("value=%d", *value);
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);

    free(tca);
    return ESP_OK;
}
