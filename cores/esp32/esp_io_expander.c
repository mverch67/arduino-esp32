/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp32-hal-log.h"
#include "esp32-hal-gpio.h"

#include "esp_io_expander.h"
 
#define VALID_IO_COUNT(handle)      ((handle)->config.io_count <= IO_COUNT_MAX ? (handle)->config.io_count : IO_COUNT_MAX)

/**
 * @brief Register type
 *
 */
typedef enum {
    REG_INPUT = 0,
    REG_OUTPUT,
    REG_DIRECTION,
} reg_type_t;

static const char *TAG = "io_expander";

static esp_err_t write_reg(esp_io_expander_handle_t handle, reg_type_t reg, uint32_t value);
static esp_err_t read_reg(esp_io_expander_handle_t handle, reg_type_t reg, uint32_t *value);

esp_err_t esp_io_expander_set_dir(esp_io_expander_handle_t handle, uint32_t pin_num_mask, esp_io_expander_dir_t direction)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    if (pin_num_mask >= BIT64(VALID_IO_COUNT(handle))) {
        ESP_LOGW(TAG, "Pin num mask out of range, bit higher than %d won't work", VALID_IO_COUNT(handle) - 1);
    }

    bool is_output = (direction == IO_EXPANDER_OUTPUT) ? true : false;
    uint32_t dir_reg, temp;
    ESP_RETURN_ON_ERROR(read_reg(handle, REG_DIRECTION, &dir_reg), TAG, "Read direction reg failed");
    temp = dir_reg;
    if ((is_output && !handle->config.flags.dir_out_bit_zero) || (!is_output && handle->config.flags.dir_out_bit_zero)) {
        /* 1. Output && Set 1 to output */
        /* 2. Input && Set 1 to input */
        dir_reg |= pin_num_mask;
    } else {
        /* 3. Output && Set 0 to output */
        /* 4. Input && Set 0 to input */
        dir_reg &= ~pin_num_mask;
    }
    /* Write to reg only when different */
    if (dir_reg != temp) {
        ESP_RETURN_ON_ERROR(write_reg(handle, REG_DIRECTION, dir_reg), TAG, "Write direction reg failed");
    }

    return ESP_OK;
}

esp_err_t esp_io_expander_set_level(esp_io_expander_handle_t handle, uint32_t pin_num_mask, uint8_t level)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    if (pin_num_mask >= BIT64(VALID_IO_COUNT(handle))) {
        ESP_LOGW(TAG, "Pin num mask out of range, bit higher than %d won't work", VALID_IO_COUNT(handle) - 1);
    }

    uint32_t dir_reg, dir_bit;
    ESP_RETURN_ON_ERROR(read_reg(handle, REG_DIRECTION, &dir_reg), TAG, "Read direction reg failed");

    uint8_t io_count = VALID_IO_COUNT(handle);
    /* Check every target pin's direction, must be in output mode */
    for (int i = 0; i < io_count; i++) {
        if (pin_num_mask & BIT(i)) {
            dir_bit = dir_reg & BIT(i);
            /* Check whether it is in input mode */
            if ((dir_bit && handle->config.flags.dir_out_bit_zero) || (!dir_bit && !handle->config.flags.dir_out_bit_zero)) {
                /* 1. 1 && Set 1 to input */
                /* 2. 0 && Set 0 to input */
                ESP_LOGW(TAG, "Pin[%d] can't set level in input mode", i);
                //return ESP_ERR_INVALID_STATE;
                ESP_RETURN_ON_ERROR(write_reg(handle, REG_DIRECTION, !dir_reg), TAG, "Overwrite direction reg failed");
            }
        }
    }

    uint32_t output_reg, temp;
    /* Read the current output level */
    ESP_RETURN_ON_ERROR(read_reg(handle, REG_OUTPUT, &output_reg), TAG, "Read Output reg failed");
    temp = output_reg;
    /* Set expected output level */
    if ((level && !handle->config.flags.output_high_bit_zero) || (!level && handle->config.flags.output_high_bit_zero)) {
        /* 1. High level && Set 1 to output high */
        /* 2. Low level && Set 1 to output low */
        output_reg |= pin_num_mask;
    } else {
        /* 3. High level && Set 0 to output high */
        /* 4. Low level && Set 0 to output low */
        output_reg &= ~pin_num_mask;
    }
    /* Write to reg only when different */
    if (output_reg != temp) {
        ESP_RETURN_ON_ERROR(write_reg(handle, REG_OUTPUT, output_reg), TAG, "Write Output reg failed");
    }

    return ESP_OK;
}

esp_err_t esp_io_expander_get_level(esp_io_expander_handle_t handle, uint32_t pin_num_mask, uint32_t *level_mask)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(level_mask, ESP_ERR_INVALID_ARG, TAG, "Invalid level");
    if (pin_num_mask >= BIT64(VALID_IO_COUNT(handle))) {
        ESP_LOGW(TAG, "Pin num mask out of range, bit higher than %d won't work", VALID_IO_COUNT(handle) - 1);
    }

    uint32_t input_reg;
    ESP_RETURN_ON_ERROR(read_reg(handle, REG_INPUT, &input_reg), TAG, "Read input reg failed");
    if (!handle->config.flags.input_high_bit_zero) {
        /* Get 1 when input high level */
        *level_mask = input_reg & pin_num_mask;
    } else {
        /* Get 0 when input high level */
        *level_mask = ~input_reg & pin_num_mask;
    }

    return ESP_OK;
}

esp_err_t esp_io_expander_print_state(esp_io_expander_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t io_count = VALID_IO_COUNT(handle);
    uint32_t input_reg, output_reg, dir_reg;
    ESP_RETURN_ON_ERROR(read_reg(handle, REG_INPUT, &input_reg), TAG, "Read input reg failed");
    ESP_RETURN_ON_ERROR(read_reg(handle, REG_OUTPUT, &output_reg), TAG, "Read output reg failed");
    ESP_RETURN_ON_ERROR(read_reg(handle, REG_DIRECTION, &dir_reg), TAG, "Read direction reg failed");
    /* Get 1 if high level */
    if (handle->config.flags.input_high_bit_zero) {
        input_reg ^= 0xffffffff;
    }
    /* Get 1 if high level */
    if (handle->config.flags.output_high_bit_zero) {
        output_reg ^= 0xffffffff;
    }
    /* Get 1 if output */
    if (handle->config.flags.dir_out_bit_zero) {
        dir_reg ^= 0xffffffff;
    }

    for (int i = 0; i < io_count; i++) {
        ESP_LOGI(TAG, "Index[%d] | Dir[%s] | In[%d] | Out[%d]", i, (dir_reg & BIT(i)) ? "Out" : "In",
                 (input_reg & BIT(i)) ? 1 : 0, (output_reg & BIT(i)) ? 1 : 0);
    }

    return ESP_OK;
}

esp_err_t esp_io_expander_reset(esp_io_expander_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(handle->reset, ESP_ERR_NOT_SUPPORTED, TAG, "reset isn't implemented");

    return handle->reset(handle);
}

esp_err_t esp_io_expander_del(esp_io_expander_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(handle->del, ESP_ERR_NOT_SUPPORTED, TAG, "del isn't implemented");

    return handle->del(handle);
}

/**
 * @brief Write the value to a specific register
 *
 * @param handle: IO Expander handle
 * @param reg: Specific type of register
 * @param value: Expected register's value
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
static esp_err_t write_reg(esp_io_expander_handle_t handle, reg_type_t reg, uint32_t value)
{   
    //log_i("reg=0x%02x -> %08x", reg, value);
    switch (reg) {
    case REG_OUTPUT:
        ESP_RETURN_ON_FALSE(handle->write_output_reg, ESP_ERR_NOT_SUPPORTED, TAG, "write_output_reg isn't implemented");
        return handle->write_output_reg(handle, value);
    case REG_DIRECTION:
        ESP_RETURN_ON_FALSE(handle->write_direction_reg, ESP_ERR_NOT_SUPPORTED, TAG, "write_direction_reg isn't implemented");
        return handle->write_direction_reg(handle, value);
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

/**
 * @brief Read the value from a specific register
 *
 * @param handle: IO Expander handle
 * @param reg: Specific type of register
 * @param value: Actual register's value
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
static esp_err_t read_reg(esp_io_expander_handle_t handle, reg_type_t reg, uint32_t *value)
{
    ESP_RETURN_ON_FALSE(value, ESP_ERR_INVALID_ARG, TAG, "Invalid value");

    switch (reg) {
    case REG_INPUT:
        ESP_RETURN_ON_FALSE(handle->read_input_reg, ESP_ERR_NOT_SUPPORTED, TAG, "read_input_reg isn't implemented");
        return handle->read_input_reg(handle, value);
    case REG_OUTPUT:
        ESP_RETURN_ON_FALSE(handle->read_output_reg, ESP_ERR_NOT_SUPPORTED, TAG, "read_output_reg isn't implemented");
        return handle->read_output_reg(handle, value);
    case REG_DIRECTION:
        ESP_RETURN_ON_FALSE(handle->read_direction_reg, ESP_ERR_NOT_SUPPORTED, TAG, "read_direction_reg isn't implemented");
        return handle->read_direction_reg(handle, value);
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

/**
 * @brief Setup IO IRQ
 *
 * @param handle   : IO Expander handle
 * @param pin      : IO expander IRQ GPIO pin
 * @param isr      : interrupt handler to be triggered
 * @param mode     : defines when the interrupt should be triggered.
 *   GPIO_INTR_DISABLE = 0,     Disable GPIO interrupt
 *   GPIO_INTR_POSEDGE = 1,     GPIO interrupt type : rising edge
 *   GPIO_INTR_NEGEDGE = 2,     GPIO interrupt type : falling edge
 *   GPIO_INTR_ANYEDGE = 3,     GPIO interrupt type : both rising and falling edge
 *   GPIO_INTR_LOW_LEVEL = 4,   GPIO interrupt type : input low level trigger
 *   GPIO_INTR_HIGH_LEVEL = 5,  GPIO interrupt type : input high level trigger
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 *
 */
esp_err_t esp_io_expander_setup_isr(esp_io_expander_handle_t handle, voidIOExpanderISRHandler cb, uint8_t pin, int mode)
{
    if (!handle->pinIOExpanderISRs) {
        log_d("esp_io_expander_setup_isr: pinIOExpanderISRs[] is NULL, allocating memory");
        handle->pinIOExpanderISRs = malloc(sizeof(ISRHandle_t) * handle->config.io_count);
        memset(handle->pinIOExpanderISRs, 0, sizeof(ISRHandle_t) * handle->config.io_count);
        attachInterruptArg(pin, cb, handle, mode);
    }
    else {
        //detachInterrupt(pin);
        attachInterruptArg(pin, cb, handle, mode);
    }
    return ESP_OK;
}

/**
 * @brief process interrupts
 *        Needs to called by e.g. a task that communicates with the ISR handler
 *
 * @param handle: IO Expander handle
 */
esp_err_t esp_io_expander_process_irq(esp_io_expander_handle_t handle)
{
    return (handle && handle->mask) ? handle->process(handle) : ESP_OK;
}

/**
 * @brief Attach interrupt handler callback
 *
 * @param handle   : IO Expander handle
 * @param pin      : IO expander IRQ GPIO pin
 * @param cb       : interrupt callback handler to be triggered
 * @param arg      : parameter to interrupt callback handler
 * @param intr_type: defines when the interrupt should be triggered (see above)
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 *
 */
esp_err_t esp_io_expander_attach_interrupt(esp_io_expander_handle_t handle, uint8_t pin, voidIOExpanderCB cb, void* arg, int intr_type)
{
    log_d("esp_io_expander_attach_interrupt: pin=%d, cb=%p, arg=%p, intr_type=%d", pin, cb, arg, intr_type);
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(cb, ESP_ERR_INVALID_ARG, TAG, "Invalid callback");
    if (pin < IO_COUNT_MAX && handle->pinIOExpanderISRs != NULL) {
        handle->pinIOExpanderISRs[pin].fn = cb;
        handle->pinIOExpanderISRs[pin].mode = intr_type;
        handle->pinIOExpanderISRs[pin].arg = arg;
        handle->pinIOExpanderISRs[pin].functional = true;
        handle->mask |= (1 << pin);
        return ESP_OK;
    }
    else {
        log_e("Invalid pin or pinIOExpanderISRs");
        return ESP_FAIL;
    }
}

/**
 * @brief Detach interrupt handler
 *
 * @param handle  : IO Expander handle
 * @param pin     : pin num with type of `esp_io_expander_pin_num_t`
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 *
 */esp_err_t esp_io_expander_detach_interrupt(esp_io_expander_handle_t handle, uint8_t pin)
{
    if (pin < IO_COUNT_MAX && handle->pinIOExpanderISRs[pin].functional) {
        handle->pinIOExpanderISRs[pin].fn = NULL;
        handle->pinIOExpanderISRs[pin].mode = 0;
        handle->pinIOExpanderISRs[pin].arg = NULL;
        handle->pinIOExpanderISRs[pin].functional = false;
        handle->mask &= ~(1 << pin);
        return ESP_OK;
    }
    else
        return ESP_FAIL;
}
