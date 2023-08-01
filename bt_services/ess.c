#include "ess.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#include "bme68x.h"

bool ess_notifications_enabled[ESS_ID_COUNT] = {0};
ess_value_t ess_current_values[ESS_ID_COUNT] = {0};

struct bme68x_dev bme;
struct bme68x_conf bme_conf;
struct bme68x_heatr_conf bme_heatr_conf;
struct bme68x_data bme_data;
static uint8_t bme_dev_addr;

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    uint8_t buff[16];
    int bytes_sent;

    buff[0] = reg_addr;
    for (size_t i = 0; i < len; i++) {
        buff[i + 1] = reg_data[i];
    }
    bytes_sent = i2c_write_blocking(BME680_I2C_INST, device_addr, buff, len + 1, false);

    return bytes_sent <= 0;
}

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    int bytes_sent;
    int bytes_received;

    bytes_sent = i2c_write_blocking(BME680_I2C_INST, device_addr, &reg_addr, 1, false);
    bytes_received = i2c_read_blocking(BME680_I2C_INST, device_addr, reg_data, len, false);

    return (bytes_sent <= 0) || (bytes_received <= 0);
}

void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    sleep_us(period);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt) {
    switch (rslt) {
        case BME68X_OK:
            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

void ess_init() {
    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(BME680_I2C_INST, 300 * 1000);
    gpio_set_function(BME680_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BME680_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BME680_I2C_SDA_PIN);
    gpio_pull_up(BME680_I2C_SCL_PIN);

    bme_dev_addr = BME68X_I2C_ADDR_LOW;
    bme.read  = bme68x_i2c_read;
    bme.write = bme68x_i2c_write;
    bme.intf  = BME68X_I2C_INTF;
    bme.delay_us = bme68x_delay_us;
    bme.intf_ptr = &bme_dev_addr;
    bme.amb_temp = 25; // The ambient temperature in deg C is used for defining the heater temperature

    int8_t rslt;
    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    // Check if rslt == BME68X_OK, report or handle if otherwise
    bme_conf.filter = BME68X_FILTER_OFF;
    bme_conf.odr = BME68X_ODR_NONE;
    bme_conf.os_hum = BME68X_OS_16X;
    bme_conf.os_pres = BME68X_OS_1X;
    bme_conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&bme_conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);

    // Check if rslt == BME68X_OK, report or handle if otherwise
    bme_heatr_conf.enable = BME68X_ENABLE;
    bme_heatr_conf.heatr_temp = 300;
    bme_heatr_conf.heatr_dur = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme_heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);
}

bool ess_any_notification_enabled() {
    bool any_enabled = false;

    for (size_t i = 0; i < ESS_ID_COUNT; i++) {
        any_enabled |= ess_notifications_enabled[i];
    }

    return any_enabled;
}

void ess_read_data(void) {
    uint8_t n_fields;
    int8_t rslt;

    rslt = bme68x_get_data(BME68X_FORCED_MODE, &bme_data, &n_fields, &bme);
    bme68x_check_rslt("bme68x_get_data", rslt);

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);

    printf("%0d, %0d, %0d, %0d, 0x%x\n",
            bme_data.temperature,
            bme_data.pressure,
            bme_data.humidity,
            bme_data.gas_resistance,
            bme_data.status);

    ess_current_values[ESS_TEMPERATURE] = bme_data.temperature;
    ess_current_values[ESS_PRESSURE] = bme_data.pressure;
    ess_current_values[ESS_HUMIDITY] = bme_data.humidity;
}