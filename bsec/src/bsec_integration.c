#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"

#include "bsec_integration.h"
#include "bsec_iaq.h"

#define NUM_USED_OUTPUTS 13

static uint8_t bme68x_dev_addr;
static struct bme68x_dev bme68x_g;
static struct bme68x_conf bme68x_conf;
static struct bme68x_heatr_conf bme68x_heatr_conf;
static struct bme68x_data bme68x_sensor_data;

uint8_t dev_addr = BME68X_I2C_ADDR_LOW;

// State change and temporary data place holders
uint8_t last_op_mode = BME68X_SLEEP_MODE;
float temperature_offset_g = 0.0f;
uint8_t op_mode;

/*!
 * @brief        Pico specific implementation of I2C write for BSEC/BME68X
 *
 * @param[in]    reg_addr         device register address to write
 * @param[in]    reg_data         pointer to a data buffer which will be used as a write data
 * @param[in]    len              length of the provided data buffer in bytes
 * @param[in]    intf_ptr         a value from BME 68X dev structure which represents current interface
 *
 * @return       transmission result, zero when successful
 */
int8_t bsec_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
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

/*!
 * @brief        Pico specific implementation of I2C read for BSEC/BME68X
 *
 * @param[in]    reg_addr         device register address to read
 * @param[out]   reg_data         pointer to a data buffer which will be filled with read data
 * @param[in]    len              length of data to read
 * @param[in]    intf_ptr         a value from BME 68X dev structure which represents current interface
 *
 * @return       transmission result, zero when successful
 */
int8_t bsec_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    int bytes_sent;
    int bytes_received;

    bytes_sent = i2c_write_blocking(BME680_I2C_INST, device_addr, &reg_addr, 1, false);
    bytes_received = i2c_read_blocking(BME680_I2C_INST, device_addr, reg_data, len, false);

    return (bytes_sent <= 0) || (bytes_received <= 0);
}

/*!
 * @brief        Pico specific implementation of sleep function
 *
 * @param[in]    period           amont of microseconds to sleep
 * @param[in]    intf_ptr         a value from BME 68X dev structure which represents current interface
 *
 * @return       none
 */
void bsec_sleep_us(uint32_t period, void *intf_ptr) {
    sleep_us(period);
}

/*!
 * @brief        Pico specific implementation of timestamps
 *
 * @return       amount of microseconds passed since boot
 */
int64_t bsec_get_timestamp_us() {
    return (int64_t)time_us_64();
}

/*!
 * @brief        Saves state to NVM. Ensure interrupts are saved and disabled
 *               as there might be problems if any vector is placed in Flash.
 *               Currently dual-core is not used, but additional measures are
 *               needed to make sure that the secod core is not using flash.
 *
 * @param[in]    state_buffer     data buffer containing state to be stored
 * @param[in]    length           length in bytes of bsec state provided
 *
 * @return       none
 */
void bsec_state_save(const uint8_t *state_buffer, uint32_t length) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(BSEC_SAVED_STATE_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(BSEC_SAVED_STATE_FLASH_OFFSET, state_buffer, length);

    flash_range_erase(BSEC_SAVED_STATE_LEN_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(BSEC_SAVED_STATE_LEN_FLASH_OFFSET, (const uint8_t*)&length, sizeof(length));
    restore_interrupts(ints);

    printf("BSEC save state, len is %d\n", length);
}

/*!
 * @brief        Reads previously saved state from NVM.
 *
 * @param[in]    state_buffer     data buffer where state will be loaded
 * @param[out]   n_buffer         length in bytes of data filled in buffer
 *
 * @return       amount of bytes returned
 */
uint32_t bsec_state_load(uint8_t *state_buffer, uint32_t n_buffer) {
    const uint8_t *saved_state_len = (const uint8_t *) (BSEC_SAVED_STATE_LEN_BASE);
    const uint8_t *saved_state = (const uint8_t *) (BSEC_SAVED_STATE_BASE);

    // CHeck if provided buffer is sufficient for restoring saved state
    assert(*saved_state_len <= n_buffer);

    for (uint32_t i = 0; i < *saved_state_len; ++i) {
        state_buffer[i] = saved_state[i];
    }

    printf("BSEC load state, len is %d\n", *saved_state_len);

    // TODO: check state integrity nicely
    if (*saved_state_len != 221) {
        printf("Error while loading state, wrong state length\n");
        return 0;
    }

    return *saved_state_len;
}

/*!
 * @brief        Reads config from NVM.
 *
 * @param[in]    config_buffer    data buffer where config will be loaded
 * @param[in]    n_buffer         length in bytes of data buffer provided
 *
 * @return       amount of bytes returned
 */
uint32_t bsec_config_load(uint8_t *config_buffer, uint32_t n_buffer) {
    for (size_t i = 0; i < n_buffer; i++) {
        config_buffer[i] = bsec_config_iaq[i];
    }

    return n_buffer;
}

/*!
 * @brief        Virtual sensor subscription
 *               Please call this function before processing of data using bsec_do_steps function
 *
 * @param[in]    sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 *
 * @return       subscription result, zero when successful
 */
static bsec_library_return_t bme68x_bsec_update_subscription(float sample_rate) {
    bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
    uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;

    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

    bsec_library_return_t status = BSEC_OK;

    requested_virtual_sensors[ 0].sensor_id = BSEC_OUTPUT_IAQ;
    requested_virtual_sensors[ 0].sample_rate = sample_rate;
    requested_virtual_sensors[ 1].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[ 1].sample_rate = sample_rate;
    requested_virtual_sensors[ 2].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[ 2].sample_rate = sample_rate;
    requested_virtual_sensors[ 3].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    requested_virtual_sensors[ 3].sample_rate = sample_rate;
    requested_virtual_sensors[ 4].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[ 4].sample_rate = sample_rate;
    requested_virtual_sensors[ 5].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[ 5].sample_rate = sample_rate;
    requested_virtual_sensors[ 6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[ 6].sample_rate = sample_rate;
    requested_virtual_sensors[ 7].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[ 7].sample_rate = sample_rate;
    requested_virtual_sensors[ 8].sensor_id = BSEC_OUTPUT_STABILIZATION_STATUS;
    requested_virtual_sensors[ 8].sample_rate = sample_rate;
    requested_virtual_sensors[ 9].sensor_id = BSEC_OUTPUT_RUN_IN_STATUS;
    requested_virtual_sensors[ 9].sample_rate = sample_rate;
    requested_virtual_sensors[10].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[10].sample_rate = sample_rate;
    requested_virtual_sensors[11].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[11].sample_rate = sample_rate;
    requested_virtual_sensors[12].sensor_id = BSEC_OUTPUT_GAS_PERCENTAGE;
    requested_virtual_sensors[12].sample_rate = sample_rate;

    status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings, &n_required_sensor_settings);

    return status;
}

/*!
 * @brief       Initialize the bme68x sensor and the BSEC library
 *
 * @param[in]   sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 * @param[in]   temperature_offset  device-specific temperature offset (due to self-heating)
 *
 * @return      zero if successful, negative otherwise
 */
return_values_init bsec_iot_init(float sample_rate, float temperature_offset) {
    return_values_init ret = {BME68X_OK, BSEC_OK};

    uint8_t bsec_state[FLASH_SECTOR_SIZE] = {0};
    uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE] = {0};
    int32_t bsec_state_len, bsec_config_len;

    temperature_offset_g = temperature_offset;

    bme68x_dev_addr = BME68X_I2C_ADDR_LOW;
    bme68x_g.read  = bsec_i2c_read;
    bme68x_g.write = bsec_i2c_write;
    bme68x_g.intf  = BME68X_I2C_INTF;
    bme68x_g.delay_us = bsec_sleep_us;
    bme68x_g.intf_ptr = &bme68x_dev_addr;
    bme68x_g.amb_temp = 25; // The ambient temperature in deg C is used for defining the heater temperature

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    if (i2c_init(BME680_I2C_INST, 300 * 1000) == 0) {
        ret.bme68x_status = BME68X_E_COM_FAIL;
        return ret;
    }

    gpio_set_function(BME680_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BME680_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BME680_I2C_SDA_PIN);
    gpio_pull_up(BME680_I2C_SCL_PIN);

    ret.bme68x_status = bme68x_init(&bme68x_g);
    if (ret.bme68x_status != BME68X_OK) {
        return ret;
    }

    ret.bsec_status = bsec_init();
    if (ret.bsec_status != BSEC_OK) {
        return ret;
    }

    bsec_config_len = bsec_config_load(bsec_config, sizeof(bsec_config));
    if (bsec_config_len != 0) {
        printf("BSEC state loading ... ");
        ret.bsec_status = bsec_set_configuration(bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer));
        if (ret.bsec_status != BSEC_OK) {
            printf("%d\n", ret.bsec_status);
            return ret;
        }
        printf("OK\n");
    }

    bsec_state_len = bsec_state_load(bsec_state, sizeof(bsec_state));
    if (bsec_state_len != 0) {
        printf("BSEC state loading ... ");
        ret.bsec_status = bsec_set_state(bsec_state, bsec_state_len, work_buffer, sizeof(work_buffer));
        if (ret.bsec_status != BSEC_OK) {
            printf("%d\n", ret.bsec_status);
            return ret;
        }
        printf("OK\n");
    }

    ret.bsec_status = bme68x_bsec_update_subscription(sample_rate);
    if (ret.bsec_status != BSEC_OK) {
        return ret;
    }

    return ret;
}

/*!
 * @brief       This function is written to process the sensor data for the requested virtual sensors
 *
 * @param[in]   bsec_inputs         input structure containing the information on sensors to be passed to do_steps
 * @param[in]   num_bsec_inputs     number of inputs to be passed to do_steps
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 *
 * @return      library function return codes, zero when successful
 */
static bsec_library_return_t bme68x_bsec_process_data(bsec_input_t *bsec_inputs, uint8_t num_bsec_inputs, output_ready_fct output_ready) {
    // Output buffer set to the maximum virtual sensor outputs supported
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_bsec_outputs = 0;

    bsec_library_return_t bsec_status = BSEC_OK;

    int64_t timestamp = 0;

    float iaq = 0.0f;
    float static_iaq = 0.0f;
    float co2_equivalent = 0.0f;
    float breath_voc_equivalent = 0.0f;
    float raw_temperature = 0.0f;
    float raw_pressure = 0.0f;
    float raw_humidity = 0.0f;
    float raw_gas = 0.0f;
    bool  stabilization_status = false;
    bool  run_in_status = false;
    float sensor_heat_compensated_temperature = 0.0f;
    float sensor_heat_compensated_humidity = 0.0f;
    float gas_percentage = 0.0f;

    // Check if something should be processed by BSEC
    if (num_bsec_inputs > 0) {
        // Set number of outputs to the size of the allocated buffer
        // BSEC_NUMBER_OUTPUTS to be defined
        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;

        // Perform processing of the data by BSEC
        // Note:
        // * The number of outputs you get depends on what you asked for during bsec_update_subscription(). This is
        //   handled under bme68x_bsec_update_subscription() function in this example file.
        // * The number of actual outputs that are returned is written to num_bsec_outputs.
        bsec_status = bsec_do_steps(bsec_inputs, num_bsec_inputs, bsec_outputs, &num_bsec_outputs);

        // Iterate through the outputs and extract the relevant ones.
        for (uint8_t index = 0; index < num_bsec_outputs; index++) {
            switch (bsec_outputs[index].sensor_id) {
                case BSEC_OUTPUT_IAQ:
                    iaq = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_STATIC_IAQ:
                    static_iaq = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_CO2_EQUIVALENT:
                    co2_equivalent = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                    breath_voc_equivalent = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_TEMPERATURE:
                    raw_temperature = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_PRESSURE:
                    raw_pressure = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_HUMIDITY:
                    raw_humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_GAS:
                    raw_gas = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_STABILIZATION_STATUS:
                    stabilization_status = bsec_outputs[index].signal != 0.0f;
                    break;
                case BSEC_OUTPUT_RUN_IN_STATUS:
                    run_in_status = bsec_outputs[index].signal != 0.0f;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                    sensor_heat_compensated_temperature = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                    sensor_heat_compensated_humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_GAS_PERCENTAGE:
                    gas_percentage = bsec_outputs[index].signal;
                    break;

                default:
                    continue;
            }

            // Assume that all the returned timestamps are the same
            timestamp = bsec_outputs[index].time_stamp;
        }

        // Pass the extracted outputs to the user provided output_ready() function.
        output_ready(timestamp,
                     iaq,
                     static_iaq,
                     co2_equivalent,
                     breath_voc_equivalent,
                     raw_temperature,
                     raw_pressure,
                     raw_humidity,
                     raw_gas,
                     stabilization_status,
                     run_in_status,
                     sensor_heat_compensated_temperature,
                     sensor_heat_compensated_humidity,
                     gas_percentage,
                     bsec_status);
    }

    return bsec_status;
}

/*!
 * @brief       Read the data from registers and populate the inputs structure to be passed to do_steps function
 *
 * @param[in]   curr_time_ns            system timestamp value passed for processing data
 * @param[in]   data                    input structure that contains the gas sensor data to be passed to process data
 * @param[in]   bsec_process_data       process data variable returned from sensor_control
 * @param[in]   output_ready            pointer to the function processing obtained BSEC outputs
 *
 * @return      function result, one when successful & zero when unsuccessful
 */
uint8_t process_data(int64_t curr_time_ns, struct bme68x_data data, int32_t bsec_process_data, output_ready_fct output_ready) {
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; // Temp, Pres, Hum & Gas
    bsec_library_return_t bsec_status = BSEC_OK;
    uint8_t n_inputs = 0;

    // Checks all the required sensor inputs, required for the BSEC library for the requested outputs
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HEATSOURCE)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[n_inputs].signal = temperature_offset_g;
        inputs[n_inputs].time_stamp = curr_time_ns;
        n_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_TEMPERATURE)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
        inputs[n_inputs].signal = data.temperature;
        inputs[n_inputs].time_stamp = curr_time_ns;
        n_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HUMIDITY)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
        inputs[n_inputs].signal = data.humidity;
        inputs[n_inputs].time_stamp = curr_time_ns;
        n_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PRESSURE)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[n_inputs].signal = data.pressure;
        inputs[n_inputs].time_stamp = curr_time_ns;
        n_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_GASRESISTOR) && (data.status & BME68X_GASM_VALID_MSK)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[n_inputs].signal = data.gas_resistance;
        inputs[n_inputs].time_stamp = curr_time_ns;
        n_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PROFILE_PART) && (data.status & BME68X_GASM_VALID_MSK)) {
        inputs[n_inputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[n_inputs].signal = 0; // Always assume BME68X_FORCED_MODE
        inputs[n_inputs].time_stamp = curr_time_ns;
        n_inputs++;
    }

    if (n_inputs > 0) {
        // Processing of the input signals and returning of output samples is performed by bsec_do_steps()
        bsec_status = bme68x_bsec_process_data(inputs, n_inputs, output_ready);

        if (bsec_status != BSEC_OK) {
            return 0;
        }
    }
    return 1;
}

/*!
 * @brief       Function to get the measurement duration in microseconds
 *
 * @param[in]   mode                sensor operation mode to calculate the shared heater duration
 *
 * @return      calculated duration
 */
uint32_t get_meas_dur(uint8_t mode) {
    if (mode == BME68X_SLEEP_MODE) {
        mode = last_op_mode;
    }

    return bme68x_get_meas_dur(mode, &bme68x_conf, &bme68x_g);
}

/**
 * @brief Set the BME68X sensor configuration to forced mode
 *
 * @param[in]   sensor_settings     settings of the bme68x sensor adopted by sensor control function
 *
 * @return      none
 */
void set_bme68x_config_forced(bsec_bme_settings_t *sensor_settings) {
    int8_t status;

    // Set the filter, odr, temperature, pressure and humidity settings
    status = bme68x_get_conf(&bme68x_conf, &bme68x_g);
    if (status != BME68X_OK) {
        return;
    }

    bme68x_conf.os_hum = sensor_settings->humidity_oversampling;
    bme68x_conf.os_temp = sensor_settings->temperature_oversampling;
    bme68x_conf.os_pres = sensor_settings->pressure_oversampling;
    status = bme68x_set_conf(&bme68x_conf, &bme68x_g);
    if (status != BME68X_OK) {
        return;
    }

    bme68x_heatr_conf.enable = BME68X_ENABLE;
    bme68x_heatr_conf.heatr_temp = sensor_settings->heater_temperature;
    bme68x_heatr_conf.heatr_dur = sensor_settings->heater_duration;
    status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme68x_heatr_conf, &bme68x_g);
    if (status != BME68X_OK) {
        return;
    }

    status = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x_g);
    if (status != BME68X_OK) {
        return;
    }

    last_op_mode = BME68X_FORCED_MODE;
    op_mode = BME68X_FORCED_MODE;
}

/*!
 * @brief       Runs the main (endless) loop that queries sensor settings, applies them, and processes the measured data
 *
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 * @param[in]   save_intvl          interval at which BSEC state should be saved (in samples)
 *
 * @return      none
 */
void bsec_iot_loop(output_ready_fct output_ready, uint32_t save_intvl) {
    int64_t time_stamp = 0;
    bsec_bme_settings_t sensor_settings;
    memset(&sensor_settings, 0, sizeof(sensor_settings));

    uint8_t bsec_state[FLASH_SECTOR_SIZE];
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;
    int8_t ret_val;

    bsec_library_return_t bsec_status = BSEC_OK;
    sensor_settings.next_call = 0;

    while (1) {
        // get the timestamp in nanoseconds before calling bsec_sensor_control()
        time_stamp = bsec_get_timestamp_us() * 1000;

        if (time_stamp >= sensor_settings.next_call) {

            // Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control
            bsec_status = bsec_sensor_control(time_stamp, &sensor_settings);
            if (bsec_status != BSEC_OK) {
                if (bsec_status < BSEC_OK) {
                    printf("ERROR: bsec_sensor_control: %d\n", bsec_status);
                    break;
                } else {
                    printf("WARNING: bsec_sensor_control: %d\n", bsec_status);
                }
            }

            switch (sensor_settings.op_mode) {
            case BME68X_FORCED_MODE:
                set_bme68x_config_forced(&sensor_settings);
                break;
            case BME68X_PARALLEL_MODE:
                printf("ERROR: bsec_sensor_control: BME68X_PARALLEL_MODE is not supported\n");
                break;
            case BME68X_SLEEP_MODE:
                if (op_mode != sensor_settings.op_mode) {
                    ret_val = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme68x_g);
                    if ((ret_val == BME68X_OK) && (op_mode != BME68X_SLEEP_MODE)) {
                        op_mode = BME68X_SLEEP_MODE;
                    }
                }
                break;
            }

            if (sensor_settings.trigger_measurement && sensor_settings.op_mode != BME68X_SLEEP_MODE) {
                uint8_t sensor_data_valid;

                bme68x_get_data(last_op_mode, &bme68x_sensor_data, &sensor_data_valid, &bme68x_g);
                if (sensor_data_valid) {
                    // Discard sensor data where gas measurements are not valid
                    if (bme68x_sensor_data.status & BME68X_GASM_VALID_MSK) {
                        if (!process_data(time_stamp, bme68x_sensor_data, sensor_settings.process_data, output_ready)) {
                            return;
                        }
                    }
                }
            }

            printf("\t%d: ", n_samples);
            n_samples++;

            // Save state regularly
            if (n_samples >= save_intvl) {
                bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
                if (bsec_status == BSEC_OK) {
                    bsec_state_save(bsec_state, bsec_state_len);
                }
                n_samples = 0;
            }
        } else {
            int64_t sleep_period_us = (sensor_settings.next_call - time_stamp) / 1000LL;
            sleep_us(sleep_period_us);
        }
    }
}
