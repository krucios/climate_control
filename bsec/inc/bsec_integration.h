#ifndef BSEC_INTEGRATION_H
#define BSEC_INTEGRATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "bme68x.h"

#include "bsec_interface.h"
#include "bsec_datatypes.h"

#define BME680_I2C_INST    (i2c1)
#define BME680_I2C_SCL_PIN (27)
#define BME680_I2C_SDA_PIN (26)

#define BSEC_CHECK_INPUT(x, shift)		(x & (1 << (shift-1)))

/* function pointer to the system specific sleep function */
typedef void (*sleep_fct)(uint32_t t_us,void *intf_ptr);

/* function pointer to the system specific timestamp derivation function */
typedef int64_t (*get_timestamp_us_fct)();

/* function pointer to the function processing obtained BSEC outputs */
typedef void (*output_ready_fct)(int64_t timestamp,
                                 int32_t iaq,
                                 int32_t static_iaq,
                                 int32_t co2_equivalent,
                                 int32_t breath_voc_equivalent,
                                 int32_t raw_temperature,
                                 int32_t raw_pressure,
                                 int32_t raw_humidity,
                                 int32_t raw_gas,
                                 bool stabilization_status,
                                 bool run_in_status,
                                 int32_t sensor_heat_compensated_temperature,
                                 int32_t sensor_heat_compensated_humidity,
                                 int32_t gas_percentage,
                                 bsec_library_return_t bsec_status);

/* function pointer to the function loading a previous BSEC state from NVM */
typedef uint32_t (*state_load_fct)(uint8_t *state_buffer, uint32_t n_buffer);

/* function pointer to the function saving BSEC state to NVM */
typedef void (*state_save_fct)(const uint8_t *state_buffer, uint32_t length);

/* function pointer to the function loading the BSEC configuration string from NVM */
typedef uint32_t (*config_load_fct)(uint8_t *state_buffer, uint32_t n_buffer);

/* structure definitions */

/* Structure with the return value from bsec_iot_init() */
typedef struct{
    /*! Result of API execution status */
    int8_t bme68x_status;
    /*! Result of BSEC library */
    bsec_library_return_t bsec_status;
} return_values_init;

/*!
 * @brief       Initialize the bme68x sensor and the BSEC library
 *
 * @param[in]   sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 * @param[in]   temperature_offset  device-specific temperature offset (due to self-heating)
 *
 * @return      zero if successful, negative otherwise
 */
return_values_init bsec_iot_init(float sample_rate, float temperature_offset);

/*!
 * @brief       Runs the main (endless) loop that queries sensor settings, applies them, and processes the measured data
 *
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 * @param[in]   save_intvl          interval at which BSEC state should be saved (in samples)
 *
 * @return      return_values_init	struct with the result of the API and the BSEC library
 */
void bsec_iot_loop(output_ready_fct output_ready, uint32_t save_intvl);

#ifdef __cplusplus
}
#endif

#endif // BSEC_INTEGRATION_H
