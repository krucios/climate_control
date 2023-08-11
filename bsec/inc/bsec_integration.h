#ifndef BSEC_INTEGRATION_H
#define BSEC_INTEGRATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "hardware/flash.h"

#include "bme68x.h"

#include "bsec_interface.h"
#include "bsec_datatypes.h"

#define BME680_I2C_INST    (i2c1)
#define BME680_I2C_SCL_PIN (27)
#define BME680_I2C_SDA_PIN (26)

// Keep saved state at the end of Flash to not interfere with the program
// if this area gets overwritten by programm then we have bigger problems
// as the program doesn't fit into provided flash.
// Use one flash sector for storage as it's the minimal size which can be
// erased at a time.
#define BSEC_SAVED_STATE_LEN_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - 2 * FLASH_SECTOR_SIZE)
#define BSEC_SAVED_STATE_LEN_BASE (XIP_BASE + BSEC_SAVED_STATE_LEN_FLASH_OFFSET)

#define BSEC_SAVED_STATE_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define BSEC_SAVED_STATE_BASE (XIP_BASE + BSEC_SAVED_STATE_FLASH_OFFSET)

static_assert(FLASH_SECTOR_SIZE >= BSEC_MAX_PROPERTY_BLOB_SIZE, "Can't fit BSEC state into one flash sector, consider increasing allocated flash storage for BSEC");

#define BSEC_CHECK_INPUT(x, shift) (x & (1 << (shift-1)))

/* function pointer to the system specific sleep function */
typedef void (*sleep_fct)(uint32_t t_us,void *intf_ptr);

/* function pointer to the system specific timestamp derivation function */
typedef int64_t (*get_timestamp_us_fct)();

/* function pointer to the function processing obtained BSEC outputs */
typedef void (*output_ready_fct)(int64_t timestamp,
                                 float iaq,
                                 float static_iaq,
                                 float co2_equivalent,
                                 float breath_voc_equivalent,
                                 float raw_temperature,
                                 float raw_pressure,
                                 float raw_humidity,
                                 float raw_gas,
                                 bool  stabilization_status,
                                 bool  run_in_status,
                                 float sensor_heat_compensated_temperature,
                                 float sensor_heat_compensated_humidity,
                                 float gas_percentage,
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
 * @return      return_values_init  struct with the result of the API and the BSEC library
 */
void bsec_iot_loop(output_ready_fct output_ready, uint32_t save_intvl);

#ifdef __cplusplus
}
#endif

#endif // BSEC_INTEGRATION_H
