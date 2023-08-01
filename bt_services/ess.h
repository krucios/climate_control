#ifndef ESS_H
#define ESS_H

#include <pico/stdlib.h>

#define BME680_I2C_INST    (i2c1)
#define BME680_I2C_SCL_PIN (27)
#define BME680_I2C_SDA_PIN (26)

// Important, don't assign values to enum entries as this will break ID_COUNT
// logic which assumes counting is done from 0
typedef enum {
    ESS_TEMPERATURE = 0,
    ESS_PRESSURE,
    ESS_HUMIDITY,
    ESS_ID_COUNT
} ess_characteristic_id_t;

typedef uint16_t ess_value_t;

extern bool ess_notifications_enabled[ESS_ID_COUNT];
extern ess_value_t ess_current_values[ESS_ID_COUNT];

void ess_init();
bool ess_any_notification_enabled();
void ess_read_data(void);

#endif // ESS_H