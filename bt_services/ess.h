#ifndef ESS_H
#define ESS_H

#include <pico/stdlib.h>

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
void ess_thread();

#endif // ESS_H