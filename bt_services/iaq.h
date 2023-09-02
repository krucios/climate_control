#ifndef IAQ_H
#define IAQ_H

#include <pico/stdlib.h>

//
// This is rather a combination of services than a separate BLE service.
// It includes required data and methods to support some Homebridge's
// (Apple HomeKit) services - https://developers.homebridge.io/#/service/
// The main idea is to extract data from BME680, run it through BSEC to
// get additional data and then map them to as much applicable services
// as possible. For this reasons, next services were selected:
// * Air Quality Sensor
// * Humidity Sensor
// * Temperature Sensor
//

// Important, don't assign values to enum entries as this will break ID_COUNT
// logic which assumes counting is done from 0
typedef enum {
    // Temperature service
    IAQ_CURRENT_TEMPERATURE = 0,
    IAQ_CURRENT_TEMPERATURE_ACTIVE,
    IAQ_CURRENT_TEMPERATURE_LOW_BATTERY,

    // Humidity service
    IAQ_RELATIVE_HUMIDITY,
    IAQ_RELATIVE_HUMIDITY_ACTIVE,
    IAQ_RELATIVE_HUMIDITY_LOW_BATTERY,

    // Air Quality service
    IAQ_AIR_QUALITY,
    IAQ_VOC_DENSITY,
    IAQ_AIR_QUALITY_ACTIVE,
    IAQ_AIR_QUALITY_LOW_BATTERY,

    IAQ_ID_COUNT
} iaq_characteristic_id_t;

extern uint16_t iaq_notifications_enabled[IAQ_ID_COUNT];

// IMPORTANT: make sure the type of storage matches related characteristic
// specification on https://developers.homebridge.io/#/service/

// Active and low battery flags are applicable for all services, so keep one
// copy of flags for them all as these are not supposed to change independently.
extern bool iaq_active;
extern bool iaq_low_battery;

extern float   iaq_current_temperature;
extern float   iaq_relative_humidity;
extern uint8_t iaq_air_quality;
extern float   iaq_voc_density;

void iaq_init();
bool iaq_any_notification_enabled();
void iaq_thread();

#endif // IAQ_H