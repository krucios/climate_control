#ifndef CLIMATE_TRACKER_H
#define CLIMATE_TRACKER_H

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
    CT_CURRENT_TEMPERATURE = 0,
    CT_CURRENT_TEMPERATURE_ACTIVE,
    CT_CURRENT_TEMPERATURE_LOW_BATTERY,

    // Humidity service
    CT_RELATIVE_HUMIDITY,
    CT_RELATIVE_HUMIDITY_ACTIVE,
    CT_RELATIVE_HUMIDITY_LOW_BATTERY,

    // Air Quality service
    CT_AIR_QUALITY,
    CT_VOC_DENSITY,
    CT_AIR_QUALITY_ACTIVE,
    CT_AIR_QUALITY_LOW_BATTERY,

    CT_ID_COUNT
} climate_tracker_characteristic_id_t;

extern uint16_t climate_tracker_notifications_enabled[CT_ID_COUNT];

// IMPORTANT: make sure the type of storage matches related characteristic
// specification on https://developers.homebridge.io/#/service/

// Active and low battery flags are applicable for all services, so keep one
// copy of flags for them all as these are not supposed to change independently.
extern bool climate_tracker_active;
extern bool climate_tracker_low_battery;

extern float   climate_tracker_current_temperature;
extern float   climate_tracker_relative_humidity;
extern uint8_t climate_tracker_air_quality;
extern float   climate_tracker_voc_density;

void climate_tracker_init();
bool climate_tracker_any_notification_enabled();
void climate_tracker_thread();

#endif // CLIMATE_TRACKER_H