#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "bsec_integration.h"

#include "climate_tracker.h"

uint16_t climate_tracker_notifications_enabled[CT_ID_COUNT] = {0};
bool climate_tracker_active = false;
bool climate_tracker_low_battery = false;

float climate_tracker_current_temperature = 0.0f;
float climate_tracker_relative_humidity = 0.0f;
uint8_t climate_tracker_air_quality = 1;
float climate_tracker_voc_density = 0.0f;

void bsec_outputs_ready(int64_t timestamp,
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
                        bsec_library_return_t bsec_status) {
    climate_tracker_current_temperature = sensor_heat_compensated_temperature;
    climate_tracker_relative_humidity = sensor_heat_compensated_humidity;
    climate_tracker_air_quality = (uint8_t)((iaq * 0.8f) / 100) + 1; // Scale 0 - 500 to 1 - 4
    climate_tracker_voc_density = breath_voc_equivalent;

    climate_tracker_active = stabilization_status;

    printf("IAQ(%3.0f), ", iaq);
    printf("sIAQ(%3.0f), ", static_iaq);
    printf("CO2(%4.0f), ", co2_equivalent);
    printf("VOC(%4.0f), ", breath_voc_equivalent);
    printf("T(%2.1f), ", raw_temperature);
    printf("P(%7.0f), ", raw_pressure);
    printf("H(%2.1f), ", raw_humidity);
    printf("G(%6.0f), ", raw_gas);
    printf("%sstb, ", stabilization_status ? "" : "n");
    printf("%srun, ", run_in_status ? "" : "n");
    printf("Tcomp(%2.1f), ", sensor_heat_compensated_temperature);
    printf("Hcomp(%2.1f), ", sensor_heat_compensated_humidity);
    printf("G\%(%6.0f), ", gas_percentage);
    printf("bsec(%d)\n", bsec_status);
}

void climate_tracker_init() {
    // TODO: adjust sample rate for ULP
    bsec_iot_init(BSEC_SAMPLE_RATE_CONT, 0.0f);
}

bool climate_tracker_any_notification_enabled() {
    bool any_enabled = false;

    for (size_t i = 0; i < CT_ID_COUNT; i++) {
        any_enabled |= climate_tracker_notifications_enabled[i];
    }

    return any_enabled;
}

void climate_tracker_thread() {
    // TODO: adjust save state interval for low-power (maybe once a day)
    bsec_iot_loop(bsec_outputs_ready, 100);
}