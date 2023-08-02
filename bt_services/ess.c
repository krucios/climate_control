#include "ess.h"
#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "bsec_integration.h"

bool ess_notifications_enabled[ESS_ID_COUNT] = {0};
ess_value_t ess_current_values[ESS_ID_COUNT] = {0};

void bsec_outputs_ready(int64_t timestamp,
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
                        bsec_library_return_t bsec_status) {
    ess_current_values[ESS_TEMPERATURE] = raw_temperature;
    ess_current_values[ESS_PRESSURE] = raw_pressure;
    ess_current_values[ESS_HUMIDITY] = raw_humidity;

    printf("IAQ(%d), ", iaq);
    printf("sIAQ(%d), ", static_iaq);
    printf("CO2(%d), ", co2_equivalent);
    printf("VOC(%d), ", breath_voc_equivalent);
    printf("T(%d), ", raw_temperature);
    printf("P(%d), ", raw_pressure);
    printf("H(%d), ", raw_humidity);
    printf("G(%d), ", raw_gas);
    printf("%sstb, ", stabilization_status ? "" : "n");
    printf("%srun, ", run_in_status ? "" : "n");
    printf("Tcomp(%d), ", sensor_heat_compensated_temperature);
    printf("Hcomp(%d), ", sensor_heat_compensated_humidity);
    printf("G\%(%d), ", gas_percentage);
    printf("bsec(%d)\n", bsec_status);
}

void ess_init() {
    bsec_iot_init(BSEC_SAMPLE_RATE_CONT, 0.0f);
}

bool ess_any_notification_enabled() {
    bool any_enabled = false;

    for (size_t i = 0; i < ESS_ID_COUNT; i++) {
        any_enabled |= ess_notifications_enabled[i];
    }

    return any_enabled;
}

void ess_thread() {
    bsec_iot_loop(bsec_outputs_ready, 100);
}