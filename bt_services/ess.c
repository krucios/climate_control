#include "ess.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

bool ess_notifications_enabled[ESS_ID_COUNT] = {0};
ess_value_t ess_current_values[ESS_ID_COUNT] = {0};

void ess_init() {
    // Initialise adc for the temp sensor
    adc_init();
    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    adc_set_temp_sensor_enabled(true);
}

bool ess_any_notification_enabled() {
    bool any_enabled = false;

    printf("Notifications settings:");

    for (size_t i = 0; i < ESS_ID_COUNT; i++) {
        printf(" %0d", ess_notifications_enabled[i]);
        any_enabled |= ess_notifications_enabled[i];
    }

    printf(" - %0d\n", any_enabled);

    return any_enabled;
}

void ess_read_data(void) {
    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    uint32_t raw32 = adc_read();
    const uint32_t bits = 12;

    // Scale raw reading to 16 bit value using a Taylor expansion (for 8 <= bits <= 16)
    uint16_t raw16 = raw32 << (16 - bits) | raw32 >> (2 * bits - 16);

    // ref https://github.com/raspberrypi/pico-micropython-examples/blob/master/adc/temperature.py
    const float conversion_factor = 3.3 / (65535);
    float reading = raw16 * conversion_factor;

    // The temperature sensor measures the Vbe voltage of a biased bipolar diode, connected to the fifth ADC channel
    // Typically, Vbe = 0.706V at 27 degrees C, with a slope of -1.721mV (0.001721) per degree.
    float deg_c = 27 - (reading - 0.706) / 0.001721;
    ess_current_values[ESS_TEMPERATURE] = deg_c * 100;

    // TODO: use actual values
    ess_current_values[ESS_PRESSURE] = deg_c * 25;
    ess_current_values[ESS_HUMIDITY] = deg_c * 50;

    printf("Update ESS data: temp %2f\n", deg_c);
}