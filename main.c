#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "btstack.h"
#include "sensor.h"
#include "bt_services/iaq.h"
#include "bsec_integration.h"

//
// BLE has a concept of connection timeout when in connection. The idea
// is to not exceed it and send something to reset that timeout. THat's
// why HEARTBEAT_PERIOD_MS shouldn't exceed ~32 sec.
// https://punchthrough.com/manage-ble-connection/
//
#define HEARTBEAT_PERIOD_MS (10 * 1000)

#define APP_AD_FLAGS 0x06 // LE only (0x4), general discoverable mdoe (0x2)

static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t adv_data[] = {
    // Flags
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x0c, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'I', 'A', 'Q', ' ', 't', 'r', 'a', 'c', 'k', 'e', 'r'
};
static const uint8_t adv_data_len = sizeof(adv_data);
static hci_con_handle_t con_handle;

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch(event_type){
        // Wait until BTSTACK is initialised and only then start advertisement and initial reading
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) {
                return;
            }

            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // Setup advertisements
            uint16_t adv_int_min = 800;
            uint16_t adv_int_max = 800;
            uint8_t adv_type = 0; // ADV_IND: Connectable undirected advertising
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
            assert(adv_data_len <= 31); // BLE limitation

            gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
            gap_advertisements_enable(1);

            break;

        // No need to send notifications if a reader disconnected
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("Client has disconnected\n");
            memset(iaq_notifications_enabled, 0, sizeof(iaq_notifications_enabled));
            break;

        // Send notifications when it's time to do so
        case ATT_EVENT_CAN_SEND_NOW:
            if (iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000011_0000_1000_8000_0026BB765291_01_VALUE_HANDLE, (uint8_t*)&iaq_current_temperature, sizeof(iaq_current_temperature));
            }
            if (iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_ACTIVE]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_01_VALUE_HANDLE, (uint8_t*)&iaq_active, sizeof(iaq_active));
            }
            if (iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_LOW_BATTERY]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_01_VALUE_HANDLE, (uint8_t*)&iaq_low_battery, sizeof(iaq_low_battery));
            }
            if (iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000010_0000_1000_8000_0026BB765291_01_VALUE_HANDLE, (uint8_t*)&iaq_relative_humidity, sizeof(iaq_relative_humidity));
            }
            if (iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_ACTIVE]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_02_VALUE_HANDLE, (uint8_t*)&iaq_active, sizeof(iaq_active));
            }
            if (iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_LOW_BATTERY]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_02_VALUE_HANDLE, (uint8_t*)&iaq_low_battery, sizeof(iaq_low_battery));
            }
            if (iaq_notifications_enabled[IAQ_AIR_QUALITY]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000095_0000_1000_8000_0026BB765291_01_VALUE_HANDLE, (uint8_t*)&iaq_air_quality, sizeof(iaq_air_quality));
            }
            if (iaq_notifications_enabled[IAQ_VOC_DENSITY]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_000000C8_0000_1000_8000_0026BB765291_01_VALUE_HANDLE, (uint8_t*)&iaq_voc_density, sizeof(iaq_voc_density));
            }
            if (iaq_notifications_enabled[IAQ_AIR_QUALITY_ACTIVE]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_03_VALUE_HANDLE, (uint8_t*)&iaq_active, sizeof(iaq_active));
            }
            if (iaq_notifications_enabled[IAQ_AIR_QUALITY_LOW_BATTERY]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_03_VALUE_HANDLE, (uint8_t*)&iaq_low_battery, sizeof(iaq_low_battery));
            }

            break;

        default:
            break;
    }
}

uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    //
    // Handle reads to client configuration characteristic descriptor as they are not handled by BTstack and
    // might cause confusion on other side. An example: read of this descriptor return nothing so receiving
    // end reported nothing to the central application. That might crash the application as it's likely to
    // expect proper behaviour of CCC which is described in Core spec.
    //
    switch (att_handle) {
        // Temperature service
        case ATT_CHARACTERISTIC_00000011_0000_1000_8000_0026BB765291_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_current_temperature, sizeof(iaq_current_temperature), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000011_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE], sizeof(iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_active, sizeof(iaq_active), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_ACTIVE], sizeof(iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_ACTIVE]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_low_battery, sizeof(iaq_low_battery), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_LOW_BATTERY], sizeof(iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_LOW_BATTERY]), offset, buffer, buffer_size);

        // Humidity service
        case ATT_CHARACTERISTIC_00000010_0000_1000_8000_0026BB765291_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_relative_humidity, sizeof(iaq_relative_humidity), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000010_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY], sizeof(iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_02_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_active, sizeof(iaq_active), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_02_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_ACTIVE], sizeof(iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_ACTIVE]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_02_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_low_battery, sizeof(iaq_low_battery), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_02_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_LOW_BATTERY], sizeof(iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_LOW_BATTERY]), offset, buffer, buffer_size);

        // Air Quality service
        case ATT_CHARACTERISTIC_00000095_0000_1000_8000_0026BB765291_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_air_quality, sizeof(iaq_air_quality), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000095_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_AIR_QUALITY], sizeof(iaq_notifications_enabled[IAQ_AIR_QUALITY]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_000000C8_0000_1000_8000_0026BB765291_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_voc_density, sizeof(iaq_voc_density), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_000000C8_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_VOC_DENSITY], sizeof(iaq_notifications_enabled[IAQ_VOC_DENSITY]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_03_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_active, sizeof(iaq_active), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_03_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_AIR_QUALITY_ACTIVE], sizeof(iaq_notifications_enabled[IAQ_AIR_QUALITY_ACTIVE]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_03_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_low_battery, sizeof(iaq_low_battery), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_03_CLIENT_CONFIGURATION_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *)&iaq_notifications_enabled[IAQ_AIR_QUALITY_LOW_BATTERY], sizeof(iaq_notifications_enabled[IAQ_AIR_QUALITY_LOW_BATTERY]), offset, buffer, buffer_size);

    }

    return 0;
}

int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);

    uint16_t value = little_endian_read_16(buffer, 0);
    bool is_notifications_enabled = value == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;

    switch (att_handle) {
        case ATT_CHARACTERISTIC_00000011_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_ACTIVE] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_CURRENT_TEMPERATURE_LOW_BATTERY] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000010_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_02_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_ACTIVE] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_02_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_RELATIVE_HUMIDITY_LOW_BATTERY] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000095_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_AIR_QUALITY] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_000000C8_0000_1000_8000_0026BB765291_01_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_VOC_DENSITY] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000075_0000_1000_8000_0026BB765291_03_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_AIR_QUALITY_ACTIVE] = is_notifications_enabled;
            break;
        case ATT_CHARACTERISTIC_00000079_0000_1000_8000_0026BB765291_03_CLIENT_CONFIGURATION_HANDLE:
            iaq_notifications_enabled[IAQ_AIR_QUALITY_LOW_BATTERY] = is_notifications_enabled;
            break;
    }

    if (iaq_any_notification_enabled()) {
        con_handle = connection_handle; // To later use inside heartbeat function
        att_server_request_can_send_now_event(connection_handle);
    }

    return 0;
}

static void heartbeat_handler(struct btstack_timer_source *ts) {
    if (iaq_any_notification_enabled()) {
        att_server_request_can_send_now_event(con_handle);
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    sleep_ms(10);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    // Restart timer
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

int main() {
    stdio_init_all();

    printf("Initial wait to connect serial monitor\n");
    sleep_ms(5000);
    printf("Let's go!\n");

    iaq_init();

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    l2cap_init();
    sm_init();

    att_server_init(profile_data, att_read_callback, att_write_callback);

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(packet_handler);

    // set one-shot btstack timer
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);

    // btstack_run_loop_execute is only required when using the 'polling' method (e.g. using pico_cyw43_arch_poll library).
    // This example uses the 'threadsafe background` method, where BT work is handled in a low priority IRQ, so it
    // is fine to call bt_stack_run_loop_execute() but equally you can continue executing user code.

#if 0 // btstack_run_loop_execute() is not required, so lets not use it
    btstack_run_loop_execute();
#else
    // this core is free to do it's own stuff except when using 'polling' method (in which case you should use
    // btstacK_run_loop_ methods to add work to the run loop.

    // this is a forever loop in place of where user code would go.
    iaq_thread();
#endif
    return 0;
}
