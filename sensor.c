#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "btstack.h"
#include "sensor.h"
#include "bt_services/ess.h"

#define HEARTBEAT_PERIOD_MS 1000
#define APP_AD_FLAGS 0x06 // LE only (0x4), general discoverable mdoe (0x2)

static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t adv_data[] = {
    // Flags
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x0d, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', 'c', 'l', 'i', 'm', 'a', 't', 'e',
    // Advertised services, no need to list all, just 'main' service to let
    // clients know about the main purpose of the device
    0x03, BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
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

            ess_read_data();

            break;

        // No need to send notifications if a reader disconnected
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("Client has disconnected\n");
            memset(ess_notifications_enabled, 0, sizeof(ess_notifications_enabled));
            break;

        // Send notifications when it's time to do so
        case ATT_EVENT_CAN_SEND_NOW:
            printf("Send notification\n");

            if (ess_notifications_enabled[ESS_TEMPERATURE]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE, (uint8_t*)&ess_current_values[ESS_TEMPERATURE], sizeof(ess_value_t));
            }
            if (ess_notifications_enabled[ESS_PRESSURE]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_PRESSURE_01_VALUE_HANDLE, (uint8_t*)&ess_current_values[ESS_PRESSURE], sizeof(ess_value_t));
            }
            if (ess_notifications_enabled[ESS_HUMIDITY]) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_HUMIDITY_01_VALUE_HANDLE, (uint8_t*)&ess_current_values[ESS_HUMIDITY], sizeof(ess_value_t));
            }

            break;

        default:
            break;
    }
}

uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);

    printf("ATT read callback: for att handle 0x%0x\n", att_handle);

    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&ess_current_values[ESS_TEMPERATURE], sizeof(ess_value_t), offset, buffer, buffer_size);
    } else if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_PRESSURE_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&ess_current_values[ESS_PRESSURE], sizeof(ess_value_t), offset, buffer, buffer_size);
    } else if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_HUMIDITY_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&ess_current_values[ESS_HUMIDITY], sizeof(ess_value_t), offset, buffer, buffer_size);
    }
    return 0;
}

int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    UNUSED(buffer_size);

    uint16_t value = little_endian_read_16(buffer, 0);
    bool is_notifications_enabled = value == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;

    printf("ATT write callback: for att handle 0x%0x, value - 0x%0x\n", att_handle, value);

    if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE) {
        ess_notifications_enabled[ESS_TEMPERATURE] = is_notifications_enabled;
    } else if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_PRESSURE_01_CLIENT_CONFIGURATION_HANDLE) {
        ess_notifications_enabled[ESS_PRESSURE] = is_notifications_enabled;
    } else if (att_handle == ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_HUMIDITY_01_CLIENT_CONFIGURATION_HANDLE) {
        ess_notifications_enabled[ESS_HUMIDITY] = is_notifications_enabled;
    }

    if (ess_any_notification_enabled()) {
        con_handle = connection_handle; // To later use inside heartbeat function
        att_server_request_can_send_now_event(connection_handle);
    }

    return 0;
}

static void heartbeat_handler(struct btstack_timer_source *ts) {
    static uint32_t counter = 0;
    counter++;

    // Update the temp every 3s
    if (counter % 3 == 0) {
        ess_read_data();

        if (ess_any_notification_enabled()) {
            att_server_request_can_send_now_event(con_handle);
        }
    }

    // Invert the led
    static int led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    // Restart timer
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

int main() {
    stdio_init_all();

    printf("Initial wait to connect serial monitor\n");
    sleep_ms(5000);
    printf("Let's go!\n");

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
    while(true) {
        sleep_ms(1000);
    }
#endif
    return 0;
}
