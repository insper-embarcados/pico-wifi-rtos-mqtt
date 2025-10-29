/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" // needed to set hostname
#include "lwip/dns.h"

#define WIFI_SSID "Arnaldojr"
#define WIFI_PASSWORD "12345678"
#define MQTT_SERVER "broker.hivemq.com"

#define MQTT_TOPIC_LEN 100

// Structure to hold MQTT client state
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[800];  // Simplified from MQTT_OUTPUT_RINGBUF_SIZE
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
    bool mqtt_connected;  // Add flag to track MQTT connection status
} MQTT_CLIENT_DATA_T;

// Temperature
#define TEMPERATURE_UNITS 'C' // Set to 'F' for Fahrenheit

// how often to measure our temperature
#define TEMP_WORKER_TIME_S 10

// keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// qos passed to mqtt_subscribe
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// topic used for last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#define MQTT_DEVICE_NAME "pico"

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
static float read_onboard_temperature(const char unit) {
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        printf("pub_request_cb failed %d\n", err);
    }
}

static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
}

static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
    if (temperature != old_temperature) {
        old_temperature = temperature;
        // Publish temperature on /temperature topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        printf("Publishing %s to %s\n", temp_str, temperature_key);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        printf("subscribe request failed %d\n", err);
        return;
    }
    state->subscribe_count++;
}

static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        printf("unsubscribe request failed %d\n", err);
        return;
    }
    state->subscribe_count--;
    if (state->subscribe_count < 0) state->subscribe_count = 0;

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        state->mqtt_connected = false; // Update connection status
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    const char *basic_topic = state->topic;
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/led") == 0)
    {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/print") == 0) {
        printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT Connected\n");
        state->connect_done = true;
        state->mqtt_connected = true; // Update connection status
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        printf("MQTT Disconnected\n");
        state->mqtt_connected = false; // Update connection status
        if (!state->connect_done) {
            printf("Failed to connect to mqtt server\n");
        }
    }
    else {
        printf("MQTT connection error: %d\n", status);
        state->mqtt_connected = false; // Update connection status
    }
}

static void start_client(MQTT_CLIENT_DATA_T *state) {
    const int port = MQTT_PORT;  // Simplified: removed TLS support for clarity
    printf("Connecting to MQTT server\n");

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        printf("MQTT client instance creation error\n");
        return;
    }
    printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        printf("MQTT broker connection error\n");
        cyw43_arch_lwip_end();
        return;
    }
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Task to handle WiFi connection and management
void wifi_task(void *args) {
    printf("WiFi Task Started\n");
    
    cyw43_arch_enable_sta_mode();
    
    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect to WiFi\n");
        // Try to reconnect indefinitely
        while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
        {
            printf("Retrying WiFi connection in 5 seconds...\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    
    printf("\nConnected to WiFi\n");

    // WiFi task loop - periodically check connection and perform tasks
    for (;;)
    {
        // Process any pending cyw43 work
        cyw43_arch_poll();
        
        // Check WiFi connection status and reconnect if needed
        if (!cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA))
        {
            printf("WiFi connection lost, attempting to reconnect...\n");
            if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) == 0)
            {
                printf("Reconnected to WiFi\n");
            }
            else
            {
                printf("Failed to reconnect to WiFi\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Check every 10 seconds
    }       

    (void)args;
}

// Task to handle MQTT operations
void mqtt_task(void *args) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)args;
    
    printf("MQTT Task Started\n");
    
    // MQTT task loop - handle MQTT operations and reconnection
    for (;;) {
        // Check if WiFi is connected before attempting MQTT connection
        bool wifi_connected = (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP);
        
        if (!wifi_connected) {
            printf("WiFi not connected, waiting before MQTT connection attempt\n");
            vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before retrying
            continue;
        }
        
        // If WiFi is connected but MQTT is not, try to connect
        if (!state->mqtt_connected) {
            // Make DNS request for MQTT server
            cyw43_arch_lwip_begin();
            int err = dns_gethostbyname(MQTT_SERVER, &state->mqtt_server_address, NULL, NULL);
            cyw43_arch_lwip_end();
            
            if (err == ERR_OK) {
                // We have the address immediately, just start the client
                start_client(state);
            } else if (err == ERR_INPROGRESS) {
                // DNS in progress, we'll need to handle the callback to connect
                // For now, we'll try to connect in the next iteration
                printf("DNS lookup in progress, will connect once address is resolved\n");
            } else {
                printf("DNS request failed, will retry...\n");
            }
        }
        
        // If MQTT is connected, perform periodic tasks
        if (state->mqtt_connected) {
            publish_temperature(state);
            
            // Check if connection is still alive
            if (!mqtt_client_is_connected(state->mqtt_client_inst)) {
                state->mqtt_connected = false;
                state->connect_done = false;
                printf("MQTT connection lost, attempting reconnection...\n");
            }
        }
        
        // Process any pending cyw43 work
        cyw43_arch_poll();
        
        // Delay to prevent excessive CPU usage
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
    
    (void)args;
}

// Task to handle sensor reading and other RTOS work
void sensor_task(void *args) {
    // Initialize ADC for temperature reading
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    while(true) {
        // Read temperature
        const uint16_t adc = adc_read();
        const float conversionFactor = 3.3f / (1 << 12);
        float tempC = 27.0f - (adc * conversionFactor - 0.706f) / 0.001721f;
        printf("Temperature: %.2f°C\n", tempC);

        // Add other RTOS tasks here (sensors, control logic, etc.)
        
        // Wait for 30 seconds before next reading
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
    
    (void)args;
}

int main(void) {
    stdio_init_all();
    printf("MQTT FreeRTOS client starting\n");

    if (cyw43_arch_init()) 
    {
        printf("Failed to initialize CYW43\n");
        return -1;
    }


    static MQTT_CLIENT_DATA_T mqtt_state;
    memset(&mqtt_state, 0, sizeof(mqtt_state)); 

    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Generate a unique name, e.g. pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    printf("Device name %s\n", client_id_buf);

    mqtt_state.mqtt_client_info.client_id = client_id_buf;
    mqtt_state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
    mqtt_state.mqtt_client_info.client_user = NULL;  // Simplified: removed MQTT authentication
    mqtt_state.mqtt_client_info.client_pass = NULL;  // Simplified: removed MQTT authentication
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&mqtt_state, MQTT_WILL_TOPIC), sizeof(will_topic));
    mqtt_state.mqtt_client_info.will_topic = will_topic;
    mqtt_state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    mqtt_state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    mqtt_state.mqtt_client_info.will_retain = true;
    mqtt_state.mqtt_connected = false;  // Initialize connection status

    // Create the WiFi task with higher priority
    if (xTaskCreate(wifi_task, "WiFi Task", 2048, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS)
    {
        printf("Failed to create WiFi task\n");
        cyw43_arch_deinit(); // Clean up if task creation fails
        return -1;
    }

    // Create the MQTT task with medium priority
    if (xTaskCreate(mqtt_task, "MQTT Task", 2048, &mqtt_state, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
    {
        printf("Failed to create MQTT task\n");
        cyw43_arch_deinit(); // Clean up if task creation fails
        return -1;
    }

    // Create sensor task with lower priority
    if (xTaskCreate(sensor_task, "Sensor Task", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        printf("Failed to create sensor task\n");
        cyw43_arch_deinit(); // Clean up if task creation fails
        return -1;
    }

    // Start the RTOS scheduler - this function should not return
    vTaskStartScheduler();

    // If we reach here, it means vTaskStartScheduler failed
    printf("Failed to start RTOS scheduler\n");
    cyw43_arch_deinit(); // Clean up
    return -1;
}