#include <FreeRTOS.h>
#include <task.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"

#define WIFI_SSID "Arnaldojr"
#define WIFI_PASSWORD "12345678"

// Task to handle WiFi management in FreeRTOS context
void wifi_task(void *args)
{
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
        
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }       

    (void)args;
}

void other_task(void *args)
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    while(true) {
        // Read temperature
        const uint16_t adc = adc_read();
        const float conversionFactor = 3.3f / (1 << 12);
        float tempC = 27.0f - (adc * conversionFactor - 0.706f) / 0.001721f;
        printf("Temperatura: %.2f°C\n", tempC);

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
    
    (void)args;
}

int main(void)
{
    stdio_init_all();
    printf("Start RTOS WiFi\n");

    if (cyw43_arch_init()) 
    {
        printf("Failed to initialize CYW43\n");
        return -1;
    }

    // Create the WiFi task with higher priority
    if (xTaskCreate(wifi_task, "WiFi Task", 1024, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
    {
        printf("Failed to create WiFi task\n");
        cyw43_arch_deinit(); // Clean up if task creation fails
        return -1;
    }

    // Create other tasks that might need to use the network
    if (xTaskCreate(other_task, "Other Task", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        printf("Failed to create other task\n");
        cyw43_arch_deinit(); // Clean up if task creation fails
        return -1;
    }
    
    // Start the RTOS scheduler - this function should not return
    vTaskStartScheduler();

    while (true)
    {

    }
}
