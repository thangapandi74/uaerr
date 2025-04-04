#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
/*********************************uart pin configuration*******************/
#define UART_NUM1 UART_NUM_1
#define UART_NUM0 UART_NUM_0
#define UART_TXD1 (GPIO_NUM_17)
#define UART_RXD1 (GPIO_NUM_16)
#define UART_RTS1 (UART_PIN_NO_CHANGE)
#define UART_CTS1 (UART_PIN_NO_CHANGE)
/*************************************************************************** */
#define UART_TXD0 (GPIO_NUM_4)
#define UART_RXD0 (GPIO_NUM_5)
#define UART_RTS0 (UART_PIN_NO_CHANGE)
#define UART_CTS0 (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)
/*********************************uart pin configuration*******************/
QueueHandle_t uart0_queue;
SemaphoreHandle_t uart1_mutex;

// UART0 ISR Handler
void IRAM_ATTR uart0_isr_handler(void *arg) {
    uint8_t data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Read data from UART0
    uart_read_bytes(UART_NUM0, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);


    // Check if data is available
    if (data > 0) {
        ESP_LOGI("UART0", "Received data: %c", data);
    } else {
        ESP_LOGI("UART0", "No data received");
    }
    // Send data to UART0 Queue
    xQueueSendFromISR(uart0_queue, &data, &xHigherPriorityTaskWoken);

    // Yield if a higher-priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// UART0 Task
void uart0_task(void *arg) {
    uint8_t data;
    while (1) {
        if (xQueueReceive(uart0_queue, &data, portMAX_DELAY)) 
        {
            ESP_LOGI("UART0", "Received data: %c", data);
        }
    }
}

// UART1 Task
void uart1_task(void *arg) {
    uint8_t data;
    while (1) {
        if (xSemaphoreTake(uart1_mutex, portMAX_DELAY) == pdTRUE) {
            int len = uart_read_bytes(UART_NUM1, &data, 1, pdMS_TO_TICKS(100));
            if (len > 0) {
                ESP_LOGI("UART1", "Received data: %c", data);
            }
            xSemaphoreGive(uart1_mutex);
        }
    }
}

void app_main(void) {
    // Initialize UART0 and UART1
    uart0_int();
    uart1_int();

    // Create queue and mutex
    uart0_queue = xQueueCreate(10, sizeof(uint8_t));
    uart1_mutex = xSemaphoreCreateMutex();

    // Install ISR for UART0
    uart_isr_free(UART_NUM0);
    uart_isr_register(UART_NUM0, uart0_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // Start tasks
    xTaskCreate(uart0_task, "uart0_task", 2048, NULL, 10, NULL);
    xTaskCreate(uart1_task, "uart1_task", 2048, NULL, 5, NULL);
}