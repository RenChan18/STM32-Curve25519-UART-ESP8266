#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"

#define UART_NUM            UART_NUM_0
#define BUF_SIZE            128

// Задача для периодической отправки сообщения
void uart_send_task(void *pvParameters) {
    const char *msg = "Hello from ESP8266 RTOS SDK\r\n";
    while (1) {
        uart_write_bytes(UART_NUM, msg, strlen(msg));
        vTaskDelay(10000 / portTICK_PERIOD_MS); // задержка 1 секунда
    }
}

// Задача для приёма данных с UART и их эхо-ответа
void uart_receive_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    while (1) {
        // Считываем данные с таймаутом 20 мс
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // завершаем строку
            // Формируем и отправляем эхо-сообщение
            uart_write_bytes(UART_NUM, "Received: ", strlen("Received: "));
            uart_write_bytes(UART_NUM, (const char *)data, len);
            uart_write_bytes(UART_NUM, "\r\n", 2);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = 74880,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Настраиваем параметры UART
    uart_param_config(UART_NUM, &uart_config);
    // Устанавливаем драйвер UART с размером RX-буфера 2048 байт (TX-буфер не используется)
    uart_driver_install(UART_NUM, 2048, 0, 0, NULL, 0);

    // Создаем задачу для отправки данных
    xTaskCreate(uart_send_task, "uart_send_task", 1024, NULL, 10, NULL);
    // Создаем задачу для приёма данных
    xTaskCreate(uart_receive_task, "uart_receive_task", 1024, NULL, 10, NULL);
}
