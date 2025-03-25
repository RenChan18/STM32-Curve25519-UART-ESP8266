#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "stdio.h"
#include <sodium.h>

#define UART_NUM            UART_NUM_0
#define BUF_SIZE            256
#define PRIV_KEY_HEX_LEN    64

const unsigned char basepoint[32] = {9};

void curve25519_task(void *pvParameters) {
    char line_buffer[BUF_SIZE] = {0};
    int line_pos = 0;
    
    // Инициализация libsodium
    if (sodium_init() < 0) {
        uart_write_bytes(UART_NUM, "Libsodium init failed!\r\n", 24);
        vTaskDelete(NULL);
        return;
    }

    uart_write_bytes(UART_NUM, "Enter private key (64 hex symbols):\r\n", 37);

    while (1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t *)&line_buffer[line_pos], 1, portMAX_DELAY);
        if (len > 0) {
            // Эхо-печать введённого символа
            uart_write_bytes(UART_NUM, &line_buffer[line_pos], 1);
            
            if (line_buffer[line_pos] == '\n' || line_buffer[line_pos] == '\r') {
                line_buffer[line_pos] = '\0';
                
                if (strlen(line_buffer) == PRIV_KEY_HEX_LEN) {
                    unsigned char private_key[32] = {0};
                    unsigned char public_key[32] = {0};
                    
                    // Конвертация hex строки в бинарный формат
                    for (int i = 0; i < 32; i++) {
                        if (sscanf(&line_buffer[i*2], "%2hhx", &private_key[i]) != 1) {
                            uart_write_bytes(UART_NUM, "\r\nError: Invalid hex format\r\n", 28);
                            break;
                        }
                    }
                    
                    // Вычисление публичного ключа
                    if (crypto_scalarmult_curve25519(public_key, private_key, basepoint) != 0) {
                        uart_write_bytes(UART_NUM, "\r\nError: Calculation failed\r\n", 30);
                    } else {
                        char public_key_hex[65] = {0};
                        for (int i = 0; i < 32; i++) {
                            sprintf(&public_key_hex[i*2], "%02x", public_key[i]);
                        }
                        uart_write_bytes(UART_NUM, "\r\nPublic key: ", 13);
                        uart_write_bytes(UART_NUM, public_key_hex, 64);
                        uart_write_bytes(UART_NUM, "\r\n", 2);
                    }
                } else {
                    uart_write_bytes(UART_NUM, "\r\nError: Need exactly 64 hex chars\r\n", 36);
                }
                
                line_pos = 0;
                memset(line_buffer, 0, BUF_SIZE);
                uart_write_bytes(UART_NUM, "\r\nEnter private key (64 hex symbols):\r\n", 39);
            } else {
                line_pos++;
                if (line_pos >= BUF_SIZE-1) {
                    line_pos = 0;
                    memset(line_buffer, 0, BUF_SIZE);
                    uart_write_bytes(UART_NUM, "\r\nBuffer overflow, try again\r\n", 29);
                }
            }
        }
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

    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreate(curve25519_task, "curve25519_task", 4096, NULL, 10, NULL);
}

