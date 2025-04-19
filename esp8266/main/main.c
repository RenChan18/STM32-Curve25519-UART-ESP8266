#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "string.h"
#include "stdio.h"
#include <ctype.h>
#include <sodium.h>

#define UART_NUM            UART_NUM_1
#define UART_SW_RX          (GPIO_NUM_13)
#define UART_BAUD_RATE      9600
#define BIT_PERIOD_US       (1000000/UART_BAUD_RATE)
#define BUF_SIZE            256
#define KEY_HEX_LEN    64

static QueueHandle_t uart_txq, uart_rxq;

static const char *TRANSMIT = "TX_UART";
static const char *RECEIVE = "RX_UART";

const unsigned char basepoint[32] = {9};

static unsigned char my_private_key[32] = {
    0xe6,0xdb,0x68,0x67,0x58,0x30,0x30,0xdb,
    0x35,0x94,0xc1,0xa4,0x24,0xb1,0x5f,0x7c,
    0x72,0x66,0x24,0xec,0x26,0xb3,0x35,0x3b,
    0x10,0xa9,0x03,0xa6,0xd8,0xf7,0xb2,0xda};                       //

void gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << UART_SW_RX),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void trim_whitespace(char *str) {
    char *start = str;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    memmove(str, start, strlen(start) + 1);
    char *end = str + strlen(str) - 1;
    while (end >= str && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
}

void filter_hex_string(const char *in, char *out) {
    int count = 0;
    for (int i = 0; in[i] != '\0' && count < KEY_HEX_LEN; i++) {
        char c = in[i];
        if ((c >= '0' && c <= '9') ||
            (c >= 'a' && c <= 'f') ||
            (c >= 'A' && c <= 'F')) {
            out[count++] = c;
        }
    }
    out[count] = '\0';
}

void uart_rx_task(void * pvParameters) {
    uint8_t rx_byte;
    char str_buf[BUF_SIZE];
    int idx = 0;
    memset(str_buf, 0, BUF_SIZE);
    ESP_LOGI(RECEIVE, "<SOFTWARE RX>");
    while(1) {
        esp_task_wdt_reset();
        if (gpio_get_level(UART_SW_RX) == 0) {
            ets_delay_us(BIT_PERIOD_US / 2);  
            rx_byte = 0;
            for (int i = 0; i < 8; i++) {
                ets_delay_us(BIT_PERIOD_US);
                int bit = gpio_get_level(UART_SW_RX);
                rx_byte |= (bit << i);
                esp_task_wdt_reset();
            }
            ets_delay_us(BIT_PERIOD_US); 
            if (rx_byte == '\n' || rx_byte == '\r') {
                if (idx > 0) {
                    str_buf[idx] = '\0';
                    trim_whitespace(str_buf);
                    ESP_LOGI(RECEIVE, "RECEIVED LINE(THEIR PUBLICK KEY): %s", str_buf);
                    if (xQueueSend(uart_rxq, str_buf, portMAX_DELAY) != pdPASS) {
                        ESP_LOGE(RECEIVE, "ERROR QUEUE OVERFLOW");
                    }
                    idx = 0;
                    memset(str_buf, 0, BUF_SIZE);
                }
            } else {
                if (idx < BUF_SIZE - 1) {
                    str_buf[idx++] = rx_byte;
                } else {
                    str_buf[idx] = '\0';
                    idx = 0;
                    memset(str_buf, 0, BUF_SIZE);
                    ESP_LOGE(RECEIVE, "ERROR: Buffer overflow, line truncated");
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void curve25519_task(void *arg) {
    char buf[BUF_SIZE];
    char acc[KEY_HEX_LEN+1];
    size_t acc_len = 0;
    unsigned char peer[32], secret[32];
    memset(acc,0,sizeof(acc));
    while (1) {
        if (xQueueReceive(uart_rxq, buf, portMAX_DELAY) == pdPASS) {
            trim_whitespace(buf);
            for (size_t i=0; buf[i] && acc_len<KEY_HEX_LEN; i++) {
                if (isxdigit((unsigned char)buf[i])) {
                    acc[acc_len++] = buf[i];
                }
            }
            if (acc_len == KEY_HEX_LEN) {
                for (int i=0; i<32; i++) {
                    char byte[3] = {acc[i*2], acc[i*2+1], 0};
                    peer[i] = (unsigned char)strtol(byte, NULL,16);
                }
                crypto_scalarmult_curve25519(secret, my_private_key, peer);
                char hexs[KEY_HEX_LEN+1]; 
                memset(hexs,0,sizeof(hexs));
                for (int i=0; i<32; i++) sprintf(&hexs[i*2], "%02x", secret[i]);
                char *out_hdr = "Shared secret:\r\n";
                xQueueSend(uart_txq, &out_hdr, portMAX_DELAY);
                xQueueSend(uart_txq, &hexs, portMAX_DELAY);
                xQueueSend(uart_txq, "\r\n", portMAX_DELAY);
                acc_len = 0; acc[0]=0;
            }
        }
    }
}

void public_key_task(void *arg) {
    unsigned char pub[32];
    char hex[KEY_HEX_LEN+1];
    memset(hex,0,sizeof(hex));
    crypto_scalarmult_curve25519(pub, my_private_key, basepoint);
    for (int i=0; i<32; i++) {
        sprintf(&hex[i*2], "%02x", pub[i]);
    }
    char *hdr = "My public key:\r\n";
    ESP_LOGI(TRANSMIT, "MY PUBLIC KEY: %s", hex);
    xQueueSend(uart_txq, &hdr, portMAX_DELAY);
    xQueueSend(uart_txq, &hex, portMAX_DELAY);
    char *nl = "\r\n";
    xQueueSend(uart_txq, &nl, portMAX_DELAY);
    vTaskDelete(NULL);
}

void uart_tx_task(void * pvParameters) {
    char tx_buffer[BUF_SIZE];
    ESP_LOGI(TRANSMIT, "<SOFTWARE TX>");
    uart_write_bytes(UART_NUM, "\r\nSECRET: ", 14);
    while(1) {
        if (xQueueReceive(uart_txq, tx_buffer, portMAX_DELAY) == pdPASS) {
            size_t len = strlen(tx_buffer);
            int sent = uart_write_bytes(UART_NUM, tx_buffer, len);
            uart_write_bytes(UART_NUM, "\r\n", 2);
            if (sent == len) {
                ESP_LOGI(TRANSMIT, "OUR SECRET: %s", tx_buffer);
            } else {
                ESP_LOGE(TRANSMIT, "FAILED TO SEND DATA");
            }
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    gpio_init();
    uart_init();

    if (sodium_init() < 0) {
        uart_write_bytes(UART_NUM, "Libsodium init failed!\r\n", 24);
        vTaskDelete(NULL);
        return;
    }

    uart_rxq = xQueueCreate(1, BUF_SIZE);
    uart_txq = xQueueCreate(1, BUF_SIZE);
    if (uart_txq == NULL || uart_rxq == NULL) {
        ESP_LOGE("MAIN", "Queue creation failed!");
        return;
    }
    xTaskCreate(public_key_task,   "pub_key",   2048, NULL, 12, NULL);
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);
    xTaskCreate(curve25519_task, "curve25519_task", 4096, NULL, 9, NULL);
    xTaskCreate(uart_tx_task, "uart_tx", 4096, NULL, 8, NULL);
}
