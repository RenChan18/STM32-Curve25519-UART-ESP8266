#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "monocypher.h"

#include <stdint.h>
#include <string.h>

#define PRIVATE_KEY_SIZE 32
#define PUBLIC_KEY_SIZE  32

#define BAUD_RATE      9600
#define RX_BUF_LEN     (PUBLIC_KEY_SIZE * 2 + 1)

static QueueHandle_t uart_rxq;
static QueueHandle_t uart_txq;
static QueueHandle_t public_key_rxq;

static const uint8_t my_private_key[PRIVATE_KEY_SIZE] = {
    0x1a,0x2b,0x3c,0x4d,0x5e,0x6f,0x70,0x81,
    0x92,0xa3,0xb4,0xc5,0xd6,0xe7,0xf8,0x09,
    0x1b,0x2c,0x3d,0x4e,0x5f,0x60,0x71,0x82,
    0x93,0xa4,0xb5,0xc6,0xd7,0xe8,0xf9,0x0a
};

static void clock_setup(void) {
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
    rcc_set_sysclk_source(RCC_HSE);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);
}

static void gpio_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
}

void uart_setup(void) {
    usart_set_baudrate(USART2, BAUD_RATE);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_enable_rx_interrupt(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);
    usart_enable(USART2);
}

void usart2_isr(void) {
    if (usart_get_flag(USART2, USART_SR_RXNE)) {
        char c = usart_recv(USART2);
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(uart_rxq, &c, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

static inline char nibble2hex(uint8_t n) {
    return (n < 10) ? ('0' + n) : ('a' + n - 10);
}

static inline void uart_putc_queue(char c) {
    xQueueSend(uart_txq, &c, portMAX_DELAY);
}

static inline void uart_puts_queue(const char *s) {
    while (*s) uart_putc_queue(*s++);
}

static void print_key_queue(const uint8_t *key) {
    for (int i = 0; i < PUBLIC_KEY_SIZE; ++i) {
        uart_putc_queue(nibble2hex(key[i] >> 4));
        uart_putc_queue(nibble2hex(key[i] & 0xF));
    }
    uart_puts_queue("\r\n");
}

static void uart_tx_task(void *unused) {
    char c;
    for (;;) {
        if (xQueueReceive(uart_txq, &c, portMAX_DELAY) == pdPASS) {
            while (!usart_get_flag(USART2, USART_SR_TXE));
            usart_send(USART2, c);
        }
    }
}

static void uart_rx_task(void *unused) {
    char line[RX_BUF_LEN];
    size_t idx = 0;
    char c;
    for (;;) {
        xQueueReceive(uart_rxq, &c, portMAX_DELAY);
        if (c == '\r' || c == '\n') {
            if (idx == PUBLIC_KEY_SIZE * 2) {
                line[idx] = '\0';
                uint8_t peer_key[PUBLIC_KEY_SIZE];
                for (int i = 0; i < PUBLIC_KEY_SIZE; ++i) {
                    char hi = line[2*i];
                    char lo = line[2*i + 1];
                    peer_key[i] = ((hi <= '9' ? hi - '0' : hi - 'a' + 10) << 4)
                                  | (lo <= '9' ? lo - '0' : lo - 'a' + 10);
                }
                xQueueSend(public_key_rxq, peer_key, portMAX_DELAY);
                uart_puts_queue("Peer key received\r\n");
                print_key_queue(peer_key);
            } else if (idx > 0) {
                uart_puts_queue("Bad key length\r\n");
            }
            idx = 0;
        } else if (idx < PUBLIC_KEY_SIZE * 2) {
            line[idx++] = c;
        }
    }
}

static void public_key_task(void *unused) {
    uint8_t my_public[PUBLIC_KEY_SIZE];
    crypto_x25519_public_key(my_public, my_private_key);
    uart_puts_queue("My public key: ");
    print_key_queue(my_public);
    vTaskDelete(NULL);
}

static void shared_secret_task(void *unused) {
    uint8_t peer_key[PUBLIC_KEY_SIZE];
    uint8_t shared[PUBLIC_KEY_SIZE];
    for (;;) {
        xQueueReceive(public_key_rxq, peer_key, portMAX_DELAY);
        crypto_x25519(shared, my_private_key, peer_key);
        uart_puts_queue("Shared secret: ");
        print_key_queue(shared);
    }
}

int main(void) {
    clock_setup();
    gpio_setup();
    uart_setup();

    uart_rxq       = xQueueCreate(128, sizeof(char));
    uart_txq       = xQueueCreate(128, sizeof(char));
    public_key_rxq = xQueueCreate(4,   PUBLIC_KEY_SIZE);

    xTaskCreate(uart_tx_task,       "uart_tx",   256, NULL, tskIDLE_PRIORITY+2, NULL);
    xTaskCreate(uart_rx_task,       "uart_rx",   256, NULL, tskIDLE_PRIORITY+2, NULL);
    xTaskCreate(public_key_task,    "pub_key",   256, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(shared_secret_task, "secret",    256, NULL, tskIDLE_PRIORITY+1, NULL);

    vTaskStartScheduler();
    for (;;) ;
}

