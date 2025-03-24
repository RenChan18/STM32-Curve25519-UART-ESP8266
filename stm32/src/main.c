#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


#ifdef BEGIN_DECLS
#undef BEGIN_DECLS
#endif
#ifdef END_DECLS
#undef END_DECLS
#endif
#define BEGIN_DECLS
#define END_DECLS
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rng.h>
#include <libopencm3/stm32/timer.h>
#include "monocypher.h"


void uart_setup(void);
void uart_send_string(const char *str);
void print_key(const uint8_t *key, size_t length);
void randombytes(uint8_t *buffer, size_t size);
void timer_setup(void);
uint32_t timer_get_time_us(void);

#define PRIVATE_KEY_SIZE 32
#define PUBLIC_KEY_SIZE 32

void uart_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);
}

void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART2, *str++);
    }
}

void print_key(const uint8_t *key, size_t length) {
    char buffer[3];
    for (size_t i = 0; i < length; i++) {
        snprintf(buffer, sizeof(buffer), "%02x", key[i]);
        uart_send_string(buffer);
    }
    uart_send_string("\r\n");
}

void randombytes(uint8_t *buffer, size_t size) {
    rcc_periph_clock_enable(RCC_RNG);
    rng_enable();

    for (size_t i = 0; i < size; i++) {
        uint32_t rand_val;
        if (!rng_get_random(&rand_val))
            buffer[i] = 0;
        else
            buffer[i] = (uint8_t)(rand_val & 0xFF);
    }

    rng_disable();
}

void timer_setup(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_counter(TIM2, 0);
    timer_set_prescaler(TIM2, (rcc_ahb_frequency / 1000000) - 1);
    timer_continuous_mode(TIM2);
    timer_enable_counter(TIM2);
}

uint32_t timer_get_time_us(void) {
    return timer_get_counter(TIM2);
}

int main(void) {
    uart_setup();
    timer_setup();

    uint8_t public_key[PUBLIC_KEY_SIZE];

    //randombytes(private_key, PRIVATE_KEY_SIZE);
    uint8_t private_key[PRIVATE_KEY_SIZE] = {
    0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f, 0x70, 0x81,
    0x92, 0xa3, 0xb4, 0xc5, 0xd6, 0xe7, 0xf8, 0x09,
    0x1b, 0x2c, 0x3d, 0x4e, 0x5f, 0x60, 0x71, 0x82,
    0x93, 0xa4, 0xb5, 0xc6, 0xd7, 0xe8, 0xf9, 0x0a
    };

    uint32_t start_time = timer_get_time_us();
    uart_send_string(" ");
    
    crypto_x25519_public_key(public_key, private_key);

    uint32_t end_time = timer_get_time_us();
    uint32_t time_taken = end_time - start_time;

    uart_send_string("Private Key: ");
    print_key(private_key, PRIVATE_KEY_SIZE);

    uart_send_string("Public Key: ");
    print_key(public_key, PUBLIC_KEY_SIZE);

    char time_str[50];
    snprintf(time_str, sizeof(time_str), "Time taken: %lu us\r\n", time_taken);
    uart_send_string(time_str);

    while (1);
    return 0;
}

