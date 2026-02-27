#pragma once
#include <stdint.h>

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char* s);
void uart_puthex(uint32_t v);
void uart_putdec(uint32_t v);
int uart_getc_nonblock(char* out);
