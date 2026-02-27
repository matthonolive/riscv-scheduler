#include "platform.h"
#include "uart.h"

#define UART_RHR 0x00 // receive holding (read)
#define UART_THR 0x00 // transmit holding (write)
#define UART_LSR 0x05 // line status
#define LSR_THRE 0x20 // THR empty
#define LSR_DR  0x01 // data ready
#define UART_RHR 0x00 // receive holding (read)

static inline void mmio_write8(uintptr_t addr, uint8_t v) {
  *(volatile uint8_t*)addr = v;
}
static inline uint8_t mmio_read8(uintptr_t addr) {
  return *(volatile uint8_t*)addr;
}

void uart_init(void) {
  // QEMU virt UART works out-of-the-box for TX; no baud config needed here.
}

void uart_putc(char c) {
  uintptr_t base = UART0_BASE;
  while ((mmio_read8(base + UART_LSR) & LSR_THRE) == 0) { }
  mmio_write8(base + UART_THR, (uint8_t)c);
}

void uart_puts(const char* s) {
  while (*s) {
    if (*s == '\n') uart_putc('\r');
    uart_putc(*s++);
  }
}

void uart_puthex(uint32_t v) {
  static const char* hexd = "0123456789abcdef";
  uart_puts("0x");
  for (int i = 7; i >= 0; --i) {
    uart_putc(hexd[(v >> (i*4)) & 0xF]);
  }
}

void uart_putdec(uint32_t v) {
  char buf[11];
  int n = 0;
  if (v == 0) { uart_putc('0'); return; }
  while (v && n < (int)sizeof(buf)) {
    buf[n++] = '0' + (v % 10);
    v /= 10;
  }
  for (int i = n-1; i >= 0; --i) uart_putc(buf[i]);
}

int uart_getc_nonblock(char* out) {
  uintptr_t base = UART0_BASE;
  if (mmio_read8(base + UART_LSR) & LSR_DR) {
    *out = (char)mmio_read8(base + UART_RHR);
    return 1;
  }
  return 0;
}
