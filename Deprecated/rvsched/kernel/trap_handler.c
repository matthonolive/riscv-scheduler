#include "trap.h"
#include "sched.h"
#include "timer.h"
#include "uart.h"
#include "csr.h"

static inline int is_interrupt(uint32_t mcause) {
  return (mcause >> 31) != 0;
}
static inline uint32_t cause_code(uint32_t mcause) {
  return mcause & 0x7FFFFFFFu;
}

TrapFrame* trap_handler(TrapFrame* tf) {
  const uint32_t mc = tf->mcause;

  if (is_interrupt(mc)) {
    const uint32_t code = cause_code(mc);

    // Machine timer interrupt = 7
    if (code == 7u) {
      timer_ack_and_set_next(timer_tick_cycles());
            return sched_on_trap(tf, 1, 0);
    }

    // Other interrupts not handled yet
    uart_puts("\n[trap] unhandled interrupt mcause=");
    uart_puthex(mc);
    uart_puts("\n");
    for (;;) { __asm__ volatile("wfi"); }
  } else {
    const uint32_t code = cause_code(mc);

    // ECALL from M-mode = 11
    if (code == 11u) {
      tf->mepc += 4; // skip the ECALL instruction
      return sched_on_trap(tf, 0, 1);
    }

    uart_puts("\n[trap] exception mcause=");
    uart_puthex(mc);
    uart_puts(" mepc=");
    uart_puthex(tf->mepc);
    uart_puts(" mtval=");
    uart_puthex(tf->mtval);
    uart_puts("\n");
    for (;;) { __asm__ volatile("wfi"); }
  }
}
