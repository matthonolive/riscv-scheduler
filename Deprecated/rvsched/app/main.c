#include "sched.h"
#include "uart.h"
#include "dispatcher.h"

#define TICK_CYCLES 10000u
#define STACK_WORDS 256

static uint32_t stack_idle[STACK_WORDS];
static uint32_t stack_disp[STACK_WORDS];

static void idle(void* _) {
  (void)_;
  for (;;) { __asm__ volatile("wfi"); }
}

int main(void) {
  sched_init(SCHED_HYBRID, TICK_CYCLES);

  task_create(idle, 0, 7, stack_idle, STACK_WORDS, 0);
  dispatcher_start(stack_disp, STACK_WORDS);

  uart_puts("\n[rvsched] main -> sched_start()\n");
  sched_start();
  return 0;
}
