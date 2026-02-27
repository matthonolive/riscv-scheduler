#include "sched.h"
#include "uart.h"
#include <stdint.h>

typedef struct {
  uint32_t id;
  task_fn_t fn;
  uint8_t prio;
  uint8_t slice;
} TaskDef;

// Example tasks
static void task_A(void* _) {
  (void)_;
  for (;;) { uart_puts("A"); task_sleep_ticks(100); }
}
static void task_b(void* _) {
  (void)_;
  for (;;) { uart_puts("b"); task_sleep_ticks(37); }
}
static void task_hog(void* _) {
  (void)_;
  for (;;) { uart_puts("."); /* no sleep => preemption test */ }
}

static const TaskDef g_tasks[] = {
  { 1, task_A, 2, 5 },
  { 2, task_b, 3, 5 },
  { 3, task_hog, 6, 0 }, // lower prio, no slice needed if PREEMPT mode
};

const TaskDef* task_lookup(uint32_t id) {
  for (unsigned i = 0; i < sizeof(g_tasks)/sizeof(g_tasks[0]); ++i)
    if (g_tasks[i].id == id) return &g_tasks[i];
  return 0;
}

void task_list_print(void) {
  uart_puts("\nAvailable task IDs:\n");
  for (unsigned i = 0; i < sizeof(g_tasks)/sizeof(g_tasks[0]); ++i) {
    uart_puts("  ");
    uart_putdec(g_tasks[i].id);
    uart_puts("\n");
  }
}
