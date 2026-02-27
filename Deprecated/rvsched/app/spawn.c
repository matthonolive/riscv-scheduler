#include "sched.h"
#include "uart.h"
#include "task_table.h"

#define STACK_WORDS 256  // 1KB per task on RV32

static uint32_t g_stack_pool[MAX_TASKS][STACK_WORDS];
static uint8_t  g_stack_used[MAX_TASKS];

static uint32_t* alloc_stack(void) {
  for (int i = 0; i < MAX_TASKS; ++i) {
    if (!g_stack_used[i]) { g_stack_used[i] = 1; return g_stack_pool[i]; }
  }
  return 0;
}

int spawn_task_id(uint32_t id) {
  const TaskDef* d = task_lookup(id);
  if (!d) {
    uart_puts("[spawn] unknown id\n");
    return -1;
  }

  uint32_t* stk = alloc_stack();
  if (!stk) {
    uart_puts("[spawn] no stack slots (task limit hit)\n");
    return -2;
  }

  int tid = task_create(d->fn, 0, d->prio, stk, STACK_WORDS, d->slice);
  if (tid < 0) {
    uart_puts("[spawn] task_create failed (MAX_TASKS reached)\n");
    return -3;
  }

  uart_puts("[spawn] id=");
  uart_putdec(id);
  uart_puts(" tid=");
  uart_putdec((uint32_t)tid);
  uart_puts("\n");
  return tid;
}
