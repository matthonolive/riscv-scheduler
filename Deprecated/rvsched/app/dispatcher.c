#include "sched.h"
#include "uart.h"
#include "spawn.h"
#include "task_table.h"

static uint32_t parse_u32(const char* s) {
  uint32_t v = 0;
  while (*s >= '0' && *s <= '9') { v = v*10 + (uint32_t)(*s - '0'); s++; }
  return v;
}

static void dispatcher(void* _) {
  (void)_;
  char buf[16];
  int n = 0;

  uart_puts("\nType a task ID and press Enter (e.g. 1, 2, 3)\n");
  task_list_print();
  uart_puts("> ");

  for (;;) {
    char c;
    if (!uart_getc_nonblock(&c)) {
      task_sleep_ticks(1); // don’t busy-spin; also proves timer IRQ works
      continue;
    }

    if (c == '\r' || c == '\n') {
      buf[n] = 0;
      uart_puts("\n");

      if (n > 0) {
        uint32_t id = parse_u32(buf);
        spawn_task_id(id);
      }

      n = 0;
      uart_puts("> ");
      continue;
    }

    if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
      uart_putc(c); // local echo
    }
  }
}

void dispatcher_start(uint32_t* stack, unsigned stack_words) {
  // priority high enough to be responsive but not starve others
  task_create(dispatcher, 0, 1, stack, stack_words, 1);
}
