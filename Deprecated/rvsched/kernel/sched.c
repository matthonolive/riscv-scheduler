#include "sched.h"
#include "trap.h"
#include "csr.h"
#include "timer.h"
#include "uart.h"
#include <stddef.h>
void* memset(void* dst, int c, size_t n);
void* memcpy(void* dst, const void* src, size_t n);

typedef struct Task {
  uintptr_t tf_sp;       // saved trapframe pointer (stack pointer at trapframe base)
  task_fn_t fn;
  void* arg;

  uint32_t wake_tick;

  uint32_t pending_events;
  uint32_t wait_mask;

  uint8_t prio;          // 0 highest
  task_state_t state;

  uint8_t slice_reload;  // for HYBRID / RR
  uint8_t slice_left;
} Task;

static Task g_tasks[MAX_TASKS];
static int  g_cur = -1;

static sched_policy_t g_policy = SCHED_HYBRID;
static uint32_t g_tick_cycles = 10000; // default: adjust if you want different tick rate

static uint32_t g_ticks = 0;

// Ready sets per priority (bitmask of task IDs)
static uint32_t g_ready_prio[MAX_PRIO];
static uint8_t  g_rr_last[MAX_PRIO];   // last chosen tid within that prio (for RR)

static inline void ready_set(int tid) {
  uint8_t p = g_tasks[tid].prio;
  g_ready_prio[p] |= (1u << tid);
  g_tasks[tid].state = TASK_READY;
}
static inline void ready_clear(int tid) {
  uint8_t p = g_tasks[tid].prio;
  g_ready_prio[p] &= ~(1u << tid);
}

static inline int any_ready(void) {
  for (int p = 0; p < MAX_PRIO; ++p) if (g_ready_prio[p]) return 1;
  return 0;
}

static int pick_next_tid(void) {
  // If nothing ready, return idle (tid 0).
  if (!any_ready()) return 0;

  // RR policy treats all tasks as same priority 0.
  if (g_policy == SCHED_RR) {
    uint32_t mask = g_ready_prio[0];
    // Ensure idle isn't the only option if others exist
    uint8_t start = (uint8_t)(g_rr_last[0] + 1);
    for (int k = 0; k < MAX_TASKS; ++k) {
      uint8_t tid = (start + k) % MAX_TASKS;
      if (mask & (1u << tid)) {
        g_rr_last[0] = tid;
        return tid;
      }
    }
    return 0;
  }

  // PREEMPT/HYBRID: pick highest priority with ready tasks
  for (int p = 0; p < MAX_PRIO; ++p) {
    uint32_t mask = g_ready_prio[p];
    if (!mask) continue;

    // HYBRID: RR within same priority. PREEMPT: deterministic smallest TID.
    if (g_policy == SCHED_PREEMPT) {
      for (int tid = 0; tid < MAX_TASKS; ++tid)
        if (mask & (1u << tid)) return tid;
      continue;
    }

    // HYBRID RR
    uint8_t start = (uint8_t)(g_rr_last[p] + 1);
    for (int k = 0; k < MAX_TASKS; ++k) {
      uint8_t tid = (start + k) % MAX_TASKS;
      if (mask & (1u << tid)) {
        g_rr_last[p] = tid;
        return tid;
      }
    }
  }
  return 0;
}

static void task_exit(void) {
  uint32_t ms = irq_disable();
  int tid = g_cur;
  g_tasks[tid].state = TASK_ZOMBIE;
  ready_clear(tid);
  irq_restore(ms);
  task_yield();
  for (;;) { } // should never return
}

static void prepare_initial_trapframe(int tid, uint32_t* stack, size_t stack_words) {
  // Stack grows down. Place a TrapFrame at the top.
  uintptr_t top = (uintptr_t)(stack + stack_words);
  uintptr_t tf_sp = (top - sizeof(TrapFrame)) & ~((uintptr_t)0xF);

  TrapFrame* tf = (TrapFrame*)tf_sp;
  memset(tf, 0, sizeof(*tf));

  tf->ra = (uint32_t)(uintptr_t)task_exit;
  tf->a0 = (uint32_t)(uintptr_t)g_tasks[tid].arg;

  // mstatus: run in M-mode, and have interrupts enabled after mret via MPIE=1
  uint32_t ms = csr_read_mstatus();
  ms &= ~MSTATUS_MPP_MASK;
  ms |= MSTATUS_MPP_M;
  ms |= MSTATUS_MPIE;
  tf->mstatus = ms;

  tf->mepc = (uint32_t)(uintptr_t)g_tasks[tid].fn;

  g_tasks[tid].tf_sp = tf_sp;
}

void sched_init(sched_policy_t policy, uint32_t tick_cycles) {
  memset(g_tasks, 0, sizeof(g_tasks));
  memset(g_ready_prio, 0, sizeof(g_ready_prio));
  memset(g_rr_last, 0, sizeof(g_rr_last));
  g_cur = -1;

  g_policy = policy;
  g_tick_cycles = tick_cycles;
  g_ticks = 0;

  uart_init();

  extern void trap_entry(void);
  csr_write_mtvec((uint32_t)(uintptr_t)&trap_entry);

  uart_puts("\n[rvsched] init ok\n");
}

int task_create(task_fn_t fn, void* arg, uint8_t prio,
                uint32_t* stack, size_t stack_words,
                uint32_t time_slice_ticks) {
  if (!fn || !stack || stack_words < 64) return -1;
  if (prio >= MAX_PRIO) prio = (MAX_PRIO - 1);

  uint32_t ms = irq_disable();

  int tid = -1;
  for (int i = 0; i < MAX_TASKS; ++i) {
    if (g_tasks[i].state == TASK_UNUSED) { tid = i; break; }
  }
  if (tid < 0) { irq_restore(ms); return -1; }

  g_tasks[tid].fn = fn;
  g_tasks[tid].arg = arg;
  g_tasks[tid].prio = prio;
  g_tasks[tid].pending_events = 0;
  g_tasks[tid].wait_mask = 0;
  g_tasks[tid].wake_tick = 0;

  if (g_policy == SCHED_PREEMPT) {
    g_tasks[tid].slice_reload = 0;
    g_tasks[tid].slice_left = 0;
  } else {
    uint8_t slice = (time_slice_ticks == 0) ? 1 : (uint8_t)time_slice_ticks;
    g_tasks[tid].slice_reload = slice;
    g_tasks[tid].slice_left = slice;
  }

  // RR policy forces all tasks into priority 0 for simplicity.
  if (g_policy == SCHED_RR) g_tasks[tid].prio = 0;

  prepare_initial_trapframe(tid, stack, stack_words);

  ready_set(tid);

  irq_restore(ms);
  return tid;
}

uint32_t sched_ticks(void) { return g_ticks; }
int sched_current_tid(void) { return g_cur; }

static void tick_wake_sleepers(void) {
  for (int tid = 0; tid < MAX_TASKS; ++tid) {
    if (g_tasks[tid].state == TASK_SLEEP && (int32_t)(g_ticks - g_tasks[tid].wake_tick) >= 0) {
      ready_set(tid);
    }
  }
}

static int exists_higher_ready(uint8_t cur_prio) {
  for (int p = 0; p < cur_prio; ++p) {
    if (g_ready_prio[p]) return 1;
  }
  return 0;
}

TrapFrame* sched_on_trap(TrapFrame* tf, int from_timer_irq, int force_resched) {
  // Save current task's trapframe pointer.
  if (g_cur >= 0) {
    g_tasks[g_cur].tf_sp = (uintptr_t)tf;
  }

  int need_resched = force_resched;

  if (from_timer_irq) {
    g_ticks++;
    tick_wake_sleepers();

    if (g_cur >= 0) {
      Task* cur = &g_tasks[g_cur];

      if (g_policy == SCHED_HYBRID || g_policy == SCHED_RR) {
        if (cur->slice_reload) {
          if (cur->slice_left > 0) cur->slice_left--;
          if (cur->slice_left == 0) {
            cur->slice_left = cur->slice_reload;
            // Only resched if there is someone else runnable at same prio or higher prio.
            if (exists_higher_ready(cur->prio) || (g_ready_prio[cur->prio] & ~(1u << g_cur))) {
              need_resched = 1;
            }
          }
        }
      } else if (g_policy == SCHED_PREEMPT) {
        if (exists_higher_ready(cur->prio)) need_resched = 1;
      }
    }
  }

  if (!need_resched) {
    if (g_cur >= 0) return (TrapFrame*)(uintptr_t)g_tasks[g_cur].tf_sp;
    return tf;
  }

  // Move RUNNING -> READY if still runnable
  if (g_cur >= 0 && g_tasks[g_cur].state == TASK_RUNNING) {
    ready_set(g_cur);
  }

  int next = pick_next_tid();
  // Mark next running; remove from ready set.
  ready_clear(next);
  g_tasks[next].state = TASK_RUNNING;
  g_cur = next;

  return (TrapFrame*)(uintptr_t)g_tasks[next].tf_sp;
}

void task_yield(void) {
  arch_ecall();
}

void task_sleep_ticks(uint32_t dt) {
  task_sleep_until(g_ticks + dt);
}

void task_sleep_until(uint32_t wake_tick) {
  uint32_t ms = irq_disable();
  int tid = g_cur;
  g_tasks[tid].wake_tick = wake_tick;
  g_tasks[tid].state = TASK_SLEEP;
  ready_clear(tid);
  irq_restore(ms);
  task_yield();
}

void task_wait_events(uint32_t mask) {
  uint32_t ms = irq_disable();
  int tid = g_cur;
  Task* t = &g_tasks[tid];

  // If already pending, consume and return immediately.
  if (t->pending_events & mask) {
    t->pending_events &= ~mask;
    irq_restore(ms);
    return;
  }

  t->wait_mask = mask;
  t->state = TASK_WAIT;
  ready_clear(tid);
  irq_restore(ms);

  task_yield();

  // After being woken, consume events.
  ms = irq_disable();
  t->pending_events &= ~mask;
  t->wait_mask = 0;
  irq_restore(ms);
}

void task_set_events(int tid, uint32_t mask) {
  if (tid < 0 || tid >= MAX_TASKS) return;
  uint32_t ms = irq_disable();
  Task* t = &g_tasks[tid];
  t->pending_events |= mask;
  if (t->state == TASK_WAIT && (t->pending_events & t->wait_mask)) {
    ready_set(tid);
  }
  irq_restore(ms);
}

void sched_start(void) {
  // Expect an idle task to exist at tid 0.
  if (g_tasks[0].state == TASK_UNUSED) {
    uart_puts("[rvsched] ERROR: create an idle task at tid 0 before sched_start()\n");
    for (;;) { __asm__ volatile("wfi"); }
  }

  uart_puts("[rvsched] starting\n");

  // Select first runnable task
  int next = pick_next_tid();
  ready_clear(next);
  g_tasks[next].state = TASK_RUNNING;
  g_cur = next;

  // Start timer and enable interrupts.
  timer_init(g_tick_cycles);

  // Enter first task by restoring its trapframe and mret.
  arch_start_first(g_tasks[next].tf_sp);
  for (;;) {}
}
