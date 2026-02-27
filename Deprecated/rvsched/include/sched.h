#pragma once
#include <stdint.h>
#include <stddef.h>

typedef void (*task_fn_t)(void*);

typedef enum {
  SCHED_RR = 0,
  SCHED_PREEMPT = 1,
  SCHED_HYBRID = 2,
} sched_policy_t;

typedef enum {
  TASK_UNUSED = 0,
  TASK_READY,
  TASK_RUNNING,
  TASK_SLEEP,
  TASK_WAIT,
  TASK_ZOMBIE,
} task_state_t;

#define MAX_TASKS 16
#define MAX_PRIO  8   // 0 = highest priority

typedef struct TrapFrame TrapFrame;

void sched_init(sched_policy_t policy, uint32_t tick_cycles);
int  task_create(task_fn_t fn, void* arg, uint8_t prio,
                 uint32_t* stack, size_t stack_words,
                 uint32_t time_slice_ticks /*0 => no slice*/);

void sched_start(void);

// Called from trap handler (timer/ecall). Returns trapframe pointer to resume.
TrapFrame* sched_on_trap(TrapFrame* tf, int from_timer_irq, int force_resched);

// Cooperative primitives (implemented as ECALL -> trap)
void task_yield(void);
void task_sleep_ticks(uint32_t dt);
void task_sleep_until(uint32_t wake_tick);

// Event-driven primitives
void task_wait_events(uint32_t mask);
void task_set_events(int tid, uint32_t mask);

// Time base
uint32_t sched_ticks(void);

// Debug
int sched_current_tid(void);
