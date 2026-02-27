/*
 * risc_v_firmware_scheduler.c
 *
 * Deterministic firmware scheduler for bare-metal RISC-V MCUs
 * Suitable for SERDES control, equalization, and monitoring loops
 */

#include <stdlib.h>
#include <stdint.h>

/* ===================== Configuration ===================== */

#define NUM_TASKS          8
#define NUM_PARAMS_RX      4
#define NUM_PARAMS_CTLE    4

#define MAX_PRIORITY       10

/* Scheduling policies */
#define SCHED_POLICY_ROUND_ROBIN 0
#define SCHED_POLICY_PREEMPTIVE  1
#define SCHED_POLICY_HYBRID      2

#define SCHED_POLICY SCHED_POLICY_HYBRID

/* ===================== Event Flags ===================== */

#define EVENT_RX_ERROR     (1U << 0)
#define EVENT_CTLE_UPDATE  (1U << 1)

volatile uint32_t event_flags;

/* ===================== System Tick ===================== */

/*
 * In real RISC-V firmware, this would be driven by:
 *  - CLINT mtime / mtimecmp
 *  - or a platform timer ISR
 */
volatile uint32_t system_ticks;

/* Simulated timer ISR hook */
void SysTick_Handler(void) {
    system_ticks++;
}

/* ===================== Task Types ===================== */

typedef enum {
    TASK_RX,
    TASK_CTLE
} TaskType;

typedef enum {
    TASK_READY,
    TASK_RUNNING,
    TASK_DONE
} TaskState;

typedef struct {
    TaskType  type;
    TaskState state;

    uint8_t   priority;

    /* Time-driven scheduling */
    uint32_t  period_ticks;
    uint32_t  next_release;

    /* Event-driven scheduling */
    uint32_t  event_mask;

    /* Bounded execution */
    int       remaining_iters;

    float localRXParams[NUM_PARAMS_RX];
    float localCTLEParams[NUM_PARAMS_CTLE];

    float rxErrorBuffer[NUM_PARAMS_RX];
    float ctleErrorBuffer[NUM_PARAMS_CTLE];

} Task;

/* ===================== Globals ===================== */

Task taskQueue[NUM_TASKS];

float globalRXParams[NUM_PARAMS_RX];
float globalCTLEParams[NUM_PARAMS_CTLE];

/* Round-robin cursor */
static int rr_index;

/* ===================== Task Implementations ===================== */

/*
 * Tasks execute ONE bounded step per scheduler invocation.
 * This enables deterministic, preemptive-like behaviour
 * without context switching.
 */

void RX(Task *task) {
    task->state = TASK_RUNNING;

    /* --- One iteration of RX adaptation --- */
    /* TODO: send bit, probe output */
    /* TODO: update rxErrorBuffer */
    /* TODO: adjust localRXParams */

    task->remaining_iters--;

    if (task->remaining_iters == 0) {
        for (int i = 0; i < NUM_PARAMS_RX; i++)
            globalRXParams[i] = task->localRXParams[i];

        task->state = TASK_DONE;
        event_flags &= ~EVENT_RX_ERROR;
    }
}

void CTLE(Task *task) {
    task->state = TASK_RUNNING;

    /* --- One iteration of CTLE update --- */
    /* TODO: send bit, probe output */
    /* TODO: update ctleErrorBuffer */
    /* TODO: adjust localCTLEParams */

    task->remaining_iters--;

    if (task->remaining_iters == 0) {
        for (int i = 0; i < NUM_PARAMS_CTLE; i++)
            globalCTLEParams[i] = task->localCTLEParams[i];

        task->state = TASK_DONE;
        event_flags &= ~EVENT_CTLE_UPDATE;
    }
}

/* ===================== Scheduler Helpers ===================== */

static int all_tasks_done(void) {
    for (int i = 0; i < NUM_TASKS; i++) {
        if (taskQueue[i].state != TASK_DONE)
            return 0;
    }
    return 1;
}

static int task_ready(Task *t) {
    if (t->state == TASK_DONE)
        return 0;

    if (system_ticks < t->next_release)
        return 0;

    if (t->event_mask && !(event_flags & t->event_mask))
        return 0;

    return 1;
}

/* ===================== Scheduler ===================== */

void schedule(void) {
    while (!all_tasks_done()) {

        SysTick_Handler(); /* Simulated timer tick */

#if (SCHED_POLICY == SCHED_POLICY_ROUND_ROBIN)

        for (int i = 0; i < NUM_TASKS; i++) {
            int idx = (rr_index + i) % NUM_TASKS;
            Task *t = &taskQueue[idx];

            if (!task_ready(t))
                continue;

            rr_index = idx + 1;
            t->next_release = system_ticks + t->period_ticks;

            if (t->type == TASK_RX) RX(t);
            else                   CTLE(t);

            break;
        }

#elif (SCHED_POLICY == SCHED_POLICY_PREEMPTIVE)

        for (int p = 0; p < MAX_PRIORITY; p++) {
            for (int i = 0; i < NUM_TASKS; i++) {
                Task *t = &taskQueue[i];

                if (t->priority != p || !task_ready(t))
                    continue;

                t->next_release = system_ticks + t->period_ticks;

                if (t->type == TASK_RX) RX(t);
                else                   CTLE(t);

                goto next_tick;
            }
        }

#elif (SCHED_POLICY == SCHED_POLICY_HYBRID)

        /* Hybrid definition:
         *  - Event-triggered tasks preempt
         *  - Otherwise round-robin
         */

        if (event_flags) {
            for (int p = 0; p < MAX_PRIORITY; p++) {
                for (int i = 0; i < NUM_TASKS; i++) {
                    Task *t = &taskQueue[i];

                    if (t->priority != p || !task_ready(t))
                        continue;

                    t->next_release = system_ticks + t->period_ticks;

                    if (t->type == TASK_RX) RX(t);
                    else                   CTLE(t);

                    goto next_tick;
                }
            }
        } else {
            for (int i = 0; i < NUM_TASKS; i++) {
                int idx = (rr_index + i) % NUM_TASKS;
                Task *t = &taskQueue[idx];

                if (!task_ready(t))
                    continue;

                rr_index = idx + 1;
                t->next_release = system_ticks + t->period_ticks;

                if (t->type == TASK_RX) RX(t);
                else                   CTLE(t);

                break;
            }
        }

#endif

    next_tick:
        ;
    }
}

/* ===================== Main ===================== */

int main(void) {

    for (int i = 0; i < NUM_TASKS; i++) {
        Task *t = &taskQueue[i];

        t->type = (i < NUM_TASKS / 2) ? TASK_RX : TASK_CTLE;
        t->priority = rand() % MAX_PRIORITY;
        t->state = TASK_READY;

        t->period_ticks = (rand() % 5) + 1;
        t->next_release = 0;

        t->remaining_iters = (rand() % 10) + 3;

        t->event_mask = (t->type == TASK_RX) ?
                         EVENT_RX_ERROR :
                         EVENT_CTLE_UPDATE;

        for (int j = 0; j < NUM_PARAMS_RX; j++)
            t->localRXParams[j] = (float)(rand() % 100) / 100.0f;

        for (int j = 0; j < NUM_PARAMS_CTLE; j++)
            t->localCTLEParams[j] = (float)(rand() % 100) / 100.0f;
    }

    /* Simulate initial events */
    event_flags = EVENT_RX_ERROR | EVENT_CTLE_UPDATE;

    schedule();
    return 0;
}
