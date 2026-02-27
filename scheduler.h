#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdlib.h>

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

#endif /* SCHEDULER_H */