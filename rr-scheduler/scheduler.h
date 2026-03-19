#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

#define MAX_TASKS 4
#define STACK_SIZE 1024

typedef struct {
    uint32_t *sp;   // stack pointer
    uint8_t active;
} task_t;

void scheduler_init();
int create_task(void (*func)(void));
void yield();
void schedule();

#endif