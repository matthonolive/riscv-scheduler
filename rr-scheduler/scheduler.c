#include "scheduler.h"

task_t tasks[MAX_TASKS];
uint32_t stacks[MAX_TASKS][STACK_SIZE];

int current_task = -1;
int num_tasks = 0;

extern void context_switch(uint32_t **old_sp, uint32_t *new_sp);

void scheduler_init() {
    for (int i = 0; i < MAX_TASKS; i++) {
        tasks[i].active = 0;
    }
}

int create_task(void (*func)(void)) {
    if (num_tasks >= MAX_TASKS) return -1;

    int id = num_tasks++;

    uint32_t *stack_top = &stacks[id][STACK_SIZE - 1];

    // Fake stack frame
    *(--stack_top) = (uint32_t)func; // return addr

    tasks[id].sp = stack_top;
    tasks[id].active = 1;

    return id;
}

void yield() {
    schedule();
}

void schedule() {
    int next = (current_task + 1) % num_tasks;

    int prev = current_task;
    current_task = next;

    if (prev == -1) {
        // first run
        uint32_t *dummy;
        context_switch(&dummy, tasks[next].sp);
    } else {
        context_switch(&tasks[prev].sp, tasks[next].sp);
    }
}