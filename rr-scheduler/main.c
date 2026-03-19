#include <stdint.h>
#include "scheduler.h"

volatile uint32_t *UART0 = (uint32_t*)0x10000000;

extern void trap_vector();
extern void timer_init();

static inline void enable_interrupts() {
    asm volatile("csrw mtvec, %0" :: "r"(trap_vector));
    
    // Enable machine timer interrupt
    asm volatile("csrs mie, %0" :: "r"(1 << 7));  // MTIE
    
    // Enable global interrupt
    asm volatile("csrs mstatus, %0" :: "r"(1 << 3)); // MIE
}

void print(const char *s) {
    while (*s) {
        *UART0 = *s++;
    }
}

void task1() {
    while (1) {
        print("Task 1 running\n");
        for (volatile int i = 0; i < 100000; i++);
        yield();
    }
}

void task2() {
    while (1) {
        print("Task 2 running\n");
        for (volatile int i = 0; i < 100000; i++);
        yield();
    }
}

int main() {
    scheduler_init();

    create_task(task1);
    create_task(task2);

    schedule(); // start scheduler

    while (1);
}