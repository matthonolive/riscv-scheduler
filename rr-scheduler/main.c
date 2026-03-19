#include <stdint.h>
#include "scheduler.h"

volatile uint32_t *UART0 = (uint32_t *)0x10000000;

void print(const char *s) {
    while (*s)
        *UART0 = *s++;
}

static void print_int(int v) {
    char buf[12];
    int  i = 0;
    if (v == 0) { print("0"); return; }
    while (v > 0) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i--) { char c = buf[i]; *UART0 = c; }
}

/* ── Task 1: cooperative yielding ──────────────────────────── */
void task1(void) {
    int count = 0;
    while (1) {
        print("Task 1 running (");
        print_int(count++);
        print(")\n");
        for (volatile int i = 0; i < 100000; i++);
        yield();
    }
}

/* ── Task 2: also yields cooperatively ─────────────────────── */
void task2(void) {
    int count = 0;
    while (1) {
        print("Task 2 running (");
        print_int(count++);
        print(")\n");
        for (volatile int i = 0; i < 100000; i++);
        yield();
    }
}

/* ── Task 3: exercises double-precision FPU ────────────────── */
void task3(void) {
    double accum = 0.0;
    int    iter  = 0;
    while (1) {
        for (int k = 0; k < 1000; k++) {
            int n = iter * 1000 + k;
            double term = 1.0 / (2.0 * n + 1.0);
            if (n & 1) accum -= term;
            else       accum += term;
        }
        iter++;
        print("Task 3 (FP): pi ~ ");
        int approx = (int)(accum * 4.0);
        print_int(approx);
        print(".");
        int frac = (int)((accum * 4.0 - approx) * 10000);
        if (frac < 0) frac = -frac;
        if (frac < 1000) print("0");
        if (frac < 100)  print("0");
        if (frac < 10)   print("0");
        print_int(frac);
        print("\n");
        yield();
    }
}

int main(void) {
    scheduler_init();

    create_task(task1);
    create_task(task2);
    create_task(task3);

    schedule();      /* launches first task – never returns */

    while (1);
}