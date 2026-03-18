/*
 * scheduler.c — Tick-Based Preemptive Round-Robin Scheduler (Max-Heap)
 *               with full register dump on every context switch.
 *
 * Compile:  gcc -O2 -D_GNU_SOURCE -o scheduler scheduler.c
 * Run:      ./scheduler
 */

#define _GNU_SOURCE   /* expose REG_* enum and mcontext gregs */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <ucontext.h>

/* ── tunables ─────────────────────────────────────────── */
#define STACK_SIZE       (64 * 1024)
#define TICK_INTERVAL_US  10000       /* 10 ms per tick */
#define TICKS_PER_SLICE   3
#define MAX_TASKS         64

/* ── task states ──────────────────────────────────────── */
typedef enum { TASK_READY, TASK_RUNNING, TASK_DONE } TaskState;

/* ── TCB ──────────────────────────────────────────────── */
typedef struct {
    int         id;
    int         priority;
    TaskState   state;
    int         ticks_remaining;
    int         total_ticks;
    ucontext_t  ctx;
    char       *stack;
    void      (*fn)(void);
} TCB;

/* ── globals ──────────────────────────────────────────── */
static TCB       *heap[MAX_TASKS];
static int        heap_size = 0;
static int        next_id   = 0;
static ucontext_t scheduler_ctx;
static TCB       *current = NULL;

/* ── heap ─────────────────────────────────────────────── */
static inline int par(int i) { return i/2; }
static inline int lc(int i)  { return i*2; }
static inline int rc(int i)  { return i*2+1; }
static void hswap(int a,int b){ TCB*t=heap[a];heap[a]=heap[b];heap[b]=t; }
static void sift_up(int i){
    while(i>1&&heap[i]->priority>heap[par(i)]->priority){hswap(i,par(i));i=par(i);}
}
static void sift_down(int i){
    int lg=i,l=lc(i),r=rc(i);
    if(l<=heap_size&&heap[l]->priority>heap[lg]->priority)lg=l;
    if(r<=heap_size&&heap[r]->priority>heap[lg]->priority)lg=r;
    if(lg!=i){hswap(i,lg);sift_down(lg);}
}
static void heap_push(TCB*t){
    if(heap_size>=MAX_TASKS){fprintf(stderr,"heap full\n");return;}
    heap[++heap_size]=t; sift_up(heap_size);
}
static TCB*heap_pop(void){
    if(!heap_size)return NULL;
    TCB*top=heap[1]; heap[1]=heap[heap_size--];
    if(heap_size)sift_down(1);
    return top;
}

/* ── register dump ────────────────────────────────────── */

/*
 * All 23 general-purpose registers saved in mcontext_t on x86-64 Linux.
 * Order matches the gregset_t array indices (REG_* enums from sys/ucontext.h).
 */
static const char *reg_names[] = {
    "r8   ", "r9   ", "r10  ", "r11  ", "r12  ", "r13  ", "r14  ", "r15  ",
    "rdi  ", "rsi  ", "rbp  ", "rbx  ", "rdx  ", "rax  ", "rcx  ",
    "rsp  ", "rip  ", "efl  ", "cs/gs/fs",
    "err  ", "trapno", "oldmask", "cr2  "
};

static void dump_regs(const char *label, int task_id, const ucontext_t *ctx) {
    const gregset_t *g = &ctx->uc_mcontext.gregs;
    printf("\n  ┌─ %s  [task %d] ──────────────────────────────\n", label, task_id);
    /* Print in rows of 4 for readability */
    for (int i = 0; i < NGREG; i++) {
        if (i % 4 == 0) printf("  │  ");
        printf("%-10s 0x%016llx   ", reg_names[i], (unsigned long long)(*g)[i]);
        if (i % 4 == 3 || i == NGREG-1) printf("\n");
    }

    /* FPU/SSE summary — just MXCSR and FP control word, not all 16 XMMs */
    if (ctx->uc_mcontext.fpregs) {
        printf("  │  mxcsr      0x%08x           "
               "fp_cw       0x%04x\n",
               ctx->uc_mcontext.fpregs->mxcsr,
               ctx->uc_mcontext.fpregs->cwd);
    }
    printf("  └────────────────────────────────────────────────\n\n");
}

/* ── tick handler (preemption) ────────────────────────── */
static void tick_handler(int sig) {
    (void)sig;
    if (!current || current->state != TASK_RUNNING) return;

    current->total_ticks++;
    current->ticks_remaining--;

    if (current->ticks_remaining <= 0) {
        current->state = TASK_READY;

        /* Dump registers being SAVED for the outgoing task */
        dump_regs("SAVING  (outgoing)", current->id, &current->ctx);

        swapcontext(&current->ctx, &scheduler_ctx);

        /*
         * Resumed here after rescheduling.
         * current now points to THIS task again (scheduler restores it).
         * Dump the registers being LOADED back in.
         */
        dump_regs("LOADING (resuming)", current->id, &current->ctx);
    }
}

static void start_timer(void) {
    struct sigaction sa = { .sa_handler = tick_handler, .sa_flags = SA_RESTART };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGALRM, &sa, NULL);
    struct itimerval itv = {
        .it_interval = {0, TICK_INTERVAL_US},
        .it_value    = {0, TICK_INTERVAL_US},
    };
    setitimer(ITIMER_REAL, &itv, NULL);
}

static void stop_timer(void) {
    struct itimerval itv = {0};
    setitimer(ITIMER_REAL, &itv, NULL);
}

/* ── trampoline + scheduler ───────────────────────────── */
static void task_entry(void) {
    current->fn();
    current->state = TASK_DONE;
    swapcontext(&current->ctx, &scheduler_ctx);
}

static void scheduler_run(void) {
    while (heap_size > 0) {
        TCB *t = heap_pop();
        if (!t || t->state == TASK_DONE) continue;

        t->state           = TASK_RUNNING;
        t->ticks_remaining = TICKS_PER_SLICE;
        current            = t;

        printf("[sched] → dispatching task %d (priority %d)\n",
               t->id, t->priority);

        /* Dump the context we're about to jump INTO */
        dump_regs("LOADING (dispatch)", t->id, &t->ctx);

        swapcontext(&scheduler_ctx, &t->ctx);

        if (t->state == TASK_DONE) {
            printf("[sched] ✓ task %d done (total ticks: %d)\n\n",
                   t->id, t->total_ticks);
            free(t->stack);
            free(t);
        } else {
            printf("[sched] ↺ task %d preempted, back to heap\n\n", t->id);
            t->state = TASK_READY;
            heap_push(t);
        }
    }

    stop_timer();
    printf("[sched] all tasks complete.\n");
}

/* ── public API ───────────────────────────────────────── */
int create_task(void (*fn)(void), int priority) {
    TCB *t = calloc(1, sizeof(TCB));
    if (!t) return -1;
    t->id = next_id++; t->priority = priority;
    t->state = TASK_READY; t->fn = fn;
    t->stack = malloc(STACK_SIZE);
    if (!t->stack) { free(t); return -1; }
    getcontext(&t->ctx);
    t->ctx.uc_stack.ss_sp   = t->stack;
    t->ctx.uc_stack.ss_size = STACK_SIZE;
    t->ctx.uc_link          = NULL;
    makecontext(&t->ctx, task_entry, 0);
    heap_push(t);
    printf("[init]  task %d created (priority %d)\n", t->id, priority);
    return t->id;
}

/* ── testbench ────────────────────────────────────────── */
static volatile long sink = 0;
static void busy(int n) { for(int i=0;i<n;i++) sink++; }

static void task_A(void) {
    printf("  [A] starting\n");
    for(int i=0;i<4;i++){ busy(5000000); printf("  [A] step %d\n",i+1); }
    printf("  [A] done\n");
}
static void task_B(void) {
    printf("  [B] starting\n");
    for(int i=0;i<3;i++){ busy(5000000); printf("  [B] step %d\n",i+1); }
    printf("  [B] done\n");
}
static void task_C(void) {
    printf("  [C] high-priority burst\n");
    busy(1000000);
    printf("  [C] done\n");
}

/* ── main ─────────────────────────────────────────────── */
int main(void) {
    printf("=== Tick-Based Preemptive Scheduler (register debug) ===\n");
    printf("    tick interval : %d us  |  ticks/slice : %d\n\n",
           TICK_INTERVAL_US, TICKS_PER_SLICE);

    create_task(task_A, 3);
    create_task(task_B, 2);
    create_task(task_C, 5);

    start_timer();
    printf("\n--- dispatch start ---\n\n");
    scheduler_run();
    printf("\n=== done ===\n");
    return 0;
}
