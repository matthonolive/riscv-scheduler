#pragma once
#include <stdint.h>

// Must match trap.S layout exactly (RV32 words).
typedef struct TrapFrame {
  uint32_t ra;
  uint32_t gp;
  uint32_t tp;
  uint32_t t0, t1, t2;
  uint32_t s0, s1;
  uint32_t a0, a1, a2, a3, a4, a5, a6, a7;
  uint32_t s2, s3, s4, s5, s6, s7, s8, s9, s10, s11;
  uint32_t t3, t4, t5, t6;
  uint32_t mepc;
  uint32_t mstatus;
  uint32_t mcause;
  uint32_t mtval;
} TrapFrame;

TrapFrame* trap_handler(TrapFrame* tf);

// Starts first task by restoring trapframe and mret.
void arch_start_first(uintptr_t tf_sp);

// Request resched using ECALL
static inline void arch_ecall(void) {
  __asm__ volatile("ecall");
}
