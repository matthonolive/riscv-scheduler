// Trap entry/exit and first-task entry.
// Saves registers to a TrapFrame on the *current task stack*.

.section .text
.align 2
.global trap_entry

.equ TF_SIZE, 136

trap_entry:
  addi sp, sp, -TF_SIZE

  // save GPRs (skip x2/sp)
  sw ra,   0(sp)
  sw gp,   4(sp)
  sw tp,   8(sp)
  sw t0,  12(sp)
  sw t1,  16(sp)
  sw t2,  20(sp)
  sw s0,  24(sp)
  sw s1,  28(sp)
  sw a0,  32(sp)
  sw a1,  36(sp)
  sw a2,  40(sp)
  sw a3,  44(sp)
  sw a4,  48(sp)
  sw a5,  52(sp)
  sw a6,  56(sp)
  sw a7,  60(sp)
  sw s2,  64(sp)
  sw s3,  68(sp)
  sw s4,  72(sp)
  sw s5,  76(sp)
  sw s6,  80(sp)
  sw s7,  84(sp)
  sw s8,  88(sp)
  sw s9,  92(sp)
  sw s10, 96(sp)
  sw s11,100(sp)
  sw t3, 104(sp)
  sw t4, 108(sp)
  sw t5, 112(sp)
  sw t6, 116(sp)

  // save CSRs
  csrr t0, mepc
  csrr t1, mstatus
  csrr t2, mcause
  csrr t3, mtval
  sw t0, 120(sp)
  sw t1, 124(sp)
  sw t2, 128(sp)
  sw t3, 132(sp)

  // call C trap handler: TrapFrame* trap_handler(TrapFrame* tf)
  mv a0, sp
  call trap_handler

  // a0 = new trapframe pointer to resume
  mv sp, a0

restore_and_mret:
  // restore CSRs
  lw t0, 120(sp)
  lw t1, 124(sp)
  csrw mepc, t0
  csrw mstatus, t1

  // restore GPRs
  lw ra,   0(sp)
  lw gp,   4(sp)
  lw tp,   8(sp)
  lw t0,  12(sp)
  lw t1,  16(sp)
  lw t2,  20(sp)
  lw s0,  24(sp)
  lw s1,  28(sp)
  lw a0,  32(sp)
  lw a1,  36(sp)
  lw a2,  40(sp)
  lw a3,  44(sp)
  lw a4,  48(sp)
  lw a5,  52(sp)
  lw a6,  56(sp)
  lw a7,  60(sp)
  lw s2,  64(sp)
  lw s3,  68(sp)
  lw s4,  72(sp)
  lw s5,  76(sp)
  lw s6,  80(sp)
  lw s7,  84(sp)
  lw s8,  88(sp)
  lw s9,  92(sp)
  lw s10, 96(sp)
  lw s11,100(sp)
  lw t3, 104(sp)
  lw t4, 108(sp)
  lw t5, 112(sp)
  lw t6, 116(sp)

  addi sp, sp, TF_SIZE
  mret

// arch_start_first(uintptr_t tf_sp):
//   tf_sp points to a valid TrapFrame on the first task's stack.
//   Set sp = tf_sp and reuse restore path.

.align 2
.global arch_start_first

arch_start_first:
  mv sp, a0
  j restore_and_mret
