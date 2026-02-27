.section .text.init
.global _start
_start:
  // Set up a temporary boot stack (top of RAM from linker script)
  la sp, _stack_top

  // Clear .bss
  la t0, _sbss
  la t1, _ebss
1:
  bgeu t0, t1, 2f
  sw zero, 0(t0)
  addi t0, t0, 4
  j 1b
2:
  // Jump to C
  call main

  // If main returns, just park.
3:
  wfi
  j 3b
