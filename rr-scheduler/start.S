.section .text
.global _start

_start:
    la sp, _stack_top   # initialize stack pointer
    call main           # jump to main

1:  j 1b                # infinite loop if main returns