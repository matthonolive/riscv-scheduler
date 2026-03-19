ENTRY(_start)

SECTIONS {
    . = 0x80000000;

    .text : {
        *(.text*)
    }

    .rodata : {
        *(.rodata*)
    }

    .data : {
        *(.data*)
    }

    .bss : {
        *(.bss*)
    }

    . = ALIGN(4);
    _stack_top = . + 0x4000; /* 16KB stack */
}