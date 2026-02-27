OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 128M
}

SECTIONS
{
  . = ORIGIN(RAM);

  .text : ALIGN(4)
  {
    *(.text.init)
    *(.text .text.*)
  } > RAM

  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*)
  } > RAM

  .data : ALIGN(4)
  {
    _sdata = .;
    *(.data .data.*)
    _edata = .;
  } > RAM

  .bss : ALIGN(4)
  {
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    _ebss = .;
  } > RAM

  . = ALIGN(16);
  _stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
