#pragma once
#include <stdint.h>

// QEMU "virt" common memory map: RAM @ 0x80000000, UART0 @ 0x10000000,
// CLINT @ 0x02000000, PLIC @ 0x0c000000.
// (For portability, prefer parsing the DTB later.)
enum {
  UART0_BASE  = 0x10000000u,
  CLINT_BASE  = 0x02000000u,
};

// CLINT offsets for hart 0 on QEMU virt:
// mtimecmp @ CLINT_BASE + 0x4000
// mtime    @ CLINT_BASE + 0xBFF8
static inline uintptr_t clint_mtimecmp_addr(uint32_t hartid) {
  return (uintptr_t)(CLINT_BASE + 0x4000u + 8u * hartid);
}
static inline uintptr_t clint_mtime_addr(void) {
  return (uintptr_t)(CLINT_BASE + 0xBFF8u);
}
