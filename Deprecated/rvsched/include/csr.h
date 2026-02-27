#pragma once
#include <stdint.h>

static inline uint32_t csr_read_mstatus(void) {
  uint32_t x; __asm__ volatile("csrr %0, mstatus" : "=r"(x)); return x;
}
static inline void csr_write_mstatus(uint32_t x) {
  __asm__ volatile("csrw mstatus, %0" :: "r"(x));
}
static inline uint32_t csr_read_mie(void) {
  uint32_t x; __asm__ volatile("csrr %0, mie" : "=r"(x)); return x;
}
static inline void csr_write_mie(uint32_t x) {
  __asm__ volatile("csrw mie, %0" :: "r"(x));
}
static inline void csr_write_mtvec(uint32_t x) {
  __asm__ volatile("csrw mtvec, %0" :: "r"(x));
}
static inline uint32_t csr_read_mhartid(void) {
  uint32_t x; __asm__ volatile("csrr %0, mhartid" : "=r"(x)); return x;
}

static inline uint32_t irq_disable(void) {
  uint32_t m = csr_read_mstatus();
  __asm__ volatile("csrc mstatus, %0" :: "r"(1u << 3)); // clear MIE
  return m;
}
static inline void irq_restore(uint32_t mstatus_prev) {
  csr_write_mstatus(mstatus_prev);
}

enum {
  MSTATUS_MIE  = (1u << 3),
  MSTATUS_MPIE = (1u << 7),
  MSTATUS_MPP_SHIFT = 11,
  MSTATUS_MPP_MASK  = (3u << MSTATUS_MPP_SHIFT),
  MSTATUS_MPP_M     = (3u << MSTATUS_MPP_SHIFT),

  MIE_MTIE = (1u << 7),
};
