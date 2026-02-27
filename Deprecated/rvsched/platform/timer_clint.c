#include "platform.h"
#include "timer.h"
#include "csr.h"

// RV32-safe read of 64-bit mtime.
static inline uint64_t clint_read_mtime(void) {
  volatile uint32_t* mtime = (volatile uint32_t*)clint_mtime_addr();
  uint32_t hi, lo;
  do {
    hi = mtime[1];
    lo = mtime[0];
  } while (hi != mtime[1]);
  return ((uint64_t)hi << 32) | lo;
}

// RV32-safe write of 64-bit mtimecmp (avoid spurious interrupts).
static inline void clint_write_mtimecmp(uint32_t hartid, uint64_t v) {
  volatile uint32_t* cmp = (volatile uint32_t*)clint_mtimecmp_addr(hartid);
  cmp[1] = 0xFFFFFFFFu;           // set high to max first
  cmp[0] = (uint32_t)(v & 0xFFFFFFFFu);
  cmp[1] = (uint32_t)(v >> 32);
}

static uint32_t g_tick_cycles = 0;

void timer_init(uint32_t tick_cycles) {
  g_tick_cycles = tick_cycles;
  uint32_t hart = csr_read_mhartid();

  uint64_t now = clint_read_mtime();
  clint_write_mtimecmp(hart, now + (uint64_t)g_tick_cycles);

  // Enable machine-timer interrupts
  uint32_t mie = csr_read_mie();
  csr_write_mie(mie | MIE_MTIE);

  uint32_t ms = csr_read_mstatus();
  csr_write_mstatus(ms | MSTATUS_MIE);
}

void timer_ack_and_set_next(uint32_t tick_cycles) {
  uint32_t hart = csr_read_mhartid();
  uint64_t now = clint_read_mtime();
  clint_write_mtimecmp(hart, now + (uint64_t)tick_cycles);
}

uint64_t timer_now_cycles(void) {
  return clint_read_mtime();
}

uint32_t timer_tick_cycles(void) { return g_tick_cycles; }
