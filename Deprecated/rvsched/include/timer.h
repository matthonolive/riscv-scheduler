#pragma once
#include <stdint.h>

void timer_init(uint32_t tick_cycles);
void timer_ack_and_set_next(uint32_t tick_cycles);
uint64_t timer_now_cycles(void);
uint32_t timer_tick_cycles(void);
