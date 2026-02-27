#pragma once
#include <stdint.h>
#include "sched.h"

typedef struct {
  uint32_t id;
  task_fn_t fn;
  uint8_t prio;
  uint8_t slice;
} TaskDef;

const TaskDef* task_lookup(uint32_t id);
void task_list_print(void);
