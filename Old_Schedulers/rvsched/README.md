# rvsched (bare-metal RV32 scheduler on QEMU virt)

This is a tiny, deterministic scheduler that runs in RISC-V machine-mode on QEMU's `virt` board.
It supports:
- Time-driven tasks (`task_sleep_ticks`, `task_sleep_until`)
- Event-driven tasks (`task_wait_events`, `task_set_events`)
- Policies: round-robin (RR), strict preemptive priority, and hybrid (priority + RR per priority)

## Build

Requires:
- riscv64-unknown-elf-gcc (bare-metal toolchain)
- qemu-system-riscv32 (comes with `qemu-system-misc` on many distros)

```bash
make
```

## Run

```bash
make run
```

Expected output: periodic `A` and `b` characters, with `[EVT]` printed occasionally.

## Notes

Tick period is controlled by `TICK_CYCLES` in `app/main.c`. If timing feels off, you can:
- adjust `TICK_CYCLES`, or
- dump QEMU's device-tree and read `timebase-frequency`.
