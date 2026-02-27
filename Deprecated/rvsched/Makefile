CROSS ?= riscv64-unknown-elf-
CC     := $(CROSS)gcc
OBJCOPY:= $(CROSS)objcopy
OBJDUMP:= $(CROSS)objdump
SIZE   := $(CROSS)size

ARCH   := rv32imac_zicsr
ABI    := ilp32

BUILD  := build
TARGET := $(BUILD)/rvsched.elf

CFLAGS := -march=$(ARCH) -mabi=$(ABI) -mcmodel=medany -msmall-data-limit=0 \
          -ffreestanding -fno-builtin -fno-common -nostdlib -nostartfiles \
          -Wall -Wextra -Wshadow -Wdouble-promotion -Wundef \
          -O2 -g -Iinclude \
          -fdata-sections -ffunction-sections

LDFLAGS := -T link.ld -Wl,--gc-sections -Wl,-Map,$(BUILD)/rvsched.map

SRCS := start.S trap.S \
        platform/uart16550.c platform/timer_clint.c \
        kernel/mini_libc.c kernel/sched.c kernel/event.c kernel/trap_handler.c \
		app/task_table.c app/spawn.c app/dispatcher.c\
        app/main.c

OBJS := $(SRCS:%.c=$(BUILD)/%.o)
OBJS := $(OBJS:%.S=$(BUILD)/%.o)

.PHONY: all clean run disasm

all: $(TARGET) $(BUILD)/rvsched.bin

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/%.o: %.S | $(BUILD)
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS) link.ld | $(BUILD)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJS) -lgcc
	$(SIZE) $@

$(BUILD)/rvsched.bin: $(TARGET)
	$(OBJCOPY) -O binary $< $@

disasm: $(TARGET)
	$(OBJDUMP) -d $(TARGET) | less

run: $(TARGET)
	qemu-system-riscv32 -M virt -m 128M -nographic -bios none -kernel $(TARGET)

clean:
	rm -rf $(BUILD)
