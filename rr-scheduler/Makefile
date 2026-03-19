CC = riscv64-unknown-elf-gcc
CFLAGS = -march=rv32imafd -mabi=ilp32d -nostdlib -ffreestanding -O2

all: kernel.elf

kernel.elf: start.S main.c scheduler.c context.S timer.S
	$(CC) $(CFLAGS) -T linker.ld $^ -o $@

run:
	qemu-system-riscv32 \
	-nographic \
	-machine virt \
	-bios none \
	-kernel kernel.elf

clean:
	rm -f *.elf