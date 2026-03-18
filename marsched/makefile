# Makefile for riscv-scheduler

CC = gcc
CFLAGS = -O2
LDFLAGS = -lm
TARGET = sched
SRCS = sched.c serdes_sim.c

CHANNEL_TAPS ?= channel_taps.txt

.PHONY: build run clean

build: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS) $(LDFLAGS)

run: build
	./$(TARGET) $(CHANNEL_TAPS)

clean:
	rm -f $(TARGET)