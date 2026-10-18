# STM32 側のコードを PC でテストする(HAL の代わりに hal_stub を使う)
#   make -C firmware/stm32_can/test
CC ?= gcc
CFLAGS = -std=c11 -Wall -Wextra -Werror -Wshadow -O1 -g -I hal_stub -I ..
SRCS = test_main.c hal_stub.c ../can_node.c ../speed_pid.c ../motor.c

.PHONY: test clean
test: test_firmware
	./test_firmware

test_firmware: $(SRCS) hal_stub/main.h ../can_node.h ../speed_pid.h ../motor.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) -lm

clean:
	rm -f test_firmware
