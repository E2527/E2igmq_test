# STM32F303K8 モーター制御ノードのビルド
#
#   make                  … ID 1 の基板用をビルド(build/node1/motor_node.{elf,hex,bin})
#   make NODE_ID=2        … ID 2 の基板用をビルド
#   make TEST_MODE=1      … CAN なしで目標速度 APP_TEST_SPEED で回す配線確認用
#   make clean
#
# 必要なもの: arm-none-eabi-gcc(dev コンテナには入っている)、firmware/lib のサブモジュール

NODE_ID ?= 1
TEST_MODE ?= 0
TARGET = motor_node
BUILD_DIR = build/node$(NODE_ID)$(if $(filter 1,$(TEST_MODE)),-test)

LIB = ../lib
COMMON = ../stm32_can
HAL = $(LIB)/stm32f3xx-hal-driver
DEVICE = $(LIB)/cmsis_device_f3

C_SOURCES = \
	Core/Src/main.c \
	Core/Src/app.c \
	Core/Src/stm32f3xx_it.c \
	Core/Src/stm32f3xx_hal_msp.c \
	Core/Src/syscalls.c \
	$(COMMON)/can_node.c \
	$(COMMON)/motor.c \
	$(COMMON)/speed_pid.c \
	$(DEVICE)/Source/Templates/system_stm32f3xx.c \
	$(addprefix $(HAL)/Src/stm32f3xx_hal, .c _cortex.c _rcc.c _rcc_ex.c _gpio.c _dma.c _flash.c \
		_flash_ex.c _pwr.c _pwr_ex.c _tim.c _tim_ex.c _can.c _exti.c)

ASM_SOURCES = $(DEVICE)/Source/Templates/gcc/startup_stm32f303x8.s
LDSCRIPT = STM32F303K8Tx_FLASH.ld

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
C_DEFS = -DUSE_HAL_DRIVER -DSTM32F303x8 -DAPP_NODE_ID=$(NODE_ID) -DAPP_TEST_MODE=$(TEST_MODE)
C_INCLUDES = -ICore/Inc -I$(COMMON) -I$(HAL)/Inc -I$(DEVICE)/Include -I$(LIB)/cmsis-core/Include

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -std=gnu11 -O2 -g3 -Wall -Wextra \
	-fdata-sections -ffunction-sections -MMD -MP
ASFLAGS = $(MCU) -g3
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--print-memory-usage

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o))) \
	$(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

.PHONY: all clean check-deps
all: check-deps $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

check-deps:
	@test -f $(HAL)/Inc/stm32f3xx_hal.h || { \
		echo "HAL がありません。リポジトリの一番上で git submodule update --init を実行してください"; exit 1; }
	@command -v $(CC) >/dev/null || { \
		echo "$(CC) がありません。dev コンテナの中で実行するか、sudo apt install gcc-arm-none-eabi を実行してください"; exit 1; }

# ST の HAL は -Wextra の「使っていない引数」の警告が出るので、HAL のファイルだけ止める
HAL_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(filter $(HAL)/%,$(C_SOURCES:.c=.o))))
$(HAL_OBJECTS): CFLAGS += -Wno-unused-parameter

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary -S $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf build

-include $(wildcard $(BUILD_DIR)/*.d)
