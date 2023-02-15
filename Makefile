CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
FLAGS = -mthumb -mcpu=cortex-m4
CPPFLAGS = -DSTM32F411xE

BUILD_DIR ?= build
SRC_DIR ?= src

EXTERNAL_SRCS = stm32/src/startup_stm32.c stm32/src/delay.c stm32/src/gpio.c

SRCS := $(shell find $(SRC_DIR) -name *.c) $(EXTERNAL_SRCS)

OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

INC_DIRS=./stm32/inc \
				 ./stm32/CMSIS/Include \
				 ./stm32/CMSIS/Device/ST/STM32F4xx/Include \
				 include \
				 $(BUILD_DIR)/generated_sounds

INC_PARAMS=$(foreach dir, $(INC_DIRS), -I$(dir))

CFLAGS = $(FLAGS) -Wall -g -std=c2x \
			-O2 -ffunction-sections -fdata-sections \
			$(INC_PARAMS)

LDFLAGS = $(FLAGS) -Wl,--gc-sections -nostartfiles \
			-L./stm32/lds -Tstm32f411re.lds

.DEFAULT_GOAL := all

.PHONY: clean

TARGET = player

.SECONDARY: $(BUILD_DIR)/$(TARGET).elf $(OBJS)

$(SRC_DIR)/player.c: $(BUILD_DIR)/generated_sounds/sound_jingle.raw11.cdata

all: $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.c.o: %.c
		mkdir -p $(dir $@)
		$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.elf : $(OBJS)
		mkdir -p $(dir $@)
		$(CC) $(LDFLAGS) $^ -o $@

$(BUILD_DIR)/%.bin : $(BUILD_DIR)/%.elf
		$(OBJCOPY) $< $@ -O binary

$(BUILD_DIR)/%.raw11: ./sounds/%.raw16
		sounds/reduce_sample_size.py --input $^ --output $@ --n 11

$(BUILD_DIR)/generated_sounds/%.raw11.cdata: $(BUILD_DIR)/%.raw11
		mkdir -p $(BUILD_DIR)/generated_sounds/
		sounds/make_cdata.py --input $^ --output $@

clean:
		rm -fr build
