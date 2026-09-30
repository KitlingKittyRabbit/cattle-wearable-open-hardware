CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
CFLAGS = -mcpu=cortex-m0plus -mthumb -Wall -O2 -g3 \
         -ffunction-sections -fdata-sections \
         -fno-builtin \
         -I源码

LDFLAGS = -mcpu=cortex-m0plus -mthumb \
          -T链接脚本/STM32G030K8T6.ld \
          -Wl,--gc-sections -Wl,--print-memory-usage \
          -nostartfiles -nostdlib \
          -lgcc

BUILD = 编译输出

SRCS = 源码/main.c
OBJS = $(SRCS:源码/%.c=$(BUILD)/%.o)

TARGET = $(BUILD)/firmware

.PHONY: all flash clean

all: $(TARGET).bin $(TARGET).hex

$(BUILD)/%.o: 源码/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $^ -lgcc -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD):
	mkdir -p $@

flash: $(TARGET).bin
	st-flash --hot-plug write $< 0x08000000

flash-hex: $(TARGET).hex
	st-flash --hot-plug --format ihex write $<

erase:
	st-flash erase

clean:
	rm -rf $(BUILD)
