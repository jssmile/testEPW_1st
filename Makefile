TARGET:=FreeRTOS
# TODO change to your ARM gcc toolchain path
TOOLCHAIN_ROOT:=/usr/local/csl/gcc-arm-none-eabi-4_7-2013q3
TOOLCHAIN_PATH:=$(TOOLCHAIN_ROOT)/bin
TOOLCHAIN_PREFIX:=arm-none-eabi

# Optimization level, can be [0, 1, 2, 3, s].
OPTLVL:=0
DBG:=-g

FREERTOS:=$(CURDIR)/FreeRTOS
STARTUP:=$(CURDIR)/hardware
LINKER_SCRIPT:=$(CURDIR)/Utilities/stm32_flash.ld

INCLUDE=-I$(CURDIR)/hardware
INCLUDE+=-I$(FREERTOS)/include
INCLUDE+=-I$(FREERTOS)/portable/GCC/ARM_CM4F
INCLUDE+=-I$(CURDIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
INCLUDE+=-I$(CURDIR)/Libraries/CMSIS/Include
INCLUDE+=-I$(CURDIR)/Libraries/STM32F4-Discovery
INCLUDE+=-I$(CURDIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc
INCLUDE+=-I$(CURDIR)/Libraries/usart
INCLUDE+=-I$(CURDIR)/config
INCLUDE+=-I$(CURDIR)/epw_lib
INCLUDE+=-I$(CURDIR)/sdio_lib
INCLUDE+=-I$(CURDIR)/fatfs
INCLUDE+=-I$(CURDIR)/epw_control

BUILD_DIR = $(CURDIR)/build
BIN_DIR = $(CURDIR)/binary

# vpath is used so object files are written to the current directory instead
# of the same directory as their source files
vpath %.c $(CURDIR)/Libraries/STM32F4xx_StdPeriph_Driver/src \
	  $(CURDIR)/Libraries/STM32F4-Discovery \
	  $(CURDIR)/Libraries/usart \
	  $(CURDIR)/Libraries/syscall $(CURDIR)/hardware $(FREERTOS) \
	  $(FREERTOS)/portable/MemMang $(FREERTOS)/portable/GCC/ARM_CM4F \
	  $(CURDIR)/epw_lib \
	  $(CURDIR)/sdio_lib \
	  $(CURDIR)/fatfs \
	  $(CURDIR)/epw_control

vpath %.s $(STARTUP)
ASRC=startup_stm32f4xx.s

# Project Source Files
SRC+=stm32f4xx_it.c
SRC+=system_stm32f4xx.c
SRC+=main.c
SRC+=syscalls.c

# STM32F4-Discovery
SRC+=stm32f4_discovery.c

# EPW libraries
#SRC+=transfer.c
SRC+=uart.c
SRC+=command.c
SRC+=motor.c
SRC+=encoder.c
SRC+=linear_actuator.c
SRC+=sensors.c
SRC+=PID.c
#----------------------

# epw control
SRC+=record.c
SRC+=control.c

# sdio
SRC+=sdio_debug.c
SRC+=stm32f4_discovery_sdio_sd.c
SRC+=stm32f4_discovery_sdio_sd_LowLevel.c
# fat
SRC+=ff.c
SRC+=diskio.c
#SRC+=fftest.c

# FreeRTOS Source Files
SRC+=port.c
SRC+=list.c
SRC+=queue.c
SRC+=tasks.c
SRC+=timers.c
SRC+=heap_4.c

# Standard Peripheral Source Files
SRC+=stm32f4xx_syscfg.c
SRC+=misc.c
SRC+=stm32f4xx_adc.c
#SRC+=stm32f4xx_dac.c
SRC+=stm32f4xx_dma.c
SRC+=stm32f4xx_exti.c
#SRC+=stm32f4xx_flash.c
SRC+=stm32f4xx_sdio.c
SRC+=stm32f4xx_gpio.c
SRC+=stm32f4xx_i2c.c
SRC+=stm32f4xx_rcc.c
#SRC+=stm32f4xx_spi.c
SRC+=stm32f4xx_tim.c
SRC+=stm32f4xx_usart.c
SRC+=stm32f4xx_rng.c

CDEFS=-DUSE_STDPERIPH_DRIVER
CDEFS+=-DSTM32F4XX
CDEFS+=-DHSE_VALUE=8000000
CDEFS+=-D__FPU_PRESENT=1
CDEFS+=-D__FPU_USED=1
CDEFS+=-DARM_MATH_CM4

MCUFLAGS=-mcpu=cortex-m4 -mthumb -mfloat-abi=hard
COMMONFLAGS=-O$(OPTLVL) $(DBG) -Wall
CFLAGS=$(COMMONFLAGS) $(MCUFLAGS) $(INCLUDE) $(CDEFS)
LDLIBS=$(TOOLCHAIN_ROOT)/arm-none-eabi/lib/armv7e-m/fpu/libc_s.a $(TOOLCHAIN_ROOT)/arm-none-eabi/lib/armv7e-m/fpu/libm.a
LDFLAGS=$(COMMONFLAGS) -fno-exceptions -ffunction-sections -fdata-sections -nostartfiles -Wl,--gc-sections,-T$(LINKER_SCRIPT) -v

CC=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
LD=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-objcopy
AS=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-as
AR=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-ar
GDB=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gdb

OBJ = $(SRC:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) $< -c -o $@

all: $(OBJ)
	$(AS) -o $(ASRC:%.s=$(BUILD_DIR)/%.o) $(STARTUP)/$(ASRC)
	$(CC) -o $(BIN_DIR)/$(TARGET).elf $(LDFLAGS) $(OBJ) $(ASRC:%.s=$(BUILD_DIR)/%.o) $(LDLIBS)
	$(OBJCOPY) -O ihex $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex
	$(OBJCOPY) -O binary $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).bin

.PHONY: clean

clean:
	rm -f $(OBJ)
	rm -f $(ASRC:%.s=$(BUILD_DIR)/%.o)
	rm -f $(BIN_DIR)/$(TARGET).elf
	rm -f $(BIN_DIR)/$(TARGET).hex
	rm -f $(BIN_DIR)/$(TARGET).bin

flash:
	st-flash write $(BIN_DIR)/$(TARGET).bin 0x8000000

onekey:
	make clean ; make ; make flash