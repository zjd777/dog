.DEFAULT_GOAL := all
SHELL := cmd.exe
.SHELLFLAGS := /C

TARGET := CtrBoard-H7_IMU
BUILD_DIR := build
PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
CXX := $(PREFIX)g++
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size
OPENOCD ?= openocd
OPT ?= -Og

include build_support/sources.mk

CPU := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
DEFINES := -DUSE_HAL_DRIVER -DSTM32H723xx -DARM_MATH_CM7 -D__PROGRAM_START=Reset_Handler
INCLUDES := -ICore/Inc -IBSP -Imy -IDevice/unitree -IDevice/remote_control \
 -IDevice/BMI088/inc -IDevice/BMI088 \
 -IDrivers/STM32H7xx_HAL_Driver/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
 -IDrivers/CMSIS/Device/ST/STM32H7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/DSP/Include
COMMON := $(CPU) $(DEFINES) $(INCLUDES) $(OPT) -g3 -ffunction-sections -fdata-sections -funsigned-char -Wall -Wextra -MMD -MP
CFLAGS := $(COMMON) -std=gnu99
CXXFLAGS := $(COMMON) -std=gnu++17 -fno-exceptions -fno-rtti -fno-use-cxa-atexit
LINKER_SCRIPT := build_support/STM32H723VG_FLASH.ld
LDFLAGS := $(CPU) -T$(LINKER_SCRIPT) -nostartfiles --specs=nano.specs --specs=nosys.specs \
 -Wl,--gc-sections -Wl,-z,noexecstack -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--print-memory-usage
OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o) $(CXX_SOURCES:.cpp=.o) $(ASM_SOURCES:.s=.o))
OBJ_DIRS := $(sort $(dir $(OBJECTS)))
ELF := $(BUILD_DIR)/$(TARGET).elf

.PHONY: all clean rebuild flash probe
all: $(ELF) $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(OBJ_DIRS):
	@if not exist "$(subst /,\,$@)" mkdir "$(subst /,\,$@)"

$(BUILD_DIR)/%.o: %.c Makefile build_support/sources.mk | $(OBJ_DIRS)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile build_support/sources.mk | $(OBJ_DIRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(OBJ_DIRS)
	$(CC) $(CPU) -g3 -x assembler-with-cpp -MMD -MP -c $< -o $@

$(ELF): $(OBJECTS) $(LINKER_SCRIPT)
	$(CXX) $(OBJECTS) $(LDFLAGS) -Wl,--start-group -lc -lm -lstdc++ -lnosys -Wl,--end-group -o $@
	$(SIZE) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

flash: all
	$(OPENOCD) -f .vscode/stlink-h723.cfg -c "program {$(ELF)} verify reset exit"

probe:
	$(OPENOCD) -f .vscode/stlink-h723.cfg -c "init; shutdown"

clean:
	powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode/embedded.ps1 Clean

rebuild:
	$(MAKE) clean
	$(MAKE) all

-include $(OBJECTS:.o=.d)
