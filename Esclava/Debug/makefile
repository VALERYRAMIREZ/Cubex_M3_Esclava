################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Esclava.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Esclava.list \

OBJCOPY_HEX += \
Esclava.hex \

OBJCOPY_BIN += \
Esclava.bin \


# All Target
all: Esclava.elf secondary-outputs

# Tool invocations
Esclava.elf: $(OBJS) $(USER_OBJS) C:\Users\valery\Desktop\Cube\Cubex2\Esclava\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "Esclava.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\valery\Desktop\Cube\Cubex2\Esclava\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Esclava.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Esclava.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Esclava.list"
	@echo 'Finished building: $@'
	@echo ' '

Esclava.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Esclava.hex"
	@echo 'Finished building: $@'
	@echo ' '

Esclava.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Esclava.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
