################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/ICM20608G/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/TIMER/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/SPI/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/I2C/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/GPIO/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/FLASH/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/FDCAN/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/BSP_COMMON/Src/subdir.mk
-include AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/ADC/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/BM1422AGMV/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/GRF/GRF_Sensor/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/FSR/SZH-HWS004/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/FSR/RA30P/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/EMG/PSL-iEMG2/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/NZR_LED/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/LED_24ch_Driver/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Buzzer/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/FileSystem/Fatfs_Integration/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/FileSystem/FATFS_SDMMC/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Communication/BLE/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/IOIF/AudioPlay/WavPlayer_SAI/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/CDI/Risk_Mngr/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/subdir.mk
-include AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/subdir.mk
-include AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/subdir.mk
-include AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/subdir.mk
-include AGR_Library_MD/Apps/Tasks/Algorithm_Ctrl/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Expansion_Board_H743AII6
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Expansion_Board_H743AII6.elf \

MAP_FILES += \
Expansion_Board_H743AII6.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Expansion_Board_H743AII6.list \


# All Target
all: main-build

# Main-build Target
main-build: Expansion_Board_H743AII6.elf secondary-outputs

# Tool invocations
Expansion_Board_H743AII6.elf Expansion_Board_H743AII6.map: $(OBJS) $(USER_OBJS) C:\Users\ChanyoungKo\STM32CubeIDE\workspace_1.17.0\Expansion_Board_H743AII6\STM32H743AIIX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Expansion_Board_H743AII6.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\ChanyoungKo\STM32CubeIDE\workspace_1.17.0\Expansion_Board_H743AII6\STM32H743AIIX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Expansion_Board_H743AII6.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Expansion_Board_H743AII6.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Expansion_Board_H743AII6.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Expansion_Board_H743AII6.elf Expansion_Board_H743AII6.list Expansion_Board_H743AII6.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
