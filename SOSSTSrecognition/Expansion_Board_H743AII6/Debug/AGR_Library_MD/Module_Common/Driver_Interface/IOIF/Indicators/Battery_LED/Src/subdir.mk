################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.c 

OBJS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/%.o AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/%.su AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/%.cyclo: ../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/%.c AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Indicators-2f-Battery_LED-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Indicators-2f-Battery_LED-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Indicators/Battery_LED/Src/ioif_battery_led.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Indicators-2f-Battery_LED-2f-Src

