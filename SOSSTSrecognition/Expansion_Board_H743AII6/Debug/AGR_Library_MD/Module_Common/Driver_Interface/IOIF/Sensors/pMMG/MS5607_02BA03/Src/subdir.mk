################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.c 

OBJS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/%.o AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/%.su AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/%.cyclo: ../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/%.c AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Sensors-2f-pMMG-2f-MS5607_02BA03-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Sensors-2f-pMMG-2f-MS5607_02BA03-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/pMMG/MS5607_02BA03/Src/ioif_ms5607_02ba03.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Sensors-2f-pMMG-2f-MS5607_02BA03-2f-Src

