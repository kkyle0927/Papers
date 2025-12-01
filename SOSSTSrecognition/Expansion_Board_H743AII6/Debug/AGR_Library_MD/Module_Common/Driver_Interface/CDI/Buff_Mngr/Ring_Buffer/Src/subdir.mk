################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.c 

OBJS += \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/%.o AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/%.su AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/%.cyclo: ../AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/%.c AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-Buff_Mngr-2f-Ring_Buffer-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-Buff_Mngr-2f-Ring_Buffer-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Buff_Mngr/Ring_Buffer/Src/ring_buffer.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-Buff_Mngr-2f-Ring_Buffer-2f-Src

