################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.c 

OBJS += \
./AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.o 

C_DEPS += \
./AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/%.o AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/%.su AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/%.cyclo: ../AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/%.c AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.c_includes.args"

clean: clean-AGR_Library_MD-2f-Apps-2f-Tasks-2f-ExpPack_Ctrl-2f-Src

clean-AGR_Library_MD-2f-Apps-2f-Tasks-2f-ExpPack_Ctrl-2f-Src:
	-$(RM) ./AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.cyclo ./AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.d ./AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.o ./AGR_Library_MD/Apps/Tasks/ExpPack_Ctrl/Src/exppack_ctrl.su

.PHONY: clean-AGR_Library_MD-2f-Apps-2f-Tasks-2f-ExpPack_Ctrl-2f-Src

