################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.c 

OBJS += \
./AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.o 

C_DEPS += \
./AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/%.o AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/%.su AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/%.cyclo: ../AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/%.c AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.c_includes.args"

clean: clean-AGR_Library_MD-2f-Apps-2f-Tasks-2f-Msg_Hdlr-2f-Src

clean-AGR_Library_MD-2f-Apps-2f-Tasks-2f-Msg_Hdlr-2f-Src:
	-$(RM) ./AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.cyclo ./AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.d ./AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.o ./AGR_Library_MD/Apps/Tasks/Msg_Hdlr/Src/msg_hdlr.su

.PHONY: clean-AGR_Library_MD-2f-Apps-2f-Tasks-2f-Msg_Hdlr-2f-Src

