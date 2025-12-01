################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.c \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_mngr.c \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_state_machine.c 

OBJS += \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.o \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_mngr.o \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_state_machine.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.d \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_mngr.d \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_state_machine.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/%.o AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/%.su AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/%.cyclo: ../AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/%.c AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-Task_State_Machine-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-Task_State_Machine-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/routine_mngr.su ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_mngr.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_mngr.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_mngr.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_mngr.su ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_state_machine.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_state_machine.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_state_machine.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/Task_State_Machine/Src/task_state_machine.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-Task_State_Machine-2f-Src

