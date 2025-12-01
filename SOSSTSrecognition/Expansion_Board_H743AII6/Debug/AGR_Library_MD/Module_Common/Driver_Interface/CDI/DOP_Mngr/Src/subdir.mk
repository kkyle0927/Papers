################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.c \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_common.c \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_dictionaries.c \
../AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_interface.c 

OBJS += \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.o \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_common.o \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_dictionaries.o \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_interface.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.d \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_common.d \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_dictionaries.d \
./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_interface.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/%.o AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/%.su AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/%.cyclo: ../AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/%.c AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-DOP_Mngr-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-DOP_Mngr-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object.su ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_common.su ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_dictionaries.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_dictionaries.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_dictionaries.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_dictionaries.su ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_interface.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_interface.d ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_interface.o ./AGR_Library_MD/Module_Common/Driver_Interface/CDI/DOP_Mngr/Src/data_object_interface.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-CDI-2f-DOP_Mngr-2f-Src

