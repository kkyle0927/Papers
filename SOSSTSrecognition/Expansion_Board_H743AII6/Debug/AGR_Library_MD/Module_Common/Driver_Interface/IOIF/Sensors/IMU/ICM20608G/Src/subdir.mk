################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.c 

OBJS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/%.o AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/%.su AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/%.cyclo: ../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/%.c AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Sensors-2f-IMU-2f-ICM20608G-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Sensors-2f-IMU-2f-ICM20608G-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/Sensors/IMU/ICM20608G/Src/ioif_icm20608g.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-Sensors-2f-IMU-2f-ICM20608G-2f-Src

