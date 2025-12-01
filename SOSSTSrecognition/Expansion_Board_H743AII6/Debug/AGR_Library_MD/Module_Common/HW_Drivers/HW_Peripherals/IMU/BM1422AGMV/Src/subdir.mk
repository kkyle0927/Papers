################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.c 

OBJS += \
./AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/%.o AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/%.su AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/%.cyclo: ../AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/%.c AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-HW_Drivers-2f-HW_Peripherals-2f-IMU-2f-BM1422AGMV-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-HW_Drivers-2f-HW_Peripherals-2f-IMU-2f-BM1422AGMV-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.cyclo ./AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.d ./AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.o ./AGR_Library_MD/Module_Common/HW_Drivers/HW_Peripherals/IMU/BM1422AGMV/Src/bm1422agmv.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-HW_Drivers-2f-HW_Peripherals-2f-IMU-2f-BM1422AGMV-2f-Src

