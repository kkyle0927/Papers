################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.c 

OBJS += \
./AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/%.o AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/%.su AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/%.cyclo: ../AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/%.c AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-HW_Drivers-2f-BSP-2f-BSP_Drivers-2f-UART-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-HW_Drivers-2f-BSP-2f-BSP_Drivers-2f-UART-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.cyclo ./AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.d ./AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.o ./AGR_Library_MD/Module_Common/HW_Drivers/BSP/BSP_Drivers/UART/Src/bsp_uart.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-HW_Drivers-2f-BSP-2f-BSP_Drivers-2f-UART-2f-Src

