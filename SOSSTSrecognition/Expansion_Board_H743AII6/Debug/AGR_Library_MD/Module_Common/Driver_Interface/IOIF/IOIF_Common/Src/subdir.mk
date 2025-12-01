################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.c \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_fdcan_common.c \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_gpio_common.c \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_i2c_common.c \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_spi_common.c \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_tim_common.c \
../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_uart_common.c 

OBJS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.o \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_fdcan_common.o \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_gpio_common.o \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_i2c_common.o \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_spi_common.o \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_tim_common.o \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_uart_common.o 

C_DEPS += \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.d \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_fdcan_common.d \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_gpio_common.d \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_i2c_common.d \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_spi_common.d \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_tim_common.d \
./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_uart_common.d 


# Each subdirectory must supply rules for building sources it contributes
AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/%.o AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/%.su AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/%.cyclo: ../AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/%.c AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" @"AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.c_includes.args"

clean: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-IOIF_Common-2f-Src

clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-IOIF_Common-2f-Src:
	-$(RM) ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_adc_common.su ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_fdcan_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_fdcan_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_fdcan_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_fdcan_common.su ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_gpio_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_gpio_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_gpio_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_gpio_common.su ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_i2c_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_i2c_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_i2c_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_i2c_common.su ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_spi_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_spi_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_spi_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_spi_common.su ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_tim_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_tim_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_tim_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_tim_common.su ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_uart_common.cyclo ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_uart_common.d ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_uart_common.o ./AGR_Library_MD/Module_Common/Driver_Interface/IOIF/IOIF_Common/Src/ioif_uart_common.su

.PHONY: clean-AGR_Library_MD-2f-Module_Common-2f-Driver_Interface-2f-IOIF-2f-IOIF_Common-2f-Src

