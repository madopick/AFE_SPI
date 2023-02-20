################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/uart/uart_driver.c 

OBJS += \
./Core/Src/uart/uart_driver.o 

C_DEPS += \
./Core/Src/uart/uart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/uart/%.o Core/Src/uart/%.su: ../Core/Src/uart/%.c Core/Src/uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/STM32/SourceTree/AFE SPI/Core/Src/spi_slave" -I"C:/STM32/SourceTree/AFE SPI/Core/Src/uart" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-uart

clean-Core-2f-Src-2f-uart:
	-$(RM) ./Core/Src/uart/uart_driver.d ./Core/Src/uart/uart_driver.o ./Core/Src/uart/uart_driver.su

.PHONY: clean-Core-2f-Src-2f-uart

