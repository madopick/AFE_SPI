################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/spi_slave/spi_slave_driver.c 

OBJS += \
./Core/Src/spi_slave/spi_slave_driver.o 

C_DEPS += \
./Core/Src/spi_slave/spi_slave_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/spi_slave/%.o Core/Src/spi_slave/%.su: ../Core/Src/spi_slave/%.c Core/Src/spi_slave/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/STM32/SourceTree/AFE SPI/Core/Src/spi_slave" -I"C:/STM32/SourceTree/AFE SPI/Core/Src/uart" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-spi_slave

clean-Core-2f-Src-2f-spi_slave:
	-$(RM) ./Core/Src/spi_slave/spi_slave_driver.d ./Core/Src/spi_slave/spi_slave_driver.o ./Core/Src/spi_slave/spi_slave_driver.su

.PHONY: clean-Core-2f-Src-2f-spi_slave

