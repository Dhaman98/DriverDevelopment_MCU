################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f401xe_gpio_driver.c \
../drivers/src/stm32f401xe_i2c_driver.c \
../drivers/src/stm32f401xe_rcc_driver.c \
../drivers/src/stm32f401xe_spi_driver.c \
../drivers/src/stm32f401xe_usart_driver.c 

OBJS += \
./drivers/src/stm32f401xe_gpio_driver.o \
./drivers/src/stm32f401xe_i2c_driver.o \
./drivers/src/stm32f401xe_rcc_driver.o \
./drivers/src/stm32f401xe_spi_driver.o \
./drivers/src/stm32f401xe_usart_driver.o 

C_DEPS += \
./drivers/src/stm32f401xe_gpio_driver.d \
./drivers/src/stm32f401xe_i2c_driver.d \
./drivers/src/stm32f401xe_rcc_driver.d \
./drivers/src/stm32f401xe_spi_driver.d \
./drivers/src/stm32f401xe_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"D:/MCU1/Target/STM32F401xE_drivers/bsp" -I"D:/MCU1/Target/STM32F401xE_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f401xe_gpio_driver.cyclo ./drivers/src/stm32f401xe_gpio_driver.d ./drivers/src/stm32f401xe_gpio_driver.o ./drivers/src/stm32f401xe_gpio_driver.su ./drivers/src/stm32f401xe_i2c_driver.cyclo ./drivers/src/stm32f401xe_i2c_driver.d ./drivers/src/stm32f401xe_i2c_driver.o ./drivers/src/stm32f401xe_i2c_driver.su ./drivers/src/stm32f401xe_rcc_driver.cyclo ./drivers/src/stm32f401xe_rcc_driver.d ./drivers/src/stm32f401xe_rcc_driver.o ./drivers/src/stm32f401xe_rcc_driver.su ./drivers/src/stm32f401xe_spi_driver.cyclo ./drivers/src/stm32f401xe_spi_driver.d ./drivers/src/stm32f401xe_spi_driver.o ./drivers/src/stm32f401xe_spi_driver.su ./drivers/src/stm32f401xe_usart_driver.cyclo ./drivers/src/stm32f401xe_usart_driver.d ./drivers/src/stm32f401xe_usart_driver.o ./drivers/src/stm32f401xe_usart_driver.su

.PHONY: clean-drivers-2f-src

