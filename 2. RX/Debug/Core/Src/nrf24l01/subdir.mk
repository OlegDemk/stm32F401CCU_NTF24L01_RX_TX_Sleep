################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/nrf24l01/nrf24L01.c 

OBJS += \
./Core/Src/nrf24l01/nrf24L01.o 

C_DEPS += \
./Core/Src/nrf24l01/nrf24L01.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/nrf24l01/%.o Core/Src/nrf24l01/%.su Core/Src/nrf24l01/%.cyclo: ../Core/Src/nrf24l01/%.c Core/Src/nrf24l01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-nrf24l01

clean-Core-2f-Src-2f-nrf24l01:
	-$(RM) ./Core/Src/nrf24l01/nrf24L01.cyclo ./Core/Src/nrf24l01/nrf24L01.d ./Core/Src/nrf24l01/nrf24L01.o ./Core/Src/nrf24l01/nrf24L01.su

.PHONY: clean-Core-2f-Src-2f-nrf24l01

