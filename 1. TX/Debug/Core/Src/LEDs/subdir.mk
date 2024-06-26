################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LEDs/leds.c 

OBJS += \
./Core/Src/LEDs/leds.o 

C_DEPS += \
./Core/Src/LEDs/leds.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LEDs/%.o Core/Src/LEDs/%.su Core/Src/LEDs/%.cyclo: ../Core/Src/LEDs/%.c Core/Src/LEDs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LEDs

clean-Core-2f-Src-2f-LEDs:
	-$(RM) ./Core/Src/LEDs/leds.cyclo ./Core/Src/LEDs/leds.d ./Core/Src/LEDs/leds.o ./Core/Src/LEDs/leds.su

.PHONY: clean-Core-2f-Src-2f-LEDs

