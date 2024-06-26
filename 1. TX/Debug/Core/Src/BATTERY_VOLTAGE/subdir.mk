################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BATTERY_VOLTAGE/adc_voltage.c 

OBJS += \
./Core/Src/BATTERY_VOLTAGE/adc_voltage.o 

C_DEPS += \
./Core/Src/BATTERY_VOLTAGE/adc_voltage.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BATTERY_VOLTAGE/%.o Core/Src/BATTERY_VOLTAGE/%.su Core/Src/BATTERY_VOLTAGE/%.cyclo: ../Core/Src/BATTERY_VOLTAGE/%.c Core/Src/BATTERY_VOLTAGE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BATTERY_VOLTAGE

clean-Core-2f-Src-2f-BATTERY_VOLTAGE:
	-$(RM) ./Core/Src/BATTERY_VOLTAGE/adc_voltage.cyclo ./Core/Src/BATTERY_VOLTAGE/adc_voltage.d ./Core/Src/BATTERY_VOLTAGE/adc_voltage.o ./Core/Src/BATTERY_VOLTAGE/adc_voltage.su

.PHONY: clean-Core-2f-Src-2f-BATTERY_VOLTAGE

