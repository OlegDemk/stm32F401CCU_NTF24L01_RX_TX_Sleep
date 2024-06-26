################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/INTERNAL_RTC_TIME/internal_rtc_time.c 

OBJS += \
./Core/Src/INTERNAL_RTC_TIME/internal_rtc_time.o 

C_DEPS += \
./Core/Src/INTERNAL_RTC_TIME/internal_rtc_time.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/INTERNAL_RTC_TIME/%.o Core/Src/INTERNAL_RTC_TIME/%.su Core/Src/INTERNAL_RTC_TIME/%.cyclo: ../Core/Src/INTERNAL_RTC_TIME/%.c Core/Src/INTERNAL_RTC_TIME/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-INTERNAL_RTC_TIME

clean-Core-2f-Src-2f-INTERNAL_RTC_TIME:
	-$(RM) ./Core/Src/INTERNAL_RTC_TIME/internal_rtc_time.cyclo ./Core/Src/INTERNAL_RTC_TIME/internal_rtc_time.d ./Core/Src/INTERNAL_RTC_TIME/internal_rtc_time.o ./Core/Src/INTERNAL_RTC_TIME/internal_rtc_time.su

.PHONY: clean-Core-2f-Src-2f-INTERNAL_RTC_TIME

