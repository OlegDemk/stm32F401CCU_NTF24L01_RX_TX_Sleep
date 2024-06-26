################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/RTC_DATA/rtc_data_write_read.c 

OBJS += \
./Core/Src/RTC_DATA/rtc_data_write_read.o 

C_DEPS += \
./Core/Src/RTC_DATA/rtc_data_write_read.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/RTC_DATA/%.o Core/Src/RTC_DATA/%.su Core/Src/RTC_DATA/%.cyclo: ../Core/Src/RTC_DATA/%.c Core/Src/RTC_DATA/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-RTC_DATA

clean-Core-2f-Src-2f-RTC_DATA:
	-$(RM) ./Core/Src/RTC_DATA/rtc_data_write_read.cyclo ./Core/Src/RTC_DATA/rtc_data_write_read.d ./Core/Src/RTC_DATA/rtc_data_write_read.o ./Core/Src/RTC_DATA/rtc_data_write_read.su

.PHONY: clean-Core-2f-Src-2f-RTC_DATA

