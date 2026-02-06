################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ugv_drivers/adc/ugv_adc.c 

OBJS += \
./Drivers/ugv_drivers/adc/ugv_adc.o 

C_DEPS += \
./Drivers/ugv_drivers/adc/ugv_adc.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ugv_drivers/adc/%.o Drivers/ugv_drivers/adc/%.su Drivers/ugv_drivers/adc/%.cyclo: ../Drivers/ugv_drivers/adc/%.c Drivers/ugv_drivers/adc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ugv_drivers-2f-adc

clean-Drivers-2f-ugv_drivers-2f-adc:
	-$(RM) ./Drivers/ugv_drivers/adc/ugv_adc.cyclo ./Drivers/ugv_drivers/adc/ugv_adc.d ./Drivers/ugv_drivers/adc/ugv_adc.o ./Drivers/ugv_drivers/adc/ugv_adc.su

.PHONY: clean-Drivers-2f-ugv_drivers-2f-adc

