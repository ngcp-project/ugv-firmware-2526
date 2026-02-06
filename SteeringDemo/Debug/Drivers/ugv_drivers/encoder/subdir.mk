################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ugv_drivers/encoder/ugv_encoder.c 

OBJS += \
./Drivers/ugv_drivers/encoder/ugv_encoder.o 

C_DEPS += \
./Drivers/ugv_drivers/encoder/ugv_encoder.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ugv_drivers/encoder/%.o Drivers/ugv_drivers/encoder/%.su Drivers/ugv_drivers/encoder/%.cyclo: ../Drivers/ugv_drivers/encoder/%.c Drivers/ugv_drivers/encoder/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ugv_drivers-2f-encoder

clean-Drivers-2f-ugv_drivers-2f-encoder:
	-$(RM) ./Drivers/ugv_drivers/encoder/ugv_encoder.cyclo ./Drivers/ugv_drivers/encoder/ugv_encoder.d ./Drivers/ugv_drivers/encoder/ugv_encoder.o ./Drivers/ugv_drivers/encoder/ugv_encoder.su

.PHONY: clean-Drivers-2f-ugv_drivers-2f-encoder

