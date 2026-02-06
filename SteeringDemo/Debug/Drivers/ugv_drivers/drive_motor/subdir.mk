################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ugv_drivers/drive_motor/motor_control.c \
../Drivers/ugv_drivers/drive_motor/ugv_drive_motor_utilities.c 

OBJS += \
./Drivers/ugv_drivers/drive_motor/motor_control.o \
./Drivers/ugv_drivers/drive_motor/ugv_drive_motor_utilities.o 

C_DEPS += \
./Drivers/ugv_drivers/drive_motor/motor_control.d \
./Drivers/ugv_drivers/drive_motor/ugv_drive_motor_utilities.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ugv_drivers/drive_motor/%.o Drivers/ugv_drivers/drive_motor/%.su Drivers/ugv_drivers/drive_motor/%.cyclo: ../Drivers/ugv_drivers/drive_motor/%.c Drivers/ugv_drivers/drive_motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ugv_drivers-2f-drive_motor

clean-Drivers-2f-ugv_drivers-2f-drive_motor:
	-$(RM) ./Drivers/ugv_drivers/drive_motor/motor_control.cyclo ./Drivers/ugv_drivers/drive_motor/motor_control.d ./Drivers/ugv_drivers/drive_motor/motor_control.o ./Drivers/ugv_drivers/drive_motor/motor_control.su ./Drivers/ugv_drivers/drive_motor/ugv_drive_motor_utilities.cyclo ./Drivers/ugv_drivers/drive_motor/ugv_drive_motor_utilities.d ./Drivers/ugv_drivers/drive_motor/ugv_drive_motor_utilities.o ./Drivers/ugv_drivers/drive_motor/ugv_drive_motor_utilities.su

.PHONY: clean-Drivers-2f-ugv_drivers-2f-drive_motor

