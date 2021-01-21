################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Pulse_Sensor/Src/Max30102.c 

OBJS += \
./Pulse_Sensor/Src/Max30102.o 

C_DEPS += \
./Pulse_Sensor/Src/Max30102.d 


# Each subdirectory must supply rules for building sources it contributes
Pulse_Sensor/Src/Max30102.o: ../Pulse_Sensor/Src/Max30102.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I"D:/Work/Programs/MAX30102/Pulse_Sensor/Src" -I"D:/Work/Programs/MAX30102/Pulse_Sensor/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Pulse_Sensor/Src/Max30102.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

