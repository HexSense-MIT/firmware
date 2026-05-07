################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/class/video/video_device.c 

OBJS += \
./tinyusb/class/video/video_device.o 

C_DEPS += \
./tinyusb/class/video/video_device.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/class/video/%.o tinyusb/class/video/%.su tinyusb/class/video/%.cyclo: ../tinyusb/class/video/%.c tinyusb/class/video/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-class-2f-video

clean-tinyusb-2f-class-2f-video:
	-$(RM) ./tinyusb/class/video/video_device.cyclo ./tinyusb/class/video/video_device.d ./tinyusb/class/video/video_device.o ./tinyusb/class/video/video_device.su

.PHONY: clean-tinyusb-2f-class-2f-video

