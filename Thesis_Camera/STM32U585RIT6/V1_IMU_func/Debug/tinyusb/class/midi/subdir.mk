################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/class/midi/midi_device.c \
../tinyusb/class/midi/midi_host.c 

OBJS += \
./tinyusb/class/midi/midi_device.o \
./tinyusb/class/midi/midi_host.o 

C_DEPS += \
./tinyusb/class/midi/midi_device.d \
./tinyusb/class/midi/midi_host.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/class/midi/%.o tinyusb/class/midi/%.su tinyusb/class/midi/%.cyclo: ../tinyusb/class/midi/%.c tinyusb/class/midi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-class-2f-midi

clean-tinyusb-2f-class-2f-midi:
	-$(RM) ./tinyusb/class/midi/midi_device.cyclo ./tinyusb/class/midi/midi_device.d ./tinyusb/class/midi/midi_device.o ./tinyusb/class/midi/midi_device.su ./tinyusb/class/midi/midi_host.cyclo ./tinyusb/class/midi/midi_host.d ./tinyusb/class/midi/midi_host.o ./tinyusb/class/midi/midi_host.su

.PHONY: clean-tinyusb-2f-class-2f-midi

