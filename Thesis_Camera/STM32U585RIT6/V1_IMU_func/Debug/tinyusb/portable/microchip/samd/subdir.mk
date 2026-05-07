################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/portable/microchip/samd/dcd_samd.c \
../tinyusb/portable/microchip/samd/hcd_samd.c 

OBJS += \
./tinyusb/portable/microchip/samd/dcd_samd.o \
./tinyusb/portable/microchip/samd/hcd_samd.o 

C_DEPS += \
./tinyusb/portable/microchip/samd/dcd_samd.d \
./tinyusb/portable/microchip/samd/hcd_samd.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/portable/microchip/samd/%.o tinyusb/portable/microchip/samd/%.su tinyusb/portable/microchip/samd/%.cyclo: ../tinyusb/portable/microchip/samd/%.c tinyusb/portable/microchip/samd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-portable-2f-microchip-2f-samd

clean-tinyusb-2f-portable-2f-microchip-2f-samd:
	-$(RM) ./tinyusb/portable/microchip/samd/dcd_samd.cyclo ./tinyusb/portable/microchip/samd/dcd_samd.d ./tinyusb/portable/microchip/samd/dcd_samd.o ./tinyusb/portable/microchip/samd/dcd_samd.su ./tinyusb/portable/microchip/samd/hcd_samd.cyclo ./tinyusb/portable/microchip/samd/hcd_samd.d ./tinyusb/portable/microchip/samd/hcd_samd.o ./tinyusb/portable/microchip/samd/hcd_samd.su

.PHONY: clean-tinyusb-2f-portable-2f-microchip-2f-samd

