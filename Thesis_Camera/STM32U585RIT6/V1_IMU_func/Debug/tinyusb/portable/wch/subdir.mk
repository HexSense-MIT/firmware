################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/portable/wch/dcd_ch32_usbfs.c \
../tinyusb/portable/wch/dcd_ch32_usbhs.c \
../tinyusb/portable/wch/hcd_ch32_usbfs.c 

OBJS += \
./tinyusb/portable/wch/dcd_ch32_usbfs.o \
./tinyusb/portable/wch/dcd_ch32_usbhs.o \
./tinyusb/portable/wch/hcd_ch32_usbfs.o 

C_DEPS += \
./tinyusb/portable/wch/dcd_ch32_usbfs.d \
./tinyusb/portable/wch/dcd_ch32_usbhs.d \
./tinyusb/portable/wch/hcd_ch32_usbfs.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/portable/wch/%.o tinyusb/portable/wch/%.su tinyusb/portable/wch/%.cyclo: ../tinyusb/portable/wch/%.c tinyusb/portable/wch/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-portable-2f-wch

clean-tinyusb-2f-portable-2f-wch:
	-$(RM) ./tinyusb/portable/wch/dcd_ch32_usbfs.cyclo ./tinyusb/portable/wch/dcd_ch32_usbfs.d ./tinyusb/portable/wch/dcd_ch32_usbfs.o ./tinyusb/portable/wch/dcd_ch32_usbfs.su ./tinyusb/portable/wch/dcd_ch32_usbhs.cyclo ./tinyusb/portable/wch/dcd_ch32_usbhs.d ./tinyusb/portable/wch/dcd_ch32_usbhs.o ./tinyusb/portable/wch/dcd_ch32_usbhs.su ./tinyusb/portable/wch/hcd_ch32_usbfs.cyclo ./tinyusb/portable/wch/hcd_ch32_usbfs.d ./tinyusb/portable/wch/hcd_ch32_usbfs.o ./tinyusb/portable/wch/hcd_ch32_usbfs.su

.PHONY: clean-tinyusb-2f-portable-2f-wch

