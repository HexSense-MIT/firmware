################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/portable/bridgetek/ft9xx/dcd_ft9xx.c 

OBJS += \
./tinyusb/portable/bridgetek/ft9xx/dcd_ft9xx.o 

C_DEPS += \
./tinyusb/portable/bridgetek/ft9xx/dcd_ft9xx.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/portable/bridgetek/ft9xx/%.o tinyusb/portable/bridgetek/ft9xx/%.su tinyusb/portable/bridgetek/ft9xx/%.cyclo: ../tinyusb/portable/bridgetek/ft9xx/%.c tinyusb/portable/bridgetek/ft9xx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-portable-2f-bridgetek-2f-ft9xx

clean-tinyusb-2f-portable-2f-bridgetek-2f-ft9xx:
	-$(RM) ./tinyusb/portable/bridgetek/ft9xx/dcd_ft9xx.cyclo ./tinyusb/portable/bridgetek/ft9xx/dcd_ft9xx.d ./tinyusb/portable/bridgetek/ft9xx/dcd_ft9xx.o ./tinyusb/portable/bridgetek/ft9xx/dcd_ft9xx.su

.PHONY: clean-tinyusb-2f-portable-2f-bridgetek-2f-ft9xx

