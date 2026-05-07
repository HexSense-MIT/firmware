################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32u585ritx.s 

OBJS += \
./Core/Startup/startup_stm32u585ritx.o 

S_DEPS += \
./Core/Startup/startup_stm32u585ritx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -g3 -DDEBUG -c -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb" -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb/class" -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb/common" -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb/device" -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb/host" -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb/osal" -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb/portable" -I"C:/DATA/Documents/Lab/github/HexSense_MIT/firmware/Thesis_Camera/STM32U585RIT6/V1_IMU_func/tinyusb/typec" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32u585ritx.d ./Core/Startup/startup_stm32u585ritx.o

.PHONY: clean-Core-2f-Startup

