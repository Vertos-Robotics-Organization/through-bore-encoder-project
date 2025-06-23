################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L0/core/src/vl53l0x_api.c \
../Drivers/VL53L0/core/src/vl53l0x_api_calibration.c \
../Drivers/VL53L0/core/src/vl53l0x_api_core.c \
../Drivers/VL53L0/core/src/vl53l0x_api_ranging.c \
../Drivers/VL53L0/core/src/vl53l0x_api_strings.c 

OBJS += \
./Drivers/VL53L0/core/src/vl53l0x_api.o \
./Drivers/VL53L0/core/src/vl53l0x_api_calibration.o \
./Drivers/VL53L0/core/src/vl53l0x_api_core.o \
./Drivers/VL53L0/core/src/vl53l0x_api_ranging.o \
./Drivers/VL53L0/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./Drivers/VL53L0/core/src/vl53l0x_api.d \
./Drivers/VL53L0/core/src/vl53l0x_api_calibration.d \
./Drivers/VL53L0/core/src/vl53l0x_api_core.d \
./Drivers/VL53L0/core/src/vl53l0x_api_ranging.d \
./Drivers/VL53L0/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0/core/src/%.o Drivers/VL53L0/core/src/%.su Drivers/VL53L0/core/src/%.cyclo: ../Drivers/VL53L0/core/src/%.c Drivers/VL53L0/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0-2f-core-2f-src

clean-Drivers-2f-VL53L0-2f-core-2f-src:
	-$(RM) ./Drivers/VL53L0/core/src/vl53l0x_api.cyclo ./Drivers/VL53L0/core/src/vl53l0x_api.d ./Drivers/VL53L0/core/src/vl53l0x_api.o ./Drivers/VL53L0/core/src/vl53l0x_api.su ./Drivers/VL53L0/core/src/vl53l0x_api_calibration.cyclo ./Drivers/VL53L0/core/src/vl53l0x_api_calibration.d ./Drivers/VL53L0/core/src/vl53l0x_api_calibration.o ./Drivers/VL53L0/core/src/vl53l0x_api_calibration.su ./Drivers/VL53L0/core/src/vl53l0x_api_core.cyclo ./Drivers/VL53L0/core/src/vl53l0x_api_core.d ./Drivers/VL53L0/core/src/vl53l0x_api_core.o ./Drivers/VL53L0/core/src/vl53l0x_api_core.su ./Drivers/VL53L0/core/src/vl53l0x_api_ranging.cyclo ./Drivers/VL53L0/core/src/vl53l0x_api_ranging.d ./Drivers/VL53L0/core/src/vl53l0x_api_ranging.o ./Drivers/VL53L0/core/src/vl53l0x_api_ranging.su ./Drivers/VL53L0/core/src/vl53l0x_api_strings.cyclo ./Drivers/VL53L0/core/src/vl53l0x_api_strings.d ./Drivers/VL53L0/core/src/vl53l0x_api_strings.o ./Drivers/VL53L0/core/src/vl53l0x_api_strings.su

.PHONY: clean-Drivers-2f-VL53L0-2f-core-2f-src

