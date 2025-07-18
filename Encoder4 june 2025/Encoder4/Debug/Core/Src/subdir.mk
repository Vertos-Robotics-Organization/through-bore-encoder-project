################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MT6835.c \
../Core/Src/error_blink.c \
../Core/Src/flash_config.c \
../Core/Src/flex_encoder.c \
../Core/Src/main.c \
../Core/Src/mlx90393.c \
../Core/Src/mlx90393_mux.c \
../Core/Src/non_blocking_morse.c \
../Core/Src/pca9555.c \
../Core/Src/rgb_led.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/userflash.c \
../Core/Src/vl53l0x_api.c \
../Core/Src/vl53l0x_api_calibration.c \
../Core/Src/vl53l0x_api_core.c \
../Core/Src/vl53l0x_api_ranging.c \
../Core/Src/vl53l0x_api_strings.c \
../Core/Src/vl53l0x_platform.c \
../Core/Src/vl53l0x_platform_log.c 

OBJS += \
./Core/Src/MT6835.o \
./Core/Src/error_blink.o \
./Core/Src/flash_config.o \
./Core/Src/flex_encoder.o \
./Core/Src/main.o \
./Core/Src/mlx90393.o \
./Core/Src/mlx90393_mux.o \
./Core/Src/non_blocking_morse.o \
./Core/Src/pca9555.o \
./Core/Src/rgb_led.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/userflash.o \
./Core/Src/vl53l0x_api.o \
./Core/Src/vl53l0x_api_calibration.o \
./Core/Src/vl53l0x_api_core.o \
./Core/Src/vl53l0x_api_ranging.o \
./Core/Src/vl53l0x_api_strings.o \
./Core/Src/vl53l0x_platform.o \
./Core/Src/vl53l0x_platform_log.o 

C_DEPS += \
./Core/Src/MT6835.d \
./Core/Src/error_blink.d \
./Core/Src/flash_config.d \
./Core/Src/flex_encoder.d \
./Core/Src/main.d \
./Core/Src/mlx90393.d \
./Core/Src/mlx90393_mux.d \
./Core/Src/non_blocking_morse.d \
./Core/Src/pca9555.d \
./Core/Src/rgb_led.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/userflash.d \
./Core/Src/vl53l0x_api.d \
./Core/Src/vl53l0x_api_calibration.d \
./Core/Src/vl53l0x_api_core.d \
./Core/Src/vl53l0x_api_ranging.d \
./Core/Src/vl53l0x_api_strings.d \
./Core/Src/vl53l0x_platform.d \
./Core/Src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/VL53L0/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MT6835.cyclo ./Core/Src/MT6835.d ./Core/Src/MT6835.o ./Core/Src/MT6835.su ./Core/Src/error_blink.cyclo ./Core/Src/error_blink.d ./Core/Src/error_blink.o ./Core/Src/error_blink.su ./Core/Src/flash_config.cyclo ./Core/Src/flash_config.d ./Core/Src/flash_config.o ./Core/Src/flash_config.su ./Core/Src/flex_encoder.cyclo ./Core/Src/flex_encoder.d ./Core/Src/flex_encoder.o ./Core/Src/flex_encoder.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mlx90393.cyclo ./Core/Src/mlx90393.d ./Core/Src/mlx90393.o ./Core/Src/mlx90393.su ./Core/Src/mlx90393_mux.cyclo ./Core/Src/mlx90393_mux.d ./Core/Src/mlx90393_mux.o ./Core/Src/mlx90393_mux.su ./Core/Src/non_blocking_morse.cyclo ./Core/Src/non_blocking_morse.d ./Core/Src/non_blocking_morse.o ./Core/Src/non_blocking_morse.su ./Core/Src/pca9555.cyclo ./Core/Src/pca9555.d ./Core/Src/pca9555.o ./Core/Src/pca9555.su ./Core/Src/rgb_led.cyclo ./Core/Src/rgb_led.d ./Core/Src/rgb_led.o ./Core/Src/rgb_led.su ./Core/Src/stm32g0xx_hal_msp.cyclo ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_it.cyclo ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.cyclo ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/userflash.cyclo ./Core/Src/userflash.d ./Core/Src/userflash.o ./Core/Src/userflash.su ./Core/Src/vl53l0x_api.cyclo ./Core/Src/vl53l0x_api.d ./Core/Src/vl53l0x_api.o ./Core/Src/vl53l0x_api.su ./Core/Src/vl53l0x_api_calibration.cyclo ./Core/Src/vl53l0x_api_calibration.d ./Core/Src/vl53l0x_api_calibration.o ./Core/Src/vl53l0x_api_calibration.su ./Core/Src/vl53l0x_api_core.cyclo ./Core/Src/vl53l0x_api_core.d ./Core/Src/vl53l0x_api_core.o ./Core/Src/vl53l0x_api_core.su ./Core/Src/vl53l0x_api_ranging.cyclo ./Core/Src/vl53l0x_api_ranging.d ./Core/Src/vl53l0x_api_ranging.o ./Core/Src/vl53l0x_api_ranging.su ./Core/Src/vl53l0x_api_strings.cyclo ./Core/Src/vl53l0x_api_strings.d ./Core/Src/vl53l0x_api_strings.o ./Core/Src/vl53l0x_api_strings.su ./Core/Src/vl53l0x_platform.cyclo ./Core/Src/vl53l0x_platform.d ./Core/Src/vl53l0x_platform.o ./Core/Src/vl53l0x_platform.su ./Core/Src/vl53l0x_platform_log.cyclo ./Core/Src/vl53l0x_platform_log.d ./Core/Src/vl53l0x_platform_log.o ./Core/Src/vl53l0x_platform_log.su

.PHONY: clean-Core-2f-Src

