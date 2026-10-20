################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../App/Bmm150/bmm150.cpp 

OBJS += \
./App/Bmm150/bmm150.o 

CPP_DEPS += \
./App/Bmm150/bmm150.d 


# Each subdirectory must supply rules for building sources it contributes
App/Bmm150/%.o App/Bmm150/%.su App/Bmm150/%.cyclo: ../App/Bmm150/%.cpp App/Bmm150/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"M:/STM32CubeIDE/workspace/F103-GPS/App/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-Bmm150

clean-App-2f-Bmm150:
	-$(RM) ./App/Bmm150/bmm150.cyclo ./App/Bmm150/bmm150.d ./App/Bmm150/bmm150.o ./App/Bmm150/bmm150.su

.PHONY: clean-App-2f-Bmm150

