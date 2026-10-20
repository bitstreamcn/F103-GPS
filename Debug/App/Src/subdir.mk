################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../App/Src/application.cpp \
../App/Src/common.cpp \
../App/Src/cppmain.cpp \
../App/Src/gnss.cpp 

C_SRCS += \
../App/Src/syscallsx.c 

C_DEPS += \
./App/Src/syscallsx.d 

OBJS += \
./App/Src/application.o \
./App/Src/common.o \
./App/Src/cppmain.o \
./App/Src/gnss.o \
./App/Src/syscallsx.o 

CPP_DEPS += \
./App/Src/application.d \
./App/Src/common.d \
./App/Src/cppmain.d \
./App/Src/gnss.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.cpp App/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"M:/STM32CubeIDE/workspace/F103-GPS/App/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"M:/STM32CubeIDE/workspace/F103-GPS/App/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/application.cyclo ./App/Src/application.d ./App/Src/application.o ./App/Src/application.su ./App/Src/common.cyclo ./App/Src/common.d ./App/Src/common.o ./App/Src/common.su ./App/Src/cppmain.cyclo ./App/Src/cppmain.d ./App/Src/cppmain.o ./App/Src/cppmain.su ./App/Src/gnss.cyclo ./App/Src/gnss.d ./App/Src/gnss.o ./App/Src/gnss.su ./App/Src/syscallsx.cyclo ./App/Src/syscallsx.d ./App/Src/syscallsx.o ./App/Src/syscallsx.su

.PHONY: clean-App-2f-Src

