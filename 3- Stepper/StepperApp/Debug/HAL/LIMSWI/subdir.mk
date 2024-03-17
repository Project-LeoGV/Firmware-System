################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/LIMSWI/LIMSWI_Program.c 

OBJS += \
./HAL/LIMSWI/LIMSWI_Program.o 

C_DEPS += \
./HAL/LIMSWI/LIMSWI_Program.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/LIMSWI/%.o HAL/LIMSWI/%.su: ../HAL/LIMSWI/%.c HAL/LIMSWI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32G431CBTx -DSTM32G4 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-HAL-2f-LIMSWI

clean-HAL-2f-LIMSWI:
	-$(RM) ./HAL/LIMSWI/LIMSWI_Program.d ./HAL/LIMSWI/LIMSWI_Program.o ./HAL/LIMSWI/LIMSWI_Program.su

.PHONY: clean-HAL-2f-LIMSWI

