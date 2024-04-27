################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/STEPPER/STEPPER_Program.c 

OBJS += \
./HAL/STEPPER/STEPPER_Program.o 

C_DEPS += \
./HAL/STEPPER/STEPPER_Program.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/STEPPER/%.o HAL/STEPPER/%.su HAL/STEPPER/%.cyclo: ../HAL/STEPPER/%.c HAL/STEPPER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32G431CBTx -DSTM32G4 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-HAL-2f-STEPPER

clean-HAL-2f-STEPPER:
	-$(RM) ./HAL/STEPPER/STEPPER_Program.cyclo ./HAL/STEPPER/STEPPER_Program.d ./HAL/STEPPER/STEPPER_Program.o ./HAL/STEPPER/STEPPER_Program.su

.PHONY: clean-HAL-2f-STEPPER

