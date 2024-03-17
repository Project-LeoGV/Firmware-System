################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/TIMER/TIMER_Program.c 

OBJS += \
./MCAL/TIMER/TIMER_Program.o 

C_DEPS += \
./MCAL/TIMER/TIMER_Program.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/TIMER/%.o MCAL/TIMER/%.su: ../MCAL/TIMER/%.c MCAL/TIMER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32G431CBTx -DSTM32G4 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MCAL-2f-TIMER

clean-MCAL-2f-TIMER:
	-$(RM) ./MCAL/TIMER/TIMER_Program.d ./MCAL/TIMER/TIMER_Program.o ./MCAL/TIMER/TIMER_Program.su

.PHONY: clean-MCAL-2f-TIMER

