################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/ADC/ADC_Program.c 

OBJS += \
./MCAL/ADC/ADC_Program.o 

C_DEPS += \
./MCAL/ADC/ADC_Program.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/ADC/%.o MCAL/ADC/%.su MCAL/ADC/%.cyclo: ../MCAL/ADC/%.c MCAL/ADC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32G431CBTx -DSTM32G4 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MCAL-2f-ADC

clean-MCAL-2f-ADC:
	-$(RM) ./MCAL/ADC/ADC_Program.cyclo ./MCAL/ADC/ADC_Program.d ./MCAL/ADC/ADC_Program.o ./MCAL/ADC/ADC_Program.su

.PHONY: clean-MCAL-2f-ADC

