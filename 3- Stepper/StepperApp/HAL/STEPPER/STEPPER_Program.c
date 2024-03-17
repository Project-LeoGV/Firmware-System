/*
 * STEPPER_Program.c
 *
 *  Created on: Dec 16, 2023
 *      Author: Farha Hany
 */
 
#include "STEPPER_Config.h"
#include "STEPPER_Interface.h"


/*Motor Initialization*/
void STEPPER_voidInitMotor(STEPPER_config_t* A_stepperMotor)
{
    /*Structs to configure the Direction, Step, USM0, and USM1 pins*/
    MGPIO_Config_t dirConfig = {
        .Port = A_stepperMotor->dir_PORT,
        .Pin = A_stepperMotor->dir_PIN,
        .Mode = GPIO_MODE_OUTPUT,
        .OutputType = GPIO_OT_PUSHPULL,
        .OutputSpeed = GPIO_SPEED_LOW
    };

    MGPIO_Config_t stepConfig = {
        .Port = A_stepperMotor->step_PORT,
        .Pin = A_stepperMotor->step_PIN,
        .Mode = GPIO_MODE_OUTPUT,
        .OutputType = GPIO_OT_PUSHPULL,
        .OutputSpeed = GPIO_SPEED_LOW
    };

    MGPIO_Config_t ISENAConfig = {
        .Port = A_stepperMotor->ISENA_PORT,
        .Pin = A_stepperMotor->ISENA_PIN,
        .Mode = GPIO_MODE_INPUT,
        .OutputType = GPIO_OT_PUSHPULL,
        .OutputSpeed = GPIO_SPEED_LOW
    };

    MGPIO_Config_t ISENBConfig = {
        .Port = A_stepperMotor->ISENB_PORT,
        .Pin = A_stepperMotor->ISENB_PIN,
        .Mode = GPIO_MODE_INPUT,
        .OutputType = GPIO_OT_PUSHPULL,
        .OutputSpeed = GPIO_SPEED_LOW
    };

    /*Initialization for each pin using the GPIO_voidInitPin function*/
    GPIO_voidInitPin(&dirConfig);
    GPIO_voidInitPin(&stepConfig);
    GPIO_voidInitPin(&ISENAConfig);
    GPIO_voidInitPin(&ISENBConfig);
}

/*Motor Movement*/
void STEPPER_voidMotorStep(STEPPER_config_t* A_stepperMotor, u8 A_dir, u32 A_delay)
{
    /*Checking the Rotation Direction*/
    GPIO_voidSetPinValue(A_stepperMotor->dir_PORT, A_stepperMotor->dir_PIN, A_dir ? GPIO_VALUE_HIGH : GPIO_VALUE_LOW);

    /*Timer Initialaization*/
	MGPIO_Config_t TIM2_ch1_pin = {.Port = GPIO_PORTA, .Pin = GPIO_PIN5, .Mode = GPIO_MODE_ALTF, .AltFunc = GPIO_AF1,.OutputType = GPIO_OT_PUSHPULL, .OutputSpeed = GPIO_SPEED_LOW, .InputPull = GPIO_NO_PULL};
    GPIO_voidInitPin(&TIM2_ch1_pin);

    /*Toggle to take one step*/
    GPIO_voidSetPinValue(A_stepperMotor->dir_PORT, A_stepperMotor->step_PIN, GPIO_VALUE_HIGH);
    TIMERx_voidDelay_ms(TIM2, A_delay);
    GPIO_voidSetPinValue(A_stepperMotor->dir_PORT, A_stepperMotor->step_PIN, GPIO_VALUE_LOW);
    TIMERx_voidDelay_ms(TIM2, A_delay);
}

/*QuadMotor Movement*/
void STEPPER_voidQuadMotorStep(STEPPER_config_t* A_stepperMotor1, STEPPER_config_t* A_stepperMotor2, STEPPER_config_t* A_stepperMotor3, STEPPER_config_t* A_stepperMotor4, u8 A_dir, TIMER_RegMap_t* timerSelect, u32 A_delay)
{
	/*Set the Motors Direction*/
	GPIO_voidSetPinValue(A_stepperMotor1->dir_PORT, A_stepperMotor1->dir_PIN, A_dir);
	GPIO_voidSetPinValue(A_stepperMotor2->dir_PORT, A_stepperMotor2->dir_PIN, A_dir);
	GPIO_voidSetPinValue(A_stepperMotor3->dir_PORT, A_stepperMotor3->dir_PIN, A_dir);
	GPIO_voidSetPinValue(A_stepperMotor4->dir_PORT, A_stepperMotor4->dir_PIN, A_dir);
	/*Quick Note: If a problem occurs with missing a step due to setup time, we can simply split the direction
	 * and step ports/pins into two different initialization/movement function, this would allow for the setup time
	 * to happen during the initialization of a pin*/

	/*Set the Motor Signals High*/
	GPIO_voidSetPinValue(A_stepperMotor1->step_PORT, A_stepperMotor1->step_PIN, 1);
	GPIO_voidSetPinValue(A_stepperMotor2->step_PORT, A_stepperMotor2->step_PIN, 1);
	GPIO_voidSetPinValue(A_stepperMotor3->step_PORT, A_stepperMotor3->step_PIN, 1);
	GPIO_voidSetPinValue(A_stepperMotor4->step_PORT, A_stepperMotor4->step_PIN, 1);
	/*Delay for 5 ms*/
	TIMERx_voidDelay_ms(timerSelect, A_delay);
	/*Set the Motor Signals to Low to Generate a step*/
	GPIO_voidSetPinValue(A_stepperMotor1->step_PORT, A_stepperMotor1->step_PIN, 0);
	GPIO_voidSetPinValue(A_stepperMotor2->step_PORT, A_stepperMotor2->step_PIN, 0);
	GPIO_voidSetPinValue(A_stepperMotor3->step_PORT, A_stepperMotor3->step_PIN, 0);
	GPIO_voidSetPinValue(A_stepperMotor4->step_PORT, A_stepperMotor4->step_PIN, 0);
	/*Delay for 5 ms*/
	TIMERx_voidDelay_ms(timerSelect, A_delay);
}

/*Homing the Motors*/
void STEPPER_voidQuadMotorHome(STEPPER_config_t* A_stepperMotor1, STEPPER_config_t* A_stepperMotor2, STEPPER_config_t* A_stepperMotor3, STEPPER_config_t* A_stepperMotor4, TIMER_RegMap_t* timerSelect)
{
	/*Set the Motors Direction*/
	GPIO_voidSetPinValue(A_stepperMotor1->dir_PORT, A_stepperMotor1->dir_PIN, STEPPER_DIR);
	GPIO_voidSetPinValue(A_stepperMotor2->dir_PORT, A_stepperMotor2->dir_PIN, STEPPER_DIR);
	GPIO_voidSetPinValue(A_stepperMotor3->dir_PORT, A_stepperMotor3->dir_PIN, STEPPER_DIR);
	GPIO_voidSetPinValue(A_stepperMotor4->dir_PORT, A_stepperMotor4->dir_PIN, STEPPER_DIR);

		/*Set the Motor Signals High*/
		GPIO_voidSetPinValue(A_stepperMotor1->step_PORT, A_stepperMotor1->step_PIN, 1);
		GPIO_voidSetPinValue(A_stepperMotor2->step_PORT, A_stepperMotor2->step_PIN, 1);
		GPIO_voidSetPinValue(A_stepperMotor3->step_PORT, A_stepperMotor3->step_PIN, 1);
		GPIO_voidSetPinValue(A_stepperMotor4->step_PORT, A_stepperMotor4->step_PIN, 1);
		/*Delay for 5 ms*/
		TIMERx_voidDelay_ms(timerSelect, 5);
		/*Set the Motor Signals to Low to Generate a step*/
		GPIO_voidSetPinValue(A_stepperMotor1->step_PORT, A_stepperMotor1->step_PIN, 0);
		GPIO_voidSetPinValue(A_stepperMotor2->step_PORT, A_stepperMotor2->step_PIN, 0);
		GPIO_voidSetPinValue(A_stepperMotor3->step_PORT, A_stepperMotor3->step_PIN, 0);
		GPIO_voidSetPinValue(A_stepperMotor4->step_PORT, A_stepperMotor4->step_PIN, 0);
		/*Delay for 5 ms*/
		TIMERx_voidDelay_ms(timerSelect, 5);
}


