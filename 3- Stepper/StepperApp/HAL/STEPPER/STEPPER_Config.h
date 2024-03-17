/*
 * STEPPER_Config.h
 *
 *  Created on: Dec 16, 2023
 *      Author: Farha Hany
 */
 
#ifndef HAL_STEPPER_STEPPER_CONFIG_H_
#define HAL_STEPPER_STEPPER_CONFIG_H_

#include "STEPPER_Interface.h"

/*		Definitions			 */
#define STEPPER_DIR 				0 			//Stepper Motors Default Direction for Homing DOWN

#define STEPPER_STEPS			800			//800 Rotations is 4 full motor rotations at Full-Step Micro-stepping



/*Stepper Motor Configuration*/
static STEPPER_config_t STEPPER_mod[4] =
{
		{
				GPIO_PORTB,
				GPIO_PIN10,
				GPIO_PORTB,
				GPIO_PIN11,
				GPIO_PORTA,
				GPIO_PIN2,
				GPIO_PORTA,
				GPIO_PIN3
		},
		{
				GPIO_PORTB,
				GPIO_PIN12,
				GPIO_PORTB,
				GPIO_PIN13,
				GPIO_PORTA,
				GPIO_PIN4,
				GPIO_PORTA,
				GPIO_PIN5,
		},
		{
				GPIO_PORTB,
				GPIO_PIN15,
				GPIO_PORTB,
				GPIO_PIN14,
				GPIO_PORTA,
				GPIO_PIN6,
				GPIO_PORTA,
				GPIO_PIN7
		},
		{
				GPIO_PORTA,
				GPIO_PIN1,
				GPIO_PORTA,
				GPIO_PIN0,
				GPIO_PORTB,
				GPIO_PIN1,
				GPIO_PORTB,
				GPIO_PIN0
		}
};


#endif /* HAL_STEPPER_STEPPER_CONFIG_H_ */
