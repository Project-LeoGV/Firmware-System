/*
 * LIMSWI_Config.h
 *
 *  Created on: Feb 16, 2024
 *      Author: es-maro19996
 */

#ifndef HAL_LIMSWI_LIMSWI_CONFIG_H_
#define HAL_LIMSWI_LIMSWI_CONFIG_H_

/*				Includes				*/
#include "../../MCAL/GPIO/GPIO_Interface.h"

/*				Limit Switch Numbering				*/

#define LIMSWI1				1
#define LIMSWI2				2

/*				Port/Pin Configuration of Limit Switches			*/

static MGPIO_Config_t LIMSWI[2] =
{
		{GPIO_PORTA, GPIO_PIN15, GPIO_MODE_INPUT, GPIO_OT_PUSHPULL, GPIO_SPEED_LOW, GPIO_PULL_UP, GPIO_AF0},
		{GPIO_PORTB, GPIO_PIN2, GPIO_MODE_INPUT, GPIO_OT_PUSHPULL, GPIO_SPEED_LOW, GPIO_PULL_UP, GPIO_AF0}
};



#endif /* HAL_LIMSWI_LIMSWI_CONFIG_H_ */
