/*
 * LIMSWI_Interface.h
 *
 *  Created on: Feb 16, 2024
 *      Author: es-maro19996
 */

#ifndef HAL_LIMSWI_LIMSWI_INTERFACE_H_
#define HAL_LIMSWI_LIMSWI_INTERFACE_H_

/*				Includes				*/
#include "LIMSWI_Config.h"
#include "../../MCAL/GPIO/GPIO_Interface.h"
#include "../../MCAL/GPIO/GPIO_Registers.h"
#include "../../MCAL/GPIO/GPIO_Config.h"

void LIMSWI_voidSwitchInit(void);
u8 LIMSWI_u8SwitchPress(u8 A_u8SwitchPort,u8 A_u8SwitchPin);

#endif /* HAL_LIMSWI_LIMSWI_INTERFACE_H_ */
