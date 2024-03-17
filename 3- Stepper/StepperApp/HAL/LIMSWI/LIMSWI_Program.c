/*
 * LIMSWI_Program.c
 *
 *  Created on: Feb 16, 2024
 *      Author: es-maro19996
 */

#include "LIMSWI_Interface.h"

void LIMSWI_voidSwitchInit(void)
{
	GPIO_voidInitPin(&LIMSWI[0]);
	GPIO_voidInitPin(&LIMSWI[1]);
}

u8 LIMSWI_u8SwitchPress(u8 A_u8SwitchPort,u8 A_u8SwitchPin)
{
	u8 reading = GPIO_u8GetPinData(A_u8SwitchPort, A_u8SwitchPin);
	return !reading;						//Inverted Logic: 0 -> Button is pressed 1-> Button is not pressed
}

