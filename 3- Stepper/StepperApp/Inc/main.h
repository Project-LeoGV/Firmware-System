/*
 * main.h
 *
 *  Created on: Feb 5, 2024
 *      Author: es-maro19996
 */

#ifndef MAIN_H_
#define MAIN_H_

/*Includes*/
#include "../MCAL/RCC/RCC_Interface.h"
#include "../MCAL/GPIO/GPIO_Interface.h"
#include "../MCAL/Timer/TIMER_Interface.h"
#include "../MCAL/FDCAN/CAN_Interface.h"
#include "../MCAL/ADC/ADC_Interface.h"
#include "../HAL/STEPPER/STEPPER_Interface.h"
#include "../HAL/STEPPER/STEPPER_Config.h"
#include "../HAL/LIMSWI/LIMSWI_Interface.h"


/* Functions Prototypes */
void APP_voidSystemClockInit(void);
void APP_voidGpioInit(void);
void APP_voidCanInit(void);
void APP_voidADCInit(void);
void APP_voidCurrentSense(void);


void APP_voidHoming(void);
void APP_voidEnsureSafety(void);

#endif /* MAIN_H_ */
