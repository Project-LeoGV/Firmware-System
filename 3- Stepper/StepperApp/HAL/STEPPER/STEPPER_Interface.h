/*
 * STEPPER_Interface.h
 *
 *  Created on: Dec 16, 2023
 *      Author: Farha Hany
 */

#ifndef HAL_STEPPER_STEPPER_INTERFACE_H_
#define HAL_STEPPER_STEPPER_INTERFACE_H_

#include "../../LIB/STD_TYPES.h"
#include "../../MCAL/GPIO/GPIO_Interface.h"
#include "../../MCAL/Timer/Timer_Interface.h"

/************************************************************/
/*DIRECTION                                                 */
/* 1 Clockwise                                              */
/* 0 Counterclockwise                                       */
/************************************************************/
/*STEP                                                      */
/* 1 Raising edge                                           */
/* 0 falling edge                                           */
/* step 1.8                                                 */
/************************************************************/
/*USM0, USM1                                                */
/*microstepping -> 1 == full step     USM0 USM1 == 0 0      */
/*microstepping -> 2 == half step     USM0 USM1 == 1 0      */
/*microstepping -> 4 == Quartar step  USM0 USM1 == 0 1      */
/*microstepping -> 8 == eighth step   USM0 USM1 == 1 1      */
/************************************************************/
typedef struct 
{
    u8 dir_PORT;
    u8 dir_PIN;
    u8 step_PORT;
    u8 step_PIN;
    u8 ISENA_PORT;
    u8 ISENA_PIN;
    u8 ISENB_PORT;
    u8 ISENB_PIN;
} STEPPER_config_t;

/*Functions Declaration*/
void STEPPER_voidInitMotor(STEPPER_config_t* A_stepperMotor);
void STEPPER_voidMotorStep(STEPPER_config_t* A_stepperMotor, u8 A_dir, u32 A_delay);
void STEPPER_voidQuadMotorStep(STEPPER_config_t* A_stepperMotor1, STEPPER_config_t* A_stepperMotor2, STEPPER_config_t* A_stepperMotor3, STEPPER_config_t* A_stepperMotor4, u8 A_dir, TIMER_RegMap_t* timerSelect, u32 A_delay);
void STEPPER_voidQuadMotorHome(STEPPER_config_t* A_stepperMotor1, STEPPER_config_t* A_stepperMotor2, STEPPER_config_t* A_stepperMotor3, STEPPER_config_t* A_stepperMotor4, TIMER_RegMap_t* timerSelect);



#endif/* HAL_STEPPER_STEPPER_INTERFACE_H_*/
