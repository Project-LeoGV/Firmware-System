/*
 * TIMER_Interface.h
 *
 *  Created on: Dec 26, 2023
 *      Author: Nada Mamdouh
 */

#ifndef MCAL_TIMER_TIMER_INTERFACE_H_
#define MCAL_TIMER_TIMER_INTERFACE_H_

#include "TIMER_Register.h"
#include "TIMER_Config.h"

/*********************************************************************/
/*Function name: TIMERx_voidCounter_Init                              */
/*Description: use this function when configuring PWM,IC and Pulse in*/
/*@param TIMx: peripheral name                                       */
/*@param A_xCNT_Cfg:struct contains config parameters                */
/*********************************************************************/
void TIMERx_voidCounter_Init(TIMER_RegMap_t *TIMx,TIMx_Counter_Cfg_t *A_xCNT_Cfg);
/*****************************************************************/
/*Function name: TIMERx_voidCfg_IC                               */
/*Description: config TIMx input capture,used to capture one edge*/
/*@param TIMx: peripheral name                                   */
/*@param A_xIC_Cfg:struct contains config parameters             */
/*****************************************************************/
void TIMERx_voidCfg_IC(TIMER_RegMap_t *TIMx, TIMx_IC_Cfg_t *A_xIC_Cfg);
/*****************************************************/
/*Function name: TIMERx_voidCfg_PWM                  */
/*Description: configuring TIMx PWM Mode             */
/*@param TIMx: peripheral name                       */
/*@param A_xPWM_Cfg:struct contains config parameters*/
/*****************************************************/
void TIMERx_voidCfg_PWM(TIMER_RegMap_t *TIMx,TIMx_PWM_Cfg_t *A_xPWM_Cfg);

/**********************************************************/
/*Function name: TIMERx_u32GetCaptureValue                */
/*Description: get captured value,used to capture one edge*/
/*@param TIMx: peripheral name                            */
/*@param A_u8ChannelNo:channel number                     */
/**********************************************************/
u32 TIMERx_u32GetCaptureValue(TIMER_RegMap_t *TIMx,u8 A_u8ChannelNo);
/************************************************************/
/*Function name: TIMERx_voidCfg_PulseIn                     */
/*Description: config PWM input mode,used to capture a pulse*/
/*   => ONLY CHANNEL 1 OR 2 COULD BE USED  <=               */
/*@param TIMx: peripheral name                              */
/*@param A_u8Channel_NO:channel number                      */
/************************************************************/
void TIMERx_voidCfg_PulseIn(TIMER_RegMap_t *TIMx,u8 A_u8Channel_NO);
/**************************************************************/
/*Function name: TIMER_xMeasurePWM                            */
/*Description: returns the period and the pulse width of a PWM*/
/*@param TIMx: peripheral name                                */
/**************************************************************/
MeasurePWM_t TIMERx_xMeasurePWM(TIMER_RegMap_t *TIMx);
/**************************************************************/
/*Function name: TIMERx_voidSetPeriod_DutyCycle               */
/*Description: to set period and pulse width                  */
/*@param TIMx: peripheral name                                */
/*@param A_u32Period: pulse period                            */
/*@param A_u32DutyCycle: takes a number from 1 to 100         */
/**************************************************************/
void TIMERx_voidSetPeriod_DutyCycle(TIMER_RegMap_t *TIMx,u8 A_u8Channel_NO, u32 A_u32Period, u32 A_u32DutyCycle);
void TIMERx_voidStartCounter(TIMER_RegMap_t *TIMx);
void TIMERx_voidStopCounter(TIMER_RegMap_t *TIMx);
void TIMERx_voidDelay_ms(TIMER_RegMap_t *TIMx,u32 A_u32Time_in_ms);

/* -------- TIMER Peripheral -------- */
/* TIM2
 * TIM3
 * TIM4
 * TIM5
 */

/* ------- Channel NO ------- */
/* TIMER_CHANNEL_1
 * TIMER_CHANNEL_2
 * TIMER_CHANNEL_3
 * TIMER_CHANNEL_4
*/
/* ------- TIMER PINS ------- */
/* TIM2_CH1   [GPIO_PORTA , GPIO_PIN0 , GPIO_AF1] or [GPIO_PORTA , GPIO_PIN5 , GPIO_AF1] or [GPIO_PORTA , GPIO_PIN15 , GPIO_AF1]
 * TIM2_CH2   [GPIO_PORTA , GPIO_PIN1 , GPIO_AF1] or [GPIO_PORTB , GPIO_PIN3 , GPIO_AF1]
 * TIM2_CH3   [GPIO_PORTA , GPIO_PIN2 , GPIO_AF1] or [GPIO_PORTB , GPIO_PIN10 , GPIO_AF1] or [GPIO_PORTA , GPIO_PIN9 , GPIO_AF10]
 * TIM2_CH4   [GPIO_PORTA , GPIO_PIN3 , GPIO_AF1] or [GPIO_PORTB , GPIO_PIN11 , GPIO_AF1] or [GPIO_PORTA , GPIO_PIN10 , GPIO_AF10]
 *
 * TIM3_CH1   [GPIO_PORTA , GPIO_PIN6 , GPIO_AF2] or [GPIO_PORTB , GPIO_PIN4 , GPIO_AF2] or [GPIO_PORTC , GPIO_PIN6 , GPIO_AF2]
 * TIM3_CH2   [GPIO_PORTA , GPIO_PIN4 , GPIO_AF2] or [GPIO_PORTA , GPIO_PIN7 , GPIO_AF2] or [GPIO_PORTB , GPIO_PIN5 , GPIO_AF2] or [GPIO_PORTC , GPIO_PIN7 , GPIO_AF2]
 * TIM3_CH3   [GPIO_PORTB , GPIO_PIN0 , GPIO_AF2] or [GPIO_PORTC , GPIO_PIN8 , GPIO_AF2]
 * TIM3_CH4   [GPIO_PORTB , GPIO_PIN1 , GPIO_AF2] or [GPIO_PORTC , GPIO_PIN9 , GPIO_AF2] or [GPIO_PORTB , GPIO_PIN7 , GPIO_AF10]
 *
 * TIM4_CH1   [GPIO_PORTB , GPIO_PIN6 , GPIO_AF2] or [GPIO_PORTA , GPIO_PIN11 , GPIO_AF10]
 * TIM4_CH2   [GPIO_PORTB , GPIO_PIN7 , GPIO_AF2] or [GPIO_PORTA , GPIO_PIN12 , GPIO_AF10]
 * TIM4_CH3   [GPIO_PORTB , GPIO_PIN8 , GPIO_AF2] or [GPIO_PORTA , GPIO_PIN13 , GPIO_AF10]
 * TIM4_CH4   [GPIO_PORTB , GPIO_PIN9 , GPIO_AF2]
 *
 * */

#endif /* MCAL_TIMER_TIMER_INTERFACE_H_ */
