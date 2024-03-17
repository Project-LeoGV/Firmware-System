/*
 * TIMER_Register.h
 *
 *  Created on: Dec 26, 2023
 *      Author: Nada Mamdouh
 */

#ifndef MCAL_TIMER_TIMER_REGISTER_H_
#define MCAL_TIMER_TIMER_REGISTER_H_


#define TIMER2_BASE_ADDR             0x40000000U
#define TIMER3_BASE_ADDR             0x40000400U
#define TIMER4_BASE_ADDR             0x40000800U
#define TIMER5_BASE_ADDR             0x40000C00U

typedef volatile struct{
	u32 CR1;
	u32 CR2;
	u32 SMCR;
	u32 DIER;
	u32 SR;
	u32 EGR;
	u32 CCMR1;
	u32 CCMR2;
	u32 CCER;
	u32 CNT;
	u32 PSC;
	u32 ARR;
	u32 Reserved1;
	u32 CCR1;
	u32 CCR2;
	u32 CCR3;
	u32 CCR4;
	u32 Reserved2[5];
	u32 ECR;
	u32 TISEL;
	u32 AF1;
	u32 AF2;
	u32 Reserved3[221];
	u32 DCR;
	u32 DMAR;
}TIMER_RegMap_t;

#define TIM2                  ((TIMER_RegMap_t*)(TIMER2_BASE_ADDR))
#define TIM3                  ((TIMER_RegMap_t*)(TIMER3_BASE_ADDR))
#define TIM4                  ((TIMER_RegMap_t*)(TIMER4_BASE_ADDR))
#define TIM5                  ((TIMER_RegMap_t*)(TIMER5_BASE_ADDR))

/* -------------------- REGISTERS -------------------- */
/* --------- CCMR1 Input --------- */
#define CC1S_0    0
#define CC1S_1    1
#define IC1PSC_0  2
#define IC1F_0    4

#define CC2S_0    8
#define CC2S_1    9
#define IC2PSC_0  10
#define IC2F_0    12
/* --------- CCMR1 Output --------- */
#define OC1PE     3
#define OC1M_0    4
#define OC1M_1    5
#define OC1M_2    6

#define OC2PE     11
#define OC2M_0    12
#define OC2M_1    13
#define OC2M_2    14


/* --------- CCMR2 Input--------- */
#define CC3S_0    0
#define CC3S_1    1
#define IC3PSC_0  2
#define IC3F_0    4

#define CC4S_0    8
#define CC4S_1    9
#define IC4PSC_0  10
#define IC4F_0    12
/* --------- CCMR2 Output --------- */
#define OC3PE     3
#define OC3M_0    4
#define OC3M_1    5
#define OC3M_2    6

#define OC4PE     11
#define OC4M_0    12
#define OC4M_1    13
#define OC4M_2    14

/* --------- CCER --------- */
#define CC1E      0
#define CC1P      1
#define CC1NP     3

#define CC2E      4
#define CC2P      5
#define CC2NP     7

#define CC3E      8
#define CC3P      9
#define CC3NP     11

#define CC4E      12
#define CC4P      13
#define CC4NP     15

/* --------- SR --------- */
#define UIF       0
#define CC1IF     1
#define CC2IF     2
#define CC3IF     3
#define CC4IF     4

#define TIF       6

#define CC1OF     9
#define CC2OF     10
#define CC3OF     11
#define CC4OF     12

#define IDXF      20
#define DIRF      21
#define IERRF     22
#define TERRF     23

/* --------- CR1 --------- */
#define CEN       0

#define DIR       4

#define ARPE      7

/* --------- SMCR --------- */
#define SMS_0     0
#define SMS_1     1
#define SMS_2     2
#define TS_0      4
#define TS_1      5
#define TS_2      6

#define TS_3      20
#define TS_4      21

/* --------- EGR --------- */
#define UG        0


#endif /* MCAL_TIMER_TIMER_REGISTER_H_ */
