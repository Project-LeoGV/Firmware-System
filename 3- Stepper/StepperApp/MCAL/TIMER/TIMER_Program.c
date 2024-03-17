/*
 * TIMER_Program.c
 *
 *  Created on: Dec 26, 2023
 *      Author: Nada Mamdouh
 */

#include "../../LIB/STD_TYPES.h"
#include "../../LIB/BIT_MATH.h"

#include "TIMER_Register.h"
#include "TIMER_Config.h"
#include "TIMER_Interface.h"


static void TIMER_voidSelectEdge(TIMER_RegMap_t *TIMx, TIMx_IC_Cfg_t *A_xIC_Cfg);

void TIMERx_voidCfg_IC(TIMER_RegMap_t *TIMx, TIMx_IC_Cfg_t *A_xIC_Cfg)
{
	/* Select external input source */
	TIMx->TISEL &= ~(0b1111<< 4*A_xIC_Cfg-> Channel_NO);
	TIMx->TISEL |= (0b0000<< 4*A_xIC_Cfg-> Channel_NO);

	/* Select the active input, link TIMx_CCRx to the tim_tix input, */
	switch(A_xIC_Cfg-> Channel_NO)
	{
	case TIMER_CHANNEL_1:
		SET_BIT(TIMx->CCMR1,CC1S_0);
		CLR_BIT(TIMx->CCMR1,CC1S_1);
		break;
	case TIMER_CHANNEL_2:
		SET_BIT(TIMx->CCMR1,CC2S_0);
		CLR_BIT(TIMx->CCMR1,CC2S_1);
		break;
	case TIMER_CHANNEL_3:
		SET_BIT(TIMx->CCMR2,CC3S_0);
		CLR_BIT(TIMx->CCMR2,CC3S_1);
		break;
	case TIMER_CHANNEL_4:
		SET_BIT(TIMx->CCMR2,CC4S_0);
		CLR_BIT(TIMx->CCMR2,CC4S_1);
		break;
	}

	/* Program filter duration */
	switch(A_xIC_Cfg-> Channel_NO)
	{
	case TIMER_CHANNEL_1:
		TIMx->CCMR1 &= ~(0b1111 << IC1F_0);
		TIMx->CCMR1 |=  (A_xIC_Cfg->Filter << IC1F_0);
		break;
	case TIMER_CHANNEL_2:
		TIMx->CCMR1 &= ~(0b1111 << IC2F_0);
		TIMx->CCMR1 |=  (A_xIC_Cfg->Filter << IC2F_0);
		break;
	case TIMER_CHANNEL_3:
		TIMx->CCMR2 &= ~(0b1111 << IC3F_0);
		TIMx->CCMR2 |=  (A_xIC_Cfg->Filter << IC3F_0);
		break;
	case TIMER_CHANNEL_4:
		TIMx->CCMR2 &= ~(0b1111 << IC4F_0);
		TIMx->CCMR2 |=  (A_xIC_Cfg->Filter << IC4F_0);
		break;
	}


	/* Select the edge of the active transition */
	TIMER_voidSelectEdge(TIMx,&A_xIC_Cfg);

	/* Program prescaler */
	switch(A_xIC_Cfg-> Channel_NO)
	{
	case TIMER_CHANNEL_1:
		TIMx->CCMR1 &= ~(0b11 <<IC1PSC_0);
		TIMx->CCMR1 &= ~( A_xIC_Cfg->IC_Prescaler <<IC1PSC_0);
		break;
	case TIMER_CHANNEL_2:
		TIMx->CCMR1 &= ~(0b11 <<IC2PSC_0);
		TIMx->CCMR1 &= ~( A_xIC_Cfg->IC_Prescaler <<IC2PSC_0);
		break;
	case TIMER_CHANNEL_3:
		TIMx->CCMR2 &= ~(0b11 <<IC3PSC_0);
		TIMx->CCMR2 &= ~( A_xIC_Cfg->IC_Prescaler <<IC3PSC_0);
		break;
	case TIMER_CHANNEL_4:
		TIMx->CCMR2 &= ~(0b11 <<IC4PSC_0);
		TIMx->CCMR2 &= ~( A_xIC_Cfg->IC_Prescaler <<IC4PSC_0);
		break;
	}

	/* Enable capture */
	switch(A_xIC_Cfg-> Channel_NO)
	{
	case TIMER_CHANNEL_1:
		SET_BIT(TIMx->CCER,CC1E);
		break;
	case TIMER_CHANNEL_2:
		SET_BIT(TIMx->CCER,CC2E);
		break;
	case TIMER_CHANNEL_3:
		SET_BIT(TIMx->CCER,CC3E);
		break;
	case TIMER_CHANNEL_4:
		SET_BIT(TIMx->CCER,CC4E);
		break;
	}
	/* Enable interrupt request */

	/* Start Counter */
	SET_BIT(TIMx->CR1,CEN);

}

static void TIMER_voidSelectEdge(TIMER_RegMap_t *TIMx, TIMx_IC_Cfg_t *A_xIC_Cfg)
{
	if(A_xIC_Cfg-> Channel_NO == TIMER_CHANNEL_1)
	{
		CLR_BIT(TIMx->CCER,CC1E);
		switch(A_xIC_Cfg->Edge_polarity)
		{
		case TIMER_EDGE_RISING:
			CLR_BIT(TIMx->CCER,CC1P);
			CLR_BIT(TIMx->CCER,CC1NP);
			break;
		case TIMER_EDGE_FALLING:
			SET_BIT(TIMx->CCER,CC1P);
			CLR_BIT(TIMx->CCER,CC1NP);
			break;
		case TIMER_EDGE_BOTH:
			SET_BIT(TIMx->CCER,CC1P);
			SET_BIT(TIMx->CCER,CC1NP);
			break;
		}
	}
	else if(A_xIC_Cfg-> Channel_NO == TIMER_CHANNEL_2)
	{
		CLR_BIT(TIMx->CCER,CC2E);
		switch(A_xIC_Cfg->Edge_polarity)
		{
		case TIMER_EDGE_RISING:
			CLR_BIT(TIMx->CCER,CC2P);
			CLR_BIT(TIMx->CCER,CC2NP);
			break;
		case TIMER_EDGE_FALLING:
			SET_BIT(TIMx->CCER,CC2P);
			CLR_BIT(TIMx->CCER,CC2NP);
			break;
		case TIMER_EDGE_BOTH:
			SET_BIT(TIMx->CCER,CC2P);
			SET_BIT(TIMx->CCER,CC2NP);
			break;
		}
	}
	else if(A_xIC_Cfg-> Channel_NO == TIMER_CHANNEL_3)
	{
		CLR_BIT(TIMx->CCER,CC3E);
		switch(A_xIC_Cfg->Edge_polarity)
		{
		case TIMER_EDGE_RISING:
			CLR_BIT(TIMx->CCER,CC3P);
			CLR_BIT(TIMx->CCER,CC3NP);
			break;
		case TIMER_EDGE_FALLING:
			SET_BIT(TIMx->CCER,CC3P);
			CLR_BIT(TIMx->CCER,CC3NP);
			break;
		case TIMER_EDGE_BOTH:
			SET_BIT(TIMx->CCER,CC3P);
			SET_BIT(TIMx->CCER,CC3NP);
			break;
		}
	}
	else if(A_xIC_Cfg-> Channel_NO == TIMER_CHANNEL_4)
	{
		CLR_BIT(TIMx->CCER,CC4E);
		switch(A_xIC_Cfg->Edge_polarity)
		{
		case TIMER_EDGE_RISING:
			CLR_BIT(TIMx->CCER,CC4P);
			CLR_BIT(TIMx->CCER,CC4NP);
			break;
		case TIMER_EDGE_FALLING:
			SET_BIT(TIMx->CCER,CC4P);
			CLR_BIT(TIMx->CCER,CC4NP);
			break;
		case TIMER_EDGE_BOTH:
			SET_BIT(TIMx->CCER,CC4P);
			SET_BIT(TIMx->CCER,CC4NP);
			break;
		}
	}
}

u32 TIMERx_u32GetCaptureValue(TIMER_RegMap_t *TIMx,u8 A_u8ChannelNo)
{
	u32 L_u32CapturedValue;
	switch(A_u8ChannelNo)
	{
	case TIMER_CHANNEL_1:
		/* Wait until detecting an edge flag is raised */
		while(GET_BIT(TIMx->SR,CC1IF) == 0);

		/* Save register value and clear flag */
		L_u32CapturedValue = TIMx->CCR1;
		break;
	case TIMER_CHANNEL_2:
		while(GET_BIT(TIMx->SR,CC2IF) == 0);
		L_u32CapturedValue = TIMx->CCR2;
		break;
	case TIMER_CHANNEL_3:
		while(GET_BIT(TIMx->SR,CC3IF) == 0);
		L_u32CapturedValue = TIMx->CCR3;
		break;
	case TIMER_CHANNEL_4:
		while(GET_BIT(TIMx->SR,CC4IF) == 0);
		L_u32CapturedValue = TIMx->CCR4;
		break;
	default:
		/*Incorrect channel number*/
		break;
	}

	return L_u32CapturedValue;
}

void TIMERx_voidCfg_PWM(TIMER_RegMap_t *TIMx,TIMx_PWM_Cfg_t *A_xPWM_Cfg)
{

	if(A_xPWM_Cfg->Channel_NO == TIMER_CHANNEL_1)
	{
		/* Turn channel off */
		CLR_BIT(TIMx->CCER,CC1E);
		/* Channel is configured as output */
		CLR_BIT(TIMx->CCMR1,CC1S_0);
		CLR_BIT(TIMx->CCMR1,CC1S_1);
		/* Select PWM Mode */
		switch(A_xPWM_Cfg->Mode)
		{
		case PWM_MODE_1 :
			CLR_BIT(TIMx->CCMR1,OC1M_0);
			SET_BIT(TIMx->CCMR1,OC1M_1);
			SET_BIT(TIMx->CCMR1,OC1M_2);
			break;
		case PWM_MODE_2:
			SET_BIT(TIMx->CCMR1,OC1M_0);
			SET_BIT(TIMx->CCMR1,OC1M_1);
			SET_BIT(TIMx->CCMR1,OC1M_2);
			break;
		default:
			/* Incorrect Mode */
			break;
		}
		/* Enable preload register */
		SET_BIT(TIMx->CCMR1,OC1PE);

		/* Enable auto-reload register */
		SET_BIT(TIMx->CR1,ARPE);

		/* Select Polarity */
		switch(A_xPWM_Cfg->Polarity)
		{
		case PWM_POLARITY_HIGH:
			CLR_BIT(TIMx->CCER,CC1P);
			break;
		case PWM_POLARITY_LOW:
			SET_BIT(TIMx->CCER,CC1P);
			break;
		default:
			break;
		}

		/* Enable tim_oc1 output */
		SET_BIT(TIMx->CCER,CC1E);
	}
	else if(A_xPWM_Cfg->Channel_NO == TIMER_CHANNEL_2)
	{
		/* Turn channel off */
		CLR_BIT(TIMx->CCER,CC2E);
		/* Channel is configured as output */
		CLR_BIT(TIMx->CCMR1,CC2S_0);
		CLR_BIT(TIMx->CCMR1,CC2S_1);
		/* Select PWM Mode */
		switch(A_xPWM_Cfg->Mode)
		{
		case PWM_MODE_1 :
			CLR_BIT(TIMx->CCMR1,OC2M_0);
			SET_BIT(TIMx->CCMR1,OC2M_1);
			SET_BIT(TIMx->CCMR1,OC2M_2);
			break;
		case PWM_MODE_2:
			SET_BIT(TIMx->CCMR1,OC2M_0);
			SET_BIT(TIMx->CCMR1,OC2M_1);
			SET_BIT(TIMx->CCMR1,OC2M_2);
			break;
		default:
			/* Incorrect Mode */
			break;
		}
		/* Enable preload register */
		SET_BIT(TIMx->CCMR1,OC2PE);

		/* Enable auto-reload register */
		SET_BIT(TIMx->CR1,ARPE);

		/* Select Polarity */
		switch(A_xPWM_Cfg->Polarity)
		{
		case PWM_POLARITY_HIGH:
			CLR_BIT(TIMx->CCER,CC2P);
			break;
		case PWM_POLARITY_LOW:
			SET_BIT(TIMx->CCER,CC2P);
			break;
		default:
			break;
		}

		/* Enable tim_oc2 output */
		SET_BIT(TIMx->CCER,CC2E);
	}
	else if(A_xPWM_Cfg->Channel_NO == TIMER_CHANNEL_3)
	{
		/* Turn channel off */
		CLR_BIT(TIMx->CCER,CC3E);
		/* Channel is configured as output */
		CLR_BIT(TIMx->CCMR2,CC3S_0);
		CLR_BIT(TIMx->CCMR2,CC3S_1);
		/* Select PWM Mode */
		switch(A_xPWM_Cfg->Mode)
		{
		case PWM_MODE_1 :
			CLR_BIT(TIMx->CCMR2,OC3M_0);
			SET_BIT(TIMx->CCMR2,OC3M_1);
			SET_BIT(TIMx->CCMR2,OC3M_2);
			break;
		case PWM_MODE_2:
			SET_BIT(TIMx->CCMR2,OC3M_0);
			SET_BIT(TIMx->CCMR2,OC3M_1);
			SET_BIT(TIMx->CCMR2,OC3M_2);
			break;
		default:
			/* Incorrect Mode */
			break;
		}
		/* Enable preload register */
		SET_BIT(TIMx->CCMR2,OC3PE);

		/* Enable auto-reload register */
		SET_BIT(TIMx->CR1,ARPE);

		/* Select Polarity */
		switch(A_xPWM_Cfg->Polarity)
		{
		case PWM_POLARITY_HIGH:
			CLR_BIT(TIMx->CCER,CC3P);
			break;
		case PWM_POLARITY_LOW:
			SET_BIT(TIMx->CCER,CC3P);
			break;
		default:
			break;
		}

		/* Enable tim_oc3 output */
		SET_BIT(TIMx->CCER,CC3E);
	}
	else if(A_xPWM_Cfg->Channel_NO == TIMER_CHANNEL_4)
	{
		/* Turn channel off */
		CLR_BIT(TIMx->CCER,CC4E);
		/* Channel is configured as output */
		CLR_BIT(TIMx->CCMR2,CC4S_0);
		CLR_BIT(TIMx->CCMR2,CC4S_1);
		/* Select PWM Mode */
		switch(A_xPWM_Cfg->Mode)
		{
		case PWM_MODE_1 :
			CLR_BIT(TIMx->CCMR2,OC4M_0);
			SET_BIT(TIMx->CCMR2,OC4M_1);
			SET_BIT(TIMx->CCMR2,OC4M_2);
			break;
		case PWM_MODE_2:
			SET_BIT(TIMx->CCMR2,OC4M_0);
			SET_BIT(TIMx->CCMR2,OC4M_1);
			SET_BIT(TIMx->CCMR2,OC4M_2);
			break;
		default:
			/* Incorrect Mode */
			break;
		}
		/* Enable preload register */
		SET_BIT(TIMx->CCMR2,OC4PE);

		/* Enable auto-reload register */
		SET_BIT(TIMx->CR1,ARPE);

		/* Select Polarity */
		switch(A_xPWM_Cfg->Polarity)
		{
		case PWM_POLARITY_HIGH:
			CLR_BIT(TIMx->CCER,CC4P);
			break;
		case PWM_POLARITY_LOW:
			SET_BIT(TIMx->CCER,CC4P);
			break;
		default:
			break;
		}

		/* Enable tim_oc4 output */
		SET_BIT(TIMx->CCER,CC4E);
	}
	TIMx->CNT = 0;
}
void TIMERx_voidCfg_PulseIn(TIMER_RegMap_t *TIMx,u8 A_u8Channel_NO)
{
	/* Select external input source */
	TIMx->TISEL &= ~(0b1111<< 4* A_u8Channel_NO);
	TIMx->TISEL |= (0b0000<< 4* A_u8Channel_NO);
	/* Disable the capture */
	CLR_BIT(TIMx->CCER,CC1E);
	CLR_BIT(TIMx->CCER,CC2E);
	if(A_u8Channel_NO == TIMER_CHANNEL_1)
	{
		/* tim_ic1 is mapped on tim_ti1 */
		SET_BIT(TIMx->CCMR1,CC1S_0);
		CLR_BIT(TIMx->CCMR1,CC1S_1);

		/* Polarity is rising edge */
		CLR_BIT(TIMx->CCER,CC1P);
		CLR_BIT(TIMx->CCER,CC1NP);

		/* tim_ic2 is mapped on tim_ti1 */
		CLR_BIT(TIMx->CCMR1,CC2S_0);
		SET_BIT(TIMx->CCMR1,CC2S_1);

		/* Polarity is falling edge */
		SET_BIT(TIMx->CCER,CC2P);
		CLR_BIT(TIMx->CCER,CC2NP);

		/* Select the valid trigger */
		SET_BIT(TIMx->SMCR,TS_0);
		CLR_BIT(TIMx->SMCR,TS_1);
		SET_BIT(TIMx->SMCR,TS_2);
		CLR_BIT(TIMx->SMCR,TS_3);
		CLR_BIT(TIMx->SMCR,TS_4);

	}
	else if(A_u8Channel_NO == TIMER_CHANNEL_2)
	{
		/* tim_ic1 is mapped on tim_ti2 */
		CLR_BIT(TIMx->CCMR1,CC1S_0);
		SET_BIT(TIMx->CCMR1,CC1S_1);

		/* Polarity is rising edge */
		CLR_BIT(TIMx->CCER,CC2P);
		CLR_BIT(TIMx->CCER,CC2NP);

		/* tim_ic2 is mapped on tim_ti2 */
		SET_BIT(TIMx->CCMR1,CC2S_0);
		CLR_BIT(TIMx->CCMR1,CC2S_1);

		/* Polarity is falling edge */
		SET_BIT(TIMx->CCER,CC1P);
		CLR_BIT(TIMx->CCER,CC1NP);
		/* Select the valid trigger */
		CLR_BIT(TIMx->SMCR,TS_0);
		SET_BIT(TIMx->SMCR,TS_1);
		SET_BIT(TIMx->SMCR,TS_2);
		CLR_BIT(TIMx->SMCR,TS_3);
		CLR_BIT(TIMx->SMCR,TS_4);
	}
	/* Configure the slave mode controller in reset mode */
	CLR_BIT(TIMx->SMCR,SMS_0);
	CLR_BIT(TIMx->SMCR,SMS_1);
	SET_BIT(TIMx->SMCR,SMS_2);

	/* Enable the captures */
	SET_BIT(TIMx->CCER,CC1E);
	SET_BIT(TIMx->CCER,CC2E);

	/* Start Counter */
	SET_BIT(TIMx->CR1,CEN);

}

MeasurePWM_t TIMERx_xMeasurePWM(TIMER_RegMap_t *TIMx)
{
	MeasurePWM_t L_xPWM;
	/* polling on rising edge to reset counter */
	while(GET_BIT(TIMx->SR,CC1IF) == 0);

	/* polling on falling edge */
	while(GET_BIT(TIMx->SR,CC2IF) == 0);
	L_xPWM.DutyCycle = (TIMx->CCR2 * TIMx->ARR) * 100 ;

	/* polling on rising edge */
	while(GET_BIT(TIMx->SR,CC1IF) == 0);
	L_xPWM.Period = TIMx->CCR1 * TIMx->ARR;                         ///check again

	L_xPWM.Time_OnPeriod = L_xPWM.DutyCycle * 1/FREQUENCY;
	return L_xPWM;
}

void TIMERx_voidStartCounter(TIMER_RegMap_t *TIMx)
{
	SET_BIT(TIMx->CR1,CEN);
}
void TIMERx_voidStopCounter(TIMER_RegMap_t *TIMx)
{
	CLR_BIT(TIMx->CR1,CEN);
}
void TIMERx_voidCounter_Init(TIMER_RegMap_t *TIMx,TIMx_Counter_Cfg_t *A_xCNT_Cfg)
{
	/* ------------ Edge-aligned mode is used ------------ */
	/* Set Counting Direction */
	switch(A_xCNT_Cfg->Count_Direction)
	{
	case TIMER_COUNT_UP:
		CLR_BIT(TIMx->CR1,DIR);
		break;
	case TIMER_COUNT_DOWN:
		SET_BIT(TIMx->CR1,DIR);
		break;
	default:
		break;
	}

	/* Set prescaler */
	if(A_xCNT_Cfg->prescaler>0 && A_xCNT_Cfg->prescaler<=65536)
	{
		/* CK_CNT = bus_clk / (PSC + 1) */
		TIMx->PSC = A_xCNT_Cfg->prescaler - 1;
	}

	/* Set auto-reload set up */
	switch(A_xCNT_Cfg->auto_reload_EN)
	{
	case TIMER_AUTORELOAD_ENABLE:
		SET_BIT(TIMx->CR1,ARPE);
		break;
	case TIMER_AUTORELOAD_DISABLE:
		CLR_BIT(TIMx->CR1,ARPE);
		break;
	default:
		break;
	}
}
void TIMERx_voidDelay_ms(TIMER_RegMap_t *TIMx,u32 A_u32Time_in_ms)
{
	/* Count down */
	SET_BIT(TIMx->CR1,DIR);

	/* PSC = (bus_clk / CK_CNT)-1       CK_CNT = 10Mhz =>time base = 1us  */
	TIMx->PSC = (FREQUENCY/1000000)-1;

	/*Set auto reload value */
	TIMx->ARR = 1000;

	TIMERx_voidStartCounter(TIMx);

	for(u32 L_u32Index =0; L_u32Index < A_u32Time_in_ms; L_u32Index++)
	{
		while(GET_BIT(TIMx->SR,UIF) == 0);
		CLR_BIT(TIMx->SR,UIF);
	}

	TIMERx_voidStopCounter(TIMx);

}
void TIMERx_voidSetPeriod_DutyCycle(TIMER_RegMap_t *TIMx,u8 A_u8Channel_NO, u32 A_u32Period, u32 A_u32DutyCycle)
{
	TIMx->ARR = A_u32Period;

	switch(A_u8Channel_NO)
		{
	      /* duty% = (CCRx/ ARR) * 100 */
		case TIMER_CHANNEL_1:
			TIMx->CCR1 = (A_u32DutyCycle * A_u32Period)/100;
			break;
		case TIMER_CHANNEL_2:
			TIMx->CCR2 = (A_u32DutyCycle * A_u32Period)/100;
			break;
		case TIMER_CHANNEL_3:
			TIMx->CCR3 = (A_u32DutyCycle * A_u32Period)/100;
			break;
		case TIMER_CHANNEL_4:
			TIMx->CCR4 = (A_u32DutyCycle * A_u32Period)/100;
			break;
		}

	/* Initialize all registers */
	SET_BIT(TIMx->EGR,UG);

	/* Start Counter */
	SET_BIT(TIMx->CR1,CEN);
}
