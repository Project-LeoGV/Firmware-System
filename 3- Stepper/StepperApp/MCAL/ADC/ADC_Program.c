/*
 * ADC_Program.c
 *
 *  Created on: Dec 24, 2023
 *      Author: Mahmoud Ahmed
 */


#include "ADC_Interface.h"

void ADC_voidInit(ADC_Config_t* A_config)
{
	ADC_RegMap_t* L_ADC_Reg;

	switch (A_config->adcNumber)
	{
	case ADC1:
		L_ADC_Reg = ADC1_MASTER;
		break;
	case ADC2:
		L_ADC_Reg = ADC2_SLAVE;
		break;
	case ADC3:
		L_ADC_Reg = ADC3_MASTER;
		break;
	case ADC4:
		L_ADC_Reg = ADC4_SLAVE;
		break;
	case ADC5:
		L_ADC_Reg = ADC5_SINGLE;
		break;
	}

	/* Wake up from Deep down power mode */
	/* Reference Manual Instructions
	 * To start ADC operations, follow the sequence below:
	 * 1. Exit Deep-power-down mode by clearing DEEPPWD bit.
	 * 2. Enable the ADC voltage regulator by setting ADVREGEN.
	 * 3. Wait for the startup time to configure the ADC (refer to the device datasheet for the value of the startup time).*/
	L_ADC_Reg->CR &= ~(1 << 29);
	L_ADC_Reg->CR |= (1 << 28);
	// Wait for startup time


	/* Disable ADC */
	if((L_ADC_Reg->CR >> 2) & 1){
		L_ADC_Reg->CR |= (1 << 4);
		while( (L_ADC_Reg->CR >> 4) & 1 );
	}
	if((L_ADC_Reg->CR >> 3) & 1){
		L_ADC_Reg->CR |= (1 << 5);
		while( (L_ADC_Reg->CR >> 5) & 1 );
	}
	if((L_ADC_Reg->CR >> 0) & 1){
		L_ADC_Reg->CR |= (1 << 1);
		while( (L_ADC_Reg->CR >> 0) & 1 );
		while( (L_ADC_Reg->CR >> 1) & 1 );
	}


	/* Calibration */
	if(A_config->mode == ADC_MODE_SINGLE)
		L_ADC_Reg->CR &= ~(1 << 30);
	else
		L_ADC_Reg->CR |= (1 << 30);
	L_ADC_Reg->CR |= (1 << 31);			// Start Calibration
	while( (L_ADC_Reg->CR >> 31 & 1) ); // Wait for calibration to be finished

	/* Channel Configuration Before Enable */
	for(u8 i = 0; i < A_config->channelsCount; i++){
		if(A_config->mode == ADC_MODE_SINGLE)
			L_ADC_Reg->DIFSEL &= ~(1 << A_config->channels[i]);
		else // Differential
			L_ADC_Reg->DIFSEL |= (1 << A_config->channels[i]);
	}

	/* Enable ADC */
	L_ADC_Reg->ISR |= (1 << 0);
	L_ADC_Reg->CR |= (1 << 0);
	while( !((L_ADC_Reg->ISR >> 0) & 1) );

	/* Channel Configuration After Enable */
	// Resolution
	L_ADC_Reg->CFGR &= ~(0b11 << 3);
	L_ADC_Reg->CFGR |= (A_config->resolution << 3);

	// Conversion Type
	if(A_config->conversionType == ADC_CONVERSION_SINGLE)
		L_ADC_Reg->CFGR &= ~(1 << 13);
	else // Continuous
		L_ADC_Reg->CFGR |= (1 << 13);

	// Channel Selection
	L_ADC_Reg->SQR[0] &= ~(0b1111 << 0);
	L_ADC_Reg->SQR[0] |= ((A_config->channelsCount - 1) << 0);

	for(u8 i = 0; i < A_config->channelsCount; i++){
		if(i < 4)
			L_ADC_Reg->SQR[0] |= (A_config->channels[i] << ((i + 1) * 6));
		else
			L_ADC_Reg->SQR[i / 5] |= (A_config->channels[i] << ((i % 5) * 6));
	}
}

void ADC_voidStartConversion(ADC_Config_t* A_config)
{
	ADC_RegMap_t* L_ADC_Reg;

	switch (A_config->adcNumber)
	{
	case ADC1:
		L_ADC_Reg = ADC1_MASTER;
		break;
	case ADC2:
		L_ADC_Reg = ADC2_SLAVE;
		break;
	case ADC3:
		L_ADC_Reg = ADC3_MASTER;
		break;
	case ADC4:
		L_ADC_Reg = ADC4_SLAVE;
		break;
	case ADC5:
		L_ADC_Reg = ADC5_SINGLE;
		break;
	}

	L_ADC_Reg->CR |= (1 << 2);
}

void ADC_voidSingleRead(ADC_Config_t* A_config, u16* result)
{
	ADC_RegMap_t* L_ADC_Reg;

	switch (A_config->adcNumber)
	{
	case ADC1:
		L_ADC_Reg = ADC1_MASTER;
		break;
	case ADC2:
		L_ADC_Reg = ADC2_SLAVE;
		break;
	case ADC3:
		L_ADC_Reg = ADC3_MASTER;
		break;
	case ADC4:
		L_ADC_Reg = ADC4_SLAVE;
		break;
	case ADC5:
		L_ADC_Reg = ADC5_SINGLE;
		break;
	}

	L_ADC_Reg->CR |= (1 << 2);

	/* Wait for EOC End of Conversion */
	for(u8 i = 0; i < A_config->channelsCount; i++){
		while( !((L_ADC_Reg->ISR >> 2) & 1) );
		result[i] = (u16) (L_ADC_Reg->DR);
	}
}
