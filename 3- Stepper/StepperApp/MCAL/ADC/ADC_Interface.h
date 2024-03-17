/*
 * ADC_Interface.h
 *
 *  Created on: Dec 24, 2023
 *      Author: Mahmoud Ahmed
 */

#ifndef ADC_ADC_INTERFACE_H_
#define ADC_ADC_INTERFACE_H_

#include "../../LIB/STD_TYPES.h"
#include "ADC_Registers.h"
#include "ADC_Config.h"

typedef enum{
	ADC1,
	ADC2,
	ADC3,
	ADC4,
	ADC5
}ADC_t;

typedef enum{
	ADC_CHANNEL1=1,
	ADC_CHANNEL2,
	ADC_CHANNEL3,
	ADC_CHANNEL4,
	ADC_CHANNEL5,
	ADC_CHANNEL6,
	ADC_CHANNEL7,
	ADC_CHANNEL8,
	ADC_CHANNEL9,
	ADC_CHANNEL10,
	ADC_CHANNEL11,
	ADC_CHANNEL12,
	ADC_CHANNEL13,
	ADC_CHANNEL14,
	ADC_CHANNEL15,
}ADC_Channel_t;

typedef struct{
	ADC_t adcNumber;
	u8 mode;
	u8 resolution;
	u8 conversionType;
	u8 channelsCount;
	ADC_Channel_t* channels;
}ADC_Config_t;

/* Mode */
#define ADC_MODE_SINGLE				0
#define ADC_MODE_DIFFERENTIAL		1

/* Resolution */
#define ADC_RESOLUTION_12			0
#define ADC_RESOLUTION_10			1
#define ADC_RESOLUTION_8			2
#define ADC_RESOLUTION_6			3

/* Conversion Type */
#define ADC_CONVERSION_SINGLE		0
#define ADC_CONVERSION_CONTINOUS	1

void ADC_voidInit(ADC_Config_t* A_config);
void ADC_voidStartConversion(ADC_Config_t* A_config);
void ADC_voidSingleRead(ADC_Config_t* A_config, u16* result);

/*
 * ADC12_IN1  --> PA0
 * ADC12_IN2  --> PA1
 * ADC1_IN3   --> PA2
 * ADC1_IN4   --> PA3
 * ADC1_IN5   --> PB14
 * ADC12_IN6  --> PC0
 * ADC12_IN7  --> PC1
 * ADC12_IN8  --> PC2
 * ADC12_IN9  --> PC3
 * ADC1_IN10  --> PF0
 * ADC1_IN11  --> PB12
 * ADC1_IN12  --> PB1
 * ADC12_IN14 --> PB11
 * ADC1_IN15  --> PB0
 */

#endif /* ADC_ADC_INTERFACE_H_ */
