/*
 * ADC_Registers.h
 *
 *  Created on: Dec 24, 2023
 *      Author: Mahmoud Ahmed
 */

#ifndef ADC_ADC_REGISTERS_H_
#define ADC_ADC_REGISTERS_H_

#define ADC12_BASE_ADDR			0x50000000U
#define ADC345_BASE_ADDR		0x50000400U

#define ADC_MASTER_OFFSET		0x000
#define ADC_SLAVE_OFFSET		0x100
#define ADC_SINGLE_OFFSET		0x200
#define ADC_COMMON_OFFSET		0x300

typedef struct{
	u32 ISR;
	u32 IER;
	u32 CR;
	u32 CFGR;
	u32 CFGR2;
	u32 SMPR1;
	u32 SMPR2;
	u32 reserved0;

	u32 TR1;
	u32 TR2;
	u32 TR3;
	u32 reserved1;

	u32 SQR[4];

	u32 DR;
	u32 reserved2[2];
	u32 JSQR;

	u32 reserved3[4];

	u32 OFR1;
	u32 OFR2;
	u32 OFR3;
	u32 OFR4;

	u32 reserved4[4];

	u32 JDR1;
	u32 JDR2;
	u32 JDR3;
	u32 JDR4;

	u32 reserved5[4];

	u32 AWD2CR;
	u32 AWD3CR;
	u32 reserved6[2];

	u32 DIFSEL;
	u32 CALFACT;
	u32 reserved7[2];

	u32 GCOMP;
}ADC_RegMap_t;

typedef struct{
	u32 CSR;
	u32 reserved0;
	u32 CCR;
	u32 CDR;
}ADC_CommonRegMap_t;

#define ADC1_MASTER 	(ADC_RegMap_t *)(ADC12_BASE_ADDR + ADC_MASTER_OFFSET)
#define ADC2_SLAVE 		(ADC_RegMap_t *)(ADC12_BASE_ADDR + ADC_SLAVE_OFFSET)
#define ADC12_COMMON 	(ADC_CommonRegMap_t *)(ADC12_BASE_ADDR + ADC_COMMON_OFFSET)

#define ADC3_MASTER 	(ADC_RegMap_t *)(ADC345_BASE_ADDR + ADC_MASTER_OFFSET)
#define ADC4_SLAVE 		(ADC_RegMap_t *)(ADC345_BASE_ADDR + ADC_SLAVE_OFFSET)
#define ADC5_SINGLE		(ADC_RegMap_t *)(ADC345_BASE_ADDR + ADC_SINGLE_OFFSET)
#define ADC345_COMMON 	(ADC_CommonRegMap_t *)(ADC345_BASE_ADDR + ADC_COMMON_OFFSET)

#endif /* ADC_ADC_REGISTERS_H_ */
