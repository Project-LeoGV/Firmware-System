/*
 * main.c
 *
 *  Created on: Feb 5, 2024
 *      Author: es-maro19996 & Farha Hany
 */

#include "../Inc/main.h"

/* MACROS */
#define CAN_IDS_COUNT				5

/* Global Variables */
u32 IDs[CAN_IDS_COUNT] = {0x21, 0x22, 0x23, 0x29, 0x0A};
// 0x01 -> Home
// 0x02 -> Lift up/down {Generic}
// 0x03 -> Current Sense
// 0x09, 0x0A -> for DEBUG_NO_CANging purposes

//Inverted Logic: 0 -> Button is pressed 1-> Button is not pressed
u8 LimSwitch = 1;
u32 currentPosition = 0;
ADC_Config_t adcConfig[2];




int main(void)
{
	APP_voidSystemClockInit();
	APP_voidGpioInit();
	APP_voidCanInit();
//	APP_voidADCInit();

	CAN_Frame_t receiveFrame;
	receiveFrame.id = 0x00;
	receiveFrame.dlc = 0;

	while(1)
	{
		LimSwitch = LIMSWI_u8SwitchPress(LIMSWI[1].Port, LIMSWI[1].Pin);
		APP_voidEnsureSafety();	// Return back to homing position only if the plate is excessively pressuring the base
//		GPIO_voidTogglePin(GPIO_PORTB, GPIO_PIN0);
//		GPIO_voidSetPinValue(GPIO_PORTB, GPIO_PIN1, GPIO_VALUE_LOW);

		if(CAN_u8GetReceivedMessagesCount(CAN1, CAN_RX_FIFO0) > 0)
			CAN_voidReceiveDataFrame(CAN1, &receiveFrame, CAN_RX_FIFO0);

		/*Switch case according to CAN Message received*/
		switch(receiveFrame.id)
		{
			case 0x21:		/* Motion to Home case */
				APP_voidHoming();
				currentPosition = 0;
				break;

			case 0x22:		/* Motion in (any direction / any speed) case */
				u8 stepDir = receiveFrame.data[0] - '0';
				u32 stepDelay = receiveFrame.data[1] - '0';
				u8 stepRot = receiveFrame.data[2] - '0';

				for(int i=0; i< stepRot *200; i++)
				{

					/*receiveData[0] -> Stepper Direction
					 * recieveData[1] -> Stepper Delay
					 * recieveData[2] -> Number of Stepper Cycles*/

					STEPPER_voidQuadMotorStep(&STEPPER_mod[0], &STEPPER_mod[1], &STEPPER_mod[2], &STEPPER_mod[3], stepDir, TIM2, stepDelay);
						LimSwitch = LIMSWI_u8SwitchPress(LIMSWI[1].Port, LIMSWI[1].Pin);						//Check for the current state of the limit switch
						if(LimSwitch == 0)																			//If the limit switch is pressed -> break the motion + store the value of i as it shows what the current position is
						{
							currentPosition = currentPosition - i;												//Negative only because the switch being pressed means that the direction of motion is downwards
							break;
						}
				}
					if(receiveFrame.data[0] == 1)																//Check for the direction
						currentPosition = currentPosition + receiveFrame.data[2] * 200;
					else
						currentPosition = currentPosition - receiveFrame.data[2] * 200;
				break;


			case 0x23:	/* Current Position Case */
				for(int i =0; i<5; i++)
				{
					GPIO_voidSetPinValue(GPIO_PORTC, GPIO_PIN13, 1);
					TIMERx_voidDelay_ms(TIM2, 500);
					GPIO_voidSetPinValue(GPIO_PORTC, GPIO_PIN13, 0);
					TIMERx_voidDelay_ms(TIM2, 500);
				}
				break;

			/* Current Sense Case */
			case 0x29:
				if (receiveFrame.rtr == CAN_FRAME_REMOTE) //empty frame, rtr =1
				{
					APP_voidCurrentSense();
				}
				break;

			default:
				break;
		}
		receiveFrame.id = 0x00;
	}
}

void APP_voidSystemClockInit(void)
{
	RCC_voidInit();
	RCC->CCIPR |= (1 << 25);
	//Initialize RCC
	RCC_voidPeripheralClockEnable(RCC_AHB2, GPIO_A);
	RCC_voidPeripheralClockEnable(RCC_AHB2, GPIO_B);
	RCC_voidPeripheralClockEnable(RCC_AHB2, GPIO_C);
	RCC_voidPeripheralClockEnable(RCC_APB1_1, TIM_2);
	RCC_voidPeripheralClockEnable(RCC_APB1_1, FDCAN);
//	RCC_voidPeripheralClockEnable(RCC_AHB2, ADC12);
	RCC->AHB2ENR |= (1<<13);
	RCC->AHB2ENR |= (1<<14);



}

void APP_voidGpioInit(void)
{
	//Only for led blinking test, should be removed form final version
	MGPIO_Config_t Test3 = {
		GPIO_PORTC,
		GPIO_PIN13,
		GPIO_MODE_OUTPUT,
		GPIO_SPEED_LOW,
		GPIO_NO_PULL,
		GPIO_AF0
	};

	//Timer configuration outside main function
	TIMx_Counter_Cfg_t timer2 = {
		TIMER_IC_PRESCALER_NO,
		TIMER_AUTORELOAD_ENABLE,
		TIMER_COUNT_DOWN
	};

	//ADC configration outside main function
		MGPIO_Config_t adc_1 = {
	        /////////GPIO_PORTA,
			/////////GPIO_PIN2,
			GPIO_MODE_ANALOG,
			GPIO_SPEED_LOW,
			GPIO_NO_PULL,
			GPIO_AF0
		};
	    MGPIO_Config_t adc_2 = {
			GPIO_PORTA,
			GPIO_PIN4,
			GPIO_MODE_ANALOG,
			GPIO_SPEED_LOW,
			GPIO_NO_PULL,
			GPIO_AF0
		};
		MGPIO_Config_t adc_3 = {
			GPIO_PORTA,
			GPIO_PIN6,
			GPIO_MODE_ANALOG,
			GPIO_SPEED_LOW,
			GPIO_NO_PULL,
			GPIO_AF0
		};
		MGPIO_Config_t adc_4 = {
			GPIO_PORTB,
			GPIO_PIN1,
			GPIO_MODE_ANALOG,
			GPIO_SPEED_LOW,
			GPIO_NO_PULL,
			GPIO_AF0
		};

	TIMERx_voidCounter_Init(TIM2, &timer2);
	STEPPER_voidInitMotor(&STEPPER_mod[0]);
	STEPPER_voidInitMotor(&STEPPER_mod[1]);
	STEPPER_voidInitMotor(&STEPPER_mod[2]);
	STEPPER_voidInitMotor(&STEPPER_mod[3]);
	LIMSWI_voidSwitchInit();
//	GPIO_voidInitPin(&adc_1);
//	GPIO_voidInitPin(&adc_2);
//	GPIO_voidInitPin(&adc_3);
//	GPIO_voidInitPin(&adc_4);
	//only for led blinking test
	GPIO_voidInitPin(&Test3);
}

void APP_voidADCInit(void)
{
    adcConfig[0].adcNumber = ADC1;
	adcConfig[0].mode = ADC_MODE_SINGLE;
	adcConfig[0].resolution = ADC_RESOLUTION_12;
	adcConfig[0].conversionType = ADC_CONVERSION_SINGLE;
	adcConfig[0].channelsCount = 4;
	u8 ADC1_Channels[] = {ADC_CHANNEL12, ADC_CHANNEL15, ADC_CHANNEL3, ADC_CHANNEL4};
	adcConfig[0].channels = ADC1_Channels;
//	adcConfig[0].channels[0] = ADC_CHANNEL12;				//A4
//	adcConfig[0].channels[1] = ADC_CHANNEL15;				//B4
//	adcConfig[0].channels[2] = ADC_CHANNEL3;				//A1
//	adcConfig[0].channels[3] = ADC_CHANNEL4;				//B1
//	ADC_voidInit(&adcConfig[0]);

	adcConfig[1].adcNumber = ADC2;
	adcConfig[1].mode = ADC_MODE_SINGLE;
	adcConfig[1].resolution = ADC_RESOLUTION_12;
	adcConfig[1].conversionType = ADC_CONVERSION_SINGLE;
	adcConfig[1].channelsCount = 4;
//	u8 ADC1_Channels[] = {ADC_CHANNEL12, ADC_CHANNEL15, ADC_CHANNEL3, ADC_CHANNEL4};
//	adcConfig[0].channels = ADC1_Channels;
//    adcConfig[1].channels[0] = ADC_CHANNEL17;				//A2
//    adcConfig[1].channels[1] = ADC_CHANNEL13;				//B2
//    adcConfig[1].channels[2] = ADC_CHANNEL3;				//A3
//    adcConfig[1].channels[3] = ADC_CHANNEL4;				//B3
	//ADC_voidInit(&adcConfig[1]);

}

void APP_voidCurrentSense(void)
{
	/*Reading ADC values*/
    u16 adcValues1[4] = {'0', '1', '5', '7'};
    u16 adcValues2[4] = {'7', '4', '3', '1'};
	//ADC_voidSingleRead(&adcConfig[0], adcValues1);
	//ADC_voidSingleRead(&adcConfig[1], adcValues2);

	/*Creating CAN frame*/
	CAN_Frame_t adcFrame = {
		.id = 0x029,  /////
		.dlc = 8,
		.rtr = CAN_FRAME_DATA,
		.ide = CAN_FRAME_STANDARD_ID
	};
    for (int i=0; i<4; i++)
	{
	    adcFrame.data[i*2] = (u8)(adcValues1[i] >> 8); //the higher byte
        adcFrame.data[i*2 + 1] = (u8)(adcValues1[i] & 0xFF); //the lower byte

	}
	CAN_voidSendDataFrame(CAN1, &adcFrame);
}

void APP_voidCanInit(void)
{
	MGPIO_Config_t canTxPin = {.Port = GPIO_PORTB, .Pin = GPIO_PIN9, .Mode = GPIO_MODE_ALTF,.AltFunc = GPIO_AF9,.OutputSpeed = GPIO_SPEED_LOW,.OutputType = GPIO_OT_PUSHPULL};
	MGPIO_Config_t canRxPin = {.Port = GPIO_PORTB, .Pin = GPIO_PIN8, .Mode = GPIO_MODE_ALTF,.AltFunc = GPIO_AF9,.OutputSpeed = GPIO_SPEED_LOW,.OutputType = GPIO_OT_PUSHPULL};

	CAN_TxConfig_t txCfg;
	txCfg.automaticTransmission = CAN_AUTOMATIC_TRANSMISSION_DISABLE;
	txCfg.bufferType = CAN_TX_BUFFER_FIFO;
	txCfg.transmitPause = CAN_TX_PAUSE_DISABLE;

	CAN_RxConfig_t rxCfg;
	rxCfg.FIFO0_Mode = CAN_RX_FIFO_OVERWRITE;
	rxCfg.FIFO1_Mode = CAN_RX_FIFO_OVERWRITE;
	rxCfg.FIFO0_numberOfIDs = CAN_IDS_COUNT;
	rxCfg.FIFO1_numberOfIDs = 0;
	rxCfg.FIFO0_IDs = IDs;
	rxCfg.FIFO1_IDs = NULL;
	rxCfg.nonMatchingFrames = CAN_RX_REJECT;

	GPIO_voidInitPin(&canTxPin);
	GPIO_voidInitPin(&canRxPin);
	CAN_voidInit(CAN1, &rxCfg, &txCfg);
}

void APP_voidHoming(void)
{
	while(LimSwitch == 1)																			//While the switch isn't pressed, exits once it is pressed
	{
		STEPPER_voidQuadMotorHome(&STEPPER_mod[0], &STEPPER_mod[1], &STEPPER_mod[2], &STEPPER_mod[3], TIM2);		//Take one step towards homing position
		LimSwitch = LIMSWI_u8SwitchPress(LIMSWI[1].Port, LIMSWI[1].Pin);											//Update the status of the switch
	}
	currentPosition = 0;
}

void APP_voidEnsureSafety(void)
{
	if(currentPosition < 0)
	{
		while(currentPosition < 0)
		{
			STEPPER_voidQuadMotorHome(&STEPPER_mod[0], &STEPPER_mod[1], &STEPPER_mod[2], &STEPPER_mod[3], TIM2);
			currentPosition+= 4;
		}
	}
}
