/*
 * main.c
 *
 *  Created on: Feb 5, 2024
 *      Author: es-maro19996
 */

#include "../Inc/main.h"

/* MACROS */
#define CAN_IDS_COUNT				5

/* Global Variables */
u32 IDs[CAN_IDS_COUNT] = {0x01, 0x02, 0x03, 0x09, 0x0A};
// 0x01 -> Home
// 0x02 -> Lift up/down {Generic}
// 0x03 -> Current Sense
// 0x09, 0x0A -> for DEBUG_NO_CANging purposes

//Inverted Logic: 0 -> Button is pressed 1-> Button is not pressed
u8 LimSwitch = 1;
u32 currentPosition = 0;



int main(void)
{
	APP_voidSystemClockInit();
	APP_voidGpioInit();
	APP_voidCanInit();

	CAN_Frame_t receiveFrame;
	receiveFrame.id = 0x000;
	receiveFrame.dlc = 0;

	while(1)
	{
		LimSwitch = LIMSWI_u8SwitchPress(LIMSWI[0].Port, LIMSWI[0].Pin);
		APP_voidEnsureSafety();	// Return back to homing position only if the plate is excessively pressuring the base


		if(CAN_u8GetReceivedMessagesCount(CAN1, CAN_RX_FIFO0) > 0)
			CAN_voidReceiveDataFrame(CAN1, &receiveFrame, CAN_RX_FIFO0);

		/*Switch case according to CAN Message recieved*/
		switch(receiveFrame.id)
		{
			case 0x01:		/* Motion to Home case */
				APP_voidHoming();
				break;

			case 0x02:		/* Motion in (any direction / any speed) case */
				for(int i=0; i</*receiveFrame.data[2] *200 */ 800; i++)
				{
					/*receiveData[0] -> Stepper Direction
					 * recieveData[1] -> Stepper Delay
					 * recieveData[2] -> Number of Stepper Cycles*/

					STEPPER_voidQuadMotorStep(&STEPPER_mod[0], &STEPPER_mod[1], &STEPPER_mod[2], &STEPPER_mod[3], /*receiveFrame.data[0]*/ 1, TIM2, /*receiveFrame.data[1]*/ 5);
//						LimSwitch = LIMSWI_u8SwitchPress(LIMSWI[0].Port, LIMSWI[0].Pin);						//Check for the current state of the limit switch
//						if(!LimSwitch)																			//If the limit switch is pressed -> break the motion + store the value of i as it shows what the current position is
//						{
//							currentPosition = currentPosition - i;												//Negative only because the switch being pressed means that the direction of motion is downwards
//							break;
//						}
				}
//					if(receiveFrame.data[0] == 1)																//Check for the direction
//						currentPosition = currentPosition + receiveFrame.data[2] * 200;
//					else
//						currentPosition = currentPosition - receiveFrame.data[2] * 200;
				receiveFrame.id = 0x03;
				break;


			case 0x03:	/* Current Position Case */
				for(int i =0; i<5; i++)
				{
					GPIO_voidSetPinValue(GPIO_PORTC, GPIO_PIN13, 1);
					TIMERx_voidDelay_ms(TIM2, 500);
					GPIO_voidSetPinValue(GPIO_PORTC, GPIO_PIN13, 0);
					TIMERx_voidDelay_ms(TIM2, 500);
				}
				receiveFrame.id = 0x02;
				break;

			/* Current Sense Case */
			case 0x09:
				GPIO_voidTogglePin(GPIO_PORTC, 13);
				TIMERx_voidDelay_ms(TIM2, 500);
				break;

			default:
				break;
		}
		//receiveFrame.id = 0x00;
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

	TIMERx_voidCounter_Init(TIM2, &timer2);
	STEPPER_voidInitMotor(&STEPPER_mod[0]);
	STEPPER_voidInitMotor(&STEPPER_mod[1]);
	STEPPER_voidInitMotor(&STEPPER_mod[2]);
	STEPPER_voidInitMotor(&STEPPER_mod[3]);
	LIMSWI_voidSwitchInit();
	//only for led blinking test
	GPIO_voidInitPin(&Test3);
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
	while(LimSwitch)																			//While the switch isn't pressed, exits once it is pressed
	{
		STEPPER_voidQuadMotorHome(&STEPPER_mod[0], &STEPPER_mod[1], &STEPPER_mod[2], &STEPPER_mod[3], TIM2);		//Take one step towards homing position
		LimSwitch = LIMSWI_u8SwitchPress(LIMSWI[0].Port, LIMSWI[0].Pin);											//Update the status of the switch
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
