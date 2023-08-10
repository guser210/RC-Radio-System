/*
 * main.cpp
 *
 *  Created on: Mar 31, 2023
 *      Author: gvargas
 */

#include "main.h"
#include "LoRa.h"
#include "string.h"
extern "C" SPI_HandleTypeDef hspi2;
extern "C" UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;


uint8_t temp[10] = {0};
uint8_t temp2[10] = {0};

volatile uint8_t TxDone = 0;
volatile uint8_t RxDone = 0;
volatile uint8_t TxRxTimeout = 0;
volatile uint8_t irqStaus15_8;
volatile uint8_t irqStatus7_0;
volatile uint8_t irqHeaderValid = 0;
volatile uint8_t irqHeaderError = 0;
volatile uint32_t crcErrorCounter = 0;
volatile uint32_t headerErrorCounter = 0;
volatile uint32_t timeoutCounter = 0;

volatile uint8_t irqCRCError = 0;

volatile uint32_t debugCounter1 = 0;
volatile uint32_t debugCounter2 = 0;
volatile uint32_t debugCounter3 = 0;
volatile uint32_t debugCounter4 = 0;
volatile uint32_t debugCounter5 = 0;
volatile uint32_t debugCounter6 = 0;
volatile uint8_t LoRaPacket[10];

LoRa *pRadio;

volatile uint8_t resetLoRa = 0;

uint16_t map(int x, int in_min, int in_max, int out_min, int out_max);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim17 )
	{
//		resetNR++;
	}

}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim17 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		resetLoRa++;
		TIM17->CNT = 0;
	}
	else if( htim == &htim14 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		LED_GPIO_Port->BRR = LED_Pin;
		LED2_GPIO_Port->BRR = LED2_Pin;
		TIM14->CNT = 0;

	}
}
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case DIO1_Pin:
		TxDone  = 1;
		if( pRadio != NULL)
		{
			pRadio->GetIrqStatus((uint8_t&)irqStaus15_8, (uint8_t&)irqStatus7_0);
			TxDone = irqStatus7_0 & 1;
			RxDone = (irqStatus7_0 & 2)>>1;
			pRadio->ClearIrqStatus(0b00000000, 0b11);
			if( RxDone == 1)
				pRadio->ReadBuffer(0, (uint8_t*)LoRaPacket,6);// rxPayloadLen > 5 ? 6 : rxPayloadLen);
		}

		break;
	case DIO2_Pin:
		if( pRadio != NULL)
		{
			TxRxTimeout = 1;
			timeoutCounter++;
			pRadio->GetIrqStatus((uint8_t&)irqStaus15_8, (uint8_t&)irqStatus7_0);
			pRadio->ClearIrqStatus(0b01000000, 0b00000000);
		}
		break;
	case DIO3_Pin:

		if( pRadio != NULL)
		{
			pRadio->GetIrqStatus((uint8_t&)irqStaus15_8, (uint8_t&)irqStatus7_0);
			irqHeaderValid = (irqStatus7_0 & 0b00010000)>>4;
			irqHeaderError = (irqStatus7_0 & 0b00100000)>>5;
			irqCRCError = (irqStatus7_0 & 0b01000000)>>6;

			if( irqHeaderError)
			{
				headerErrorCounter++;
				/*
				 * 16.2 LoRa Modem: Additional Header Checks Required
				 * Page 150.
				 * 16.2.1 Problem Description
					If the LoRa modem is used with the header enabled, in the presence of a header error no RxDone or RxTimeout will be
					generated.
					16.2.2 Problem Solution
					To prevent the radio becoming blocked in LoRa receive mode in the presence of the corrupted header, the HeaderError
					interrupt must be mapped to the DIO lines and routed to the host microcontroller. Upon reception of the HeaderError
					interrupt the host could, for example, put the radio back in receiver mode.
					The process for mapping the interrupt is shown in Section 11.9 of the datasheet.
				 */
				pRadio->SetRx(LoRa::ms_0,0, 0);
			}
			if( irqCRCError)
				crcErrorCounter++;


			pRadio->ClearIrqStatus(0b00000000, 0b01100000);

		}
		break;
	default:
		break;
	}

}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
}
uint16_t map(int x, int in_min, int in_max, int out_min, int out_max) {
	if (x < in_min)
		x = in_min;

	if (x > in_max)
		x = in_max;

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void SBUS_Decode(volatile uint8_t *sbus, volatile uint16_t *channel, uint8_t size)
{
	uint8_t bits = 0;
	uint8_t channelIndex = 0;
	uint32_t buffer = 0;

	for( int index = 1; index < size; index += 2)
	{
		buffer |= sbus[index]<<bits;
		bits += 8;
		buffer |= sbus[index+1]<<bits;
		channel[channelIndex++] = buffer & 0x07FF;
		buffer >>= 11;
		if( (bits -= 3) >= 11)
		{
			channel[channelIndex++] = buffer & 0x07FF;
			buffer >>= 11;
			bits -= 11;

		}
	}
}


volatile uint8_t lostOfSignal = 0;
void SBUS_Encode(volatile uint16_t* channel, uint8_t size, volatile uint8_t *sbus)
{
	//uint8_t sbus[25] = { 0 }; // SBUS bucket buffer.
	uint8_t bits = 0;			// bit counter for staging buffer.
	uint8_t busIndex = 1;		// current bucket being assigned
	uint32_t buffer = 0;		// staging buffer of bits, will append 11 bits per channel.

	sbus[0] = 0x0f;				// set SBUS flag (0x0F).

	for (int i = 0; i < size; i++)
	{
		buffer |= (channel[i] & 0x07ff)<<bits;	// Append 11 bits of channel data MSB.
		sbus[busIndex++] = buffer & 0xff;		// assign 8 LSB to sbus bucket byte.
		buffer >>= 8;							// Shift 8 bits to the right, those bits were assigned to sbus bucket.
		if ((bits += 3) >= 9)					// Increment bit count in buffer of bits and compare to 9.
		{										//
			sbus[busIndex++] = buffer & 0xff;	// assign 8 bits to next sbus bucket to prevent overflow.
			buffer >>= 8;						// Shift 8 bits to the right, those bits were assigned to sbus bucket.
			bits -= 8;							// decrement bit count in buffer of bits.
		}

	}
	sbus[busIndex++] = buffer & 0xff;			// assign left over bits to next two buckets
	sbus[busIndex] = (buffer>>8) & 0xff;

	sbus[23] = 0;								//assing las 2 bytes incase they were assigned by mistake.
	sbus[24] = 0;

}
volatile uint8_t uart_tx_ready = 1;
volatile uint8_t uart_rx_ready = 0;

volatile uint8_t rx_buffer[25] = {0};
volatile uint8_t packet[26];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_tx_ready = 1;
}

volatile HAL_StatusTypeDef ret;
volatile uint8_t buffer[16] = {"This is LoRa"};
volatile uint32_t packetsSent = 0;
volatile uint8_t outpuPower = 0;
volatile uint16_t outputCounter = 0;
volatile uint16_t channel[17] = {0};
volatile uint16_t channelR[17] = {0};

uint8_t rxPayloadLen;
uint8_t rxStartBufferPointer;

//volatile uint8_t radioChannel = 1;
//void ResetLoRagOOD(uint8_t radioChannel, uint8_t LoRaWord)
//{
//	pRadio->Init();
//
//
//	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);
//	//pRadio->SetStandby(LoRa::STDBY_RC_13MHz);
//	pRadio->SetPacketType(LoRa::PACKET_TYPE_LORA);
//
//	pRadio->SetChannel(radioChannel);
//	pRadio->SetBufferBaseAddress(0x80,0);
//
//	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_400,LoRa::LORA_CR_4_5);
//	pRadio->SetPacketParams(0x08, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);
////	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);
//
////	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_1600,LoRa::LORA_CR_4_5);
////	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 25, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);
//
//	pRadio->SetDioIrqParams(0b01000000,0b01100011
//						   ,0b00000000,0b00000011
//						   ,0b01000000,0b00000000
//						   ,0b00000000,0b01100000);
//
//	pRadio->SetLoRaWord(LoRaWord);
//	TxDone = 0;
//	RxDone = 0;
//
////	buffer[0] = 0;
////	buffer[1] = 0;
////	buffer[2] = 0;
////	buffer[3] = 0;
////	pRadio->ReadRegister(0x944,  (uint8_t*)buffer, 4);
////
//	RX_EN_GPIO_Port->BSRR = RX_EN_Pin; // enable RX_EN_Pin, not sure what this does still.
//
//	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);
//
//}
void ResetLoRa(uint8_t radioChannel, uint8_t LoRaWord)
{
	pRadio->Init();



	pRadio->SetStandby(LoRa::STDBY_RC_13MHz);

	volatile uint8_t gain[1] = {0}; // High sensitivity gain by 3db page30.
	pRadio->ReadRegister(0x0891, (uint8_t*)gain, 1);
	gain[0] |= (0xc0);
	pRadio->WriteRegister(0X891, (uint8_t*)gain, 1);
	gain[0] = (0);
	pRadio->ReadRegister(0x0891, (uint8_t*)gain, 1);


	pRadio->SetAutoFS(); // reduce switching time Page85.
	pRadio->SetRegulatorMode(1); // use dcdc page143.

	pRadio->SetPacketType(LoRa::PACKET_TYPE_LORA);

	radioChannel = 50;
	pRadio->SetChannel(radioChannel);
	pRadio->SetBufferBaseAddress(0x80,0);

	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_400,LoRa::LORA_CR_4_5);
	pRadio->SetPacketParams(0x08, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_INVERTED, 0x00, 0x00);
//	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);

//	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_1600,LoRa::LORA_CR_4_5);
//	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 25, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);

	pRadio->SetDioIrqParams(0b01000000,0b01100011
						   ,0b00000000,0b00000011
						   ,0b01000000,0b00000000
						   ,0b00000000,0b01100000);

	pRadio->SetLoRaWord(LoRaWord);
	TxDone = 0;
	RxDone = 0;

//	buffer[0] = 0;
//	buffer[1] = 0;
//	buffer[2] = 0;
//	buffer[3] = 0;
//	pRadio->ReadRegister(0x944,  (uint8_t*)buffer, 4);
//
	RX_EN_GPIO_Port->BSRR = RX_EN_Pin; // enable RX_EN_Pin, not sure what this does still.

	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);

}
volatile uint8_t radioConnected = 0;
volatile uint8_t testPacket = 0;

volatile uint8_t status = 0;

void setup()
{
	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,500);
	if(HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	TIM14->CCR1 = 6000;
	if(HAL_TIM_OC_Start_IT(&htim14, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
}

volatile uint8_t rssiValue = 0;
volatile double rssiAgv = 0.0;
volatile double rssiW = 0.01;
volatile double rssiMultiplier = (6666/255);
void maincpp()
{


	setup();


	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	uint8_t radioChannel = 1;

	uint8_t LoRaWord = 210;
	uint8_t rxPayloadLen = 0;
	uint8_t rxStartBufferPointer = 0;

	LoRa radio;
	pRadio = &radio;
	pRadio->nresetPort = NRESET_GPIO_Port;
	pRadio->nreset = NRESET_Pin;

	pRadio->busyPort = BUSY_GPIO_Port;
	pRadio->busy = BUSY_Pin;

	pRadio->nssPort = NSS_GPIO_Port;
	pRadio->nss = NSS_Pin;
	pRadio->port = &hspi2;

	ResetLoRa(radioChannel, LoRaWord);


	memset((void*)buffer,0,16);

	for( int i = 0; i < 16; i++)
		channel[i] = 172;




	while(1)
	{

		TxRxTimeout = 0;
		irqHeaderError = 0;
		irqCRCError = 0;
		pRadio->ClearIrqStatus(0xff, 0xff);
		pRadio->SetRx(LoRa::ms_0,0,0); // always receving..
		RxDone = 0;

		while(RxDone == 0 )
		{
			if( resetLoRa >= 3)
				break;
		}


		if( RxDone == 1)
		{
			RxDone = 0;
			radioConnected = 1;
			resetLoRa = 0;
		}

		if( resetLoRa >= 3 )
		{

			/*
			 * Lost of signal
			 * send disarm command.
			 * Reset radio just in case.
			 */

			radioConnected = 0;
			//			packet[23] |= (1); // digital channel 17
			//			packet[23] |= (2); // digital channel 18
			//			packet[23] |= (4); lost frame
			//          packet[23] |= (8); // <-------- failsafe

			TxRxTimeout = 0;


//			SBUS_Decode(packet, channel, 25);
			channel[2] = 172; // set Throttle to 0;
			channel[4] = 1811; // disarm.


			SBUS_Encode(channel, 16, packet);



			for( int i = 0; i < 5; i++)
			{
				/*
				 * Disarm on lost of signal.
				 * TODO: add GPS before implementing failsafe.
				 * packet[23] |= (8); //failsafe
				 */

				while(uart_tx_ready == 0)
				{
					HAL_Delay(1);
				}

				HAL_UART_Transmit_IT(&huart1, (uint8_t*)packet,25);
				uart_tx_ready = 0;

				HAL_Delay(3);
			}


			LED2_GPIO_Port->BSRR = LED2_Pin;

			HAL_Delay(20);
			LED2_GPIO_Port->BRR = LED2_Pin;
			LED_GPIO_Port->BRR = LED_Pin;
			TIM17->CNT = 0;
			TIM14->CNT = 0;
			ResetLoRa(radioChannel,LoRaWord);
			TIM14->CNT = 0;
			TIM17->CNT = 0;
			resetLoRa = 2;

			continue;

		}

		TIM17->CNT = 0;
		TIM14->CNT = 0;
		LED_GPIO_Port->BSRR = LED_Pin;
		LED2_GPIO_Port->BSRR = LED2_Pin;

		// Read message parameters.
//		pRadio->GetRxBufferStatus(rxPayloadLen, rxStartBufferPointer);


		// Read incoming message.



		RxDone = 0;
		if( radioConnected
				&& uart_tx_ready == 1
				&& irqCRCError == 0
				&& irqHeaderError == 0)// && rxPayloadLen >= 6)
		{
			//memset((void*)packet,0,25);
//			pRadio->ReadBuffer(0, (uint8_t*)LoRaPacket,6);// rxPayloadLen > 5 ? 6 : rxPayloadLen);
			/*
			 * pass data to FC if error free.
			 * TODO: detech overlapping Synch word.
			 */


		    // 0 Roll.
		    // 1 Pitch.
		    // 2 Throttle
		    // 3 Yaw
			volatile uint16_t bigBuffer = 0;

			bigBuffer = LoRaPacket[1]<<8 |
					LoRaPacket[0];


			channel[3] = ((bigBuffer & 0x03ff) * 1.56402738) ;//map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer = (bigBuffer>>10) |  (LoRaPacket[2]<<6);
			channel[2] = ((bigBuffer & 0x03ff) * 1.56402738) ;// map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer = (bigBuffer>>10) |  (LoRaPacket[3]<<4);
			channel[1] = ((bigBuffer & 0x03ff) * 1.56402738) ;// map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer = (bigBuffer>>10) |  (LoRaPacket[4]<<2);
			channel[0] = ((bigBuffer & 0x03ff) * 1.56402738) ;//map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer = (bigBuffer>>10) |  (LoRaPacket[5]);
			//channel[13] = channel[2]; // Duplicate throttle for Betaflight modes.

			channel[3] += 192;
			channel[2] += 192;
			channel[1] += 192;
			channel[0] += 192;
//			127.5


			//channel[13] = pRadio->GetRssiInst()>>1;

			rssiValue = pRadio->GetRssiInst();

			rssiAgv = (rssiValue * rssiW) + ( rssiAgv * (1.0 - rssiW));
//
//			rssiCounter++;
//			rssiCounter %= 32;
//			if( rssiCounter == 0)
//			{
//				rssiTotal = 0;
//				for( int i = 0; i < 32; i++)
//					rssiTotal += rssiRunningValues[i];
//
//				rssiValue =  1792 - ((rssiTotal>>5) * 12.54901961) ;
//
//			}

			channel[14] =  1792 - (((uint16_t)rssiAgv>>1) * (7.0275098 ) );// 1000 + rssiAgv;
			TIM16->CCR1 = 6666.0 - (rssiAgv * rssiMultiplier);

			//			channel[14]  =1792-((pRadio->GetRssiInst()/2)* 12.54901961);//*100 6.2745098);
			channel[15] = channel[2]; // Duplicate throttle for Betaflight modes.



			channel[4] = (map( (bigBuffer & 0b1) ,0, 1, 192, 1792) & 0x07FF); // 2-Pos switch

			channel[5 + ((bigBuffer>>1) & 0b1111)] = (map( (bigBuffer>>5) ,0, 6, 192, 1792) & 0x07FF); // 2-Pos switch


			SBUS_Encode(channel, 16, packet);



			HAL_UART_Transmit_IT(&huart1, (uint8_t*)packet,25);

			debugCounter1++;

			if( debugCounter1 > 10000)
			{
				debugCounter1 = 0;
				debugCounter2 = 0;
			}
		}
		else if( uart_tx_ready == 1)
		{
			uint8_t packetJunk[0x80] = {0};
			pRadio->ReadBuffer(rxStartBufferPointer, (uint8_t*)packetJunk, rxPayloadLen);// > 5 ? 6 : rxPayloadLen);
			LED_GPIO_Port->BRR = LED_Pin;
			LED2_GPIO_Port->BRR = LED2_Pin;
			debugCounter2++;
//			SBUS_Encode(channel, 16, packet);
//			SBUS_Decode(packet, channelR, 25);
//			// Send old data.
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)packet,25);
//
//			HAL_Delay(3);
		}
		packetsSent++;

	}


}



