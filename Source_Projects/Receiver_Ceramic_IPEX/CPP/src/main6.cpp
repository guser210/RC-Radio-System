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
volatile uint8_t packet[25];
volatile uint8_t LoRaPacket[10];
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_tx_ready = 1;
}

volatile HAL_StatusTypeDef ret;
volatile uint8_t buffer[16] = {"This is LoRa"};
volatile uint32_t packetsSent = 0;
volatile uint8_t outpuPower = 0;
volatile uint16_t outputCounter = 0;
volatile uint16_t channel[20] = {0};

uint8_t rxPayloadLen;
uint8_t rxStartBufferPointer;

//volatile uint8_t radioChannel = 1;
void ResetLoRa(uint8_t radioChannel, uint8_t LoRaWord)
{
	pRadio->Init();


//// LoRa
	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);// 0 = 13MHz RC, 1 = 52MHz XTAL.
	pRadio->SetPacketType(LoRa::PACKET_TYPE_LORA); //  * 0 = GSKF  * 1 = LORA	  * 2 = RANGING	  * 3 = FLRC	  * 4 = BLE

	pRadio->SetChannel(radioChannel);
	pRadio->SetBufferBaseAddress(0x80,0);

//	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_800,LoRa::LORA_CR_4_5);
//	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);
	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_800,LoRa::LORA_CR_4_5);
	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);


//	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_1600,LoRa::LORA_CR_4_5);
//	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 25, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);

	pRadio->SetDioIrqParams(0b01000000,0b01100011
						   ,0b00000000,0b00000011
						   ,0b01000000,0b00000000
						   ,0b00000000,0b01100000);

	pRadio->SetLoRaWord(LoRaWord);
	TxDone = 0;
	RxDone = 0;

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;
	pRadio->ReadRegister(0x944,  (uint8_t*)buffer, 4);

	RX_EN_GPIO_Port->BSRR = RX_EN_Pin;

	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);

}
volatile uint8_t status = 0;
void setup()
{
	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,500);
	if(HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

}

volatile uint8_t radioConnected = 0;
volatile uint8_t testPacket = 0;
volatile uint8_t channelInfo = 0;
void maincpp()
{

	HAL_Delay(1000);
	setup();

	uint8_t radioChannel = 26;

	uint8_t LoRaWord = 210;

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



	while(1)
	{


		TxRxTimeout = 0;
		irqHeaderError = 0;
		irqCRCError = 0;
		pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);
		pRadio->ClearIrqStatus(0xff, 0xff);
		pRadio->SetRx(LoRa::ms_0,0,0); // always receving..

		RxDone = 0;
		LED_GPIO_Port->BRR = LED_Pin;
		while(RxDone == 0 )
		{
			if( resetLoRa >= 4)
				break;
		}


		TIM17->CNT = 0; // Reset timeout counter.

		if( RxDone == 1)
		{
			radioConnected = 1;
			resetLoRa = 0;
			LED_GPIO_Port->BSRR = LED_Pin;
		}

		if( resetLoRa >= 4)
		{

			LED2_GPIO_Port->BSRR = LED2_Pin;
			/*
			 * Lost of signal
			 * send disarm command.
			 * Reset radio just in case.
			 */

			radioConnected = 0;
			//			packet[23] |= (1); // digital channel 17
			//			packet[23] |= (2); // digital channel 18
			//			packet[23] |= (4); lost frame
			packet[23] |= (8); //failsafe
			TxRxTimeout = 0;
			resetLoRa = 3;

			//SBUS_Decode(packet, channel, 25);
			for( int i = 0; i < 16; i++)
				channel[16] = 172;
			channel[2] = 172; // set Throttle to 0;
			channel[4] = 1811; // disarm.

			SBUS_Encode(channel, 16, packet);

			//packet[23] |= (8); // <-------- failsafe

			for( int i = 0; i < 5; i++)
			{
				/*
				 * Disarm on lost of signal.
				 * TODO: add GPS before implementing failsafe.
				 */
				while(uart_tx_ready == 0)
				{
					HAL_Delay(1);
				}

				HAL_UART_Transmit_IT(&huart1, (uint8_t*)packet,25);
				uart_tx_ready = 0;

				HAL_Delay(3);
			}




			HAL_Delay(40);
			LED_GPIO_Port->BSRR = LED_Pin;
			HAL_Delay(10);
			LED2_GPIO_Port->BRR = LED2_Pin;
			LED_GPIO_Port->BRR = LED_Pin;
			TIM17->CNT = 0;
			ResetLoRa(radioChannel,LoRaWord);
			TIM17->CNT = 0;

		}
		TIM17->CNT = 0;

		uint8_t rxPayloadLen = 0;
		uint8_t rxStartBufferPointer = 0;

		// Read message parameters.
		pRadio->GetRxBufferStatus(rxPayloadLen, rxStartBufferPointer);


		// Read incoming message.




		if( radioConnected
				&& uart_tx_ready == 1
				&& irqCRCError == 0
				&& irqHeaderError == 0 && rxPayloadLen == 6)
		{
			memset((void*)LoRaPacket,0,25);
			pRadio->ReadBuffer(rxStartBufferPointer, (uint8_t*)LoRaPacket, rxPayloadLen );
			/*
			 * pass data to FC if error free.
			 * TODO: detech overlapping Synch word.
			 */




			volatile uint64_t bigBuffer = 0;
		    // 0 Roll.
		    // 1 Pitch.
		    // 2 Throttle
		    // 3 Yaw
			for( int i = 5; i >= 0; i--)
			{
				bigBuffer |= LoRaPacket[i];
				if( i > 0)
					bigBuffer <<= 8;
			}

	//		channelInfo = bigBuffer & 0xff;
	//		bigBuffer >>= 8;
			//111000011111111111000000000000000000001111111111
			//111000011111111111000000000000000000001111111111
			//111000101000000000011111111111111111111000000000
			//111001001000000000011111111011111111111000000000
			//11100011 1000000000011111111111111111111000000000
	//	    channel[4] = (map( divClosest(adcValues[2],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
	//	    channel[6] = (map( divClosest(adcValues[4],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
	//	    channel[7] = (map( divClosest(adcValues[5],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
	//	    channel[9] = (map( divClosest(adcValues[7],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
	//		channelR[4] = channelInfo & 0b1;
	//		channelR[5] = (channelInfo>>5) & 0b111;
			channel[3] = map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer >>= 10;
			channel[2] = map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer >>= 10;
			channel[1] = map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer >>= 10;
			channel[0] = map(int(bigBuffer & 0x03ff) ,0,1023, 192,1792) ;
			bigBuffer >>= 10;
			channel[13] = channel[2]; // Duplicate throttle for Betaflight modes.


//			channel[4] = (map( (bigBuffer & 0b1) ,0, 1, 192, 1792) & 0x07FF); // 2-Pos switch
////			channelInfo >>= 1;
//			if( ((bigBuffer>>1) & 0b1111) == 1)
//		{
//			channelInfo = bigBuffer & 0xff;
//			if( ((channelInfo>>1) & 0b1111) == 0)
//				channel[5 + ((channelInfo>>1) & 0b1111)] = (map( (channelInfo>>5) ,0, 7, 192, 1792) & 0x07FF); // 2-Pos switch
//		}
			//	    channel[6] = (map( divClosest(adcValues[4],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
			//	    channel[7] = (map( divClosest(adcValues[5],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
			//	    channel[9] = (map( divClosest(adcValues[7],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch


			//SBUS_Decode(packet, channel, 25);
//
//		    channel[4] = (map( (channelInfo & 0b00000001)>>0 ,0, 1, 192, 1792) & 0x07FF); // 2-Pos switch
//		    channel[5] = (map( (packet[7] & 0b00000110)>>1 ,0, 2, 172, 1811) & 0x07FF); // 3-Pos switch
//		    channel[6] = (map( (packet[7] & 0b00001000)>>3 ,0, 1, 172, 1811) & 0x07FF); // 2-Pos switch
//		    channel[7] = (map( (packet[7] & 0b00010000)>>4 ,0, 1, 172, 1811) & 0x07FF); // 2-Pos switch
//		    channel[8] = (map( (packet[7] & 0b01100000)>>5 ,0, 2, 172, 1811) & 0x07FF); // 2-Pos switch
//		    channel[9] = (map( (packet[7] & 0b10000000)>>7 ,0, 1, 172, 1811) & 0x07FF); // 2-Pos switch

//		    testPacket = packet[7];
//		    channel[13] = channel[2]; // Duplicate throttle for Betaflight modes.

			SBUS_Encode(channel, 16, packet);
//
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)packet,25);
			HAL_Delay(1);
			uart_tx_ready = 0;
			debugCounter1++;


		}
		else
		{
			uint8_t packetJunk[25] = {0};
			pRadio->ReadBuffer(rxStartBufferPointer, (uint8_t*)packetJunk, rxPayloadLen = 6 ? 6 : rxPayloadLen);


			// Send old data.
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)packet,25);

			HAL_Delay(3);
		}
		packetsSent++;
		RxDone = 0;
	}


}



