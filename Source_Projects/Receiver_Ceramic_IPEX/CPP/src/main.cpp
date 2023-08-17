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

int16_t memorySettings[4] = {0};
const int sizeOfSettings = sizeof(memorySettings) / 2;
volatile uint8_t pairing = 0;
uint16_t comSettingsDefault[ 3 ] = {210,(uint16_t)LoRa::LORA_BW_1600, 0};
volatile uint8_t TxRxTimeout = 0;
volatile uint8_t irqStaus15_8;
volatile uint8_t irqStatus7_0;

volatile uint8_t irqHeaderValid = 0;
volatile uint8_t irqHeaderError = 0;
volatile uint8_t irqCRCError = 0;

volatile uint32_t debugCounter1 = 0;
volatile uint32_t debugCounter2 = 0;
//volatile uint32_t debugCounter3 = 0;
//volatile uint32_t debugCounter4 = 0;
//volatile uint32_t debugCounter5 = 0;
//volatile uint32_t debugCounter6 = 0;
volatile uint8_t LoRaPacket[10];

//volatile HAL_StatusTypeDef ret;
//volatile uint8_t buffer[16] = {"This is LoRa"};
volatile uint16_t channel[17] = {0};


volatile uint8_t rssiValue = 0;
volatile double rssiAvg = 0.0;
volatile double rssiW = 0.01;
volatile double rssiMultiplier = (6666/255);

volatile uint8_t uart_tx_ready = 1;
volatile uint8_t uart_rx_ready = 0;

volatile uint8_t packet[26];


LoRa *pRadio;

void ReadMemory(int16_t* data, int size, int location);
void WriteMemory(int16_t* data, int size, int location);

uint16_t map(int x, int in_min, int in_max, int out_min, int out_max);
void SBUS_Encode(volatile uint16_t* channel, uint8_t size, volatile uint8_t *sbus);

void ReadMemory(int16_t* data, int size, int location){
	location *= 16;

	for( int index = 0; index < size; index++){

		data[index] =   (uint16_t)(*(uint64_t*)(0x0800F800UL + location)); // 64kB

		location += 16;
	}

}


void WriteMemory(int16_t* data, int size, int location){

	int16_t memsettings[32] ={0};
	ReadMemory(memsettings, sizeOfSettings, 0);

	size = size > sizeOfSettings ? sizeOfSettings : size;
	location = location >= sizeOfSettings ? sizeOfSettings : location;

	for( int index = 0; index < size; index++){
		location = location >= sizeOfSettings ? sizeOfSettings : location;
		memsettings[location++] = data[index];
	}

	FLASH_EraseInitTypeDef epage;
	epage.TypeErase = FLASH_TYPEERASE_PAGES;
	epage.Page = 31; // 64kB 2k pages
	epage.NbPages = 1;


	uint32_t error = 0;
	HAL_StatusTypeDef ret = HAL_FLASH_Unlock();

	ret = HAL_FLASHEx_Erase(&epage, &error);
	for( int index = 0; index < sizeOfSettings; index++){
		ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0800F800UL + (index * 16), memorySettings[index]); // 64kB

	}
	HAL_FLASH_Lock();


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim17 )
	{
	}

}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(pairing)
	{
		TIM14->CNT = 0;
		TIM17->CNT = 0;
		return;
	}
	if( htim == &htim17 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{

		TIM17->CNT = 0;
		TIM14->CNT = 0;
		TIM14->CCR1 = 60000; // Lost of signal indicator timeout.

		LED2_GPIO_Port->BSRR = LED2_Pin;
		pRadio->ClearIrqStatus(0xff, 0xff);
		pRadio->SetRx(LoRa::ms_0,0,0); // always receving..


	}
	else if( htim == &htim14 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		LED_GPIO_Port->BRR = LED_Pin; // reset both LEDs after x timeout.
		LED2_GPIO_Port->BRR = LED2_Pin;
		TIM14->CNT = 0;

	}
}


void FailSafe()
{

}

volatile uint8_t sense = 0;
volatile uint8_t pairingRSSi = 0;
void RxDone_Pairing()
{

	pairingRSSi = pRadio->GetRssiInst();
	if( pairingRSSi > 60)
		return;
	if( LoRaPacket[3] == 170 &&
		LoRaPacket[4] == 170 &&
		LoRaPacket[5] == 170 )
	{
		memorySettings[0] = LoRaPacket[0]; // SynchWord.
		memorySettings[1] = LoRaPacket[1]; // Bandwidth.
		memorySettings[2] = LoRaPacket[2]; // channel.
		WriteMemory(memorySettings, 3, 0);
		pairing = 2;
	}
}
void RxDone()
{
	TIM14->CCR1 = 7000; // turn off both LEDs after 6ms.


	volatile uint16_t bigBuffer = 0;

	TIM17->CNT = 0;
	TIM14->CNT = 0;
	LED_GPIO_Port->BSRR = LED_Pin;
	LED2_GPIO_Port->BSRR = LED2_Pin;

	bigBuffer = LoRaPacket[1]<<8 |LoRaPacket[0]; // append 1st 16 Bits.


	channel[3] = ((bigBuffer & 0x03ff) * 1.56402738) ; // transfer 10 LSB.
	bigBuffer = (bigBuffer>>10) |  (LoRaPacket[2]<<6); // shift the 10 bits transfered in previous operation and append 8 MSB.
	channel[2] = ((bigBuffer & 0x03ff) * 1.56402738) ; // transfer 10 LSB.
	bigBuffer = (bigBuffer>>10) |  (LoRaPacket[3]<<4); // shift the 10 bits transfered in previous operation and append 8 MSB.
	channel[1] = ((bigBuffer & 0x03ff) * 1.56402738) ; // transfer 10 LSB.
	bigBuffer = (bigBuffer>>10) |  (LoRaPacket[4]<<2); // shift the 10 bits transfered in previous operation and append 8 MSB.
	channel[0] = ((bigBuffer & 0x03ff) * 1.56402738) ; // transfer 10 LSB.
	bigBuffer = (bigBuffer>>10) |  (LoRaPacket[5]); //    append 8 bits..
	//channel[13] = channel[2]; // Duplicate throttle for Betaflight modes.

	channel[3] += 192; // Add 192 base range Betaflight.
	channel[2] += 192; // Add 192 base range Betaflight.
	channel[1] += 192; // Add 192 base range Betaflight.
	channel[0] += 192; // Add 192 base range Betaflight.
//			127.5
	//channel[13] = pRadio->GetRssiInst()>>1;

	rssiValue = pRadio->GetRssiInst();

	rssiAvg = (rssiValue * rssiW) + ( rssiAvg * (1.0 - rssiW));

	channel[14] =  1792 - (((uint16_t)rssiAvg>>1) * (7.0275098 ) );// 1000 + rssiAgv;
	TIM16->CCR1 = 6666.0 - (rssiAvg * rssiMultiplier);

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
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case DIO1_Pin:
//		TxDone  = 1;
		if( pRadio != NULL)
		{
			pRadio->GetIrqStatus((uint8_t&)irqStaus15_8, (uint8_t&)irqStatus7_0);

			irqHeaderValid = (irqStatus7_0 & 0b00010000)>>4;
			irqHeaderError = (irqStatus7_0 & 0b00100000)>>5;
			irqCRCError = (irqStatus7_0 & 0b01000000)>>6;

			pRadio->ReadBuffer(0, (uint8_t*)LoRaPacket,6);// rxPayloadLen > 5 ? 6 : rxPayloadLen);

			pRadio->ClearIrqStatus(0b00000000, 0b11);
			if( ((irqStatus7_0 & 2)>>1) == 1 &&
					!irqHeaderValid &&
					!irqHeaderError &&
					!irqCRCError)
			{
				if( pairing == 1)
					RxDone_Pairing();
				else
					RxDone();
			}
			pRadio->ClearIrqStatus(0xff, 0xff);
			pRadio->SetRx(LoRa::ms_0,0,0); // always receving..

		}

		break;
	case DIO2_Pin:
		if( pRadio != NULL)
		{
			TxRxTimeout = 1;

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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_tx_ready = 1;
}

void LoRaSetup( uint8_t synchWord, uint8_t bw, uint8_t channel )
{


	pRadio->Init();

	pRadio->SetStandby(LoRa::STDBY_RC_13MHz);

	volatile uint8_t gain[1] = {0}; // High sensitivity gain by 3db page30.
	pRadio->ReadRegister(0x0891, (uint8_t*)gain, 1);
	gain[0] |= (0xc0); // setting bits 7:6 at address 0x891 to 0x3. (0b11000000).
	pRadio->WriteRegister(0X891, (uint8_t*)gain, 1);
	gain[0] = (0);
	pRadio->ReadRegister(0x0891, (uint8_t*)gain, 1);


	pRadio->SetAutoFS(); // reduce switching time Page85.
	pRadio->SetRegulatorMode(1); // use dcdc page143.

	pRadio->SetPacketType(LoRa::PACKET_TYPE_LORA);

	pRadio->SetChannel(channel);

	pRadio->SetBufferBaseAddress(0x80,0);
	pRadio->SetModulationParams( LoRa::LORA_SF_6 ,(LoRa::_LoRa_BW )bw,LoRa::LORA_CR_4_5);
	pRadio->SetPacketParams(0x08, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_INVERTED, 0x00, 0x00);


	pRadio->SetDioIrqParams(0b01000000,0b01100011
						   ,0b00000000,0b00000011
						   ,0b01000000,0b00000000
						   ,0b00000000,0b01100000);

	uint8_t LoRaWord = synchWord;
	pRadio->SetLoRaWord(LoRaWord);

//
	RX_EN_GPIO_Port->BSRR = RX_EN_Pin; // enable RX_EN_Pin, not sure what this does still.

	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);






}
void ResetLoRa(uint8_t radioChannel, uint8_t LoRaWord)
{
	pRadio->Init();

	pRadio->SetStandby(LoRa::STDBY_RC_13MHz);

	volatile uint8_t gain[1] = {0}; // High sensitivity gain by 3db page30.
	pRadio->ReadRegister(0x0891, (uint8_t*)gain, 1);
	gain[0] |= (0xc0); // setting bits 7:6 at address 0x891 to 0x3. (0b11000000).
	pRadio->WriteRegister(0X891, (uint8_t*)gain, 1);
	gain[0] = (0);
	pRadio->ReadRegister(0x0891, (uint8_t*)gain, 1);


	pRadio->SetAutoFS(); // reduce switching time Page85.
	pRadio->SetRegulatorMode(1); // use dcdc page143.

	pRadio->SetPacketType(LoRa::PACKET_TYPE_LORA);

	radioChannel = 50;
	pRadio->SetChannel(radioChannel);
	pRadio->SetBufferBaseAddress(0x80,0);

	pRadio->SetModulationParams( LoRa::LORA_SF_6 ,LoRa::LORA_BW_400,LoRa::LORA_CR_4_5);
	pRadio->SetPacketParams(8, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_INVERTED, 0, 0);
//	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);

//	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_1600,LoRa::LORA_CR_4_5);
//	pRadio->SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 25, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);

	pRadio->SetDioIrqParams(0b01000000,0b01100011
						   ,0b00000000,0b00000011
						   ,0b01000000,0b00000000
						   ,0b00000000,0b01100000);

	pRadio->SetLoRaWord(LoRaWord);
//
	RX_EN_GPIO_Port->BSRR = RX_EN_Pin; // enable RX_EN_Pin, not sure what this does still.

	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);

}
volatile uint8_t radioConnected = 0;
volatile uint8_t testPacket = 0;

volatile uint8_t status = 0;

void setup()
{
	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,1000);
	if(HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	TIM14->CCR1 = 6000;
	if(HAL_TIM_OC_Start_IT(&htim14, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
}
void blinkLEDs(uint16_t delay, uint16_t cycles)
{
	for( uint16_t index = 0; index < cycles; index++)
	{
		LED_GPIO_Port->ODR ^= LED_Pin;
		LED2_GPIO_Port->ODR ^= LED2_Pin;
		HAL_Delay(delay);

	}

}

void maincpp()
{
	HAL_Delay(500); // <-- Needs Delay for voltage to settle.
	setup();


	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	uint8_t radioChannel = 1;

	uint8_t LoRaWord = 210;
//	uint8_t rxPayloadLen = 0;
//	uint8_t rxStartBufferPointer = 0;

	LoRa radio;
	pRadio = &radio;
	pRadio->nresetPort = NRESET_GPIO_Port;
	pRadio->nreset = NRESET_Pin;

	pRadio->busyPort = BUSY_GPIO_Port;
	pRadio->busy = BUSY_Pin;

	pRadio->nssPort = NSS_GPIO_Port;
	pRadio->nss = NSS_Pin;
	pRadio->port = &hspi2;


	pairing = 1;
	LoRaSetup((uint8_t)comSettingsDefault[0], (uint8_t)comSettingsDefault[1], (uint8_t)comSettingsDefault[2]);
	TxRxTimeout = 0;
	irqHeaderError = 0;
	irqCRCError = 0;
	pRadio->ClearIrqStatus(0xff, 0xff);
	pRadio->SetRx(LoRa::ms_0,0,0); // always receving..
//
	uint16_t pairingTimeout = 0;

	while(pairing == 1 && (pairingTimeout++) < 100)
	{
		HAL_Delay(50);
		LED2_GPIO_Port->ODR ^= LED2_Pin;
		LED_GPIO_Port->ODR ^= LED_Pin;

	}

	ReadMemory(memorySettings, 3, 0);
	LoRaSetup((uint8_t)memorySettings[0], (uint8_t)memorySettings[1], (uint8_t)memorySettings[2]);
	if( pairing ==2)
	{ // got valid connection data.

		blinkLEDs(500, 4);
	}
	pairing = 0;
	LED2_GPIO_Port->BSRR = LED2_Pin;
	LED_GPIO_Port->BSRR = LED_Pin;

	//ResetLoRa(radioChannel, LoRaWord);


	//memset((void*)buffer,0,16);

	for( int i = 0; i < 16; i++)
		channel[i] = 172;


	TxRxTimeout = 0;
	irqHeaderError = 0;
	irqCRCError = 0;
	pRadio->ClearIrqStatus(0xff, 0xff);
	pRadio->SetRx(LoRa::ms_0,0,0); // always receving..
//	RxDone = 0;

	while(1)
	{

	}

}



