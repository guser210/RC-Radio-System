/*
 * main.cpp
 *
 *  Created on: May 27, 2023
 *      Author: gvargas
 */

#include "main.h"
#include "LoRa.h"

#include "string.h"

extern "C"{
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern SPI_HandleTypeDef hspi2;
}

volatile double batteryVoltage = 0;
volatile uint16_t adcValues[13] = {0};
volatile const double maxSVolts = 4.23;
volatile const double minSVolts = 3.3;
volatile const double voltageBase = 3.3/4095;
volatile double voltage122Ref = 0.0;
volatile double batteryS[3] = {maxSVolts, maxSVolts * 2.0, maxSVolts * 3.0};

volatile uint16_t joystickMin[2][2] = {0};
volatile uint16_t joystickMax[2][2] = {0};
volatile uint16_t joystickRaw[4][16] = {0};
volatile uint16_t joystick[4] = {0};
volatile uint8_t joystickReadCount = 0;
volatile uint16_t channel[22] = {0};

volatile uint8_t nextAuxChannelToTransmit = 0;
volatile uint8_t channelInfo = 0;


volatile uint8_t packet[25];
volatile int16_t trim[3] = {0};


int16_t memorySettings[32] = {0};
volatile uint16_t comSettingsNew[ 3 ] = {0};
uint16_t comSettings[ 3 ] = {0};
uint16_t comSettingsDefault[ 3 ] = {210,(uint16_t)LoRa::LORA_BW_400, 50};
int sizeOfSettings = sizeof(memorySettings) / 2;


LoRa *pRadio;
volatile uint8_t gimbalsFlipped = 0;

volatile uint32_t debugCounter1 = 0;
volatile uint32_t debugCounter2 = 0;
volatile uint32_t debugCounter3 = 0;
volatile uint32_t debugCounter4 = 0;
volatile uint32_t debugCounter5 = 0;
volatile uint8_t TxDone = 0;
volatile uint8_t RxDone = 0;
volatile uint8_t TxRxTimeout = 0;

volatile uint8_t p1;
volatile uint8_t p2;

volatile uint8_t radioReady = 0;

uint16_t divClosest( uint16_t a, uint16_t b);
uint16_t map(int x, int in_min, int in_max, int out_min, int out_max);

void SBUS_Encode(volatile uint16_t* channel, uint8_t size, volatile uint8_t *sbus);
void SBUS_Decode(volatile uint8_t *sbus, volatile uint16_t *channel, uint8_t size);
void writeMemory(int16_t* data, int size= 16, int location = 0);
void ReadMemory(int16_t* data, int size = 16, int location = 0);

uint16_t map(int x, int in_min, int in_max, int out_min, int out_max) {
	if (x < in_min)
		x = in_min;

	if (x > in_max)
		x = in_max;

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t divClosest( uint16_t a, uint16_t b)
{
	return (a + b/ 2) / b;
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static  double divValue = 11.0 * 1.22;
	// vrefint = 1.23 * 11 (11 = volt divider)

	voltage122Ref = divValue / adcValues[12];
	batteryVoltage = voltage122Ref * adcValues[10];

	joystickRaw[0][joystickReadCount] = adcValues[0]>>4; // Left - x.
	joystickRaw[1][joystickReadCount] = adcValues[1]>>4; // Left - y.

	joystickRaw[2][joystickReadCount] = adcValues[8]>>4; // right - x.
	joystickRaw[3][joystickReadCount] = adcValues[9]>>4; // rigth - y.


	joystickReadCount++;
	joystickReadCount %= 16;


	joystick[gimbalsFlipped == 0 ? 0 : 1] = (
			joystickRaw[0][0] + joystickRaw[0][1] + joystickRaw[0][2] + joystickRaw[0][3] +
			joystickRaw[0][4] + joystickRaw[0][5] + joystickRaw[0][6] + joystickRaw[0][7] +
			joystickRaw[0][8] + joystickRaw[0][9] + joystickRaw[0][10] + joystickRaw[0][11] +
			joystickRaw[0][12] + joystickRaw[0][13] + joystickRaw[0][14] + joystickRaw[0][15]);


	joystick[gimbalsFlipped == 0 ? 1 : 0] = (
			joystickRaw[1][0] + joystickRaw[1][1] + joystickRaw[1][2] + joystickRaw[1][3] +
			joystickRaw[1][4] + joystickRaw[1][5] + joystickRaw[1][6] + joystickRaw[1][7] +
			joystickRaw[1][8] + joystickRaw[1][9] + joystickRaw[1][10] + joystickRaw[1][11] +
			joystickRaw[1][12] + joystickRaw[1][13] + joystickRaw[1][14] + joystickRaw[1][15]);

	joystick[gimbalsFlipped == 0 ? 2 : 3] = (
			joystickRaw[2][0] + joystickRaw[2][1] + joystickRaw[2][2] + joystickRaw[2][3] +
			joystickRaw[2][4] + joystickRaw[2][5] + joystickRaw[2][6] + joystickRaw[2][7] +
			joystickRaw[2][8] + joystickRaw[2][9] + joystickRaw[2][10] + joystickRaw[2][11] +
			joystickRaw[2][12] + joystickRaw[2][13] + joystickRaw[2][14] + joystickRaw[2][15]);

	joystick[gimbalsFlipped == 0 ? 3 : 2] = (
			joystickRaw[3][0] + joystickRaw[3][1] + joystickRaw[3][2] + joystickRaw[3][3] +
			joystickRaw[3][4] + joystickRaw[3][5] + joystickRaw[3][6] + joystickRaw[3][7] +
			joystickRaw[3][8] + joystickRaw[3][9] + joystickRaw[3][10] + joystickRaw[3][11] +
			joystickRaw[3][12] + joystickRaw[3][13] + joystickRaw[3][14] + joystickRaw[3][15]);


//	// data prep
//    channel[0] = ((map(joystick[2],810, 3520, 172, 1811) + trim[0]) & 0x07FF); // Roll
//    channel[1] = ((map(joystick[3],580, 3330, 1811, 172) + trim[1]) & 0x07FF); // Pitch
//    channel[2] = (map(joystick[1] ,635, 3335, 1811, 172) & 0x07FF); // Throttle
//    channel[3] = ((map(joystick[0],600, 3286, 172, 1811) + trim[2]) & 0x07FF); // Yaw
//    channel[13] = channel[2]; // Duplicate throttle for Betaflight modes.
//
//
//
//    channel[4] = (map( divClosest(adcValues[2],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
//    channel[6] = (map( divClosest(adcValues[4],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
//    channel[7] = (map( divClosest(adcValues[5],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
//    channel[9] = (map( divClosest(adcValues[7],2000) ,1, 2, 0, 1) & 0x07FF); // 2-Pos switch
//
////    channel[4] = (map( divClosest(adcValues[2],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch
////    channel[6] = (map( divClosest(adcValues[4],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch
////    channel[7] = (map( divClosest(adcValues[5],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch
////    channel[9] = (map( divClosest(adcValues[7],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch
//
//    static uint16_t tempValue = 0;
//
//    tempValue = divClosest(adcValues[3],1000);
//    channel[5] = tempValue == 2 ? 0 : tempValue == 4 ? 1 : 2; // 3-Pos switch
////    channel[5] = tempValue == 4 ? 1000 : tempValue == 2 ? 172 : 1811; // 3-Pos switch
//
//    tempValue = divClosest(adcValues[6],1000);
//    channel[8] = tempValue == 2 ? 0 : tempValue == 4 ? 1 : 2; // 3-Pos switch
////    channel[8] = tempValue == 4 ? 1000 : tempValue == 2 ? 172 : 1811; // 3-Pos switch
//
//    // data prep end
//
	//
	//	joystickMin[0][0] = memorySettings[4]; // Yaw.
	//	joystickMin[0][1] = memorySettings[5]; // Throttle.
	//	joystickMin[1][0] = memorySettings[6]; // Roll.
	//	joystickMin[1][1] = memorySettings[7]; // Pitch.
	//
	//	joystickMax[0][0] = memorySettings[8]; // Yaw.
	//	joystickMax[0][1] = memorySettings[9]; // Throttle.
	//	joystickMax[1][0] = memorySettings[10]; // Roll.
	//	joystickMax[1][1] = memorySettings[11]; // Pitch.


		// Gimbal data prep
	    channel[3] = ((map(joystick[0],(joystickMin[0][0] + 100) + trim[2], (joystickMax[0][0] - 100) + trim[2], 0, 1023) ) & 0x07FF); // Yaw
	    channel[2] = (map(joystick[1] ,(joystickMin[0][1] + 100) , (joystickMax[0][1] - 70), 1023, 0) & 0x07FF); // Throttle
	    channel[0] = ((map(joystick[2],(joystickMin[1][0] + 100) + trim[0], (joystickMax[1][0] - 100) + trim[0], 0, 1023) ) & 0x07FF); // Roll
	    channel[1] = ((map(joystick[3],(joystickMin[1][1] + 100) + trim[1], (joystickMax[1][1] - 100) + trim[1], 1023, 0) ) & 0x07FF); // Pitch


	    // Plastic joysticks.
//	    channel[2] = ((map(joystick[0],(joystickMin[0][0] + 100) + trim[2], (joystickMax[0][0] - 100) + trim[2], 0, 1023) ) & 0x07FF); // Yaw
//	    channel[3] = (map(joystick[1] ,(joystickMin[0][1] + 100) , (joystickMax[0][1] - 70), 1023, 0) & 0x07FF); // Throttle
//	    channel[1] = ((map(joystick[2],(joystickMin[1][0] + 100) + trim[0], (joystickMax[1][0] - 100) + trim[0], 0, 1023) ) & 0x07FF); // Roll
//	    channel[0] = ((map(joystick[3],(joystickMin[1][1] + 100) + trim[1], (joystickMax[1][1] - 100) + trim[1], 1023, 0) ) & 0x07FF); // Pitch
//
	    // 0 Roll.
	    // 1 Pitch.
	    // 2 Throttle
	    // 3 Yaw
	    //    channel[3] = ((map(joystick[0],600, 3286, 172, 1811) + trim[2]) & 0x07FF); // Yaw
	//    channel[2] = (map(joystick[1] ,635, 3335, 1811, 172) & 0x07FF); // Throttle
	//    channel[0] = ((map(joystick[2],810, 3520, 172, 1811) + trim[0]) & 0x07FF); // Roll
	//    channel[1] = ((map(joystick[3],580, 3330, 1811, 172) + trim[1]) & 0x07FF); // Pitch

	    channel[13] = channel[2]; // Duplicate throttle for Betaflight modes.



	    channel[4] = (map( divClosest(adcValues[2],2000) ,1, 2, 0, 7) & 0x07FF); // 2-Pos switch
	    channel[6] = (map( divClosest(adcValues[4],2000) ,1, 2, 0, 7) & 0x07FF); // 2-Pos switch
	    channel[7] = (map( divClosest(adcValues[5],2000) ,1, 2, 0, 7) & 0x07FF); // 2-Pos switch
	    channel[9] = (map( divClosest(adcValues[7],2000) ,1, 2, 0, 7) & 0x07FF); // 2-Pos switch

	//    channel[4] = (map( divClosest(adcValues[2],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch
	//    channel[6] = (map( divClosest(adcValues[4],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch
	//    channel[7] = (map( divClosest(adcValues[5],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch
	//    channel[9] = (map( divClosest(adcValues[7],2000) ,1, 2, 172, 1811) & 0x07FF); // 2-Pos switch

	    static uint16_t tempValue = 0;

	    tempValue = divClosest(adcValues[3],1000);
	    channel[5] = tempValue == 2 ? 0 : tempValue == 4 ? 3 : 6; // 3-Pos switch
	//    channel[5] = tempValue == 4 ? 1000 : tempValue == 2 ? 172 : 1811; // 3-Pos switch

	    tempValue = divClosest(adcValues[6],1000);
	    channel[8] = tempValue == 2 ? 0 : tempValue == 4 ? 3 : 6; // 3-Pos switch
	//    channel[8] = tempValue == 4 ? 1000 : tempValue == 2 ? 172 : 1811; // 3-Pos switch

	    // data prep end

}
volatile int8_t pin[7] =  {0};
volatile uint8_t trimChanged = 0;

volatile uint16_t cnt = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( !radioReady)
		return;
	if( htim->Instance == TIM17)
	{// Turn OFF status LED (White)
		LED_1_GPIO_Port->BRR = LED_1_Pin;

	}

}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if( !radioReady)
		return;
	if( htim->Instance == TIM17)
	{// Turn ON status LED (White)
		LED_1_GPIO_Port->BSRR = LED_1_Pin;
	}
}
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if( !radioReady)
		return;

	/*
	 * LoRa IRQ code
	 */
	switch(GPIO_Pin)
	{
	case  LORA_DIO1_Pin:
		TxDone = 1;
		if( pRadio != NULL)
		{
			//LORA_TX_EN_GPIO_Port->BRR = LORA_TX_EN_Pin;
			pRadio->GetIrqStatus((uint8_t&)p1,(uint8_t&)p2);
			pRadio->ClearIrqStatus( (uint8_t&)p1,(uint8_t&)p2);
//			pRadio->ClearIrqStatus( 0b01000000, 0b1);

		}
		return;
		break;
	case LORA_DIO2_Pin:
		RxDone = 1;
		if( pRadio != NULL)
//			LORA_TX_EN_GPIO_Port->BRR = LORA_TX_EN_Pin;
			pRadio->ClearIrqStatus(0b01000000, 0b10);
		return;
		break;
	case LORA_DIO3_Pin:
		TxRxTimeout = 1;

		if( pRadio != NULL)
		{
//			LORA_TX_EN_GPIO_Port->BRR = LORA_TX_EN_Pin;
//			pRadio->SetTx(LoRa::ms_1, 0, 0);
			pRadio->ClearIrqStatus(0b01000000, 0b0);
		}
		return;
		break;
	}



	/*
	 * Trim button IRQ code.
	 * adds/subtracts value set on Falling_Callback.
	 * there needs to be 50 or more ms between press and release for
	 * the action to be recognized.
	 * TODO: change 	TIM17 to TIM16.
	 */
	if( TIM17->CNT < 20) return;


	switch(GPIO_Pin)
	{
	case TRIM_1_Pin: // roll -
		trim[0] += pin[1];
		pin[1] = 0;
		trimChanged = 1;
		break;
	case TRIM_2_Pin: // roll +
		trim[0] += pin[2];
		pin[2] = 0;
		trimChanged = 1;
		break;
	case TRIM_3_Pin: // pitch +
		trim[1] += pin[3];
		pin[3] = 0;
		trimChanged = 1;
		break;
	case TRIM_4_Pin: // pitch -
		trim[1] += pin[4];
		pin[4] = 0;
		trimChanged = 1;
		break;
	case TRIM_5_Pin: // Yaw -
		cnt = TIM17->CNT;
		trim[2] += pin[5];
		pin[5] = 0;
		trimChanged = 1;
		break;
	case TRIM_6_Pin: // Yaw +
		trim[2] += pin[6];
		pin[6] = 0;
		trimChanged = 1;
		break;
	 default:
		for( uint8_t index = 0 ; index < 7; index++ )
			pin[index] = 0;
		break;
}



}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if( !radioReady)
		return;
	/*
	 * Trim IRQ inital press.
	 * resets counter for later use.
	 * TODO: change 	TIM17 to TIM16.
	 */
	TIM17->CNT = 0;
		switch(GPIO_Pin)
		{
		case TRIM_1_Pin: // roll -
			pin[1] = 1;
			break;
		case TRIM_2_Pin: // roll +
			pin[2] = -1;
			break;
		case TRIM_3_Pin: // pitch +
			pin[3] = 1;
			break;
		case TRIM_4_Pin: // pitch -
			pin[4] = -1;
			break;
		case TRIM_5_Pin: // Yaw -
			pin[5] = 1;
			break;
		case TRIM_6_Pin: // Yaw +
			pin[6] = -1;
			break;
		 default:
			for( uint8_t index = 0 ; index < 7; index++ )
				pin[index] = 0;
			break;
	}

}
void ReadMemory(int16_t* data, int size, int location){
	location *= 16;

	for( int index = 0; index < size; index++){

		data[index] =   (uint16_t)(*(uint64_t*)(0x0800F800UL + location)); // 64kB

		location += 16;
	}

}


void WriteMemory(int16_t* data, int size, int location){

	int16_t memsettings[16] ={0};
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
void setup()
{
	if(HAL_TIM_Base_Start(&htim14) != HAL_OK)
		Error_Handler(); // Timer for Trim buttons
	if(HAL_TIM_Base_Start(&htim16) != HAL_OK)
		Error_Handler(); // Timer for Trim buttons

	if(HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
		Error_Handler(); // TImer for LED status.

	if(HAL_TIM_PWM_Start_IT(&htim17, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	HAL_ADC_Stop(&hadc1);

	/*
	 * BUG Fix:HAL_ADCEx_Calibration_Start
	 * Link: https://community.st.com/s/question/0D53W00001aJM92SAG/stm32cubewlv120-haladcexcalibrationstart-does-not-work
	 *     /* Apply calibration factor
	* LL_ADC_Enable(hadc->Instance);
	* HAL_Delay(10);
	* LL_ADC_SetCalibrationFactor(hadc->Instance, calibration_factor_accumulated);
	* LL_ADC_Disable(hadc->Instance);
	 *
	 */
	if(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
		Error_Handler();


	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 13) != HAL_OK)
		Error_Handler();


}
volatile HAL_StatusTypeDef ret;
volatile uint8_t buffer[16] = {"This is LoRa"};
volatile uint8_t bufferMessage[16] = {"This is LoRa"};
volatile uint32_t packetsSent = 0;
volatile uint8_t outpuPower = 0;
volatile uint16_t outputCounter = 0;
volatile uint16_t decodedChannel[16] = {0};
volatile uint8_t readyToTransmit = 0;
volatile uint8_t radioChannel = 26;
volatile uint8_t testPacket = 0;
volatile uint8_t trigger1 = 0;

volatile uint8_t status1, status2, status3, status4, status5;

void LoRaSetup( uint8_t synchWord, uint8_t bw, uint8_t channel )
{
	pRadio->nresetPort = LORA_RESET_GPIO_Port;// NRESET_GPIO_Port;
	pRadio->nreset = LORA_RESET_Pin;// NRESET_Pin;

	pRadio->busyPort = LORA_BUSY_GPIO_Port;// BUSY_GPIO_Port;
	pRadio->busy = LORA_BUSY_Pin;// BUSY_Pin;

	pRadio->nssPort = LORA_SPI_NSS_GPIO_Port;// NSS_GPIO_Port;
	pRadio->nss = LORA_SPI_NSS_Pin;// NSS_Pin;
	pRadio->port = &hspi2;
	pRadio->Init();


	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);
	pRadio->SetPacketType(LoRa::PACKET_TYPE_LORA);

	pRadio->SetAutoFS(); // reduce switching time Page85.
	radioChannel = channel;
	pRadio->SetChannel(channel);

	pRadio->SetBufferBaseAddress(0x80,0);
	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,(LoRa::_LoRa_BW)bw,LoRa::LORA_CR_4_5);
	pRadio->SetPacketParams(0x08, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_INVERTED, 0x00, 0x00);


	//LORA_BW_200 500mW 0.43Amp. 145pps
//	pRadio->SetTxParams(13, 0x40);//0x40); // needs brief, power, and ramp.

	// LORA_BW_400 420mW 0.49Amp 280pps.
	pRadio->SetTxParams(31, 0x40);//0x40); // needs brief, power, and ramp.
	// LORA_BW_800 420mW 0.66Amp 510pps. shuts down.
//	pRadio->SetTxParams(20, 0x40);//0x40); // needs brief, power, and ramp.

	pRadio->SetDioIrqParams(0b01000000,0b11
						 ,0b00000000,0b01
						 ,0b00000000,0b10
						 ,0b01000000,0b0);
	TxDone = 0;
	RxDone = 0;

//	uint8_t txEnabled = 0;

	uint8_t LoRaWord = synchWord;
	pRadio->SetLoRaWord(LoRaWord);

}
void LoRaReset()
{
	pRadio->nresetPort = LORA_RESET_GPIO_Port;// NRESET_GPIO_Port;
	pRadio->nreset = LORA_RESET_Pin;// NRESET_Pin;

	pRadio->busyPort = LORA_BUSY_GPIO_Port;// BUSY_GPIO_Port;
	pRadio->busy = LORA_BUSY_Pin;// BUSY_Pin;

	pRadio->nssPort = LORA_SPI_NSS_GPIO_Port;// NSS_GPIO_Port;
	pRadio->nss = LORA_SPI_NSS_Pin;// NSS_Pin;
	pRadio->port = &hspi2;
	pRadio->Init();


	pRadio->SetStandby(LoRa::STDBY_XOSC_52MHz);
	pRadio->SetPacketType(LoRa::PACKET_TYPE_LORA);

	pRadio->SetAutoFS(); // reduce switching time Page85.
	radioChannel = 50;
	pRadio->SetChannel(radioChannel);

	pRadio->SetBufferBaseAddress(0x80,0);
	pRadio->SetModulationParams( LoRa::LORA_SF_5 ,LoRa::LORA_BW_400,LoRa::LORA_CR_4_5);
	pRadio->SetPacketParams(0x08, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_INVERTED, 0x00, 0x00);

	//	radio.SetModulationParams( LoRa::LORA_SF_10 ,LoRa::LORA_BW_1600,LoRa::LORA_CR_4_5);
	//	radio.SetPacketParams(0x0C, LoRa::EXPLICIT_HEADER, 6, LoRa::LORA_CRC_ENABLE, LoRa::LORA_IQ_STD, 0x00, 0x00);

	//LORA_BW_200 500mW 0.43Amp. 145pps
//	pRadio->SetTxParams(13, 0x40);//0x40); // needs brief, power, and ramp.

	// LORA_BW_400 420mW 0.49Amp 280pps.
	pRadio->SetTxParams(31, 0x40);//0x40); // needs brief, power, and ramp.
	// LORA_BW_800 420mW 0.66Amp 510pps. shuts down.
//	pRadio->SetTxParams(20, 0x40);//0x40); // needs brief, power, and ramp.

	pRadio->SetDioIrqParams(0b01000000,0b11
						 ,0b00000000,0b01
						 ,0b00000000,0b10
						 ,0b01000000,0b0);
	TxDone = 0;
	RxDone = 0;

//	uint8_t txEnabled = 0;

	uint8_t LoRaWord = 210;
	pRadio->SetLoRaWord(LoRaWord);
}

volatile uint32_t packetsPerSecond = 0;
volatile uint32_t lastDebugCounter1 = 0;

void maincpp()
{
	gimbalsFlipped = 0;

	HAL_Delay(100);
	setup();
	HAL_Delay(100);
	HAL_Delay(100);
	LoRa radio;
	pRadio = &radio;

	uint32_t pins = TRIM_1_Pin | TRIM_2_Pin | TRIM_3_Pin | TRIM_4_Pin;
	uint32_t pinsLeft = TRIM_5_Pin | TRIM_6_Pin;

	uint32_t comSettingsCurrent = 0;
	ReadMemory(memorySettings, 15, 0);

	comSettings[0] = memorySettings[13]; // synch word.
	comSettings[1] = memorySettings[14]; // Bandwidth.
	comSettings[2] = memorySettings[15]; // channel.


	comSettingsCurrent = 2;
	if( (TRIM_1_GPIO_Port->IDR & pins) == 0)
	{

		joystickMin[0][0] = joystick[0]; // Yaw
		joystickMin[0][1] = joystick[1]; // Throttle
		joystickMin[1][0] = joystick[2]; // Roll
		joystickMin[1][1] = joystick[3]; // Pitch

		joystickMax[0][0] = joystick[0]; // Yaw
		joystickMax[0][0] = joystick[1]; // Throttle
		joystickMax[0][0] = joystick[2]; // Roll
		joystickMax[0][0] = joystick[3]; // Pitch




		while(1)
		{

			if(gimbalsFlipped != (channel[9] > 0 ? 0 : 1))
			{
				gimbalsFlipped = channel[9] > 0 ? 0: 1;

				joystickMin[0][0] = joystick[0]; // Yaw
				joystickMin[0][1] = joystick[1]; // Throttle
				joystickMin[1][0] = joystick[2]; // Roll
				joystickMin[1][1] = joystick[3]; // Pitch

				joystickMax[0][0] = joystick[0]; // Yaw
				joystickMax[0][0] = joystick[1]; // Throttle
				joystickMax[0][0] = joystick[2]; // Roll
				joystickMax[0][0] = joystick[3]; // Pitch

			}

			if( gimbalsFlipped)
				LED_4_GPIO_Port->ODR ^= LED_4_Pin;
			else
				LED_4_GPIO_Port->ODR &= ~LED_4_Pin;
			HAL_Delay(30);
			LED_1_GPIO_Port->ODR ^= LED_1_Pin;

//		    channel[0] = ((map(joystick[2],810, 3520, 172, 1811) + trim[0]) & 0x07FF); // Roll
//		    channel[1] = ((map(joystick[3],580, 3330, 1811, 172) + trim[1]) & 0x07FF); // Pitch
//		    channel[2] = (map(joystick[1] ,635, 3335, 1811, 172) & 0x07FF); // Throttle
//		    channel[3] = ((map(joystick[0],600, 3286, 172, 1811) + trim[2]) & 0x07FF); // Yaw

			if( joystickMin[0][0] > joystick[0])
				joystickMin[0][0] = joystick[0]; // Yaw.
			if( joystickMin[0][1] > joystick[1])
				joystickMin[0][1] = joystick[1]; // Throttle.
			if( joystickMin[1][0] > joystick[2])
				joystickMin[1][0] = joystick[2]; // Roll.
			if( joystickMin[1][1] > joystick[3])
				joystickMin[1][1] = joystick[3]; // Pitch.

			if( joystickMax[0][0] < joystick[0])
				joystickMax[0][0] = joystick[0]; // Yaw.
			if( joystickMax[0][1] < joystick[1])
				joystickMax[0][1] = joystick[1]; // Throttle.
			if( joystickMax[1][0] < joystick[2])
				joystickMax[1][0] = joystick[2]; // Roll.
			if( joystickMax[1][1] < joystick[3])
				joystickMax[1][1] = joystick[3]; // Pitch.


			if((TRIM_5_GPIO_Port->IDR & TRIM_5_Pin) == 0 &&
					(TRIM_6_GPIO_Port->IDR & TRIM_6_Pin) == 0)
			{
				//save
				ReadMemory(memorySettings, 15, 0);
				memorySettings[4] = joystickMin[0][0]; // Yaw.
				memorySettings[5] = joystickMin[0][1]; // Throttle.
				memorySettings[6] = joystickMin[1][0]; // Roll.
				memorySettings[7] = joystickMin[1][1]; // Pitch.

				memorySettings[8] = joystickMax[0][0]; // Raw.
				memorySettings[9] = joystickMax[0][1]; // Throttle.
				memorySettings[10] = joystickMax[1][0]; // Roll.
				memorySettings[11] = joystickMax[1][1]; // Pitch.

				memorySettings[12] = gimbalsFlipped;

				WriteMemory(memorySettings, 16, 0);

				for( int i = 0; i < 10; i++ )
				{
					LED_4_GPIO_Port->ODR ^= LED_4_Pin;
					HAL_Delay(100);
				}
				LED_4_GPIO_Port->BRR = LED_4_Pin;
				break;
			}

		}
	}else if( 1 == 11 ||  (TRIM_1_GPIO_Port->IDR & pinsLeft) == 0)
	{

		while(1)
		{


			if(comSettingsCurrent != (channel[9] > 0 ? 0 : 1))
			{

				comSettingsCurrent = channel[9] > 0 ? 0: 1;
				if( comSettingsCurrent == 0)
				{

					// Random SynchWord from voltage value.
					comSettingsNew[0] = (uint16_t)((uint32_t)(batteryVoltage * 10000.0) & 0xff);
				}
				LoRaSetup((uint8_t)comSettingsDefault[0], (uint8_t)comSettingsDefault[1], (uint8_t)comSettingsDefault[2]);
				//LoRaReset();
				radioReady = 1;
				TxDone = 1;
				TIM17->CCR1 = 470;
				TIM17->ARR = 500;
				TIM17->CNT = 0;

				//Update chanel Settings
			}

			if( ( TxRxTimeout == 1 || TxDone == 1) )
			{
				if( comSettingsCurrent == 0)
				{

				}
				packet[0] = (uint8_t)comSettings[0];
				packet[1] = (uint8_t)comSettings[1];
				packet[2] = (uint8_t)comSettings[2];
				radio.WriteBuffer(0x80, (uint8_t*)packet, 6);//25);

				TxDone = 0;
				/*
				 * send Buffer with no timeout.
				 */
				pRadio->ClearIrqStatus(0xff, 0xff);
				radio.SetTx(LoRa::ms_0,0,0);

				//channel[2] // throttle.
			}

		}
	}


	for( int i = 0; i < 16; i++)
		channel[i] = 172;

	LoRaReset();

	uint8_t txEnabled = 0;



	ReadMemory(memorySettings, 13, 0);
	trim[0] = memorySettings[0];
	trim[1] = memorySettings[1];
	trim[2] = memorySettings[2];


	joystickMin[0][0] = memorySettings[4]; // Yaw.
	joystickMin[0][1] = memorySettings[5]; // Throttle.
	joystickMin[1][0] = memorySettings[6]; // Roll.
	joystickMin[1][1] = memorySettings[7]; // Pitch.

	joystickMax[0][0] = memorySettings[8]; // Yaw.
	joystickMax[0][1] = memorySettings[9]; // Throttle.
	joystickMax[1][0] = memorySettings[10]; // Roll.
	joystickMax[1][1] = memorySettings[11]; // Pitch.
	gimbalsFlipped = memorySettings[12];

	radioReady = 1;

	TxDone = 1; // init var
	TIM14->CNT = 0;
	volatile uint16_t bigBuffer = 0;


//	// debug
//	readyToTransmit = 1;
//	TxDone = 1;
//	TIM17->CCR1 = 470;
//	TIM17->ARR = 500;
//	TIM17->CNT = 0;

	// debug end

	while(1)
	{
	    // 0 Roll.
	    // 1 Pitch.
	    // 2 Throttle
	    // 3 Yaw

		readyToTransmit = 1;
//		}
		if( ( TxRxTimeout == 1 || TxDone == 1) && readyToTransmit == 1)
		{
			//			if( channel[7] == 0)
							//LORA_TX_EN_GPIO_Port->BRR = LORA_TX_EN_Pin;

			bigBuffer = channel[3] & 0x3ff; // 10 bits
			packet[0] = bigBuffer & 0xff; // -8bits = 2
			bigBuffer = (bigBuffer>>8) | (channel[2]<<2); // +10 = 12
			packet[1] = bigBuffer & 0xff; // -8 = 4
			bigBuffer = (bigBuffer>>8) | (channel[1]<<4); // +10 = 14
			packet[2] = bigBuffer & 0xff; // -8 = 6
			bigBuffer = (bigBuffer>>8) | (channel[0]<<6); // +10 = 16
			packet[3] = bigBuffer & 0xff; // -8 = 8
			packet[4] = (bigBuffer>>8) & 0xff; // -8 = 8


			packet[5] =  ((channel[5 + nextAuxChannelToTransmit]) & 0b111);
			packet[5] <<= 4;
			packet[5] |= nextAuxChannelToTransmit & 0b1111;
			packet[5] <<= 1;
			packet[5] |= ((channel[4] & 0b1) & 0b1); // Aux1 Arm. 2 pos


			nextAuxChannelToTransmit++;
			nextAuxChannelToTransmit %= 5;


//			TIM16->CNT = 0;
//			while(TIM16->CNT < 500)
//			{}
//

			TxRxTimeout = 0;
			debugCounter1++;

			if( TIM14->CNT >= 1000)
			{
				packetsPerSecond = ( debugCounter1 -  lastDebugCounter1);
				lastDebugCounter1 = debugCounter1;
				TIM14->CNT = 0;
			}

//			SBUS_Encode(channel, 4, packet);
//			SBUS_Decode(packet, decodedChannel, 16);
			if( txEnabled == 0)
			{
			//	LORA_TX_EN_GPIO_Port->BSRR = LORA_TX_EN_Pin;
				txEnabled = 1;
			}

			if( channel[7] == 0)
				LORA_TX_EN_GPIO_Port->BSRR = LORA_TX_EN_Pin;

			/*
			 * pre-load buffer
			 */
			radio.WriteBuffer(0x80, (uint8_t*)packet, 6);//25);

			TxDone = 0;
			/*
			 * send Buffer with no timeout.
			 */
//			pRadio->GetIrqStatus((uint8_t&)p1,(uint8_t&)p2);
			pRadio->ClearIrqStatus(0xff, 0xff);
			radio.SetTx(LoRa::ms_0,0,0);


		}

		if( channel[7] == 0)
			LORA_TX_EN_GPIO_Port->BSRR = LORA_TX_EN_Pin;
		else
			LORA_TX_EN_GPIO_Port->BRR = LORA_TX_EN_Pin;



		//pRadio->GetIrqStatus((uint8_t&)p1,(uint8_t&)p2);
		//volatile uint8_t status = pRadio->GetStatus();
		/*
		 * Logic for enabling Radio.
		 */
		if( readyToTransmit != 1)
		{

			if( channel[4] == 0 )
			{
				if( readyToTransmit == 0)
				{
					if( channel[2] > 900 && channel[0] > 900)
					{
						LED_1_GPIO_Port->BRR = LED_1_Pin;

						for( int i = 0; i < 10; i++)
						{
							HAL_Delay(50);
							LED_1_GPIO_Port->ODR ^= LED_1_Pin;
							readyToTransmit = 2;
							if(channel[2] < 900 || channel[0] < 900 || channel[4] != 0)
							{
								readyToTransmit = 0;
								break;
							}
						}
						LED_1_GPIO_Port->BRR = LED_1_Pin;
					}
				}
				else if( readyToTransmit == 2)
				{
					if( channel[2] == 0 && channel[0] > 900)
					{
						LED_1_GPIO_Port->BRR = LED_1_Pin;

						for( int i = 0; i < 10; i++)
						{
							HAL_Delay(50);
							LED_1_GPIO_Port->ODR ^= LED_1_Pin;
							readyToTransmit = 1;
							if(channel[2] > 0 || channel[0] < 900  || channel[4] != 0)
							{
								readyToTransmit = 0;
								break;
							}
						}
						LED_1_GPIO_Port->BRR = LED_1_Pin;
						if( readyToTransmit == 1)
						{
							TxDone = 1;
							TIM17->CCR1 = 470;
							TIM17->ARR = 500;
							TIM17->CNT = 0;
						}

					}
				}

			}
		}



		if( trimChanged == 1)
		{
			/*
			 * save trim to memory if button pressed detected.
			 */
			trimChanged = 0;
			memorySettings[0] = trim[0];
			memorySettings[1] = trim[1];
			memorySettings[2] = trim[2];

			WriteMemory(memorySettings, 3, 0);
		}



		/*
		 * Battery status logic.
		 */
		if( batteryVoltage >= ( minSVolts * 3.0) && batteryVoltage <= (maxSVolts * 3.0))
		{

			if( batteryVoltage > ((maxSVolts - 0.2) * 3.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;
			}
			else if( batteryVoltage > ((maxSVolts - 0.4) * 3.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin | LED_3_Pin ;
				LED_2_GPIO_Port->BRR=  LED_4_Pin;
			}
			else if( batteryVoltage > ((maxSVolts - 0.6) * 3.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin ;
				LED_2_GPIO_Port->BRR=  LED_3_Pin | LED_4_Pin;
			}
			else
			{
				LED_2_GPIO_Port->BRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;
			}
		}
		else if( batteryVoltage >= ( minSVolts * 2.0) && batteryVoltage <= (maxSVolts * 2.0))
		{
			if( batteryVoltage > ((maxSVolts - 0.2) * 2.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;
			}
			else if( batteryVoltage > ((maxSVolts - 0.4) * 2.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin | LED_3_Pin ;
				LED_2_GPIO_Port->BRR=  LED_4_Pin;
			}
			else if( batteryVoltage > ((maxSVolts - 0.6) * 2.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin ;
				LED_2_GPIO_Port->BRR=  LED_3_Pin | LED_4_Pin;
			}
			else
			{
				LED_2_GPIO_Port->BRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;
			}

		}
		else if( batteryVoltage >= ( minSVolts * 1.0) && batteryVoltage <= (maxSVolts * 1.0))
		{
			if( batteryVoltage > ((maxSVolts - 0.2) * 1.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;
			}
			else if( batteryVoltage > ((maxSVolts - 0.4) * 1.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin | LED_3_Pin ;
				LED_2_GPIO_Port->BRR=  LED_4_Pin;
			}
			else if( batteryVoltage > ((maxSVolts - 0.6) * 1.0)  )
			{
				LED_2_GPIO_Port->BSRR= LED_2_Pin ;
				LED_2_GPIO_Port->BRR=  LED_3_Pin | LED_4_Pin;
			}
			else
			{
				LED_2_GPIO_Port->BRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;
			}

		}
		else
		{
			debugCounter2++;
			LED_2_GPIO_Port->BSRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;
//			HAL_Delay(50);
			LED_2_GPIO_Port->BRR= LED_2_Pin | LED_3_Pin | LED_4_Pin;

		}

	}


}





