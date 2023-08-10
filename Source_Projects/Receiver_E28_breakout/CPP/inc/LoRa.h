/*
 * LoRa.h
 *
 *  Created on: Apr 3, 2023
 *      Author: gvargas
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"

class LoRa {
public:
	SPI_HandleTypeDef* port;
	GPIO_TypeDef *nresetPort;
	GPIO_TypeDef *nssPort;
	GPIO_TypeDef *busyPort;

	GPIO_TypeDef *txEnPort;
	GPIO_TypeDef *rxEnPort;
	GPIO_TypeDef *dio1Port;
	GPIO_TypeDef *dio2Port;
	GPIO_TypeDef *dio3Port;


	uint16_t nreset;
	uint16_t nss;
	uint16_t busy;
	uint16_t tx_en;
	uint16_t rx_en;
	uint16_t dio1;
	uint16_t dio2;
	uint16_t dio3;

	const uint16_t TransmitTimeout = 500;
	const uint16_t ReceiveTimeout = 500;

	enum _StandByConfig{
		STDBY_RC_13MHz = 0,
		STDBY_XOSC_52MHz = 1,
	};

	enum _SetPacketType{
		PACKET_TYPE_GFSK = 0x00,
		PACKET_TYPE_LORA = 0x01,
		PACKET_TYPE_RANGING_LORA = 0x02,
		PACKET_TYPE_FLRC = 0x03,
		PACKET_TYPE_BLE = 0x04,

	};

	enum _PacketType{
		GSKF = 0,
		LORA = 1,
		RANGING = 2,
		FLRC = 3,
		BLE = 4

//		0 = GSKF  * 1 = LORA	  * 2 = RANGING	  * 3 = FLRC	  * 4 = BLE
	};

	_PacketType packetType;

	enum _LoRa_SpreadingFactor{
		LORA_SF_5 = 0x50,
		LORA_SF_6 = 0x60,
		LORA_SF_7 = 0x70,
		LORA_SF_8 = 0x80,
		LORA_SF_9 = 0x90,
		LORA_SF_10 = 0xa0,
		LORA_SF_11 = 0xb0,
		LORA_SF_12 = 0xc0,
	};

	enum _LoRa_BW{
		LORA_BW_1600 = 0x0a,
		LORA_BW_800 = 0x18,
		LORA_BW_400 = 0x26,
		LORA_BW_200 = 0x34,
	};

	enum _LoRa_CR{
		LORA_CR_4_5 = 0x01,
		LORA_CR_4_6 = 0x02,
		LORA_CR_4_7 = 0x03,
		LORA_CR_4_8 = 0x04,
		LORA_CR_LI_4_5 = 0x05,
		LORA_CR_LI_4_6 = 0x06,
		LORA_CR_LI_4_8 = 0x07,
	};

	enum PacketParams2{
		EXPLICIT_HEADER = 0x00,
		IMPLICIT_HEADER = 0x80,
	};
	enum PacketParams4{
		LORA_CRC_ENABLE = 0x20,
		LORA_CRC_DISABLE = 0x00,
	};
	enum PacketParams5{
		LORA_IQ_INVERTED = 0x00,
		LORA_IQ_STD = 0x40,
	};

	enum _PeriodBase{
		ms_0 = 0x0,
		ms_1 = 0x2,
		ms_4 = 0x3,
	};
	LoRa();
	~LoRa();

	void Init();
	void WaitForBusy();
	uint32_t IsBusy();
	void Reset();


	void WriteCommand(uint8_t cmd);
	void WriteCommand(uint8_t cmd,uint8_t *data, uint8_t size);

	void nssHigh(){ nssPort->BSRR = nss;};
	void nssLow(){  nssPort->BRR = nss;};

	uint8_t GetStatus();
	void WriteRegister(uint16_t reg, uint8_t *data,uint8_t size);
	void ReadRegister(uint16_t reg, uint8_t *status, uint8_t size);

	void WriteBuffer(uint8_t offset, uint8_t *buffer,uint8_t size);
	void ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);

	void SetSleep(uint8_t sleepConfig);
	void SetStandby(_StandByConfig standbyConfig);

	void SetFs();

	void SetTx(_PeriodBase periodBase, uint8_t periodBaseCount1, uint8_t periodBaseCount0);
	void SetRx(_PeriodBase periodBase, uint8_t periodBaseCount1, uint8_t periodBaseCount0);

	void SetRxDutyCycle(uint8_t periodBase, uint8_t rxPeriodBaseCount1, uint8_t rxPeriodBaseCount0,
											uint8_t sleepPeriodBaseCount1, uint8_t sleepPeriodBaseCount0);

	void SetCad();

	void SetTxContinuousWave();
	void SetTxContinuousPreamble();

	void SetPacketType(_SetPacketType packetType);
	uint8_t GetPacketType();

	void SetLoRaWord(uint8_t LoRaWord);
	void SetChannel(uint16_t channel);
	void SetRfFrequency(uint8_t rfFrequency2, uint8_t rfFrequency1, uint8_t rfFrequency0);


	void SetTxParams(uint8_t power, uint8_t rampTime);

	void SetCadParams(uint8_t cadSymbolNum);

	void SetBufferBaseAddress(uint8_t txBaseAddress=0x80,uint8_t rxBaseAddress=0x00);

	void SetModulationParams(_LoRa_SpreadingFactor param1, _LoRa_BW param2, _LoRa_CR param3);

	void SetPacketParams(uint8_t param1, PacketParams2 param2, uint8_t param3, PacketParams4 param4, PacketParams5 param5, uint8_t param6, uint8_t param7);

	void GetRxBufferStatus(uint8_t &rxPayloadLen, uint8_t &rxStartBufferPointer);

	void GetPacketStatus(uint8_t &status1, uint8_t &status2, uint8_t &status3, uint8_t &status4, uint8_t &status5);

	uint8_t GetRssiInst(); // returns rssIlnst.

	void SetDioIrqParams(uint8_t irqMask1, uint8_t irqMask0, uint8_t dio1Mask1, uint8_t dio1Mask0, uint8_t dio2Mask1, uint8_t dio2Mask0, uint8_t dio3Mask1, uint8_t dio3Mask0);

	void GetIrqStatus(uint8_t &irqStatus1, uint8_t &irqStatus0);

	void ClearIrqStatus( uint8_t irqMask1, uint8_t irqMask0);

	void SetRegulatorMode(uint8_t regModeParam);

	void SetSaveContext();

	void SetAutoFS(uint8_t enable=1);

	void SetAutoTx(uint8_t time1, uint8_t time0);

	void SetLongPreamble(uint8_t enable=1);

	void SetRangingRole(uint8_t master=1);

	void SetAdvancedRanging(uint8_t enable=1);

	void irq();
private:
};


#endif /* INC_LORA_H_ */
