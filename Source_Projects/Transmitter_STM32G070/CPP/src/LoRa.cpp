/*
 * LoRa.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: gvargas
 */

#include <stdio.h>
//#include <stdlib.h>
#include "main.h"
#include "LoRa.h"
#include "string.h"


LoRa::LoRa()
{

}
LoRa::~LoRa()
{

}

void LoRa::Init()
{
	nresetPort->BSRR = nreset;
	nssHigh();
	Reset();
	WaitForBusy();
}
void LoRa::WaitForBusy()
{
	while(IsBusy())
			;
}

uint32_t LoRa::IsBusy()
{
	return busyPort->IDR & busy;
}

void LoRa::Reset()
{
	nresetPort->BRR = nreset;
	HAL_Delay(100);
	nresetPort->BSRR = nreset;

}

void LoRa::ReadRegister(uint16_t regValue, uint8_t *status, uint8_t size)
{
	/*SX1281 Page 75, 4 byte command.
	 * ReadRegister: 0x19
	 * reg byte1 ,reg byte2, status = 0x0
	 *
	 */

	uint8_t reg[4] = {0};
	reg[0] = 0x19; 				// Read command
	reg[1] = regValue>>8 & 0xff; 		// H byte
	reg[2] = regValue & 0xff;	// L byte
	reg[3] = 0; 				// NOP to start receving data.

	WaitForBusy();
	HAL_StatusTypeDef ret;
	nssLow();

	ret = HAL_SPI_Transmit(port, reg, 4, TransmitTimeout);
	ret = HAL_SPI_Receive( port,status, size, ReceiveTimeout);


	nssHigh();
}
void LoRa::WriteRegister(uint16_t regValue, uint8_t *data,uint8_t size)
{
	/*SX1281 Page 74, 1 byte command.
	 * ReadRegister: 0x18
	 * reg byte1 ,reg byte2, status = 0x0
	 *
	 */
	uint8_t reg[3 + size];
	reg[0] = 0x18; 				// Write Command
	reg[1] = regValue>>8; 		// H Byte
	reg[2] = regValue & 0xff;	// L Byte.

	memcpy(reg + 3, data, size);

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, reg ,size + 3, TransmitTimeout);
	nssHigh();
}

void LoRa::WriteBuffer(uint8_t offset, uint8_t* data, uint8_t size)
{
	/*SX1281 Page 75, 1 byte command.
	 * ReadRegister: 0x1A
	 * offset: 0x20
	 *
	 *
	 */
	uint8_t reg[2];
	reg[0] = 0x1A; 		// Write buffer Command
	reg[1] = offset; 		// offset

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, reg, 2, TransmitTimeout);
	HAL_SPI_Transmit(port, data,size, TransmitTimeout);
	nssHigh();

}
void LoRa::ReadBuffer(uint8_t offset, uint8_t* data, uint8_t size)
{
	/*SX1281 Page 76, 1 byte command.
	 * ReadRegister: 0x1B
	 * offset: 0x20
	 *
	 *
	 */
	uint8_t reg[3];
	reg[0] = 0x1B; 		// Read buffer Command
	reg[1] = offset; 		// offset
	reg[2] = 0; // NOP

	uint8_t buffer[50];

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, reg, 3, TransmitTimeout);
	HAL_SPI_Receive(port, data,size, ReceiveTimeout);

	nssHigh();

}
void LoRa::WriteCommand(uint8_t cmd)
{
	uint8_t command[1] = {cmd};

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command,1,TransmitTimeout); // Page 144.
	nssHigh();
}
void LoRa::WriteCommand(uint8_t cmd,uint8_t *data, uint8_t size)
{
	uint8_t command[1 + size] = {cmd};
	memcpy(command + 1, data, size);

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command,size + 1,TransmitTimeout); // Page 144.
	nssHigh();


}
void LoRa::SetSleep(uint8_t sleepConfig)
{
	WriteCommand(0xd5);
}

/*
 * @brief Page 78, Set Standby MHz clock.
 * 0 = 13MHz
 * 1 = 52MHz
 */
//void LoRa::SetStandby(uint8_t standbyConfig)
void LoRa::SetStandby(_StandByConfig standbyConfig)
{
	uint8_t data[1] = {(uint8_t)standbyConfig};
	WriteCommand(0x80,data, 1);
}

void  LoRa::SetFs()
{
	WriteCommand(0xc1);
}

void LoRa::SetTx(_PeriodBase periodBase, uint8_t periodBaseCount1, uint8_t periodBaseCount0)
{ // Page 79
	uint8_t command = 0x83;
	uint8_t data[3] = {(uint8_t)periodBase, periodBaseCount1, periodBaseCount0};
	WriteCommand(command, data,sizeof(data));
}

void LoRa::SetRx(_PeriodBase periodBase, uint8_t periodBaseCount1, uint8_t periodBaseCount0)
{ // Page 80
	uint8_t command = 0x82;
	uint8_t data[3] = {(uint8_t)periodBase, periodBaseCount1, periodBaseCount0};
	WriteCommand(command, data,sizeof(data));

}

void LoRa::SetRxDutyCycle(uint8_t periodBase, uint8_t rxPeriodBaseCount1, uint8_t rxPeriodBaseCount0,
		uint8_t sleepPeriodBaseCount1, uint8_t sleepPeriodBaseCount0)
{ // Page 81
	uint8_t command = 0x94;
	uint8_t data[5] = {periodBase, rxPeriodBaseCount1,rxPeriodBaseCount0, sleepPeriodBaseCount1, sleepPeriodBaseCount0};
	WriteCommand(command, data,sizeof(data));

}

void LoRa::SetCad()
{ // Page 83
	WriteCommand(0xc5);
}

void LoRa::SetTxContinuousWave()
{ // Page 83
	WriteCommand(0xd1);
}

void LoRa::SetTxContinuousPreamble()
{ // Page 84
	WriteCommand(0xd2);
}

/**
  * @brief  Page 86 Set LORA or other types
  * @param  packetType 0,1,2,3,4
  * 0 = GSKF
  * 1 = LORA
  * 2 = RANGING
  * 3 = FLRC
  * 4 = BLE
  */

void LoRa::SetPacketType(_SetPacketType packetType)
{
	uint8_t command[1] = { (uint8_t)packetType};
	WriteCommand(0x8a,command,1);
}

uint8_t LoRa::GetStatus()
{
	uint8_t command[2] = {0xc0,0};
	uint8_t result [1] = {0};

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command, 1, TransmitTimeout);
	HAL_SPI_Receive(port, result, 1, ReceiveTimeout);
	nssHigh();
	return result[0];

}
uint8_t LoRa::GetPacketType()
{ // Page 86
	uint8_t command[2] = {0x03,0};
	uint8_t result [1] = {0};

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command, 2, TransmitTimeout);
	HAL_SPI_Receive(port, result, 1, ReceiveTimeout);
	nssHigh();
	return result[0];
}

/*
 * @brief Page 87,
 * @param rfFrequency2 [23:16]
 * @param rfFrequency1 [18:8]
 * @param rfFrequency0 [7:0]
 *
 */
void LoRa::SetRfFrequency(uint8_t rfFrequency2, uint8_t rfFrequency1, uint8_t rfFrequency0)
{ // Page 87
	uint8_t command = 0x86;
	uint8_t data[3] = {rfFrequency2, rfFrequency1,rfFrequency0};
	WriteCommand(command, data,sizeof(data));
}

void LoRa::SetLoRaWord(uint8_t LoRaWord)
{
//	uint8_t LoRaWord = 234;

	/*
	 * LoRa Synch word allows to further filter data packets
	 * is effective withint +/- 10 points
	 * for example if the word is 130 it can filter cross traffic outside of 120 and 140 on the same channel.
	 *
	 * traffic from other radios on the same channel
	 * for other radios with the same channel and a word within 120-140 will filter through the
	 * the farther to the center the less they can filter through for instance a 121 will filter
	 * sporadically vs a 129 which will filter almost 100% through.
	 */
	uint8_t buffer[2]  = {0};
	buffer[0] = 0;
	buffer[1] = 0;
	ReadRegister(0x944,  (uint8_t*)buffer, 2);
	buffer[0] &= ~(0b11110000);
	buffer[1] &= ~(0b11110000);
	buffer[0] |= ((LoRaWord>>4)<<4);
	buffer[1] |= ((LoRaWord & 0x0f)<<4);

	WriteRegister(0x944, (uint8_t*)buffer,2); // LoRa Synch Word value.




}
void LoRa::SetChannel(uint16_t channel)
{
	if( channel < 1 ) channel = 1;
	channel--;


	uint32_t frequency = ( uint32_t )( ( double )(2400000000 +( channel * 1000000)) / ( double )198.3642578125 );






	SetRfFrequency( (frequency>>16) & 0xff, (frequency>>8) & 0xff, frequency & 0xff);

}
void LoRa::SetTxParams(uint8_t power, uint8_t rampTime)
{ // Page 87
	uint8_t command = 0x8e;
	uint8_t data[2] = {power, rampTime};
	WriteCommand(command, data,sizeof(data));
}

void LoRa::SetCadParams(uint8_t cadSymbolNum)
{ // Page 88
	WriteCommand(0x88,&cadSymbolNum,1);
}

/*
 * @brief Page 89,This command fixes the base address for the packet handing operation in Tx and Rx mode for all packet types.
 * 	Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
 * 		byte of the data payload by sending the command
 * 		Note:
 * 		txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map
 * @param txBaseAddress TX base address 0x80
 * @param rxBaseAddress RX base address 0x00
 */
void LoRa::SetBufferBaseAddress(uint8_t txBaseAddress ,uint8_t rxBaseAddress )
{ // Page 89
	uint8_t command = 0x8f;
	uint8_t data[2] = {txBaseAddress, rxBaseAddress};
	WriteCommand(command, data,sizeof(data));
}


/*
 * @brief Page 89, Define the modulation parameter
 *
 * 						spreading factor:
 * @param parm0 0x50 = 5, 0xx6 = 6... 6-12
 * 						bandwidth:
 * @param parm1 0x0a = 1625.0, 0x18 = 812.5, 0x26 = 406.25, 0x34 = 203.125
 *						coding rate:
 * @param parm2 0x01-0x07 = 4/5, 4/6, 4/7, 4/8, 4/5*, 4/6*, 4/8*.
 *
 *
 *
 */

void LoRa::SetModulationParams(_LoRa_SpreadingFactor param1, _LoRa_BW param2, _LoRa_CR param3)
{ // Page 89
	uint8_t command = 0x8b;
	uint8_t data[3] = {param1, param2,param3};
	WriteCommand(command, data,sizeof(data));


	if( packetType == LORA) // See page 130
	{
		switch(param1)
		{
		case LORA_SF_5: case LORA_SF_6:
			data[0] = 0x1E;
			break;
		case LORA_SF_7: case LORA_SF_8:
			data[0] = 0x37;
			break;
		case LORA_SF_9: case LORA_SF_10: case LORA_SF_11: case LORA_SF_12:
			data[0] = 0x32;

			break;
		default:
			data[0] = 0x1E;
			break;

		}
		WriteRegister( 0x925, data , 1 );
		data[0] = 0x01;
		WriteRegister(0x093C,data,1);//In all cases 0x1 must be written to the Frequency Error Compensation mode register 0x093C


	}



}
void LoRa::SetPacketParams(uint8_t param1,
		PacketParams2 param2, uint8_t param3,
		PacketParams4 param4, PacketParams5 param5,
		uint8_t param6, uint8_t param7)
{ // Page 90
	uint8_t command = 0x8c;
	uint8_t data[7] = {param1,(uint8_t)param2,param3,(uint8_t)param4,(uint8_t)param5,param6,param7};
	WriteCommand(command, data,sizeof(data));
}

void LoRa::GetRxBufferStatus(uint8_t &rxPayloadLen, uint8_t &rxStartBufferPointer)
{ // Page 92
	uint8_t command[2] = {0x17,0};

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command, 2, TransmitTimeout);
	HAL_SPI_Receive(port, command, 2, ReceiveTimeout);
	nssHigh();

	rxPayloadLen = command[0];
	rxStartBufferPointer = command[1];
}

void LoRa::GetPacketStatus(uint8_t &status1, uint8_t &status2, uint8_t &status3, uint8_t &status4, uint8_t &status5)
{ // Page 93
	uint8_t command[5] = {0x1d, 0};

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command, 2, TransmitTimeout);
	HAL_SPI_Receive(port, command, 5, ReceiveTimeout);
	nssHigh();

	status1 = command[0];
	status2 = command[1];
	status3 = command[2];
	status4 = command[3];
	status5 = command[4];
}

uint8_t LoRa::GetRssiInst()
{ // Page 95
	uint8_t command[2] = {0x1f,0};

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command, 2, TransmitTimeout);
	HAL_SPI_Receive(port, command, 1, ReceiveTimeout);
	nssHigh();
	return command[0];
}

void LoRa::SetDioIrqParams(uint8_t irqMask1, uint8_t irqMask0,
							uint8_t dio1Mask1, uint8_t dio1Mask0,
							uint8_t dio2Mask1, uint8_t dio2Mask0,
							uint8_t dio3Mask1, uint8_t dio3Mask0)
{ // Page 96
	uint8_t command = 0x8d;
	uint8_t data[8] = {irqMask1, irqMask0,
						dio1Mask1, dio1Mask0,
						dio2Mask1, dio2Mask0,
						dio3Mask1, dio3Mask0};
	WriteCommand(command, data,sizeof(data));

}

void LoRa::GetIrqStatus(uint8_t &irqStatus1, uint8_t &irqStatus0)
{ // Page 97
	uint8_t command[] = {0x15, 0};
	uint8_t result[] = {0,0};

	WaitForBusy();
	nssLow();
	HAL_SPI_Transmit(port, command, 2, TransmitTimeout);
	HAL_SPI_Receive(port, result, 2, ReceiveTimeout);
	nssHigh();

	irqStatus1 = result[0];
	irqStatus0 = result[1];
}
void LoRa::ClearIrqStatus( uint8_t irqMask1, uint8_t irqMask0)
{ // Page 97
	uint8_t command = 0x97;
	uint8_t data[2] = {irqMask1, irqMask0};
	WriteCommand(command, data,sizeof(data));
}

void LoRa::SetRegulatorMode(uint8_t regModeParam)
{ // Page 143
	uint8_t command = 0x96;
	uint8_t data[1] = {regModeParam};
	WriteCommand(command, data,sizeof(data));
}

void LoRa::SetSaveContext()
{ // Page 144
	WriteCommand(0xd5);
}

void LoRa::SetAutoFS(uint8_t enable)
{ // Page 85
	uint8_t command = 0x9e;
	uint8_t data[1] = {enable};
	WriteCommand(command, data,sizeof(data));
}

void LoRa::SetAutoTx(uint8_t time1, uint8_t time0)
{ // Page 119
	uint8_t command = 0x98;
	uint8_t data[2] = {time1,time0};
	WriteCommand(command, data,sizeof(data));
}
void LoRa::SetLongPreamble(uint8_t enable)
{ // Page 83
	uint8_t command = 0x9b;
	uint8_t data[1] = {enable};
	WriteCommand(command, data,sizeof(data));
}
void LoRa::SetRangingRole(uint8_t master)
{ // Page 138
	uint8_t command = 0xa3;
	uint8_t data[1] = {master};
	WriteCommand(command, data,sizeof(data));
}
void LoRa::SetAdvancedRanging(uint8_t enable)
{ // Page ??
	uint8_t command = 0x9a;
	uint8_t data[1] = {enable};
	WriteCommand(command, data,sizeof(data));
}


