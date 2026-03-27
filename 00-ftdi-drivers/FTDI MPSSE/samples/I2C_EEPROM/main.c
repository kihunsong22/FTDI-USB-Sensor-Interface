/* Standard C libraries */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "ftd2xx.h"
#include "libmpsse_i2c.h"


/******************************************************************************/
/*								Macro and type defines							   */
/******************************************************************************/
/* Helper macros */
#define APP_CHECK_STATUS(exp) {if(exp!=FT_OK){printf("%s:%d:%s(): status(0x%x) \
	!= FT_OK\n",__FILE__, __LINE__, __FUNCTION__,exp);exit(1);}else{;}};
#define APP_CHECK_STATUS_NOEXIT(exp) {if(exp!=FT_OK){printf("%s:%d:%s(): status(0x%x) \
	!= FT_OK\n",__FILE__, __LINE__, __FUNCTION__,exp);}else{;}};

/* Application specific macro definations */
#define I2C_DEVICE_ADDRESS_EEPROM		0x50
#define I2C_DEVICE_BUFFER_SIZE			256
#define START_ADDRESS_EEPROM 			0x00 /*read/write start address inside the EEPROM*/
#define END_ADDRESS_EEPROM				0x10 /*16 bytes buffer only*/
#define EEPROM_DATA_LEN					(END_ADDRESS_EEPROM-START_ADDRESS_EEPROM)
#define CHANNEL_TO_OPEN					0	/*0 for first available channel, 1 for next... */

/* Application configuration/debugging */
#define FAST_TRANSFER					1

/******************************************************************************/
/*								Global variables							  	    */
/******************************************************************************/
static FT_HANDLE ftHandle;
static uint8_t  buffer[I2C_DEVICE_BUFFER_SIZE] = {0};

static FT_STATUS write_bytes(uint8_t  slaveAddress, uint8_t  registerAddress, const uint8_t  *data, uint32_t  numBytes)
{
	FT_STATUS status;
	uint32_t  bytesToTransfer = 0;
	uint32_t  bytesTransfered = 0;
	uint32_t  options = 0;
	uint32_t  trials = 0;

	
#if FAST_TRANSFER
	options = I2C_TRANSFER_OPTIONS_START_BIT|I2C_TRANSFER_OPTIONS_STOP_BIT|I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;
#else // FAST_TRANSFER
	options = I2C_TRANSFER_OPTIONS_START_BIT|I2C_TRANSFER_OPTIONS_STOP_BIT;
#endif // FAST_TRANSFER

	buffer[bytesToTransfer++] = registerAddress;
	memcpy(buffer + bytesToTransfer, data, numBytes);
	bytesToTransfer += numBytes;


	status = I2C_DeviceWrite(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);

	while (status != FT_OK && trials < 10)
	{
		APP_CHECK_STATUS_NOEXIT(status);
        sleep(1);
		status = I2C_DeviceWrite(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);
		trials++;
	}

	return status;
}


static FT_STATUS read_bytes(uint8_t  slaveAddress, uint8_t  registerAddress, uint8_t  bRegisterAddress, uint8_t  *data, uint32_t  numBytes)
{
	FT_STATUS status = FT_OK;
	uint32_t  bytesToTransfer = 0;
	uint32_t  bytesTransfered = 0;
	uint32_t  options = 0;
	uint32_t  trials = 0;

	
	if (bRegisterAddress)
	{
#if FAST_TRANSFER
		options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;
#else // FAST_TRANSFER
		options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT;
#endif // FAST_TRANSFER
		buffer[bytesToTransfer++] = registerAddress;
		status = I2C_DeviceWrite(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);
		trials = 0;
		while (status != FT_OK && trials < 10)
		{
			APP_CHECK_STATUS_NOEXIT(status);
			status = I2C_DeviceWrite(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);
			trials++;
		}
		if (status != FT_OK)
		{
			return status;
		}
		//APP_CHECK_STATUS(status);
	}

	bytesTransfered = 0;
	bytesToTransfer = numBytes;
#if FAST_TRANSFER
	options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE | I2C_TRANSFER_OPTIONS_FAST_TRANSFER_BYTES;
#else // FAST_TRANSFER
	options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE;
#endif // FAST_TRANSFER


	status |= I2C_DeviceRead(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);

	trials = 0;
	while (status != FT_OK && trials < 10)
	{
		APP_CHECK_STATUS_NOEXIT(status);

		status = I2C_DeviceRead(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);
        sleep(1);
		trials++;
	}
	if (status == FT_OK)
	{
		memcpy(data, buffer, bytesToTransfer);
	}

	return status;
}


void TestDeviceEEPROM()
{
	FT_STATUS status = FT_OK;
	uint32_t glitch = 0;
	uint32_t count=0;
	uint8_t dataOUT[EEPROM_DATA_LEN] = {0};		
	uint8_t address = 0;

	// Read the data multiple times
	while (++count)
	{
		// Prepare the data to write
		for(address=START_ADDRESS_EEPROM; address<END_ADDRESS_EEPROM; address++)
		{
			dataOUT[address] = address + (uint8_t)count; // count changes so data always changes every while loop
		}
			
		// Write the data
		{
			address = START_ADDRESS_EEPROM;	
			status = write_bytes(I2C_DEVICE_ADDRESS_EEPROM, address, dataOUT, EEPROM_DATA_LEN);
			if (status != FT_OK)
			{
				printf("Glitch 0 write_bytes failed! count=%ud glitch=%ud\n", (unsigned int)count, (unsigned int)glitch);
				glitch++;
				continue;
			}
			//printf("write_bytes %d\n", status);
			//Sleep(1000);
		}		

		
		do
		{
			uint8_t dataIN[EEPROM_DATA_LEN] = {0};
			address = START_ADDRESS_EEPROM;
			status = read_bytes(I2C_DEVICE_ADDRESS_EEPROM, address, 1, dataIN, EEPROM_DATA_LEN);
			if (status != FT_OK)
			{
				printf("Glitch 1 write_bytes failed! count=%ud glitch=%ud\n", (unsigned int)count, (unsigned int)glitch);
				glitch++;
				break;
			}

			// Compare the write and read data
			if (memcmp(dataIN, dataOUT, EEPROM_DATA_LEN) == 0)
			{
				printf("write[%d] & read[%d] :%d bytes same (glitch = %ud)\n",dataOUT[address], dataIN[address], EEPROM_DATA_LEN, (unsigned int)glitch);
				break;
			}
			else
			{
				for(address=START_ADDRESS_EEPROM; address<END_ADDRESS_EEPROM; address++)
				{
					if (dataOUT[address] == dataIN[address])
					{
						printf("[%d] write %d read %d\n", address, dataOUT[address], dataIN[address]);
					}
					else
					{
						printf("[%d] write %d read %d ERROR!!!\n", address, dataOUT[address], dataIN[address]);
					}
				}
				glitch++;

				break;
			}

		} while (count > 20);
		if(count > 20 ) break;
	}
}

int main(int argc, char *argv[])
{
	FT_STATUS status = FT_OK;
	FT_DEVICE_LIST_INFO_NODE devList = {0};
	ChannelConfig channelConf;
	uint32_t  channels = 0;
	uint32_t  i = 0;

	// Initialize channel configurations
	memset(&channelConf, 0, sizeof(channelConf));
	channelConf.ClockRate = I2C_CLOCK_FAST_MODE;
#if FAST_TRANSFER
	channelConf.LatencyTimer = 8;
#else // FAST_TRANSFER
	channelConf.LatencyTimer = 255;
#endif // FAST_TRANSFER

	// Get the number of channels
	status = I2C_GetNumChannels(&channels);
	APP_CHECK_STATUS(status);
	printf("Number of available I2C channels = %d\n",(int)channels);
	if(channels>0)
	{
		for(i=0;i<channels;i++)
		{
			// Get the channel information
			status = I2C_GetChannelInfo(i,&devList);
			APP_CHECK_STATUS(status);
			printf("Information on channel number %ud:\n",(unsigned int)i);
			/*print the dev info*/
			printf("		Flags=0x%x\n",devList.Flags);
			printf("		Type=0x%x\n",devList.Type);
			printf("		ID=0x%x\n",devList.ID);
			printf("		LocId=0x%x\n",devList.LocId);
			printf("		SerialNumber=%s\n",devList.SerialNumber);
			printf("		Description=%s\n",devList.Description);
			printf("		ftHandle=0x%x\n",(unsigned int)devList.ftHandle);/* 0 if not open*/
		}

		// Open the first available channel
		status = I2C_OpenChannel(CHANNEL_TO_OPEN,&ftHandle);
		APP_CHECK_STATUS(status);
		printf("\nhandle=0x%x status=%d\n",(unsigned int)ftHandle,(unsigned int)status);
		
		// Initialize the channel
		status = I2C_InitChannel(ftHandle,&channelConf);
		APP_CHECK_STATUS(status);

		TestDeviceEEPROM();

        while(1);
		status = I2C_CloseChannel(ftHandle);
	}

	return 0;
}

