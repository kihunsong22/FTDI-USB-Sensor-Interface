/*
File:Ambient2Click_i2c.c
$ gcc Ambient2Click_i2c.c -o i2c  ./libmpsse.a
$ sudo ./i2c 
*/

#include<stdio.h>
#include<stdint.h>
#include<stdlib.h>
#include<string.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "ftd2xx.h"
#include "libmpsse_i2c.h"
#define APP_CHECK_STATUS(exp) {if(exp!=FT_OK){printf("%s:%d:%s(): status(0x%x) \
	!= FT_OK\n",__FILE__, __LINE__, __FUNCTION__,exp);}else{;}};

/* Application specific macro definitions */
#define I2C_DEVICE_BUFFER_SIZE 256
#define I2C_DEVICE_ADDRESS_ADC			0x44
#define ADC_DATA_LEN					2
#define CHANNEL_TO_OPEN					0	/*0 for first available channel, 1 for next... */
uint32_t channels;
FT_HANDLE ftHandle;
ChannelConfig channelConf;

/*!
 * \brief Writes to Ambient 2 Click
 *
 * This function writes a number of bytes to a specified address within the Ambient 2 Click
 *
 * \param[in] slaveAddress Address of the I2C slave (Ambient 2 Click)
 * \param[in] registerAddress Address of the memory location inside the slave to where the byte
 *			is to be written
 * \param[in] data Bytes that is to be written
 * \param[in] numBytes Number of bytes to read
 * \return Returns status code of type FT_STATUS(see D2XX Programmer's Guide)
 * \sa Datasheet of OPT3001 https://download.mikroe.com/documents/datasheets/OPT3001_datasheet.pdf
 * \note
 * \warning
 */
static FT_STATUS write_bytes(uint8_t slaveAddress, uint8_t registerAddress, const uint8_t *data, uint32_t numBytes)
{
	FT_STATUS status;
    uint8_t buffer[I2C_DEVICE_BUFFER_SIZE];
	uint32_t bytesToTransfer = 0;
	uint32_t bytesTransfered = 0;
	uint32_t options = 0;

	options = I2C_TRANSFER_OPTIONS_START_BIT|I2C_TRANSFER_OPTIONS_STOP_BIT;

	buffer[bytesToTransfer++] = registerAddress;
	memcpy(buffer + bytesToTransfer, data, numBytes);
	bytesToTransfer += numBytes;

	status = I2C_DeviceWrite(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);
	return status;
}

/*!
 * \brief Reads from Ambient 2 Click
 *
 * This function reads a number of bytes from a specified address within the OPT3001 Ambient 2 Click
 *
 * \param[in] slaveAddress Address of the I2C slave (Ambient 2 Click)
 * \param[in] registerAddress Address of the memory location inside the slave from where the
 *			byte is to be read
 * \param[in] *data Address to where the bytes are to be read
 * \param[in] numBytes Number of bytes to read
 * \return Returns status code of type FT_STATUS(see D2XX Programmer's Guide)
 * \sa Datasheet of OPT3001 https://download.mikroe.com/documents/datasheets/OPT3001_datasheet.pdf
 * \note
 * \warning
 */
static FT_STATUS read_bytes(uint8_t slaveAddress, uint8_t registerAddress, uint32_t numBytes)
{
	FT_STATUS status = FT_OK;
    uint8_t buffer[I2C_DEVICE_BUFFER_SIZE];
	uint32_t bytesToTransfer = 0;
	uint32_t bytesTransfered = 0;
	uint32_t options = 0;
	uint32_t trials = 0;

    options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT;
    buffer[bytesToTransfer++] = registerAddress;
    status = I2C_DeviceWrite(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);
    if(status != FT_OK)
    {
        return status;
    }

	bytesTransfered = 0;
	bytesToTransfer = numBytes;
	options = I2C_TRANSFER_OPTIONS_START_BIT | I2C_TRANSFER_OPTIONS_STOP_BIT | I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE;

	trials = 0;
	while (trials < 20)
	{
		APP_CHECK_STATUS(status);
		status = I2C_DeviceRead(ftHandle, slaveAddress, bytesToTransfer, buffer, &bytesTransfered, options);
        if(status != FT_OK)
        {
            printf("I2C_DeviceRead Failed........ status=%d\n",status);
        }
        else{
            uint16_t shiftdata ;
            //Convert bytes to light intensity
            shiftdata = (buffer[0] << 8) | buffer[1];
            printf("Convert bytes to light intensity = %d\n",shiftdata);
        }
#ifndef _WIN32
        sleep(1);
#else
        Sleep(1000);
#endif
		trials++;
	}
	return status;
}

int main(void)
{
    FT_STATUS status = FT_OK;
    FT_DEVICE_LIST_INFO_NODE devList;
    uint32_t i;
   
    // Initialize channel configurations
	memset(&channelConf, 0, sizeof(channelConf));
    channelConf.ClockRate = I2C_CLOCK_FAST_MODE;/*i.e. 400000 KHz*/
    channelConf.LatencyTimer= 255;
    channelConf.Options = 0;
    printf("I2C_GetNumChannels ...\n");
    status = I2C_GetNumChannels(&channels);
    APP_CHECK_STATUS(status);
    printf("Number of available I2C channels = %d\n",channels);
    if(channels>0)
    {
        for(i=0;i<channels;i++)
        {
            status = I2C_GetChannelInfo(i,&devList);
            APP_CHECK_STATUS(status);
            printf("Information on channel number %d:\n",i);
            /*print the dev info*/
            printf(" Flags=0x%x\n",devList.Flags);
            printf(" Type=0x%x\n",devList.Type);
            printf(" ID=0x%x\n",devList.ID);
            printf(" LocId=0x%x\n",devList.LocId);
            printf(" SerialNumber=%s\n",devList.SerialNumber);
            printf(" Description=%s\n",devList.Description);
            printf(" ftHandle=0x%p\n",devList.ftHandle);/*is 0 unless open*/
        }
        /* Open the first available channel */
        status = I2C_OpenChannel(CHANNEL_TO_OPEN,&ftHandle);
        APP_CHECK_STATUS(status);
        printf("\nhandle=0x%p status=%d\n",ftHandle,status);
        status = I2C_InitChannel(ftHandle,&channelConf);
        APP_CHECK_STATUS(status);

        uint8_t dataOUT[ADC_DATA_LEN] = {0};		

	    // Write to config register
		dataOUT[0] = 0xCE;	// Value to write to upper byte of config reg
		dataOUT[1] = 0x10;	// Value to write to lower byte of config reg
		status = write_bytes(I2C_DEVICE_ADDRESS_ADC, 0x1, dataOUT, ADC_DATA_LEN);
		if(status != FT_OK){
            printf("Write to config register is failed....\n");
            return 0;
        }
	
        status = read_bytes(I2C_DEVICE_ADDRESS_ADC, 0, ADC_DATA_LEN);
        if(status != FT_OK)
        {
            printf("read_bytes is Failed........ status=%d\n",status);
        }
    }
    I2C_CloseChannel(ftHandle);

return 0;
}