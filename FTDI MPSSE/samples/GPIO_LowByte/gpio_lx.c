//
// Example code to demonstrate how to use I2C MPSSE API to manage the GPIOL0-L3 pinstate.
//

#include <stdio.h>

#include "ftd2xx.h"
#include "libmpsse_i2c.h"

#define DIR_IN	0
#define DIR_OUT 1

#define GPIO_L0 4
#define GPIO_L1 5
#define GPIO_L2 6
#define GPIO_L3 7


#define INITIAL_DIRECTION		  ((DIR_OUT << GPIO_L0) | (DIR_OUT << GPIO_L1) | (DIR_OUT << GPIO_L2) | (DIR_OUT << GPIO_L3))
#define INITIAL_VALUES			  ((1 << GPIO_L0) | (1 << GPIO_L1) | (1 << GPIO_L2) | (1 << GPIO_L3))
#define FINAL_DIRECTION			  ((DIR_OUT << GPIO_L0) | (DIR_OUT << GPIO_L1) | (DIR_OUT << GPIO_L2) | (DIR_OUT << GPIO_L3))
#define FINAL_VALUES			  0 /* drive low*/


int main()
{
	FT_STATUS ftStatus = FT_OK;
	DWORD dwNumChannels = 0;
	DWORD dwIndex = 0;
	BOOL bRet = FALSE;
	FT_DEVICE_LIST_INFO_NODE devInfo;
	FT_HANDLE ftHandle;
	ChannelConfig config;

	Init_libMPSSE();

	ftStatus = I2C_GetNumChannels(&dwNumChannels);
	ftStatus = I2C_GetChannelInfo(dwIndex, &devInfo);
	ftStatus = I2C_OpenChannel(dwIndex, &ftHandle);
	if (ftStatus != FT_OK)
	{
		printf("could not open the channel: %d\n", dwIndex);
		return 0;
	}


	config.LatencyTimer = 1;
	

	config.Pin = (INITIAL_DIRECTION |     /* BIT7   -BIT0:   Initial direction of the pins	*/
				INITIAL_VALUES << 8 |	  /* BIT15 -BIT8:   Initial values of the pins		*/
				FINAL_DIRECTION << 16 |	  /* BIT23 -BIT16: Final direction of the pins		*/
				FINAL_VALUES << 24);	  /* BIT31 -BIT24: Final values of the pins		    */
	/* initial dir and values are used for initchannel API and final dir and values are used by CloseChannel API */

	config.Options = I2C_ENABLE_PIN_STATE_CONFIG;   /* set this option to enable GPIO_Lx pinstate management */
	config.ClockRate = I2C_CLOCK_STANDARD_MODE;

	ftStatus = I2C_InitChannel(ftHandle, &config);
	/*
	*   After the InitChannel call, expect the GPIOL0-L3 to be in the state indicated by Init_dir and Init_val fields in the pin state.
	*/


	I2C_CloseChannel(ftHandle); 
	/*
	*   After the CloseChannel call, expect the GPIOL0-L3 to be in the state indicated by FInal_dir and Final_val fields in the pin state.
	*/

	return 0;
}

