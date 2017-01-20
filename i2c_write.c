/* Copyright (c) 2009 John Myers
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 
*/
#include "i2c.h"
#include <stdbool.h>
extern bool I2c_RepeatStart;


uint8_t i2c_readWord ( void *const data, const uint8_t n, uint8_t DeviceAddress, uint8_t *const data_out, const uint8_t m);
/*===============================================================*
                         i2c_write()
    Function to write (as master) to another I2C device.
 *===============================================================*/
/// The address is an \b 8-bit number, which is index
/// zero or the first member of the struct that is passed in to \b *Data.
/// \param[in] *data array or struct that holds data to be sent
/// \param[in] n number of bytes to write
/// \return I2cStatus
uint8_t i2c_write ( void *const data, const uint8_t n)
{
    register uint8_t TimeOut = 0x00;
  
    I2cDataPtr = (uint8_t *)data;//-Point to data that will be sent.
    *I2cDataPtr &= WRITE_I2C_DATA;
    I2cNumByte = n;              //-Set number of bytes to send.
    I2cStatus = I2C_BUSY;        //-Set status of I2C bus in global variable.  
    i2c_start_tx_();

//-wait until I2C is no longer busy.
    while (I2cStatus == I2C_BUSY)
    {
        _delay_ms (8);
        if (TimeOut++ & I2C_TIMEOUT_VALUE)
        {
            I2cStatus |= I2C_TIMEOUT;
            break;
        }
    }

//-Return status
    return I2cStatus;
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   I2c_readWord

ORIGINATOR: PvB

DESCRIPTION: \brief read a 16bit word + PEC
 by sending a command to the Melaxis, then without stopping, read the data all in one packet

ON ENTRY:data pointer for command, data_out pointer for output, n for command length, m for read byte length

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

uint8_t i2c_readWord ( void *const data, const uint8_t n, uint8_t DeviceAddress, uint8_t *const data_out, const uint8_t m)
{
	register uint8_t TimeOut = 0x00;
	
	I2cDataPtr = (uint8_t *)data;//-Point to data that will be sent.
	*I2cDataPtr = DeviceAddress;  
	*I2cDataPtr &= WRITE_I2C_DATA;  //clear bit0 to signal write PvB
	I2cNumByte = n;              //-Set number of bytes to send.
	I2cStatus = I2C_BUSY;        //-Set status of I2C bus in global variable.
	//Now that write portion is ready to go, prepare for read portion
	I2c_RepeatStart=true;   //signal to isr to use repeat start, not stop PvB
	I2cReadPtr = (uint8_t *)data_out;
	*I2cReadPtr = DeviceAddress;   //force first byte to slave address
	*I2cReadPtr |= READ_I2C_DATA;  //set bit0 to signal read PvB
	I2cNumReadBytes = m;  

	 
	
	i2c_start_tx_();  //command to start TWI hardware with a start command, ISR takes over from here

	//-wait until I2C is no longer busy.
	while (I2cStatus == I2C_BUSY)
	{
		_delay_ms (8);
		if (TimeOut++ & I2C_TIMEOUT_VALUE)
		{
			I2cStatus |= I2C_TIMEOUT;
			return I2cStatus;  //failure
		}
	}
    
    
    //-if 0xFF then use ReturnData for address
   /*screw this! PvB if (DeviceAddress == I2C_ADDRESS_IN_ARRAY)
    --I2cNumByte; //-Don't count the address for number of bytes to load.
    else*/
 
	//-Return status
	return I2cStatus;
}
