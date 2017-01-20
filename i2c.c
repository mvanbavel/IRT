/** 
 *  \file   i2c.c  
 *  \brief  Driver for I2C bus communications.
 *
 *          This module contains macros, structures, defines and functions 
 *      to implement a master and slave I2C interface using the TWI.
 *
 *  \author John Myers, modified by Peter van Bavel
 *  \version $Revision$
 */
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
#include <avr/interrupt.h>
#include <stdbool.h>


/*
 *  Variable definitions
 */
volatile uint8_t * I2cDataPtr;
volatile uint8_t * I2cReadPtr;  //pointer for read data, no longer shared with write
volatile I2CSTAT_T I2cStatus;   //global holds status of i2 PvB
volatile uint8_t I2cNumByte;    //-Holds the # of bytes in the I2C data struct/array
volatile uint8_t I2cNumReadBytes; //Holds the # of bytes to be read
volatile bool I2c_RepeatStart;   //if true repeat don't stop after write.
//volatile rbuff_t I2cTxBuff;     //-(slave-mode) Data to be sent on master request 
volatile rbuff_t I2cRxBuff;     //-(slave-mode) Data to be read from master

/*================================================================*
                              i2c_isr()
    An interrupt is requested whenever an event occurs,
 on the I2C bus, causing the I2C peripheral to change state.
 The state is held in the I2STAT SFR.
 *================================================================*/
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:  i2c_isr

ORIGINATOR: john myers comments by PvB  

DESCRIPTION: \brief service various I2C events,indicated and controlled by I2STAT register PvB. 

ON ENTRY:   
          
ON EXIT:hardware is written

GLOBALS: I2cDataPtr points to data sent? PvB
		I2cNumByte holds number of bytes, modified PvB

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ISR(TWI_vect)
{
    // used in commented out code static uint8_t I2cNumRxByte;
    register uint8_t status = I2STAT;   //read hardware status register PvB
    //////////////////////
    // Master Transmit  //
    //////////////////////
    if ((status == 0x08) || (status == 0x10))
    {  //-START CONDITION TRANSMITTED
	    //-or REPEATED START CONDITION TRANSMITTED

    //-clear start condition flag so another start condition is not requested
        i2c_no_start_();                        
    //-transmit address
		if(status == 0x10)
		{
			I2DAT = *I2cReadPtr;  //send slave address + read command, next interrupt should be master read
		}
		else
		{			
        I2DAT = *I2cDataPtr;   //shove a byte into output data register, presumably loaded with address? PvB
		}		
   }
//-SLAVE ADDRESS + WRITE TRANSMITTED - ACK RECEIVED
    else if (status == 0x18)
    {//-Device has acknowledged that the master is talking to it so... send first 'data' byte
        ++I2cDataPtr;  //how did the status become 18?PvB
        I2DAT = *I2cDataPtr;
        I2cNumByte -= 2;            //-Decrement the value after tx of byte 
    }
//-SLAVE ADDRESS + WRITE TRANSMITTED - NO ACK RECEIVED
    else if (status == 0x20)
    {
    //-Device is TOO busy to respond or is not present.
        i2c_stop_tx_();             //-no response.
        I2cStatus = I2C_ERROR;      //-status of operation is ERROR
    }
//-DATA BYTE - ACK RECEIVED
    else if (status == 0x28)
    {   //-first data byte was received so send next... if more
        if( I2cNumByte > 0)//-If not zero then send another byte
        {
            ++I2cDataPtr;
            I2DAT = *I2cDataPtr;    //-Send 'data byte' 2 through...  
            --I2cNumByte;     

        }//-send stop command since there are no more bytes to send
        else
        {
			if(I2c_RepeatStart==true)
		   {		// repeat start code for CRADA
			//move to calling routine so data can be setup i2c_start_tx_();
			I2c_RepeatStart=false;
			i2c_start_tx_();		//causes a repeat start, since stop never occurred, should cause interrupt
		   }			
		   else
		   {   //exit normal write pvB
			  I2cStatus = I2C_OK;

				i2c_stop_tx_();
		   }				
        }
    }
//-DATA BYTE TRANSMITTED - NO ACK RECEIVED
    else if (status == 0x30)
    {
        I2cStatus = I2C_ERROR;      //-status of operation is ERROR
        i2c_stop_tx_();             //-no response.
    }
//-Arbitration lost
    else if (status == 0x38)
    { 
        i2c_stop_tx_();     //-Release bus
        //I2CON &= 0x65;    //-Retry when bus is free (STA=1)
    }
    //////////////////////
    //  Master Receive  //
    //////////////////////
//-SLAVE ADDRESS + READ TRANSMITTED - ACK RECEIVED and data byte? pvB
    else if (status == 0x40)
    {
      /*do nothing, slave sends next byte  if ( I2cNumReadBytes > 1)*/
            i2c_ack_();     //-More bytes... go to case 50
        /*else  //bytes all transmitted, so stop transmission by slave? PvB
            i2c_nack_();    //-Nack... go to case 58*/
    }
//-SLAVE ADDRESS + READ TRANSMITTED - NO ACK RECEIVED
    else if (status == 0x48)
    {   //-Device may be too busy to respond or is not there
        i2c_stop_tx_();
        I2cStatus = I2C_ERROR;      //-status of operation is ERROR
    }
//-DATA BYTE RECEIVED - ACK TRANSMITTED
    else if (status == 0x50)
    {
        if( I2cNumReadBytes > 1)//-If more then two receive another byte
        {
            *I2cReadPtr = I2DAT;    //-Load data buffer.
            ++I2cReadPtr;           //-Get ready for next byte
            --I2cNumReadBytes;           //-
        }
        else//-Get ready to stop receiving
        {//-Part 1 of 2 of the stop sequence
			i2c_stop_tx_();
            *I2cReadPtr = I2DAT;    //-Load data buffer - second to last byte to receive
            ++I2cReadPtr;           //-Get ready for next byte
            --I2cNumReadBytes;           //-
			I2cStatus = I2C_OK;
		 }
    }
//-DATA BYTE RECEIVED - NO ACK TRANSMITTED
    else if (status == 0x58)
    {
        *I2cReadPtr = I2DAT;        //-Load data buffer.
        //-send stop command since there are no more bytes to send
		I2cStatus = I2C_OK;
        i2c_stop_tx_();
        
/// \todo add code to enable repeated starts
        I2cStatus = I2C_OK;         //-status of operation is OK
    }
 /*  CRADA doesn support slave operations PvB   //////////////////////
    //   Slave Receive  //
    //////////////////////
//-OWN ADDRESS + WRITE RECEIVED - ACK Sent
    else if (status == 0x60)
    {
        if (I2cRxBuff.dptr)
        {
            i2c_ack_();
            I2cStatus = I2C_BUSY;    
        }
        else  //-ring buffer not initialized
        {
            i2c_nack_();
            I2cStatus = I2C_NO_RBUF;
        }
    }
//-Arbitration lost in SLA+R || Own SLA+W received - ACK RECEIVED
    else if (status == 0x68)//-lost arbitration with device trying to address this one
    {/// \todo write code to handle lost Arbitration 
    }
//-General call address received - ACK RECEIVED
    else if (status == 0x70)
    {//Get ready to Rx data(case-90).
        I2cStatus = I2C_BUSY;
        I2cNumRxByte = 0x00;
        if (I2cRxBuff.dptr)
            i2c_ack_();
        else  //-ring buffer not initialized
            i2c_nack_();            
    }
//-Arbitration lost in SLA+R to General call - ACK RECEIVED
    else if (status == 0x78)
    {}
//-DATA BYTE RECEIVED
    else if (status == 0x80)
    {
        *I2cRxBuff.tail = I2DAT;    //-Load data buffer.   
        i2c_ack_();
        //-Increment pointer
        if ((I2cRxBuff.tail-I2cRxBuff.dptr) & I2C_RX_RING_BUFFER_SIZE)
            I2cRxBuff.tail = I2cRxBuff.dptr;
        else
            ++(I2cRxBuff.tail);         
    }
//-DATA BYTE RECEIVED - Nack returned
    else if (status == 0x88) 
        i2c_ack_();  //-Re-enable reception
//-General call 'DATA' received - ACK RECEIVED
    else if (status == 0x90)
    {   //first time entering
        if ( I2cNumRxByte == 0x00 )
        {
            //-Hardware master (e.g keyboard)
            if (I2DAT & 0x01) 
            { 
                *I2cRxBuff.tail = (I2DAT>>1);//get address
                if ((I2cRxBuff.tail-I2cRxBuff.dptr) & I2C_RX_RING_BUFFER_SIZE)
                    I2cRxBuff.tail = I2cRxBuff.dptr;
                else
                    ++(I2cRxBuff.tail);
                i2c_ack_();
                I2cNumRxByte++;
            }
            else if (I2DAT == 0x04)  //-Re-read i2c-HW address
            {
                i2c_nack_();
                if (i2c_get_address)
                    i2c_get_address ();
            }
            else if (I2DAT == 0x06)  //-Reset then re-read i2c-HW address.
            {
            /// \todo Do a reset
            }                
        }
        else //-Two or more times through
        {
            *I2cRxBuff.tail = I2DAT;    //-Load data buffer.
            if ((I2cRxBuff.tail-I2cRxBuff.dptr) & I2C_RX_RING_BUFFER_SIZE)
                I2cRxBuff.tail = I2cRxBuff.dptr;
            else
                ++(I2cRxBuff.tail);
            i2c_ack_();
            I2cNumRxByte++;
        }
    }
//-General call 'DATA' received - Nack RECEIVED
    else if (status == 0x98)
    {
        i2c_ack_();
    //-Received General call but didn't ack because ring buffer wasn't initialized 
        I2cStatus = I2C_NO_RBUF; 
    }
//-Stop or reStart received while being addressed(talked to)
    else if (status == 0xA0)
    {
        i2c_ack_(); //-(re)enable slave reception
        I2cStatus = I2C_OK; 
    }
    //////////////////////
    //  Slave Transmit  //
    //////////////////////
//-OWN ADDRESS + read Requested - ACK RECEIVED
    else if (status == 0xA8)            
        i2c_nack_(); /// \todo finish slave transmit section
    ///TODO:set arbitration to OK
//-OWN ADDRESS + read Requested(Arbitration lost)- ACK RECEIVED
//      case 0xB0:
//             if (I2cTxBuff.head == I2cTxBuff.tail) //-Empty
//             {
//                 I2cTxBuff.head = I2cTxBuff.dptr;  //-Reset  
//                 I2DAT = 0xFF;
//                 i2c_nack();
//             }
//             else
//             {
//                 I2DAT = *I2cTxBuff.head;
//                 ++(I2cTxBuff.head);
//                 //-next byte...
//                 if (I2cTxBuff.head == I2cTxBuff.tail) //-Empty
//                     i2c_nack();  
//                 else
//                     i2c_ack();  //-More bytes... got to case B8h 
//             }
//      break;
//- Read Request conti - DATA BYTE ACK RECEIVED
//       case 0xB8:
//          I2DAT = *I2cTxBuff.head;
//             ++(I2cTxBuff.head);
//             //-next byte...
//             if (I2cTxBuff.head == I2cTxBuff.tail) //-Empty
//              i2c_nack();  
//          else
//              i2c_ack();  //-More bytes... I'll be back
//      break;
//-DATA BYTE RECEIVED - STOP TX REQUEST(nACK) FROM MASTER
    else if (status == 0xC0)
    {
        I2cStatus = I2C_OK;
        i2c_ack_(); //re-enable slave reception
    }
//-DATA BYTE RECEIVED - MASTER REQ FOR ANOTHER BYTE(ack), but not present [BUFFER OVERRUN]
    else if (status == 0xC8)
    {
        I2cStatus = I2C_ERROR;      //-status of operation is ERROR
        i2c_ack_(); //re-enable slave reception
    }  */
//-UNKNOWN STATE
    else
    {
        i2c_stop_tx_();             //-request to transmit stop condition
        I2cStatus = I2C_ERROR;      //-status of operation is ERROR
    }

    i2c_clr_intrpt_flag_();            //-Start next tx/rx seq.
}


#if defined(__DOXYGEN__)
/*================================================================*
                        i2c_get_address ()
                <General Call addressing feature>
 *================================================================*/
/// This function must be written by the USER when using General Call addressing
/// \post The SFR TWAR must be written to. 
/// \return USER definable
uint8_t i2c_get_address (){}
#endif
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:  ring_buff_init

ORIGINATOR:PvB  

DESCRIPTION: \brief initialize ring buffer. 

ON ENTRY:   ringbuffer: structure type to be initialized
			ring: address of the raw data array
          
ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void ring_buff_init (volatile rbuff_t *const rb, uint8_t *const ring)
{
	//-Point to array buffer
	rb->dptr = ring;
	rb->head = ring;
	rb->tail = ring;
}
