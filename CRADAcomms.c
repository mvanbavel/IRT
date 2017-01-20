/**
* \file CRADAcomms.c
* \brief CRADA Dynamax serial communications library

*
* Copyright (c) Dynamax Inc, all rights reserved, 6/25/2013
*
* Created: 6/25/2014
* \author: Peter van Bavel
*
*/
/**
* \ingroup CRADA_Dynamax_Main
* \defgroup CRADA_Dynamax_Drivers_group CRADA Low Level Drivers
*
* @{
	*/

	#include <avr/io.h>

	// Standard Input/Output functions
	#include <stdio.h>
	#include <stdbool.h>

	// not supported #include <delay.h>
	#include <float.h>
	#include <string.h>
	#include "i2c.h"
	#include "CRADA.h"
	#include "tc_timeout.h"
	#include <avr/eeprom.h>

	#define TRUE	1    //added by PvB
	#define FALSE	0

	extern volatile uint8_t command[40];
	extern char XBAPIstr[255];         // API Packet
	extern volatile char XBstr[100];             // string to be transmitted
	extern void delayCRADA(double ms);   //local wrapper for delay functions, convert to interrupt driven with sleep cycles to save power
	extern bool uart_char_waiting();
	extern void uart_putchar(uint8_t data);
	extern char uart_getchar();
	extern int8_t input_count;
	extern int8_t numberofSamples;
	extern int8_t numberofSamples;
	extern volatile int8_t output_index;   //tells when to output data
	extern int8_t numberofSamples;
	extern int32_t irTamb_out;
	extern int32_t irTobj_out;
	extern bool sampleCommand;
	extern void ControlZigBee(bool resetZ,bool sleepZ);
	extern void StatusLED(bool on);
	extern char *date_p;
	extern volatile enum charge_mode_type charge_mode;
	extern volatile uint16_t NIMH_charge_counter;

	void API_Tx(void);
	void API_AT(unsigned char C1, unsigned char C2, unsigned int value);
	bool getPacket(void);
	void sendACK();
	void sendNACK();
	//uint8_t ascii2hex(char in);


	/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: API_Tx(int8_t count)

	ORIGINATOR: Susan, header by PvB

	DESCRIPTION: \brief send frame to Xbee radio
	NOTE limit is 84 bytes in a frame without fragmentation.  see p55 Zigbee "unicast transmissions,fragmentation"

	ON ENTRY:XBstr has payload, count is number of chars

	ON EXIT:XBAPIstr has formatted packet

	GLOBALS:XBstr is read XBAPIstr is written

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void API_Tx(void)
	{
		uint8_t i;
		unsigned char csum;
		//volatile unsigned char x=0;
		volatile uint8_t state=0;
		volatile uint16_t fail_count=0, ack_count=0,packet_size;
		
		/*uart_init();
		sei();   //clear buffers and re-start*/
		fail_count=0;
		do
		{  //stay in state machine until legitimate exit, or failure
			
			if(state==0)
			{	//send packet
				
				XBAPIstr[0] = 0x7E;                     // frame start
				XBAPIstr[1] = (14 + strlen(XBstr))>>8;    // MSB Length
				XBAPIstr[2] = 14 + strlen(XBstr);       // LSB length
				XBAPIstr[3] = 0x10;                     // type (tx data)
				XBAPIstr[4] = (XBstr[0]-0x30)+1;  //frame '0'-'9'->1-a                   // frame ID, set to zero to suppress ACK/NACK
				for (i=5;i<13;i++)
				XBAPIstr[i] = 0;                      // coordinator reserved 64-bit addr =0
				
				XBAPIstr[13] = 0x0FF;                   // coor 16-bit unknown, because who cares?
				XBAPIstr[14] = 0x0FE;
				XBAPIstr[15] = 0x15;                   // number hops
				XBAPIstr[16] = 0x00;                   // transmission option=0, retries enabled, encryption off, extended timeout off
				for(i=0;i<=strlen(XBstr);i++)
				{           // add data
					XBAPIstr[17+i] = XBstr[i];
				}
				csum = 0xff;                              // calculate checksum
				for(i=3;i<strlen(XBstr)+17;i++)
				{
					csum = csum - XBAPIstr[i];
				}
				/*if(XBstr[14]>=0x33 )
				 XBAPIstr[31]=0x30;   //experiment to see what happens if checksum is bad, answer, it doesn't send the packet to the farend at all, and no NACK*/
				for(i=0;i<strlen(XBstr)+17;i++)
				{      // send data typ "0,23,3.5,?  ,?  cr  bad? PvB
					uart_putchar(XBAPIstr[i]);
				}
				uart_putchar(csum);
				delayCRADA(200);   //allow time for ACK to arrive, saves 1 second from previous implementation, experiments with one sensor in net show 100msec is too short, 200 works.   
				state=1;   //move to next state
			}  //end state 0
			if(state==1)
			{		//wait for 7e ack/nack packet start
				while(uart_char_waiting()==false)
				{ //wait for a reasonable time for a char
					if(ack_count++ > 50)
					{     //no comms in a long time, OR 50 junk chars != 7E -- failed;
						ControlZigBee(true,false); //reset and wakeup radio
						state=0xf;  //terminate and discard this packet
						break;
					}
					delayCRADA(200);//50*.2=10sec use a sleep delay TBD
				}
				
				if(state==1)
				{//not failed
					if(uart_getchar()==0x7e)
					state=2;  //found start of packet
					else
					state=1;  //junk char, go get another
				}
				
			}	//end state 1
			if(state==2)
			{
				getPacket();
				//parse command
				//read size and compare checksum
				packet_size=command[1]*256+command[2];
				if(packet_size <= 0 || packet_size > 40)
				{							//packet size fails
					XBstr[0]= 'P';
					state = 0;   //bad packet, retransmitt, what if it keeps on failing? PvB
					break;
				}
				
				csum=0xff;
				for(i=3;i<packet_size+4;i++)
				{	//calculate checksum
					
					csum-= command[i];
				}
				if(csum!=0)
				{	//checksum fails
					XBstr[0]= 'C';
					state = 0;   //bad packet, retransmitt, what if it keeps on failing? PvB
					break;
				}
				else if(command[3] == 0x8a )
				{  //status packet
					
					state = 0;  //retransmitt, just joined? PvB
					break;
				}
				else if(command[3] == 0x8b)
				{	//found a ack/nack packet
					
					if(command[8]==0)
					{  //ack is OK
						//x=0;
						fail_count=0;
						state=0xf;//ack received, normal exit
					}
					else
					{	//nack bad packet, retransmitt PvB
						fail_count++;
						if(fail_count< 4)
						{
							XBstr[0]= 'N';    //N to indicate retransmitt
							state = 0;   //RETRANSMITT packet!!!! start over
						}
						else
						{	//NACK received too many times
							ControlZigBee(true,false); //reset and wakeup radio
							fail_count=0;
							state=0xf;  //exit failed
						}
						//x=command[8];  //pass out the unrecognized command
					}
				}
				else
				{
					state=0;  //do nothing unknown packet.
				}
			}  //end of state2
			
		}while(state<4);

	}
	/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE:   API_AT

	ORIGINATOR:  Susan, header by PvB

	DESCRIPTION: \brief send a command to Xbee radio? PvB

	ON ENTRY:   two command words

	ON EXIT:XBAPIstr has formatted packet

	GLOBALS:XBAPIstr is written

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void API_AT(unsigned char C1, unsigned char C2, unsigned int value)
	{
		unsigned char i;
		unsigned char csum;
		XBAPIstr[0] = 0x7E;                     // frame start
		XBAPIstr[1] = 0;                        // MSB Length
		XBAPIstr[2] = 6;                       // LSB Length
		XBAPIstr[3] = 0x08;                     // type
		XBAPIstr[4] = 0x0;                      // frame ID
		XBAPIstr[5] = C1;                       // command char 1
		XBAPIstr[6] = C2;                       // command char 2
		XBAPIstr[7] = value >> 8;                // param value MSB
		XBAPIstr[8] = value;                     // param value LSB
		csum = 0xff;                              // calculate checksum
		for(i=3;i<9;i++)
		csum = csum - XBAPIstr[i];
		for(i=0;i<9;i++)                       // send packet
		uart_putchar(XBAPIstr[i]);
		uart_putchar(csum);                         // send checksum
	}
	/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE:   handleIncomingPacket

	ORIGINATOR: PvB

	DESCRIPTION: \brief handle serial data from radio

	ON ENTRY:radio must be on

	ON EXIT:

	GLOBALS:

	Dates:9/30/14 removed conditions on eeprom settings. 
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	bool handleIncomingPacket()
	{
		uint8_t input_char;
		volatile uint8_t i,eeprom_data;
		int8_t eeprom_length;
		volatile int16_t eeprom_address;
		if(eeprom_read_byte((uint8_t*)EEPROM_RADIO_OUTPUT)==true)
		{
			//accept commands from serial port (maybe the radio)
			ControlZigBee(false,false);
			delayCRADA(30); //allow one char to arrive.
			if(uart_char_waiting()==true)
			{	//got a char
				input_char=uart_getchar(); // get it
				if(input_char==0x7E)
				{	//its a valid start of record
					if(getPacket()==FALSE )  //get rest of packet,
					{  //its a bad packet and acknowledgment is required
						//I think the radio sends the nack...sendNACK();
						return(false);  //exit handle packet
					}
					/*if(command[4]!=0)
					sendACK();  //its a good packet*/
					
					switch(command[3])  //examine packet type
					{
						case 0x90:
						{  //incoming data packet
							if(command[15]=='S')
							{  //a synchronize packet
								sampleCommand=true;
								input_count=0;	//restart packet counter
								irTamb_out=irTobj_out=0L;   //restart output accumulators
								output_index= -1;
								numberofSamples = 1;   //FORCE A QUICK 1 SAMPLE DATA RECORD
								ControlZigBee(false,true); //put back to sleep
								return(true);  //signal sample command received.
							}
							if(command[15]=='R')
							{
								//reset firmware
								StatusLED(true);delayCRADA(250); //for debug flash LED
								StatusLED(false);delayCRADA(500);
								StatusLED(true);delayCRADA(250);
								StatusLED(false);delayCRADA(500);
								StatusLED(true);delayCRADA(250);
								StatusLED(false);delayCRADA(500);
								StatusLED(true);delayCRADA(250);
								asm ("JMP 0");   //rewind everything.
							}
							if(command[15]=='V')
							{
								XBstr[0]= 0x30;  //always zero
								XBstr[1]='V';   //signal a version string
								
								strncpy(&XBstr[2],date_p,12);  //date_p is initialized in crada_usa, kind of awkward but only way it will work.
								API_Tx(/*strlen(XBstr)*/);   //send string to radio, get ack
							}
							if(command[15]=='F')
							{
								NIMH_charge_counter=0;  //start 4 hour timer
								charge_mode = top_off;  //start fast charge mode, stops when termination condition reached, then goes to trickle
								
							}
							if(command[15]=='P')
							{  //program EEPROM
								eeprom_length=command[16];
								if(eeprom_length==1||eeprom_length==2||eeprom_length==4)
								{
									eeprom_address= ( command[17]<<8) | (command[18]);
									if( eeprom_address >3 && eeprom_address < 128 && eeprom_address != EEPROM_SERIAL_COMMAND_MODE)
									{  //NOTE programing serial number OR COMMAND MODE is NOT allowed.
										for(i=0;i<eeprom_length;i++)
										{  //reverse indian, and wait for eeprom ready?
											eeprom_data = command[19+i];
											eeprom_write_byte((uint8_t *)(eeprom_address+eeprom_length-i-1),eeprom_data);
										}
										XBstr[0]= 0x30;  //always zero
										XBstr[1]='P';   //signal a program command received
										XBstr[2]=0;  //null fill
										
										API_Tx(/*3*/);   //send string to radio, get ack
										
									}
								}
							}
							if(command[15]=='X')
							{  //read eeprom
								eeprom_length=command[16];
								eeprom_address= ( command[17]<<8) | (command[18]);
								XBstr[0]= 0x30;  //always zero
								XBstr[1]='X';   //signal a program command received
								XBstr[2]=eeprom_length+0x30;
								for(i=0;i<eeprom_length;i++)
								{
									eeprom_data= eeprom_read_byte((uint8_t *)eeprom_address+eeprom_length-i-1);
									XBstr[3+i*2]=(eeprom_data>>4)+0x30;  //put one byte into output
									XBstr[3+i*2+1]= (eeprom_data&0xF)+0x30;
								}
								XBstr[3+eeprom_length*2]=0;
								

								
								
								API_Tx(/*3+eeprom_length*/);   //send string to radio, get ack
							}
						}
						case 0x8a:
						{  //status packet, ignore
						}
						case  0x8b:
						{
							//ack/nack packet, ignore
						}
						
					}
					
					
				} //end of 7e data
			}//end of some data
			ControlZigBee(false,true); //put back to sleep
		}
		else
		{	//radio output turned off
			delayCRADA(30); //same delay as radio to equalize things
		}
		return(false);  //no sample or not used
	}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE:   getPacket

	ORIGINATOR: PvB

	DESCRIPTION: \brief get a packet of data from serial port
	NOTE: doesn't exit until it gets one. 

	ON ENTRY:radio must be on

	ON EXIT:data in command[], returns False for bad packet, true for good one

	GLOBALS:command

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	bool getPacket(void)
{
	uint8_t input_char,	input_char_count=0,i;
	volatile uint16_t packet_size;
	unsigned char csum;

	// 7e start of packet is received, get the rest
	command[input_char_count++]=0x7e;
	delayCRADA(10);  //allow time for read char to clear
	while(uart_char_waiting()==true)
	{
		//stay in loop until all are read
		input_char=uart_getchar(); // get it
		if(input_char_count<sizeof(command))
		command[input_char_count++]=input_char; // store in buffer if not overflowed, otherwise they are discarded
		delayCRADA(1);  //allow time for read char to clear
		if(input_char_count>100)
		break;//escape clause
	}
 	packet_size=command[1]*256+command[2];
	if(packet_size <= 4 || packet_size > 40)
	{							//packet size fails
		return FALSE;
	}
	
	csum=0xff;
	for(i=3;i<packet_size+4;i++)
	{	//calculate checksum
		
		csum-= command[i];
	}
	if(csum!=0)
	{	//checksum fails
		return FALSE;
	}
	return TRUE;
	
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE:   ascii2hex

	ORIGINATOR: PvB

	DESCRIPTION: \brief convert ascii char to a hex 8 bit

	ON ENTRY:char input

	ON EXIT: hex.  if out of range, 0xff

	GLOBALS:

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* not used	uint8_t ascii2hex(char in)
	{
		
		if(in >= 'a'&& in <= 'f')
			in = in - 0x20;   //convert lower case to upper if a -> f
		if(in >='0' && in <= '9')
			return in-'0';
		if(in >= 'A' && in <= 'F')
			return in - 'A' + 0xa;
		return 0xFF;
	}*/