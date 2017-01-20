/**
* \file CRADASDI.c
* \brief CRADA Dynamax serial SDI mode program

*
* Copyright (c) Dynamax Inc, all rights reserved, 6/25/2013
*
* Created: 8/4/2015
* \author: Peter van Bavel
*
*/
/**
* \ingroup CRADA_Dynamax_Main
* \defgroup CRADA_Dynamax_Drivers_group CRADA Low Level Drivers
*
* @{
	*/
#include <avr/interrupt.h>
#include <avr/eeprom.h>

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

#define TRUE	1    //added by PvB
#define FALSE	0
#define SLEEP_STATE 0
#define COMMAND_STATE 1
#define ADDRESS_STATE 2
#define EXECUTE_STATE 3
#define WAKEUP_STATE 4 
#define VALID_COMMAND 2
#define INVALID_COMMAND 3
#define VALID_ADDRESS 4
#define INVALID_ADDRESS 5
#define MARK_8MSEC 6
#define FAIL 7
#define PARITY_ERROR 8
#define BREAK 9
#define SPACE 1 //5 volts approx 3.5 to 5.5 specification SDI 3.1 page 3; NOTE! a '1' here refers to the logic hardware command, not the serial data meaning which is zero.
#define MARK 0 //0 volts approx RS232 logic but ttl levels, i.e. RS232 should be -12-> -3, instead for SDI it -0.5 to 1.0 ~0.0
#define SDI_TRANSMIT (1<<PORTB1)
enum cmd_type sdi_cmd;
#define NUMBEROFSAMPLES 6   //take this many samples in an instant to average temperature

	



extern void StatusLED(bool on);
extern void delayCRADA(double ms);   //local wrapper for delay functions, convert to interrupt driven with sleep cycles to save power
extern void uart_putchar(uint8_t data);
extern char uart_getchar();
extern char *date_p;
extern volatile char XBstr[100];             // string to be transmitted
extern void ControlBatteryCharger(enum charge_mode_type);
extern void IRTSleep();
extern void IRTWakeup();
extern void getReading(unsigned char);
extern volatile enum charge_mode_type charge_mode;
extern void terminateCharge(void);
extern volatile int32_t irTobj, irTamb,irTobj_accum[NUMBEROFSAMPLES],irTamb_accum[NUMBEROFSAMPLES];
extern int8_t numberofSamples;
extern uint16_t vbatt2;
extern uint16_t NIMH_charge_counter;

void SDI(void);
int8_t get_SDI_status();
int8_t get_SDI_data();
int8_t get_SDI_address();
int8_t get_SDI_command();
void send_SDI_string(char *data_string);
void delay_832us(int16_t del);
int8_t asciitohex(int8_t);
char hextoascii(int8_t in);
int8_t sdi_port();
void sdi_port_recieve();
void send_ack_string(void );
//void uPulseLED(uint8_t code);

int8_t state;
uint8_t incoming;
uint8_t command;
static volatile char out_string[40];   //to send out
static volatile char address_string[2]={'H',0};; //address
static volatile char serial_str[8]; //serial number
static volatile	char incomingStr[20];  //incoming command
void get_Measurement_data();
static volatile char object_temp_str[]={" +12.34"};
static volatile char ambient_temp_str[]={" +56.78"};
static volatile char vbatt_str[]={"+06.0"};
static volatile char version[]="100\r";

uint16_t simulation_count;
uint8_t sim[] = {'2','D','0'};  //'2',1  '2',I' '2','M','!','2','D','0'

/*		uart_putchar('U');
		//		StatusLED(true);
		delayCRADA(10);
		//		StatusLED(false);
		  test by sending continuous measurment string do{				get_Measurement_data();
			sprintf((char*)out_string,"%s%s",(char*)address_string,(char*)object_temp_str);
			send_SDI_string(out_string);
			delayCRADA(1000);
		}while(1);*/


/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: SDI()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief communicate by SDI( serial data interface)
	

	ON ENTRY: hardware is initialized

	ON EXIT:

	GLOBALS:

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SDI(void)
{
	bool data_flag = true;   //debug should default to false.. signals that a measurement cycle has occurred and is available for transmission
	bool checksum_flag = false;  // data defaults to no checksum
	uint16_t checksum;
	simulation_count=0;  //debug
	state = SLEEP_STATE;  //state machine starts in sleep mode.
	int8_t status, i,j;
	volatile char node_str[7], temp[10],check[4];
	uint8_t eeprom_data;
	int8_t eeprom_length;
	int16_t eeprom_address;
	volatile	uint16_t termChargeTimer=0,addressStateTimer=0;


	uint32_t node_id;
/*	do{//debug transmission capability
		send_SDI_string("
	}while(1);*/
	/*wont work because its setup for sleep timeer I think.cli();   //stop interrupts while setting up so no spurious events
	tc_timeout_init();
	sei();*/
	
	
	node_id=eeprom_read_word((uint16_t*)EEPROM_SENSOR_ID); //now 16bit hex
	
	
	sprintf((char*)temp,"%5.0ld",node_id);
	for(i=0;i<5;i++)
	{	//convert leading spaces to ascii zero's
		if(temp[i]==0x20)
		temp[i]='0';
	}
	strncpy(node_str+1,temp,6);
	node_str[0]='+';
	
	
	do{  //endless loop while executing SDI interface, can't get out!
		
		PORTC |= (1<<PC2);    //set DACEN, old name for signal, now should be called 5VEN   
		switch(state)
		{
		case SLEEP_STATE:
		{   //unit is asleep?  waiting for a break on line;
			StatusLED(false);
			termChargeTimer++;
			if(termChargeTimer==1764)
				{    //ready to check battery charge, that takes 50msec so might miss a break..

					termChargeTimer=0;  //rollover at 50
					StatusLED(true);
					terminateCharge();  // only check set timer for 10 sec, then check only if expired.
				}
			
			if(get_SDI_status()==BREAK)
			{  //come out of sleep, got a break*/
			state = WAKEUP_STATE;
				
			}
			//status return of mark or fail is ignored, so stays in this state
			break;
		}
		case WAKEUP_STATE:
		{
			
			//initialize sensor, whatever that means.   but keep stored data
			if(get_SDI_status()==MARK_8MSEC)
			{	// got a long mark after break, so get address
				state = ADDRESS_STATE;
			}
			/*else
			{	//something bad, give up
				state = SLEEP_STATE;   //unexpected result
			}*/
			break;
		}
		case ADDRESS_STATE:
		{	// get a char and see if its a valid address matching this unit's address
			StatusLED(true);
			tc_timeout_start_singleshot(0,100);   //start 100msec timer
			if((status=get_SDI_address())==VALID_ADDRESS)
			{		//got a good address go get rest of command
				state = COMMAND_STATE;
			}
			else if(status==INVALID_ADDRESS  )
			{		//could be something bad happened, so start over at sleep state, its normal to come here after a completed command?
				state = SLEEP_STATE;
			}
			else if( tc_timeout_test_and_clear_expired(0)==true)
			{
				state = SLEEP_STATE;				
			}
			else if(status==BREAK)
			{  //break start over with new address
				state = WAKEUP_STATE;
			}
			addressStateTimer++;
			if(addressStateTimer==200)
			{
				addressStateTimer=0;
				state = SLEEP_STATE;
			}
			
			break;
		}
		case COMMAND_STATE:
		{	//address is good so now get a command
			switch(get_SDI_command())
			{
				case VALID_COMMAND:
				{   //its good so execute
					state = EXECUTE_STATE;
					break;
				}
				case MARK_8MSEC:
				{   //GOT A long mark, so jump for another command
					state= ADDRESS_STATE;
					break;
				}
				default: // non-valid command or other problem
				{	//controller sent something bad, what about timeouts? suggest a 100msec restart at each state
					state= SLEEP_STATE;  //experiment to remove lockup, was wakeup state, where did that come from?
					break;
				}
			}
			break;
		}
		case EXECUTE_STATE:
		{		//******************  here is the payload, where the work gets done *********************
			//execute requested command
			if(sdi_cmd == a)
			{  //ack simple hello
				send_ack_string();
				state=ADDRESS_STATE;
			}
			else if(sdi_cmd == aI)
			{  //send identification data
				strcpy((char*)out_string,(char*)address_string);
				strcat((char*)out_string,"13");
				strcat((char*)out_string,"DYNAMAX ");
				strcat((char*)out_string,"SP IRT");
				strncat((char*)out_string, version,3);  //10 digit date
				sprintf((char*)serial_str,"%4.0X",eeprom_read_word((uint16_t*)EEPROM_SENSOR_ID));  //first 16bits
				strncat((char*)out_string,(char*)serial_str,4);
				strcat((char*)out_string, "\r\n");    //carriage return, line feed
				
				send_SDI_string((char*)out_string);
				state=ADDRESS_STATE;
			}
			else if(sdi_cmd == aAb)
			{   //change address
				eeprom_write_byte((uint8_t*)EEPROM_SDI_ADDRESS,incomingStr[1]);   //change address
				address_string[0]=eeprom_read_byte((uint8_t*)EEPROM_SDI_ADDRESS),address_string[1]=0;
				sprintf((char*)out_string,"%s\r\n",(char*)address_string);
				send_SDI_string((char*)out_string);
				state=ADDRESS_STATE;
			}
			else if(sdi_cmd == aM || sdi_cmd== aMC)
			{//    get Measurement 
				data_flag=false;
				if(sdi_cmd== aMC)
					checksum_flag = true;
				else
					checksum_flag = false;
				sprintf((char*)out_string,"%s0034\r\n",(char*)address_string);
				send_SDI_string((char*)out_string);  //send measurement will be ready in 3 seconds, 2 measurements
				
				get_Measurement_data();  //****************** the REAL deep payload--  get the measurement!  
				
				send_ack_string();
				
				data_flag= true;   // we have data
				state=ADDRESS_STATE;
			}
			else if(sdi_cmd == aDx)
			{	// send a data packet
				if( data_flag==true)
				{   //send a complete data packet with both temperatures
					sprintf((char*)out_string,"%s%s%s%s%s",(char*)address_string,(char*)node_str,(char*)vbatt_str,(char*)object_temp_str,(char*)ambient_temp_str);
					if(checksum_flag==true)
					{
						checksum=0;
						for(i=0;i<strlen(out_string);i++)
						{
							checksum = checksum ^ out_string[i];
							for(j=0;j<8;j++)
							{
								if((checksum & 1)== 1 )
								{
									checksum = checksum >> 1;
									checksum = checksum ^ 0xa001;
								}
								else
								{
									checksum = checksum >> 1;
								}
							}
						}
						check[0] = 0x40 | checksum>>12;
						check[1] = 0x40 | (checksum>>6 & 0x3F);
						check[2] = 0x40 | (checksum & 0x3F);
						check[3]=0;  //null
						strncat(out_string,check,4);
						
					}  //end of checksum
				strncat(out_string,"\r\n",3);
				send_SDI_string((char*)out_string);
				} //end of if data true
				state=ADDRESS_STATE;
			} //end of data transmission
			else if(sdi_cmd == aXF)
			{	
				charge_mode = top_off;  //start fast charge mode, stops when termination condition reached, then goes to trickle
				ControlBatteryCharger(charge_mode);
				NIMH_charge_counter= 0;
				send_ack_string();

				state=ADDRESS_STATE;
			}
			else if(sdi_cmd == aXP)
			{		// packet is sAAAAdddd, or sAAAAd, or sAAAAdd s=size AAAA address d = data X size
				eeprom_length=asciitohex(incomingStr[2]);  //was 16
				for(i=3;i< 7+eeprom_length*2;i++)
				{
					incomingStr[i]=asciitohex(incomingStr[i]);
				}
				if(eeprom_length==1||eeprom_length==2||eeprom_length==4)
				{
					eeprom_address= ( incomingStr[3]<<12) | (incomingStr[4]<<8 | incomingStr[5]<<4 |incomingStr[6]);
					if( eeprom_address >3 && eeprom_address < 128 && eeprom_address != EEPROM_SERIAL_COMMAND_MODE)
					{  //NOTE programing serial number OR COMMAND MODE is NOT allowed.  too dangerous
						send_ack_string();
						//output message put here to solve an apparent delay problem with eeprom write
						for(i=0;i<eeprom_length;i++)
						{  //reverse indian, and wait for eeprom ready?
							eeprom_data = incomingStr[7+i*2]<<4 | incomingStr[8+i*2];
							eeprom_write_byte((uint8_t *)(eeprom_address+eeprom_length-i-1),eeprom_data);
						}
					}
				}

				state=ADDRESS_STATE;
			}
			else if(sdi_cmd == aXX)
			{		// packet is sAAAA s=size AAAA address
				eeprom_length=asciitohex(incomingStr[2]);  //was 16
				if(eeprom_length>4)
				eeprom_length =4;   //guard against overruns
				for(i=3;i< 7+eeprom_length*2;i++)
				{
					incomingStr[i]=asciitohex(incomingStr[i]);
				}
				eeprom_address= ( incomingStr[3]<<12) | (incomingStr[4]<<8 | incomingStr[5]<<4 |incomingStr[6]);
				out_string[0]=address_string[0];
				out_string[1]='X';out_string[2]='X';  //header for response
				for(i=0;i<eeprom_length;i++)
				{
					eeprom_data= eeprom_read_byte((uint8_t *)eeprom_address+eeprom_length-i-1);
					out_string[3+i*2]=hextoascii(eeprom_data>>4);  //put one byte into output
					out_string[3+i*2+1]= hextoascii(eeprom_data&0xF);
				}
				out_string[3+eeprom_length*2]=0;  //null term
				strncat(out_string,"\r\n",3);
				send_SDI_string((char*)out_string);

				state=ADDRESS_STATE;

			}
			else if(sdi_cmd == aXR)
			{	//reset, jump to orign point of eeprom
				send_ack_string();
				asm("JMP 0");
			}

			//commands not in this list cause a big nothing
			break;
	}
	default:
		{
		
		//illegal state?  do something!
		state = SLEEP_STATE; // restart state machine
		}
	}  //end of state switch
	}while(1);  //loop forever
}
void send_ack_string(void )
{
				sprintf((char*)out_string,"%s\r\n",(char*)address_string);
				send_SDI_string((char*)out_string);
}

/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: get_SDI_status()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief obtain SDI interface status, BEFORE data is expected
	see figure 3 of SDI-12 specification 1.3 p 23
	

	ON ENTRY:  assumes port is not transmitting and is idle( marking).

	ON EXIT:returns defined status either a BREAK, or 8msec SPACE or noisey conditions( might be data or transition to break or long space)

	GLOBALS:

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int8_t get_SDI_status()
{
	int16_t timer_tick= 0;
	sdi_port_recieve();
	_delay_us(1);//allow  hardware to settle
	if(sdi_port() == SPACE)//note, look for glitches?pvB
	{  //port is currently receiving a space = 3v, might be a break
				//StatusLED(true);
		for(timer_tick=0;timer_tick< 1200;timer_tick++)
		{		//look for 120msec is battery charging?
				if(sdi_port()==MARK)
				{  //exit at end of break
					break; 
				}
			
			_delay_us(100);  //wait some more
		}
		//PulseLED(2);
		//StatusLED(false);
		if(timer_tick>=100  && timer_tick < 200)  //break is between 12 and 20msec long, could tighten upper limit check teraterm break? PvB
			return BREAK;
		else
			return FAIL;  //break too short or too long
	}
	else
	{   //port is mark which it would normally be at idle //test for 8msec mark;
		for(timer_tick=0;timer_tick< 200;timer_tick++)
		{
			if( timer_tick >=75)
			{	//END OF MARK FOR 8 MSEC, calibrated with pulse timing
				//StatusLED(false);
				return MARK_8MSEC;
			}
			if(sdi_port() == SPACE)
				return FAIL;   //noisey data, or it could be a transition to break.
			_delay_us(100);  //wait some more
		}
		return FAIL;   //mark lasted too long
		
	}
	return FAIL; //should never get here
}
/*void uPulseLED(uint8_t code)
{// used to time uart behavior
	PORTD |= 1<<PORTD7;    //set bit turns led off
	//	_delay_us(1);
		PORTD &= ~(1<<PORTD7); // turns on
		_delay_us(1);
	PORTD |= 1<<PORTD7;    //set bit turns led off
			
}*/
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: get_SDI_data()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief obtain SDI interface data
	

	ON ENTRY:a break has been received.  

	ON EXIT:returns defined line status: mark8msec, TRUE or SPACE8msec? and one data byte if true 

	GLOBALS:data ( a global) has output

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int8_t get_SDI_data()
{
	/*data = sim[simulation_count++]; //simulation code
	return true;*/
	int16_t timer_tick;
	volatile uint8_t d,bit,p;
	incoming= 0;
	
	for(timer_tick=0;timer_tick < 77;timer_tick++)
	{
		if(sdi_port() == SPACE)
			break; //found start bit
		_delay_us(104);		//1/8 baud
	}
	if( timer_tick==77)
		return MARK_8MSEC;  //waited too long, no start bit
	// start bit detected, get data
	//StatusLED(true);
	_delay_us(416);  //shift data sampling point a towards the middle
	_delay_us(832);  // shift to next bit
	//StatusLED(false);
	for(timer_tick=0;timer_tick<7; timer_tick++)
	{
		//uPulseLED(1);
		incoming |= (sdi_port()<<7);  //MOVE PORT B0 TO B7 AND MERGE WITH DATA
		incoming = incoming>>1; // shift to right BECAUSE LSB ARRIVES FIRST, MSB LAST 
		_delay_us(832);  // shift to next bit
	}
	//uPulseLED(1);
	incoming |= (sdi_port()<<7);  //MOVE PORT B0 TO B7 AND MERGE WITH DATA
	incoming = ~incoming;
	_delay_us(832);  // shift to next bit
	//check for long space, return break if found
	//so what about parity?	
	for(timer_tick=0;timer_tick< 77; timer_tick++)
	{	//wait for stop bit
		if(sdi_port()== MARK)
			break;  //found stop 
		_delay_us(104);		//1/8 baud
	}
	//uPulseLED(10);
	if(timer_tick==77)
	{  //8msec space,   thats bad
		return FAIL;  //STOP BIT NOT FOUND
	}
	if(timer_tick> 16 && incoming ==0)
		return BREAK;  //>16 = 1.68msec + 8.3msec 1 baud = 10msec space
	d = incoming;
	incoming &= 0x7F;  //clear off parity bit
	p=0;
	for(timer_tick=0;timer_tick<8;timer_tick++)
	{	//count logic 1 bits for parity
		bit = d & 0x01;
		if(bit==0x01) p++;
		d = d>>1; //right shift
	}
	if(p % 2 == 0)
	{ //even parity- OK
		return TRUE; 
	}
	else
	{
		return PARITY_ERROR;
	}
	 
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: sdi_port()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief read sdi-port, if needed, change direction
	

	ON ENTRY:  

	ON EXIT:returns state of line, either mark or space 

	GLOBALS:

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void sdi_port_recieve()
{
	uint8_t r;
		r= PORTB;
		r &=  ~(1<<PORTB0);
		PORTB = r;   //force data to zero so pullup is not enabled.
		DDRB &= ~(1<<PORTB0); //set port as input pin
		
		r= PORTB;
		r &= ~SDI_TRANSMIT;  //set to receive mode just in case
		PORTB =r;
		
		_delay_us(1);

}
int8_t sdi_port()
{
//debug force return SPACE;
	uint8_t r;
	r = PINB;   //get data
	r &= (1<<PORTB0);  //clear all except lsb
	return r;

}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: get_SDI_address()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief get address data and validate
	

	ON ENTRY:  

	ON EXIT:address string and status 

	GLOBALS:

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int8_t get_SDI_address()
{
	volatile int8_t status;

	incoming=0x7F;   //stupid data kills stale 	
	status = get_SDI_data();
	if(status==true)
	{
		if(incoming == eeprom_read_byte((uint8_t*)EEPROM_SDI_ADDRESS) || incoming == '?')
		{	//either correct address or wildcard char
			address_string[0]=eeprom_read_byte((uint8_t*)EEPROM_SDI_ADDRESS),address_string[1]=0;   //make a string and store
			return VALID_ADDRESS;
		}
		return INVALID_ADDRESS;   //something else, bad data 
	}
	return status;   //signal its just a timeout
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: get_SDI_command()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief command data and validate
	

	ON ENTRY:  

	ON EXIT:command type and status 

	GLOBALS:

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int8_t get_SDI_command()
{
	int8_t i=0;
	for(i=0;i<17;i++)
		incomingStr[i]=0;  //erase stale data
	i=0;
	do{
		if(get_SDI_data()== true)// provide escape if marking? PvB
		{
			incomingStr[i]= incoming;i++;
			if(incoming == '!' )
			{
				simulation_count--;  //debug
			break;
			}
		}
	}while(i<17);
	//check for buffer overun
	if(i>=17)
	return INVALID_COMMAND;
	if(i==1)
	{		//ACKNOWLEDGE ACTIVE
		sdi_cmd = a;
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'I' && i == 2)
	{		//SEND IDENTIFICATION
		sdi_cmd =  aI;
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'A' && i == 3)
	{		//CHANGE ADDRESS
		sdi_cmd =  aAb;  //
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'M' && i == 2)
	{		//MEASURE 
		sdi_cmd =  aM;  //
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'M' && incomingStr[1]== 'C' && i == 3)
	{		//MEASURE
		sdi_cmd =  aMC;  //
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'D' && i == 3)
	{	//SEND DATA NUMBER x
		sdi_cmd =  aDx;  //
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'X' && incomingStr[1]=='F' && i == 3)
	{	//fast charge
		sdi_cmd =  aXF;  //
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'X' && incomingStr[1]=='P' )
	{	//program eeprom
		sdi_cmd =  aXP;  //
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'X' && incomingStr[1]=='X' && i == 8)
	{	//fast read eeprom
		sdi_cmd =  aXX;  //
		return VALID_COMMAND;
	}
	else if(incomingStr[0]== 'X' && incomingStr[1]=='R' )
	{	//resetsoftware
		sdi_cmd =  aXR;  //
		return VALID_COMMAND;
	}
	
	return INVALID_COMMAND;
	
}
void delay_832us(int16_t del)
{
		_delay_us(833);
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: send_SDI_string()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief send a string out the sdi port
	

	ON ENTRY:  

	ON EXIT:

	GLOBALS:

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void send_SDI_string(char *data_string)
{
	uint8_t i,j,d,r;
	volatile uint8_t bit,p;
	//set port to mark for 8.33 sec.
	DDRB |= 0x1; //set port as output pin
	r= PORTB;
	r &= ~((1<<PORTB1) | (1<<PORTB0));   
	r |= SDI_TRANSMIT + MARK;//set enable transmitt=1 + marking=0v) 
	PORTB = r;
	delayCRADA(9); //signal to controller: ready to transmitt  
	for(i=0;i<strlen(data_string);i++)
	{
		//****************** calculate and set parity
		//StatusLED(true);
		d=data_string[i]; //word to transmitt
		p=0;   //parity clear
		for(j=0;j<7;j++)
		{	//count logic 1 bits for parity
			bit = d & 0x01;
			if(bit==0x01) p++;
			d = d>>1; //right shift
		}
		d=data_string[i]; //word to transmitt
		if(p % 2 == 0)
		{ //even #bits
			d &= 0x7F;  //clear high bit = parity bit zero
		}
		else
		{  //odd high bits so add one more so sum is even
			d |= 0x80; //set high bit parity bit 1
		}
		//*************** begin transmission of byte	
		r=PORTB;
		r &= ~(1<<PORTB0);  // START bit set to space = 0= 3volt// shorten to portb = r & 0x7e | START)
		r |= SPACE;
		PORTB = r;
		delay_832us(832) ; //1 baud
		for(j=0;j<8;j++)
		{		// send 8 bits of data
			bit = d & 0x01;
			if(bit==0x01)
			{  //transmitt a mark bit
				r=PORTB;
				r &= ~(1<<PORTB0);  // mark  = 1 logic 0volt// shorten to portb = r & 0x7e | START)
				r |= MARK; //zero volt
				PORTB = r;
				p++;
			}
			else
			{  //bit is 0 logic transmitt a space bit
				r=PORTB;
				r &= ~(1<<PORTB0);  // space = 0// shorten to portb = r & 0x7e | START)
				r |= SPACE;	//3 volts			
				PORTB = r;
				p++;
			}
			d = d>>1; //right shift for next bit
			delay_832us(832) ; //1 baud
		}
		r=PORTB;
		r &= ~(1<<PORTB0);  // Stop bit set to mark = 0// shorten to portb = r & 0x7e | START)
		r |= MARK;
		PORTB = r;
		//StatusLED(false);
		delay_832us(832) ; //1 baud stop bit time
		_delay_us(400);  
		delay_832us(832) ; //1 baud  a little space between chars, less than 1.664msec per sdi spec
		
		
	}
	// last word in string sent so set to receive mode
	r= PORTB;
	r &= ~SDI_TRANSMIT;
	r &= ~(1<<PORTB0);
	r |=  MARK;//clear enable transmit + marking)
	PORTB = r;
	
	DDRB &= 0xFE;  //bit 0 back to input mode, ready to receive

}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	TITLE: get Measurement data()

	ORIGINATOR: Peter van Bavel

	DESCRIPTION: \brief get temperatures format output
	

	ON ENTRY:  

	ON EXIT:status?		

	GLOBALS:object_temp_str, ambient_temp_str,vbatt_str

	Dates:
	S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void get_Measurement_data()
{
	char temp[10];
	volatile int8_t i;
				ControlBatteryCharger(trickle);   //force charger top-off so we get a clean reading with no noise
				// build data string  c is "minute" here its ONLY 0,
				if(vbatt2<0)
					vbatt2=0;   //no negatives!per mike, if it happens, truncate to zero
				//convert vbatt to string, forcing decimal point
				sprintf((char*)temp,"%3.0d",vbatt2);
				for(i=0;i<2;i++)
				{	//convert leading spaces to ascii zero's
					if(temp[i]==0x20)
						temp[i]='0';
				}
				
				vbatt_str[0]='+';  //always +
				vbatt_str[1]= temp[0];
				vbatt_str[2]= temp[1];
				vbatt_str[3]='.'; //decimal point
				vbatt_str[4]= temp[2];//xx.x string
				vbatt_str[5]=0;  //terminate
			ControlBatteryCharger(charger_off);  //charger is off during measurements
			//StatusLED(true);   //turn on LED during measurements
			IRTWakeup();  //HAS BUILT IN 300MSEC DELAY
			delayCRADA(141);   //CALIBRATE SO TOTAL IS 2.00SEC SO THAT TIMING IS ACCURATE, manually done 9/30/13
			//StatusLED(false);   //turn off LED during measurements per mike 12/16/13
			getReading(0);  //throw away measurement, iir filter not engaged
			/*if(handleIncomingPacket()==true)  
			{//incoming sample command--- exit  1 minute loop.
				//i=0;
				break; //go send data
			}*/
			delayCRADA(1260);   //wait for new measurementj, data sheet for IRT calls for 1.33 sec settling time (Melexis p13) manually calibrated  to give 2.00 sec sample time 1/22/15
			getReading(0);  //*************** get final temperature from sensor!   i is index where temperature data is stored PvB
			ControlBatteryCharger(charge_mode);  //resume charging whatever current mode is.
			IRTSleep();    //send sleep command to IRT
			terminateCharge();  //control charging algorithm
		//typ 70degF = 21.1111 -> 0x83f = 2111 millidegC
			irTamb = irTamb_accum[0];	//transfer to output accumulators
			irTobj = irTobj_accum[0];
				//Object Temperature
				if(irTobj>12000L)
				strcpy((char*)object_temp_str,"+999.99");//999.99 damn hot!
				else if(irTobj< -4000L)
				strcpy((char*)object_temp_str,"-200.00");
				else
				{ //in linear range convert to string, forcing decimal point
					sprintf((char*)temp,"%6.0d",(int16_t)irTobj);  // convert temperature
					for(i=0;i<6;i++)
					{	//convert leading spaces or - signs to ascii zero's
						if(temp[i]==0x20 || temp[i]== '-')
							temp[i]='0';
					}
					if(irTobj<0)  //force sign character at beginning
						temp[0]='-'; //negative
						else
						temp[0]='+';  //positive
					strncpy((char*)object_temp_str,(char*)temp,6);  //integer portion
					object_temp_str[4]='.';  //insert decimal effectively multiplying by 100
					object_temp_str[5]=temp[4];  //shift decimal digits
					object_temp_str[6]=temp[5];
					object_temp_str[7]=0;//null term*/
				}
				
				//Ambient sensor temp
				if(irTamb>12000L)
				strcpy((char*)ambient_temp_str,"+999.99");// hot!
				else if(irTamb< -4000L)
				strcpy((char*)ambient_temp_str,"-200.00");
				else
				{//in linear range convert to string, forcing decimal point
					sprintf((char*)temp,"%6.0d",(int16_t)irTamb);  // convert temperature
					for(i=0;i<6;i++)
					{	//convert leading spaces to ascii zero's
						if(temp[i]==0x20|| temp[i]== '-')
							temp[i]='0';
					}
					if(irTamb<0)  //force sign character at beginning
						temp[0]='-'; //negative
						else
						temp[0]='+';  //positive
					strncpy((char*)ambient_temp_str,(char*)temp,6);  //integer portion
					ambient_temp_str[4]='.';  //insert decimal effectively multiplying by 100
					ambient_temp_str[5]=temp[4];  //shift decimal digits
					ambient_temp_str[6]=temp[5];
					ambient_temp_str[7]=0;//null term*/
	

				}
				

}
int8_t asciitohex(int8_t in)
{
	
	if(in>= 'A' && in <= 'F')
		return in-0x37;
	if(in >= '0' && in <= '9')
		return in - '0';
	return 0;//0xFF to signal error?
}
char hextoascii(int8_t in)
{
	if(in>9 && in <= 0xF)
		return in + 0x37;
	else if(in >= 0 && in <=9)
		return in+0x30;
	return '?';
}
