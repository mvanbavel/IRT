/**
* \file CRADA UDSA.c
* \brief CRADA Dynamax IRT module main program

*
* Copyright (c) Dynamax Inc, all rights reserved, 6/25/2013
*
* Created: 6/25/2014
* \author: Peter van Bavel
*
*/


/**********************************************************************
 * 
 * modifications by Marcus van Bavel
 *
 *
 */

/************************************************************************/
/**
* \ingroup CRADA_Dynamax_Main
* 
*
* @{
	*/



//not much use! #include "compiler.h"   //from mega_gpio_example
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

// Melexis Memory Addresses are actually SCI commands PvB
#define T_AMB 0x06       // read command for case/ambient Temperature PvB
#define T_OBJ 0x07       // read command for Object viewed Temperature PvB
#define NUMBEROFSAMPLES 6   //take this many samples in an instant to average temperature


// ********************** prototypes
// *************** function prototypes PvB ****************
void sleep(void);
unsigned int readIRTmem(unsigned char);
unsigned char readSwBank(void);
void getReading(unsigned char);
void SwitchIRTtoSMB(void);

extern void configAVR();
extern void PowerMelexis(bool on);
extern void StatusLED(bool on);
extern void delayCRADA(double ms);   //local wrapper for delay functions, convert to interrupt driven with sleep cycles to save power
extern unsigned int read_adc(unsigned char adc_input_channel);
extern void ControlZigBee(bool resetZ,bool sleepZ);
//extern void ControlLinearSupply(bool on);  //probably obsolete,schematic changed. ? PvB

// **************************
extern bool uart_char_waiting();
extern void uart_init(void);
extern char test_string[];
extern void uart_putchar(uint8_t data);
extern char uart_getchar();
extern void ControlBatteryCharger(enum charge_mode_type);
extern void spi_put_word( uint16_t dataword); 
extern uint8_t i2c_readWord ( void *const data, const uint8_t n, uint8_t DeviceAddress, uint8_t *const data_out, const uint8_t m);
extern void IRTSleep();
extern void IRTWakeup();
extern void ToggleLED();
extern void PulseLED(uint8_t code);
extern void CodeLED(uint8_t code);
extern void sleepCRADA(uint16_t sleep_time);
extern void wakeupCRADA();
extern uint8_t crc8_pic(int8_t old_crc,int8_t newbyte);
extern int16_t readBattTemp();
extern int16_t readBattVolts();
extern void terminateCharge(void);
extern void OutputDAC(int32_t temperature);
extern void sleepDAC();
extern bool handleIncomingPacket();
extern void checkBattery();
extern void API_Tx(void);
extern void API_AT(unsigned char C1, unsigned char C2, unsigned int value);

extern void SDI(void);   


const char eeprdata[] __attribute__ ((section (".eeprom"))) =
{0x78, 0x56, 0x34, 0x12,
	0x17, 0x00,
	0x00,
	0x00,
	0x01, 0x00,
	0x29, 0x59,
	0x01,
0x01,
0x00, 0x1e };
const char fusedata[] __attribute__ ((section (".fuse"))) =
#ifdef WATCHCRYSTAL
	{0x62,0xD5,0xF9};  //use internal rc oscillator
#else
    {0xFB, 0xD5, 0xF9};//use 2MHz external oscillator
#endif

// ************************ global variables
// global memory declarations PvB
volatile int32_t irTobj, irTamb,irTobj_accum[NUMBEROFSAMPLES],irTamb_accum[NUMBEROFSAMPLES],irTobj_out,irTamb_out;
volatile char XBstr[100];             // string to be transmitted
//char partStr[30];
char XBAPIstr[255];         // API Packet
volatile char temp[10];		//debugging buffers
volatile char temp2[10];
volatile char temp3[10];
volatile char temp4[10];
volatile uint8_t I;  //used to debug for loops

volatile uint8_t command[40];		//input command buffer
uint16_t vbatt2;
volatile uint16_t delayLoop;
bool sampleCommand;
int8_t input_count;
int8_t numberofSamples;
volatile int8_t output_index;   //tells when to output data
uint16_t vbat_calibration;  //stores vbat calibration value from eeprom
volatile enum charge_mode_type charge_mode;
volatile int16_t thermistor_voltage,old_thermistor_voltage; 
volatile uint16_t NIMH_charge_counter;


static volatile char copyright[]="Copyright Dynamax Inc, all rights reserved, 4/20/2016\r";
char *date_p=&copyright[44];


/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:  main

ORIGINATOR:  PvB

DESCRIPTION: \brief main execution loop, its all here!

ON ENTRY:

ON EXIT: never

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

int main(void)
{
	volatile uint8_t i,j, c;
	volatile uint16_t errorTimer;  //only used for debugging of resets
	uint16_t node_id;
	volatile int16_t test;  //used to test
	bool	after_reset= true;   //signal that a reset ocurred. 
	cli();
#ifdef WATCHCRYSTAL
		CLKPR = (1<<CLKPCE);   //SETUP TO WRITE TO CLOCK SPEED
		CLKPR = (1<<CLKPS1);  //DIVIDE 8MHz RC oscillator by 4 = 2MHz clock
#endif
	errorTimer=0;
	i = MCUSR;   //read cpu reset source codes
	j = i & (1<<PORF);
	if(j != 0)
	{
		MCUSR = 0;
		errorTimer=0;
		i= 0; //clear both low voltage resets 
	}
	j = i&(1<<WDRF);
	if(j != 0)
		errorTimer=5;  //watchdog
	j = i&(1<<BORF);
	if(j != 0)
		errorTimer=10; //brownout
	j = i&(1<<EXTRF);
	if(j != 0)
		errorTimer=20; //external
	configAVR();  //88ms

	//PulseLED(5);
	if(eeprom_read_byte((uint8_t *)EEPROM_RADIO_OUTPUT)==true)
		uart_init();  //skip this to save 1 second if radio not used.
	//not used tc_timeout_init();   //setup timers
	
	
	sei();   //enable valid interrupts
	//PulseLED(7);	
	StatusLED(false);   //turn off LED during initialization
	

	
	NIMH_charge_counter=0;  //reset global fast charge counter	
	if(eeprom_read_byte((uint8_t *)EEPROM_SOLAR_PANEL)==01)
	{
		charge_mode=top_off;
	}
	else if(eeprom_read_byte((uint8_t *)EEPROM_SOLAR_PANEL)==02)
	{
		charge_mode=charger_off;
	}
	else
	{//not a solar panel so trickle, this is really a crude battery management system..
		charge_mode=trickle;
	}
	ControlBatteryCharger(charge_mode);   
	//PulseLED(10);
	vbat_calibration = eeprom_read_word((uint16_t*)EEPROM_VBAT_CALIBRATION);  //GET calibration set at factory
	read_adc(7); //a dummy read to tickle the adc and get it running.
	delayCRADA(100);//was 5000);  //wait for battery to stabilize, ADC to stabilize
	checkBattery();	//halt here if battery too low
	//PulseLED(15);
	//get node id from eeprom for zigbee
	node_id=eeprom_read_word((uint16_t*)EEPROM_SENSOR_ID); //now 16bit hex
	ControlZigBee(true,false);       //Send reset to ZigBee radio (3.3v)
	//PulseLED(20);
	
	SwitchIRTtoSMB();  //Infra Red sendor: force SMB communications, as opposed to PWM mode 20mS
	spi_put_word(0x4000);   //force DAC chip into powerdown mode with 1K load to ground for static protection..  if enabled by eeprom variable will stay powered up.
	
	output_index = eeprom_read_word((uint16_t*)EEPROM_UPDATE_INTERVAL);// PRELOAD output index used for loop timing
	output_index--;  //pre decrement as if it had gone through the loop once to make logic work
	irTamb_out=irTobj_out=0L;   //start output accumulators
	input_count=0;
	old_thermistor_voltage = readBattTemp();  //start dT/dt 
	if(eeprom_read_byte((uint8_t *)EEPROM_SDI_MODE)==true)
		SDI();   //jumps here and never exits
	//**************  HERE IS WHERE THE MAIN EXECUTION LOOP BEGINS!  AFTER A POWER RESET, IT STAYS HERE FOREVER! *****
	while(1)
	{	//top level measurement loop, runs forever never stops, at end of loop sends data to outside world
		numberofSamples = NUMBEROFSAMPLES;  
		if(output_index== -1)
			numberofSamples = 1;  //UPDATE INTERVAL 0, SO DO AT LEAST ONE SAMPLE
		sampleCommand=false;   //no sample command received yet
		handleIncomingPacket();	//CHECK FOR POSSIBLE 'S' received during radio transmission
		ControlZigBee(false,true);  //put in sleep mode PvB
		// ************** get temperature measurements *******************
		for(i=0;i<numberofSamples;i++)
		{
			ControlBatteryCharger(charger_off);  //charger is off during measurements
			StatusLED(true);   //turn on LED during measurements
			IRTWakeup();  //HAS BUILT IN 300MSEC DELAY
			delayCRADA(141);   //CALIBRATE SO TOTAL IS 2.00SEC SO THAT TIMING IS ACCURATE, manually done 9/30/13
			StatusLED(false);   //turn off LED during measurements per mike 12/16/13
			getReading(0);  //throw away measurement, iir filter not engaged
			//PulseLED(3);
			if(handleIncomingPacket()==true)  
			{//incoming sample command--- exit  1 minute loop.
				//i=0;
				break; //go send data
			}
			delayCRADA(1260);   //wait for new measurementj, data sheet for IRT calls for 1.33 sec settling time (Melexis p13) manually calibrated  to give 2.00 sec sample time 1/22/15
			getReading(i);  //*************** get final temperature from sensor!   i is index where temperature data is stored PvB
			if(eeprom_read_byte((uint8_t *)EEPROM_DAC_OUTPUT)==true && after_reset == true)
			{	// the first DAC output after a power on.
				OutputDAC(irTobj_accum[0]);//get the raw first temperature
			}
			after_reset = false;   //from now on we don't update DAC until the normal end of the sequence
			ControlBatteryCharger(charge_mode);  //resume charging whatever current mode is.
			IRTSleep();    //send sleep command to IRT
			terminateCharge();  //control charging algorithm
			sleepDAC();  //DAC goes to sleep why is this here? PvB
			//sleepCRADA(8);  //8 seconds sleep
			if(handleIncomingPacket()==true)  
			{//incoming sample command--- exit  1 minute loop.
				//i=0;
				break; 
			}
			//StatusLED(false);
					
			if(eeprom_read_byte((uint8_t *)EEPROM_SERIAL_COMMAND_MODE)==true)
			{
				if(output_index < 1 && i==0)
				{   //540 second delay on the last delay before output to radio
#ifdef WATCHCRYSTAL
					for(j=0;j<7 ;j++) //revB board with 32KHz crystal
					{ 
						sleepCRADA(1);   //sleep 1 sec
						if(handleIncomingPacket()==true)  //check for incoming sample command---reorder handle and delay to even out sampling
						{ //stop sleep cycle
							i=0;   //restart sample loop, don't know how this works..PvB
							break; //exit sleep loop
						}
					}
					delayCRADA(428);  //tuned to compensate for ack for one unit network serial #9997 2/16/2015 using 10second interval=0
#else //revA board					
					for(j=0;j<6 ;j++)
					{ 
						sleepCRADA(1);   //sleep 1 sec
						if(handleIncomingPacket()==true)  //check for incoming sample command---reorder handle and delay to even out sampling
						{ //stop sleep cycle
							i=0;   //restart sample loop, don't know how this works..PvB
							break; //exit sleep loop
						}
					}
					delayCRADA(888);  //tuned to compensate for ack for one unit network - 7 packet delays
#endif					
				}
				else //not the last delay before output, a normal delay cycle
				{
#ifdef WATCHCRYSTAL // revB
						for(j=0;j<8 ;j++)
					{ //8 second delay
						sleepCRADA(1);   //sleep 1 sec
						if(handleIncomingPacket()==true)  //check for incoming sample command---reorder handle and delay to even out sampling
						{ //stop sleep cycle
							i=0;   //restart sample loop, don't know how this works..PvB
							break; //exit sleep loop
						}
					}
					//delayCRADA(459);  //compensate for incoming packet delays
#else		//revA 
						for(j=0;j<7 ;j++)
					{ //8 second delay
						sleepCRADA(1);   //sleep 1 sec
						if(handleIncomingPacket()==true)  //check for incoming sample command---reorder handle and delay to even out sampling
						{ //stop sleep cycle
							i=0;   //restart sample loop, don't know how this works..PvB
							break; //exit sleep loop
						}
					}
					delayCRADA(459);  //compensate for incoming packet delays
#endif					
				}
			}	
			else
			{	//serial command mode off, so do one single 8 sec delay without checking for commands every second.
				sleepCRADA(8);
				if(handleIncomingPacket()==true)  //check for incoming sample command---reorder handle and delay to even out sampling
				{ 
					//i=0;   //restart sample loop
					break;   //force exit from sample loop
				}
			}				
			//PulseLED(10);
		}
			
		//typ 70degF = 21.1111 -> 0x83f = 2111 millidegC
		irTamb=0;
		irTobj=0;
		for(i=0;i<numberofSamples;i++)
		{
			irTamb += irTamb_accum[i];	//add to output accumulators
			irTobj += irTobj_accum[i];
		}		   
		irTamb/=numberofSamples;
		irTobj/=numberofSamples;
		irTamb_out += irTamb;
		irTobj_out += irTobj;
		


		if(output_index < 1)
		{		//requested interval is over, time to output data to radio

			//PulseLED(errorTimer);  //indicate what caused the previous reset
			StatusLED(true);  //pd7  turn on LED
			output_index = eeprom_read_word((uint16_t*)EEPROM_UPDATE_INTERVAL);//reset  timer for output averaging and messaging
			if(output_index!=0 && sampleCommand==false)
			{  //normal case interval >0 
				irTamb = irTamb_out/(int32_t)output_index;   //divide out the number of 1 minute samples accumulated for the average
				irTobj = irTobj_out/(int32_t)output_index;
			}
			else
			{  //index zero, or sample command received so no division
				irTamb = irTamb_out;
				irTobj = irTobj_out;
			}			
			irTamb_out=0L;	//restart accumulators
			irTobj_out=0L;
			
			
			if(eeprom_read_byte((uint8_t*)EEPROM_RADIO_OUTPUT)==true)
			{
				//GET battery voltage
				ControlBatteryCharger(trickle);   //force charger top-off so we get a clean reading with no noise
				checkBattery();  //built in 100msec delay, reads voltage into vbatt, turns off fast charge if needed
				// build data string  c is "minute" here its ONLY 0,
				if(vbatt2<0)
				vbatt2=0;   //no negatives!per mike, if it happens, truncate to zero
				//convert vbatt to string, forcing decimal point
				sprintf((char*)temp,"%3.0d",vbatt2);
				
				temp2[0]=temp[0];  //copy msdigit
				temp2[1]= temp[1];
				temp2[2]='.'; //decimal point
				temp2[3]= temp[2];//xx.x string
				temp2[4]=0;  //terminate
				
				//Object Temperature
				if(irTobj>12000L)
				strcpy((char*)temp3,"+999.99");//999.99 damn hot!
				else if(irTobj< -4000L)
				strcpy((char*)temp3,"-200.00");
				else
				{ //in linear range convert to string, forcing decimal point
					sprintf((char*)temp,"%+6.0d",(int16_t)irTobj);  // convert temperature
					strncpy((char*)temp3,(char*)temp,4);
					temp3[4]='.';
					temp3[5]=temp[4];
					temp3[6]=temp[5];
					//doesn';t work..  strncat(&temp3[5],&temp[4],2);
					temp3[7]=0;  //null term
				}
				
				//Ambient sensor temp
				if(irTamb>12000L)
				strcpy((char*)temp4,"+999.99");// hot!
				else if(irTamb< -4000L)
				strcpy((char*)temp4,"-200.00");
				else
				{//in linear range convert to string, forcing decimal point
					sprintf((char*)temp,"%+6.0d",(int16_t)irTamb);  // convert temperature
					strncpy((char*)temp4,(char*)temp,4);  //integer portion
					temp4[4]='.';
					temp4[5]=temp[4];
					temp4[6]=temp[5];
					//doesn't work strncat(&temp4[5],&temp[4],2);//decimal portion
					temp4[7]=0;//null term
				}
				
				sprintf((char*)XBstr,"%i,%4.0X,%s,%s,%s\r",input_count++,node_id,(char*)temp2,(char*)temp3,(char*)temp4);
				if(input_count>=10)
				input_count=0;  //rollover
				//"r/" means line feed
				// concatenate each string into one large one
				//strcat(XBstr,partStr);
				//PulseLED(4);
				test = 10*(0x0f & eeprom_read_byte((uint8_t*)EEPROM_SERIAL_NUMBER));   //RANDOM DELAY TO PREVENT COLLISIONS PER MIKE(even though it takes ~60msec so send data
				delayCRADA(test);   //RANDOM DELAY TO PREVENT COLLISIONS PER MIKE(even though it takes ~60msec so send data

				//PulseLED(5);
				ControlZigBee(false,false);   //turn off sleep mode pin, wakeup radio  
				API_Tx(/*strlen(XBstr)*/);		// transmit string to radio, all chars must be ascii, no nulls
				delayCRADA(200-test); //normalize random delay so total delay is 200msec, worst case for "test" is 150, so this should be no less than 50
				//PulseLED(5);

			}
			if(eeprom_read_byte((uint8_t *)EEPROM_DAC_OUTPUT)==true)
			{
				OutputDAC(irTobj);
			}
			StatusLED(false);  //pd7  turn off LED
	
		}	//end of output process, return to data accumulation

		output_index--;   //count down for next sample		

		ControlZigBee(false,true);  //put radio in sleep mode so its off during delay PvB

		for (c=0;c<NUMBEROFSAMPLES;c++)     // reset short term accumulated values & counts
		{
				//irTobj_count[c] =0.0;
				//irTamb_count[c] =0.0;
			irTobj_accum[c] =0.0;
			irTamb_accum[c] =0.0;
		}
	}  //end of endless loop
}



/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:  Fuse documentation

ORIGINATOR:PvB

DESCRIPTION: \brief list of fuses and their required settings.
NOTE this is for documentation only, no code is written,
Could be used as a basis for setting fuses in the device programmer
[X] means programmed = 0.   NOTE the logic inversion!!!!!!
[ ] means unprogrammed = 1

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*BOOTSZ = 1024W_0C00   size of boot sector, not used
extended byte
BOOTRST = [ ]
end extended
start High
RSTDISBL = [ ]
DWEN = [X]    set for debugWire NOTE! if dwen is set, spien is non-functional regardless of its fuse!
SPIEN = [X]		programmed for serial port download enable.  
WDTON = [ ] watchdog not always on, unprogrammed
EESAVE = [X]   required for debugWire to work
BODLEVEL =[x][][x]= 101 = 2.7 volts brownout level
end high  
start low byte
CKDIV8 = [ ]  bit 8 clock runs at full speed of crystal or at least 1/2 of nominal crystal freq.
CKOUT = [ ] bit7  if set clock comes out PortB0, not normally a good idea, that is SDI RX/TX
SUT = bit6,bit5
CKSEL = bits3-0
SUT_CKSEL = EXTCLK_6CK_14CK_65MS   external clock from STK500, CKSEL = 0000, 6clks delay from powerup, 65ms delay from reset SUT= 0x10
end low

EXTENDED = 0xF9 (validx)  three bytes in mega88pa
HIGH = 0x95 (validx)  =D5 for production, turns off DWEN
LOW = 0xFB (valid)   was E0 for stk500 62 for 32khz*/
/*const char lockbits[] __attribute__ ((section (".lockbits"))) = 
    {0xFC};
const char userdata[] __attribute__ ((section (".user_signatures"))) = 
    "Hello User Signatures";*/



/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:  getReading(unsigned char c)

ORIGINATOR:  Susan, header by PvB

DESCRIPTION: \brief get temperature of object in degC*100
, add to accumulated values in irTxxx_accum
do same for temperature of body or ambient.

ON ENTRY:   index to accumulator, WARNING c must be from zero to 5, or disaster

ON EXIT:code now does nothing unique if I2C returns zero, no warning given PvB

GLOBALS:irTobj_accum,irTamb_accum arrays

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void getReading(uint8_t index)
{
	volatile int32_t x;  //for debug remove later PvB
	x = readIRTmem(T_OBJ);
	x= x*2L - 27300L;
	irTobj_accum[index]= x;  //old code added but that was stupid. PvB
	x = readIRTmem(T_AMB)*2L - 27300L;
	irTamb_accum[index]= x;
	
}
/*old code float methodvoid getReading(unsigned char c)
{
	irTobj = (readIRTmem(T_OBJ) * .02 - 273.0);  // 0.02 is gain eqn for Melaxis? PvB
	if (irTobj != 0)  //guard against blank data PvB  what happens if 0? PvB
	{ irTobj_accum[c] = irTobj_accum[c] + irTobj;
		irTobj_count[c] = irTobj_count[c] + 1.0;  //why use a float here? PvB
	}
	
	irTamb = (readIRTmem(T_AMB) * .02 - 273.0);
	if (irTamb != 0)
	{ irTamb_accum[c] = irTamb_accum[c] + irTamb;
		irTamb_count[c] = irTamb_count[c] + 1.0;
	}
}*/

/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   readIRTmem(unsigned char command)

ORIGINATOR: Susan, header by PvB

DESCRIPTION: \brief Read temperature
from Melaxis sensor using I2C interface

ON ENTRY:Command is understood by Melaxis, in this case probably Tamb or Tobj

ON EXIT: integer temperature in degrees Kelvin, except failure of I2C return zero.

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
unsigned int readIRTmem(unsigned char command)
{
	//all replaced by one call to i2c-read PvB
	uint8_t temp_buffer[4],temp_readBuffer[4];   //place to store data and CRC
	uint8_t crc,failCount=0;

	//load with address and command( which better be a read address!)
	while(1)
	{
		temp_buffer[0]=0;   //slave address, good for all devices see page 14 melaxis spec
		temp_buffer[1]=(uint8_t)command;
		crc = crc8_pic(0,0);   //slave address
		crc = crc8_pic(crc,temp_buffer[1]);
		crc= crc8_pic(crc,1);    //slave address + 1 for read command
		//put first three bytes of CRC calculation here

		if(i2c_readWord(&temp_buffer[0],(uint8_t)2,(uint8_t)0,&temp_readBuffer[0],3)==I2C_OK)    //get three reads last byte is CRC not used for now TBD
		{
			crc = crc8_pic(crc,temp_readBuffer[0]);  //check LSB
			crc = crc8_pic(crc,temp_readBuffer[1]);	//check MSB
			if( crc == temp_readBuffer[2])
			{		//crc match send data
				return 256*temp_readBuffer[1]+temp_readBuffer[0];   //combine into 16bit data
				break;   //exit while
			}
		}
		failCount++;   //crc not oK or timeout failure in i2C interface
		if(failCount>10)
		{
			IRTSleep();
			IRTWakeup();  //retry wakeup sequence per mike 1/10/14 in case wakeup from sleep earlier failed. 	
		
			break;  //100 tries to get it right, exit
		}
	}
	failCount=0;
	return 1;  //signal failure to get data						
}
