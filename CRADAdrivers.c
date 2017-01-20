/**
 * \file CRADAdrivers.c
 * \brief CRADA Dynamax low level drivers
	all direct calls to hardware registers are here
 *
 * Copyright (c) Dynamax Inc, all rights reserved, 5/25/2013
 *
 * Created: 3/12/2013 5:14:12 PM
 * \author: Peter van Bavel
 *
*/
/**
 * \ingroup CRADA_Dynamax_Main
 * \defgroup CRADA_Dynamax_Drivers_group CRADA Low Level Drivers
 *
 * @{
 */

#include "compiler.h"   //from mega_gpio_example
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
//prototypes

void configAVR();
void PowerMelexis(bool on);
void StatusLED(bool on);
void delayCRADA(double ms);   //local wrapper for delay functions, convert to interrupt driven with sleep cycles to save power
unsigned int read_adc(unsigned char adc_input_channel);
void ControlZigBee(bool resetZ,bool sleepZ);
void ControlBatteryCharger(enum charge_mode_type);
void spi_interrupt_enable();
void spi_interrupt_disable();
void spi_disable();
void spi_put(uint8_t data);
void spi_put_word( uint16_t dataword);
bool handleIncomingPacket();
void PulseLED(uint8_t code);
void CodeLED(uint8_t code);
int16_t readBattVolts();
void OutputDAC(int32_t temp);
void terminateCharge();
void checkBattery();
int16_t readBattTemp();



void sleepDAC();
#define TRUE	1    //added by PvB
#define FALSE	0
#define ADC_VREF_TYPE 0x42  // Vcc reference, right adjusted, multiplexor = 2( ADC2 is supply voltage )

volatile uint16_t SPI_output_buffer;   //on 16bit output word, transmitted 8 bits at a time
volatile bool spi_done;  //set when interrupt complete
volatile bool wdt_done;

extern volatile uint8_t command[40];
extern 	bool sampleCommand;
extern bool uart_char_waiting();
extern void uart_init(void);
extern void uart_putchar(uint8_t data);
extern char uart_getchar();
extern int8_t input_count;
extern int8_t output_index;
extern int8_t numberofSamples;
extern int32_t irTamb_out;
extern int32_t irTobj_out;
extern uint16_t vbatt2 ;
extern uint16_t vbat_calibration;  //stores vbat calibration value from eeprom
extern volatile enum charge_mode_type charge_mode;
extern volatile int16_t thermistor_voltage,old_thermistor_voltage;
extern volatile uint16_t NIMH_charge_counter;


/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:  configAVR

ORIGINATOR:PvB

DESCRIPTION: \brief configure AVR processor with desired I/O and initialize services.

ON ENTRY:

ON EXIT:hardware is written

GLOBALS:

Dates:1/17/14 change portC4 to output so IRT wakeup will work
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void configAVR()
{

	int8_t junk;   //store worthless junk here 
	// Input/Output Ports initialization
	// Port B initialization
	
	PORTB=0x00;	//data register cleared so outputs are zero.
	DDRB=0x00|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5);	//all except bit 0 outputs, bit6,7 are crystal inputs
	PINB=0x00;  //none are toggle pins

	// Port C initialization

	PORTC=0x00 ;  //drives all outputs to zero
	DDRC=0x00| (1<<DDC0)|(1<<DDC1)|(1<<DDC2)|(1<<DDC3)|(1<<DDC4)|(1<<DDC5); //PC0,PC1,DACEN, THermEN SDA, SCL are outputs, 
	PINC=0;   //no toggle pins
	PORTC=0x00 |(1<<PORTC1)|(1<<PORTC5)|(1<<PORTC4); ;  //IRT clock data driven high, trickle mode charge no pullup on pC6 

	// Port D initialization
	PORTD=0x00 ;  //no pullup resistors
	DDRD=0x00 | (1<<DDD1)|(1<<DDD4)|(1<<DDD5)|(1<<DDD6)|(1<<DDD7);  //RADIORESET is an output
	PIND=0;   //no toggle pins
	PORTD=0x00 | (1<<PORTD5)|(1<<PORTD6);  //radio in sleep, reset, rts off for low power
	PulseLED(2);

	// Timer/Counter 0 initialization
	TCCR0A=0x00;  // OC0A OC0B output: Disconnected, PWM normal mode, top count is 0xFF, bottom 00, rolls over
	TCCR0B=0x00;	//forceoutputcompare A,B cleared, WGM02 = normal pwm, CS0-2 = 0 no clock=timer stopped
	TCNT0=0x00;  //removes compare match( if any)
	OCR0A=0x00;  //compare match zeroed, has no effect
	OCR0B=0x00;	//compare match zeroed, has no effect
	//TIFR0  //set to zero? PvB

	// Timer/Counter 1 initialization
	// Clock value: Timer 1 Stopped
	//
	// OC1A output: Discon.
	// OC1B output: Discon.
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer 1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=0x00;	//Normal port operation, OC1A/OC1B OC1A/OC1B disconnected, Mode: Normal top=FFFFh
	TCCR1B=0x00;	//noise cancel off, negative edge input capture, clock off
	TCNT1H=0x00;	//zero count high word register
	TCNT1L=0x00;	//zero count low word
	ICR1H=0x00;		//capture registers zeroed
	ICR1L=0x00;
	OCR1AH=0x00;	//zero output compare registers high and low, A&B
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization Timer2
	// Clock source: 32Khz if watch crystal installed
	// Clock value: Timer 2 Stopped
	// Mode: Normal top=FFh
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	#ifdef WATCHCRYSTAL
	ASSR = (1<<AS2);   //b6 0x20 means 32KHz crystal is enabled.
	#else
	ASSR = 0;  //no timer2 input at all.. 
	#endif

	// Configure Timer/Counter to CTC mode, and set desired prescaler in control registers
	//b7,b6 Com2ra=0 A output pin disconnected; B5,B4 Com2rB normal, output pin disconnected
	TCCR2A = (1 << WGM21);  //0x2 clear timer on compare match(CTC) mode, counter resets with OCR2A, TOV flag on reset.
	TCCR2B = (1<<CS22)|(1<<CS20);// WGM22 = 0, 32KHz/128 = 250Hz 4msec resolution;
	// Set compare to value for desired tick rate
	OCR2A = 0xfa;  //=250 * 4msec = 1sec timeout!  calculated defines dont work something is overiding them. TC_TIMEOUT_COMP;
	OCR2B = 0xff;  //set to max so it never compares, if interrupt actually cancels tcnt
	TCNT2= 0;   //clearcount to start process
	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// Interrupt on any change on pins PCINT0-7: Off
	// Interrupt on any change on pins PCINT8-14: Off
	// Interrupt on any change on pins PCINT16-23: Off
	EICRA=0x00;	//low level interrupt on INT0, 1
	EIMSK=0x00; //INT0,1 masked off, disabled
	EIFR= (1>>INTF0)|(1>>INTF1); //JUST IN CASE set these to clear them, paradoxical PvB
	PCICR=0x00; //pinchange INT2,1,0 masked off, disabled
	PCIFR=(1>>PCIF0)|(1>>PCIF1)|(1>>PCIF2);  //JUST IN CASE set these to clear them, paradoxical PvB
	WDTCSR = (1<<WDIF);   //FORCE WATCHDOG CONTROL TO DISABLE, CLEAR PENDING

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=0x00;  //off
	TIFR0 = (1<<OCF0A)|(1<<OCF0B)|(1>>TOV0);
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=0x00;
	TIFR1 =(1<<ICF1)| (1<<OCF1A)|(1<<OCF1B)|(1>>TOV1);
	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=0x00;
	TIFR0 = (1<<OCF2A)|(1<<OCF2B)|(1>>TOV2);
	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: Off
	// USART Transmitter: On
	// USART0 Mode: Asynchronous
	// USART Baud Rate: 9600
	UCSR0A=0x00 |(1<<TXC0);// clear status and control reg, normal speed shouldn't it be 0x20 with buffer empty set, and must be double speed? PvB, force Txcomplete off
	UCSR0B=0x00;  //RCV off, TX OFF , not 9bit, will clear RX interrupt, no interrupts enabled
	UCSR0C=0x06;	//Asynchronous USART, no parity,1 stop bit,8bit data
	UBRR0H=0x00;  //high byte of baud rate = 0
	UBRR0L=0x0C;  //low byte of baud rate = 12 this will only work if in double speed mode, which its not at this point? PvB

	//eeprom control initialization
	EECR=0; //default state, interrupt disabled
	//flash memory control initialization
	SPMCSR= 0; //interrupts off
	
	// Analog Comparator initialization


	ACSR=(1<<ACD)|(1<<ACI);// Analog Comparator: Off, interrupt cleared
	ADCSRB=0x00;// Analog Comparator Input Capture by Timer/Counter 1: Off
	//DIDR1 should be zero

	// ADC initialization
	// ADC Clock frequency: 500 kHz
	// ADC Voltage Reference: AVCC pin
	// ADC Auto Trigger Source: None
	DIDR0=0x00;   //disable digital input buffer, should be done to channel 2? PvB
	ADMUX=ADC_VREF_TYPE & 0xff;//Vcc reference, right adjusted, multiplexor = 2( ADC2 is supply voltage )
	ADCSRA=0x81|(1<<ADIF);  //ADC enabled, interrupts off, pending interrupt cleared, clock frequency is 500KHz
	ADCSRB=0;   //free running trigger

	// I2C Bus initialization, a library call? PvB
	TWBR= 0x18;   //divide prescaler by 24, if prescaler is 1, FCPU=2Mhz, then i2c clock will be 83.33KHz
	TWSR &= ~((1<<TWPS0)|(1<<TWPS1));  //PREscaler = 0 means scale by 1
	i2c_disable_intrpt_();
	//not yet i2c_enable_();   //turns on i2c ready for business PvB
	i2c_enable_intrpt_();   //system is interrupt driven PvB
	/*//WARNING"note that
clearing this flag starts the operation of the TWI, so all accesses to the TWI Address Register (TWAR), TWI Status
Register (TWSR), and TWI Data Register (TWDR) must be complete before clearing this flag*/
	i2c_clr_intrpt_flag_(); 
	
	//spi interface used for DAC voltage output
	SPCR = (1<<MSTR)|(1<<CPHA);   //interrupt and spi crkt disabled, clock divider = fclk/4 = 250KHz, TRAILING EDGE SAMPLE
	SPSR = 0;   //CLEAR SP12X SO spi clock is not double speed
	junk = SPCR;  
	junk = SPDR; //clears pending interrupts datasheet 19.5.2
	
	PRR = (1<<PRTIM1)|(1<<PRTIM0);   //turn off timers 1, 0 to save power, NOTE turn on to use!
	SMCR = 0;  //clear all sleep mode bits
	
	
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:  read_adc(unsigned char adc_input)

ORIGINATOR:  Susan, header by PvB

DESCRIPTION: \brief get an ADC reading

ON ENTRY:   channel number on portC

ON EXIT:unsigned int raw ADC value

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

uint16_t read_adc(unsigned char adc_input_channel)
{
	volatile uint16_t temp=0;  //debug counter
	ADMUX=adc_input_channel | (1<<REFS1) | (1<<REFS0);//   USE INTERNAL 1.1V REFERENCE SUPPLY,  set channel as requested
	// Delay needed for the stabilization of the ADC input voltage
	_delay_us(10);
	// Start the AD conversion //ADC enabled, interrupts off, clock frequency is 500KHz  systemclock/2
	ADCSRA|=0x40;	//set convert start bit PvB
	// Wait for the AD conversion to complete
	while((ADCSRA&0x40)==0x40)
		temp++;    //wait for adc to convert
	//while ((ADCSRA & 0x10)==0);  //old codeADC interrupt flag?
	//ADCSRA|=0x10;	//why? PvB
	return ADCW;  //16bit read? PvB
}

/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   PowerThermistor( on)

ORIGINATOR: PvB

DESCRIPTION: \brief turn on or off thermistor bias voltage

ON ENTRY:boolean yes= on, no = off

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void PowerThermistor(bool on)
{
	
	if(on==TRUE)
	PORTC |= (1<<PORTC3);       //Send Power to thermistor by turning on OPA341
	else
	PORTC &= ~(1<<PORTC3);    //turn off power to OP amp
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   StatusLED( on)

ORIGINATOR: PvB

DESCRIPTION: \brief turn on or off status LED

ON ENTRY:boolean yes= on, no = off

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void StatusLED(bool on)
{
	if(on==TRUE)
	{
	PORTD &= ~(1<<PORTD7);}
	else
	PORTD |= 1<<PORTD7;    //set bit turns led off
}
void ToggleLED(void)
{
	static volatile bool on;
	if(on==true)
	{
		PORTD |= 1<<PORTD7;    //set bit turns led off
		on=false;
	}
	else
	{
		PORTD &= ~(1<<PORTD7);
		on=true;
	}		
		
}
void PulseLED(uint8_t code)
{
	uint8_t i; 
	for(i=0;i<code;i++)
	{	PORTD |= 1<<PORTD7;    //set bit turns led off
		delayCRADA(1);
		PORTD &= ~(1<<PORTD7); // turns on
		delayCRADA(1);
	}
	delayCRADA(1);   //last led on is 2msec so we can prove that this is an intentional event not a longer pulse code that was interrupted.		
	PORTD |= 1<<PORTD7;    //set bit turns led off
			
}
void CodeLED(uint8_t code)
{
	uint8_t i;
		PORTD &= ~(1<<PORTD7); // turns on
		_delay_us(104);  //1 start bit
	for(i=0;i<8;i++)
	{
		if((code & 0x01)== 1) //test lsb
		PORTD &= ~(1<<PORTD7); // turns on
		else
		PORTD |= 1<<PORTD7;    //set bit turns led off
		_delay_us(104);
		code = code>>1;  //shift right to do the next bit
	}
		PORTD |= 1<<PORTD7;    //set bit turns led off
	_delay_us(104);  //one stop bit
	for(i=0;i<5;i++)
	{//wiggle line to signal end of pulse train 
		PORTD &= ~(1<<PORTD7); // turns on
		_delay_us(5);
		PORTD |= 1<<PORTD7;    //set bit turns led off
		_delay_us(5);
	}		
		
		

	
}
	
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   delayCRADA( ms)

ORIGINATOR: PvB

DESCRIPTION: \brief delay specified msec

ON ENTRY:double # of msec

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void delayCRADA(double ms)
{
	if(ms>6553)
		ms=6553; //limit of 1/10 sec resolution 
	_delay_ms(ms);    //used default until otherwise coded PvB
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   ControlZigBee()

ORIGINATOR: PvB

DESCRIPTION: \brief control all zigbee radio pins 
From zigbee data sheet:
Pin sleep allows the module to sleep and wake according to the state of the Sleep_RQ pin (pin 9). Pin sleep
mode is enabled by setting the SM command to 1.
When Sleep_RQ is asserted (high), the module will finish any transmit or receive operations and enter a low
power state. For example, if the module has not joined a network and Sleep_RQ is asserted (high), the module
will sleep once the current join attempt completes (i.e. when scanning for a valid network completes). The
module will wake from pin sleep when the Sleep_RQ pin is de-asserted (low).

Reset true pulses zigbee reset* low for 1usec.  

ON ENTRY:reset on or off, sleep = on, or sleep off

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void ControlZigBee(bool resetZ,bool sleepZ)
{
	
	if(resetZ==TRUE)
	{     //power on 
		PORTD &= ~(1<<PORTD4);      //Reset* turned on.( low)
		_delay_us(1);   //minimum reset 200nS per zigbee
	}	
	PORTD |= (1<<PORTD4);       //Reset* turned off( high), just in case it was left low for some reason
	if(sleepZ==true)
	{
		PORTD |= (1<<PORTD5);  //request sleep
		PORTD |= (1<<PORTD6);  //CLEAR RTS(rst*=1) TO PREVENT TRANSMISSION WHILE SLEEPING
	}
	else
	{
		PORTD &= ~(1<<PORTD5); //not sleep
		PORTD &= ~(1<<PORTD6);  //set RTS(rst*=0) just in case its needed
	}		

}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   ControlBatteryCharger()

ORIGINATOR: PvB

DESCRIPTION: \brief control battery charger

ON ENTRY:enum charge_mode  off=0, trickle, top_off, fast(not supported by decision MvB)

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void ControlBatteryCharger(enum charge_mode_type mode)
{
	
	if(mode==trickle)
	{	//65mA
			PORTC |= (1<<PORTC1);	//d1=1
			PORTC &= ~(1<<PORTC0);   //d0=0
	}			
	else if(mode==top_off)
	{   //0.375Amp
			PORTC |= (1<<PORTC0);	//d0=1
			PORTC &= ~(1<<PORTC1);   //d1=0
	}
	else
	{//default charger_off fast   //zero
		PORTC &= ~( (1<<PORTC0)|(1<<PORTC1) );   //d1, d0=0
	}
		

};
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   SwitchIRTtoSMB() IRTWakeup()

ORIGINATOR: PvB

DESCRIPTION: \brief send special signal forcing sMB mode

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void SwitchIRTtoSMB(void)
{
	
	i2c_disable_();  //turn off TWI hardware, port pin controls now work
	PORTC |= (1<<PORTC5);  //set clock SCL
	delayCRADA(10);   //not sure this is required spec from melaxis is mum
	PORTC &= ~(1<<PORTC5);  //force clock low
	delayCRADA(10);  //minimum is 1.44msec
	PORTC |=  (1<<PORTC5);  //force clock high thus requesting cmb mode
	i2c_enable_();
	i2c_enable_intrpt_();   //twi hardware ready to go!
}	
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   IRTWakeup

ORIGINATOR: PvB

DESCRIPTION: \brief wakeup irt into default power 
see Melexis data sheet p19 8.4.8.2 Exit from Sleep Mode (Wake up request) 

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void IRTWakeup(void)
{   //from p19 of melexis spec
	i2c_disable_();  //turn off TWI hardware, port pin controls now work
	PORTC |= (1<<PORTC5);  //set clock SCL,assuming it was low
	delayCRADA(1);   //not sure this is required spec from melaxis is mum
	PORTC &= ~(1<<PORTC4);  //force data low, assuming it is high
	delayCRADA(50);   //minimum 33msec WARNING make sure this really meets spec
	PORTC |=  (1<<PORTC4);  //force data high thus waking up sleeping IRT
	delayCRADA(250);//required wakeup period per melaxis
	i2c_enable_();
	i2c_enable_intrpt_();   //twi hardware ready to go!
	
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   IRTSleep

ORIGINATOR: PvB

DESCRIPTION: \brief put IRT to sleep
see Melexis data sheet p19 8.4.8.1

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void IRTSleep(void)
{
	uint8_t temp_buffer[4];   //place to store data and CRC

	//load with address and command( which better be a read address!)
	temp_buffer[0]=0xB4;   //factory slave address(5A left shifted one bit), good for all devices if still factory default!
	temp_buffer[1]=(uint8_t)0xFF;   //sleep command
	temp_buffer[2]=(uint8_t)0xE8;   //  PEC calculated with http://www.smbus.org/faq/crc8Applet.htm
	i2c_write(temp_buffer,(uint8_t)3);   //send three byte command to sleep
	/*  since pullup resistors are hardware this won't work*/i2c_disable_();  //turn off TWI hardware, port pin controls now work
	PORTC &= ~(1<<PORTC5);  //force clock low, might then turn off pullup resistor see Melexis 8.4.8 p 19
	PORTC |=  (1<<PORTC4);  //force data high
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   ISR(SPI)

ORIGINATOR: PvB

DESCRIPTION: \brief handle spi interface interrupts

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ISR(SPI_STC_vect)
{
	
	//assume interrupt caused by completion of the high 8bits of the 16bit word, BTW slave to master transfers better not happen!
	spi_done=true;  //signal to driver that transmission is complete
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   spi_interrupt_enable, spi_interrupt_enable and spi_put

ORIGINATOR: PvB

DESCRIPTION: \brief control spi interface

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void spi_interrupt_enable()
{
	SPCR |= (1<<SPIE)|(1<<SPE);   //READY for interrupt, enable interface
}
void spi_interrupt_disable()
{
	SPCR &= ~(1<<SPIE);   //turn off interrupt, NOTE interface is still enabled, turn off later to save power?
}
void spi_disable()
{
	SPCR &=~((1<<SPIE)|(1<<SPE));  //turn off everything.   
}
void spi_put(uint8_t data)
{
	
	SPDR = data;    //8bits to output register, better be enabled at least; if interrupt enabled will execute ISR when done
}	
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   spi_put_word

ORIGINATOR: PvB

DESCRIPTION: \brief transmitt a 16 bit word on the SPI bus

ON ENTRY: 16bit word to transmitt

ON EXIT:interrupt is enabled, but should not happen..   

GLOBALS:spi_output_buffer

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void spi_put_word( uint16_t dataword)
{
	volatile int8_t fail;
	PORTB |= (1<<PORTB2);   //SET chip select*( synch*) to off = high
	_delay_us(1);  //wait  for minimum of 33nS   TBD after debug to save power keep cs* low 
	spi_done= false;		//clear completion flag
	PORTB &= ~(1<<PORTB2);   //enable CS* = 0 ready for data
	SPI_output_buffer = dataword;   //store payload for use
	spi_interrupt_enable();  //get set
	
	spi_put((uint8_t)(dataword>>8));    //WHAM!  when high byte is complete it interrupts 
	
	fail=0;
	while(spi_done==false)
	{	//high word not sent yet
		_delay_us(1);
		fail++;
		if(fail> 100)
			break;  //failed to exit properly 
	}
	//high byte transmission complete
	spi_done= false;
	spi_put((uint8_t)dataword);  //get low 8bits and slam into output register
	fail = 0;
	while(spi_done==false)
	{	//high word not sent yet
		_delay_us(1); 
		fail++;
		if(fail> 100)
			break;  //failed to exit properly 
	}
	spi_interrupt_disable();
	PORTB |= (1<<PORTB2);   //de-assert chip select* to 1
	_delay_us(50);   //after 50usec, put into lower power state see p17 of DAC data sheet
	PORTB &= ~(1<<PORTB2);   //enable CS* = 0 ready for data
	

}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   sleepCRADA, wakeupCRADA

ORIGINATOR: PvB

DESCRIPTION: \brief put all hardware in minimum sleep mode or wakeup
start interrupt timer so that when expired wakes everything back up.

ON ENTRY: uint16 seconds to sleep

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void sleepCRADA(uint16_t sleep_time )
{
	
		//turn off ADC
	//turn off comparator; never turned on
	//turn off brown out detector,disabled att
	//turn off internal voltage reference -- adc, comparator, brown out off= refer off
	//turn off watchdog timer
	//clear all port pins to zero if possible
	//if charge power available keep charging going
	//go to extended standby mode
	PORTD |= (1<<PORTD6);  //CLEAR RTS(rst*=1) TO PREVENT TRANSMISSION WHILE SLEEPING
	ADCSRA = 0;  //turn off adc
	
	if(sleep_time == 8)
	{
	TCCR2B = (1<<CS22)|(1<<CS21)   |(1<<CS20);   //32.768KHz/1024 = 32Hz or 31.25msec/tick 
	OCR2A = 255; //=   31.25msec *255 = 7.97sec. 
	}
	else
	{
	TCCR2B = (1<<CS22)|(1<<CS20);// WGM22 = 0, 32KHz/128 = 256Hz 3.90625msec tick;
	OCR2A = 248; //=  3.90625msec * 248 = 0.96875 sec( compensate for handling packet delays)  
	}	 
	TCNT2= 0;  //restart counter
	while((ASSR&0x1F) != 0);
#ifdef WATCHCRYSTAL
#ifdef EMULATE
	PRR = (1<<PRADC)|(1<<PRTIM1)|(1<<PRTIM0)|(PRTIM2)|(PRUSART0)|(1<<PRTWI);//  //all off except spi( needed for debug)
#else
	PRR = (1<<PRADC)|(1<<PRTIM1)|(1<<PRTIM0)|(PRTIM2)|(PRUSART0)|(1<<PRTWI)|(PRSPI);//  //all off debug emulator will NOT work!!
#endif //emulate
	//watch crystal mode
	wdt_done=false;   //clear the flag
	TIMSK2 = 1 << OCIE2A;  //0x70=(0x02) enable timer2 compare match interrupt
	SMCR = (1<<SM1)|(1<<SM0)|(1<<SE);   //sleep PowerSave mode on execution of SLEEP command is: power down all clocks, except watchdog and Tasynchronous, wait for interrupt PvB
	asm ("Sleep");			//assembly language sleep command
	//!!!!!!!!!!!!after waking up starts execution here
	SMCR = (1<<SM1)|(1<<SM0);  //disable SLEEP, LEAVE MODE IN POWER save
	TIMSK2 = 0; //disable interrupt
#else
	PRR = (1<<PRADC)|(1<<PRTIM1)|(1<<PRTIM0)|(PRTIM2)|(PRUSART0)|(1<<PRTWI);//  //all off except spi( needed for debug)
	asm ("wdr");   //Assembly directive-- watch dog reset PvB
	WDTCSR=(1<<WDIE)|(1<<WDCE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1);//0x1E;    //WDCE + WDP= 6 =131K prescale 128KHzclk/512K= 1Hz ->1sec timeout
	WDTCSR=(1<<WDIE)|(1<<WDP2)|(1<<WDP1); //0x46;   //WDIE =1, enable interrupt and Reset.  
	
	wdt_done=false;   //clear the flag
	SMCR = (1<<SM1)|(1<<SE);   //sleep PowerDown mode on execution of SLEEP command is: power down all clocks, except watchdog, wait for interrupt PvB
		asm ("Sleep");
		asm ("wdr");   //reset watchdog timer to restart counter
	SMCR = 0;  //disable SLEEP, go to full power "idle " mode
	WDTCSR=(1<<WDIE)|(1<<WDCE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1);//0x1E;    //WDCE + WDP= 6 =131K prescale 128KHzclk/512K= 1Hz ->1sec timeout
	WDTCSR=(1<<WDIE)|(1<<WDP2)|(1<<WDP1); //0x46;   //WDIE =1, enable interrupt and Reset.  
#endif
	PRR= 0; // turn all peripherals back on*/
	
	PORTD &= ~(1<<PORTD6);  //set RTS(rst*=0) just in case its needed
	ADCSRA = 0x81|(1<<ADIF);  //ADC enabled, interrupts off, pending interrupt cleared, clock frequency is 500KHz


}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   ISR(TIMER2_OCR)

ORIGINATOR: PvB

DESCRIPTION: \brief handle TIMER2 TIMED OUT

ON ENTRY:

ON EXIT:wdt_done true, not really needed for anything just to verify and debug

GLOBALS:wdt_done

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ISR(TIMER2_COMPA_vect)
{
#ifdef WATCHCRYSTAL
	wdt_done=true;  //signal to driver that watchdog has timed out
#else
	CodeLED(0x8);  //unwanted timer2 interrupt
	
#endif

}

/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   crc8_pic

ORIGINATOR: PvB

DESCRIPTION: \brief compute 8bit CRC8
using the x8 + x2 + x + 1 polynomial also known as CRC-8-CCITT 


ON ENTRY: uint8 old crc from previous byte, set to zero on first one
			uint8 newbyte which is to be computed and merged with old crc

ON EXIT:new crc result

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int8_t crc8_pic(int8_t old_crc,int8_t newbyte) 
 { 
	int i; 
	int data; 
	 
	data=old_crc^newbyte; //merge old with new
	 
	for(i=0;i<8;i++) 
	{   //scan over 8 bits
	   if((data&0x80)!=0) 
	   { //high bit true
		  data<<=1; //shift left
		  data^=0x07; //exclusive or with polynomial 7
	   } 
	   else 
	   { //high bit zero
		  data<<=1; 
	   } 
	} 
	return(data); 
 }

/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   ISR(WDT)

ORIGINATOR: PvB

DESCRIPTION: \brief handle watchdog interrupts

ON ENTRY:

ON EXIT:wdt_done true, not really needed for anything just to verify and debug

GLOBALS:wdt_done

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ISR(WDT_vect)
{
	//debug CodeLED(0xA);
	wdt_done=true;  //signal to driver that watchdog has timed out
}
	
int16_t readBattVolts()
{
	volatile uint16_t adc_in,adc_in2;
	uint32_t vbatt;

	ControlBatteryCharger(trickle);   //force charger top-off so we get a clean reading with no noise, causes current 
										//glitch when mode is chargeroff.  perhaps put a if(top-off then trickle)?PvB
	delayCRADA(50);  //stabilize
	adc_in = read_adc(7);// stk500 doesn't support adc7!!typ 0.238v after resistor divider ADC output typ 221= 0xdd
	delayCRADA(50);  //stabilize
	adc_in2 = read_adc(7);// stk500 doesn't support adc7!!typ 0.238v after resistor divider ADC output typ 221= 0xdd
	if(adc_in2< adc_in)
	adc_in = adc_in2;    //pick the lowest one in case we get a glitch in trickle mode
	ControlBatteryCharger(charge_mode); //restore to whatever
	vbatt = (uint32_t)adc_in * vbat_calibration;  /* was, calibrated to 22825, nominal 22580L; */ // uV per bit = 1.1v/1023 *(1.02M+51K)/51K  changed to fixed point typ 5003244=0x4c57ec
	vbatt2 = (uint16_t)(vbatt/100000);   //convert uV to 100mV typ 0x32= 50 -> 05.0volts
	// build data string  c is "minute" here its ONLY 0,
	if(vbatt2<0)
	vbatt2=0;   //no negatives!per mike, if it happens, truncate to zero
	return(vbatt2);
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   checkBattery

ORIGINATOR: PvB

DESCRIPTION: \brief check battery voltage, trap here if too low

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void checkBattery()
{
	
	readBattVolts();  //read voltage
	if(vbatt2<38)
	{	//too low, stop here
		
		while(1)
		{	//stay in this loop, pulsing the LED every second until the battery recovers
			StatusLED(false);   //turn off LED
			delayCRADA(10000);
			StatusLED(true);
			delayCRADA(10);
			readBattVolts();
			if(vbatt2>42)
				break;  //exit stop mode, voltage high enough
		}
	}

}
/* old battery read function obsolete?void readBatt()
{
	uint16_t adc_in,adc_in2;
	uint32_t vbatt;
		adc_in = read_adc(7);// stk500 doesn't support adc7!!typ 0.238v after resistor divider ADC output typ 221= 0xdd
		delayCRADA(50);  //stabilize
		adc_in2 = read_adc(7);// stk500 doesn't support adc7!!typ 0.238v after resistor divider ADC output typ 221= 0xdd
		if(adc_in2< adc_in)
		adc_in = adc_in2;    //pick the lowest one in case we get a glitch in trickle mode
		vbatt = (uint32_t)adc_in * vbat_calibration;  // was, calibrated to 22825, nominal 22580L;  // uV per bit = 1.1v/1023 *(1.02M+51K)/51K  changed to fixed point typ 5003244=0x4c57ec
		vbatt2 = (uint16_t)(vbatt/100000);   //convert uV to 100mV typ 0x32= 50 -> 05.0volts

}*/
ISR(INT0_vect){CodeLED(2);}
ISR(INT1_vect){CodeLED(3);}
ISR(PCINT0_vect){CodeLED(4);}
ISR(PCINT1_vect){CodeLED(5);}
ISR(PCINT2_vect){CodeLED(6);}
ISR(TIMER2_COMPB_vect){CodeLED(9);}
ISR(TIMER2_OVF_vect){CodeLED(10);}
ISR(TIMER1_CAPT_vect){CodeLED(11);}
ISR(TIMER1_COMPA_vect){CodeLED(12);}
ISR(TIMER1_COMPB_vect){CodeLED(13);}
ISR(TIMER1_OVF_vect){CodeLED(14);}
ISR(TIMER0_COMPA_vect){CodeLED(15);}
ISR(TIMER0_COMPB_vect){CodeLED(16);}
ISR(TIMER0_OVF_vect){CodeLED(17);}
ISR(USART_TX_vect){CodeLED(21);}
ISR(ADC_vect){CodeLED(22);}
ISR(EE_READY_vect){CodeLED(23);}
ISR(ANALOG_COMP_vect){CodeLED(24);}
ISR(SPM_READY_vect){CodeLED(25);}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   readBattTemperature

ORIGINATOR: PvB

DESCRIPTION: \brief read battery thermistor
output is in volts.  an Excel model will convert that to temperature..
battery charge temperature limits and differential rise limits will be converted back to volts
and the software will use volts.   this will save on nasty logarithm conversions, polynomial expansions etc.

ON ENTRY:

ON EXIT: thermistor voltage. in millivolts

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int16_t readBattTemp()
{
	int16_t adc_in;
	if(eeprom_read_byte((uint8_t *)EEPROM_DAC_OUTPUT)==false)
		OutputDAC(0);   //turn on dac to -40degC = 0 v, to turn on reference supply

	PowerThermistor(ON);
	delayCRADA(50);  //stabilize
	adc_in = read_adc(6);//
	PowerThermistor(OFF);
	return(adc_in);
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   OutputDAC

ORIGINATOR: PvB

DESCRIPTION: \brief send data on SPI to DAC

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void OutputDAC(int32_t temperature)
{
	
	uint16_t DAC_output; //digital word sent to DAC
	int32_t correctedtemp;
	//temperature= 2000;   //test forces 20deg C.
	correctedtemp = temperature - eeprom_read_word((uint16_t *)EEPROM_DAC_OFFSET);
	if( correctedtemp<-4000L || correctedtemp > 12000L )
	{		//reading out of range, so force to 2.00volts per mike in email feb 24, 2015
		correctedtemp= 16000L;    
	}

	DAC_output = (uint16_t)(((correctedtemp+4000L)*65532L)/100000L);	//add 40.00deg to offset output by design-- so 0v = -40
	spi_put_word(DAC_output);   //send to dac, if we need control of power modes of DAC, write a wrapper function to add bits
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   sleepDAC

ORIGINATOR: PvB

DESCRIPTION: \brief send data on SPI turn off DAC(if required)
WARNING! also turns off reference supply and therefore bias to battery termistor won't work.

ON ENTRY:

ON EXIT:

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void sleepDAC()
{
	if(eeprom_read_byte((uint8_t *)EEPROM_DAC_OUTPUT)==false)
	{

		spi_put_word(0x4000);   //force DAC chip into powerdown mode with 1K load to ground for static protection..  if enabled by eeprom variable will stay powered up.
	}
}
/*! S+ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
TITLE:   terminateCharge

ORIGINATOR: PvB

DESCRIPTION: \brief decide if charge should be terminated
Ultimately this routine will turn off the charge, or set it to trickle
based on either: maximum voltage, or maximum temperature, or a particular rise in temperature
the maximums are for safety or if you like, long term health of the battery.
the temperature rise should be the normal exit method.
if required, we can also exit on a maximum time of charge.

ON ENTRY:

ON EXIT: 

GLOBALS:

Dates:
S-* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void terminateCharge(void)
{
	volatile bool terminate = false;
	int16_t delta_thermistor=0;
	readBattVolts();
	if(vbatt2 < 42 && eeprom_read_byte((uint8_t *)EEPROM_SOLAR_PANEL)==01)
	{// 4.2 volts battery is going low so switch to top-off until temperature rise
		charge_mode=top_off;   //force high current, better to command remotely..  PvB
	}
	if(vbatt2 > 61)  // 0x46 exceeded maximum volts, changed to 6.2 volts per mike.  
	{
		terminate=true;
	}
	if(eeprom_read_byte((uint8_t *)EEPROM_THERMISTOR)==true)
	{
		//FUTURE IF EEPROM-DAC NOT TRUE, THEN TURN ON A NOMINAL 0VOLTS TO WAKUP DAC
			
		if((thermistor_voltage = readBattTemp()) < 303)
		{	//too HOT! calculated thermistor voltage for 55degC, 60C max according to textbook, allow 5deg margin TBD refine this
			//voltage is 0.325v   Reading  is 0.325*1024/1.1 = 303
			terminate=true;
		}
		if(thermistor_voltage>1103)
		terminate = true;    //temperature below -20, NO CHARGE.   seems too broad, check battery specs TBD PvB
		//don't know why this was here, its nonsense PvBif((thermistor_voltage < 12))
		delta_thermistor = old_thermistor_voltage - thermistor_voltage;  //delta positive means temperature rise
		if(delta_thermistor<0)
		delta_thermistor=0;  //don't care about temp decreases
		if(delta_thermistor > 75)  //75 = 5.5degC rise  should be 1degC/minute per battery university.  setup 1 minute counter
		{	//voltage drops as temp increases, so if batt is hot, new voltage is lower-> positive difference
			terminate=true;
		}
		old_thermistor_voltage = thermistor_voltage;
	}
	NIMH_charge_counter++;
	if(NIMH_charge_counter==1800)
	{
		NIMH_charge_counter=0;
		terminate=true;	//aproximately 5 hours
	}
	if(terminate==true)
	{
		if(eeprom_read_byte((uint8_t *)EEPROM_SOLAR_PANEL)==02)
		{
			charge_mode=charger_off;
		}
		else
		{
			charge_mode=trickle;
		}
	}
	ControlBatteryCharger(charge_mode);
	sleepDAC();   //if dac not needed power down
	//for debug, might cause crashes?  return(thermistor_voltage);
}


