/**
 * \file mega_uart_interrupt_example.c
 *
 * \brief megaAVR STK600 UART interrupt example
 *
 * Copyright (c) 2011 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

 // _ASSERT_ENABLE_ is used for enabling assert, typical for debug purposes
#define _ASSERT_ENABLE_
#include <string.h>
#include "compiler.h"

/**
 * \def BUFFER_SIZE
 * \brief The size of the UART buffer
 */
#define  BUFFER_SIZE 49   //was 40, now try 100 to solve api radio problem? was 20 PvB saw overrun, now has guard so could go back to 20 to save memory? PvB

// set the correct BAUD and F_CPU defines before including setbaud.h
#include "config/conf_clock.h"
#include "config/conf_uart.h"

/**
 * \name avr_libc_inc avr libc include files
 * @{
 */
#include <util/setbaud.h>
#include <avr/interrupt.h>
//! @}

#include "ring_buffer.h"

extern void delayCRADA(double ms);

// buffers for use with the ring buffer (belong to the UART)
uint8_t out_buffer[BUFFER_SIZE+1];  //add a 1 to the buffer because a bug caused a 1byte overrun! PvB
uint8_t in_buffer[BUFFER_SIZE+1];

// the string we send and receive on UART
const char test_string[] = "the quick brown fox jumped over the lazy dog";

//! ring buffer to use for the UART transmission
struct ring_buffer ring_buffer_out;
//! ring buffer to use for the UART reception
struct ring_buffer ring_buffer_in;

/**
 * \brief UART data register empty interrupt handler
 *
 * This handler is called each time the UART data register is available for
 * sending data.
 */
ISR(UART0_DATA_EMPTY_IRQ)
{
	// if there is data in the ring buffer, fetch it and send it
	if (!ring_buffer_is_empty(&ring_buffer_out)) {
		UDR0 = ring_buffer_get(&ring_buffer_out);  //send a char to uart register
	}
	else {
		// no more data to send, turn off data ready interrupt
		UCSR0B &= ~(1 << UDRIE0);
	}
}

/**
 * \brief Data RX interrupt handler
 *
 * This is the handler for UART receive data
 */
ISR(UART0_RX_IRQ)   //vector 18
{
	uint8_t junk; 
		if(abs(ring_buffer_in.read_offset - ring_buffer_in.write_offset) > BUFFER_SIZE/2)
		{
			PORTD|=(1<<PORTD6);  //deassert RTS so radio can't send anymore data, gives application time to absorb it
		}//WARNING!!!  getchar has to examine the buffer and if buffer is more or less clear, restart RTS*   

		if((UCSR0A & (1<<FE0)) == 0)
			ring_buffer_put(&ring_buffer_in, UDR0);  //read data from uart register
		else
			junk = UDR0;  //SOMETIMES WE GET A blank char with a frame error at the start,  this rejects it
	
}

/**
 * \brief Initialize the UART with correct baud rate settings
 *
 * This function will initialize the UART baud rate registers with the correct
 * values using the AVR libc setbaud utility. In addition set the UART to
 * 8-bit, 1 stop and no parity.
 */
void uart_init(void)
{
	uint8_t junk;
	cli();

#if defined UBRR0H
	// get the values from the setbaud tool
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#else
#error "Device is not supported by the driver"
#endif

#if USE_2X
	UCSR0A |= (1 << U2X0);
#endif
	// initialize the in and out buffer for the UART, moved init before interrupts to maybe solve false interrupts
	ring_buffer_out = ring_buffer_init(out_buffer, BUFFER_SIZE);
	ring_buffer_in = ring_buffer_init(in_buffer, BUFFER_SIZE);

	// enable RX and TX and set interrupts on rx complete
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	// 8-bit, 1 stop bit, no parity, asynchronous UART
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (0 << USBS0) |
			(0 << UPM01) | (0 << UPM00) | (0 << UMSEL01) |
			(0 << UMSEL00);
	delayCRADA(1000);  //allow time for strange events to pass
	junk = UDR0;   //read data register to erase junk data PvB
	PORTD &= ~(1<<PORTD6);  //assert RTS* so data can come in
	sei();   //re-enable interrupts.    I think everything is setup so its oK PvB
}

/**
 * \brief Function for putting a char in the UART buffer
 *
 * \param data the data to add to the UART buffer and send
 *
 */
void uart_putchar(uint8_t data)
{
	uint8_t fail_count=0;
	if(ring_buffer_is_full(&ring_buffer_out))
	{	//buffer is full so wait around and see if it clears PvB
		delayCRADA(10); //give time to process some characters
		if(ring_buffer_is_full(&ring_buffer_out))  //still full?
		{
			delayCRADA(1);  //a dummy call to fool optimizer.   
			return;     //its not working so skip this one this better not happen, replace with Assert later pvB
		}			
	}
	while(1)
	{		//TBD add escape counter in case it never asserts cts
		if((PORTD &(1<<PORT3))==0)  //test cts from radio if not true wait
			break;
		else
			delayCRADA(20);  //enough time for clearing the radio buffer
		fail_count++;
		if(fail_count>500)
			break;    //no failure indication.   tough!
			
			//TBD add failure indication
	}			
	// Disable interrupts to get exclusive access to ring_buffer_out.
	cli();
	if (ring_buffer_is_empty(&ring_buffer_out)) {
		// First data in buffer, enable data ready interrupt
		UCSR0B |=  (1 << UDRIE0);
	}
	// Put data in buffer  no mechanism for what happens if buffer is full.
	ring_buffer_put(&ring_buffer_out, data);

	// Re-enable interrupts
	sei();
}

/**
 * \brief Function for getting a char from the UART receive buffer
 *
 * \retval Next data byte in receive buffer
 */
uint8_t uart_getchar(void)
{
	if(abs(ring_buffer_in.read_offset - ring_buffer_in.write_offset) < BUFFER_SIZE/4)
	{	//if RTS was previously deasserted, then when some room opens up in the buffer, re-assert it
			PORTD &= ~(1<<PORTD6);  //assert RTS* so more data can come in
	}
	
	return ring_buffer_get(&ring_buffer_in);
}


/**
 * \brief Function to check if we have a char waiting in the UART receive buffer
 *
 * \retval true if data is waiting
 * \retval false if no data is waiting
 */
bool uart_char_waiting(void)
{
	return !ring_buffer_is_empty(&ring_buffer_in);
}

/**
 * old main application from demo program The main application
 *
 * This application will initialize the UART, send a character and then receive
 * it and check if it is the same character as was sent.
 *
 * \note The RX and TX pins should be externally connected in order to pass the
 * test.
 */
/*int main(void)
{
	uint8_t data;
	uint8_t cnt;
	cli();
	uart_init();
	sei();

	// Send the test string
	for (cnt = 0; cnt < strlen(test_string); cnt++) {
		uart_putchar(test_string[cnt]);
	}

	// Check if we have received the string we sent
	cnt = 0;
	do {
		// Wait for next character
		while (!uart_char_waiting());
		data = uart_getchar();
		// Compare to what we sent
		Assert (data == test_string[cnt++]);
	} while (cnt < strlen(test_string));

	while (true);
}*/
