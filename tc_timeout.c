/**
 * \file
 *
 * \brief megaAVR Timer/Counter (TC) timeout driver implementation
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
#include "compiler.h"
#include "tc_timeout.h"

/**
 * \ingroup tc_timeout_group
 * \defgroup tc_timeout_internals_group Timer/Counter (TC) Timeout Driver \
 *                                      internals
 *
 * @{
 */

//! \brief TC Timeout timekeeping data
struct tc_timeout_struct {
	/**
	 * Current count-down value. Counts down for every tick.
	 * Will be considered as expired when it reaches 0, and
	 * may then be reloaded with period.
	 */
	uint16_t count;
	/**
	 * Period between expires. Used to reload count.
	 * If 0, the count won't be reloaded.
	 */
	uint16_t period;
};

// Validate number of timeouts configured
#if CONFIG_TC_TIMEOUT_COUNT > 8
# error "Too many timeouts configured! Maximum is 8"
#endif

/**
 * \brief Array of configurable timeout timekeeping data
 *
 * \see tc_timeout_configuration
 */
static struct tc_timeout_struct tc_timeout_array[CONFIG_TC_TIMEOUT_COUNT];

//! \brief Bitmask of active timeouts
static uint8_t tc_timeout_active;

//! \brief bitmask of expired timeouts
static uint8_t tc_timeout_expired;

// Resolve mask to set in ASSR register based on configuration
#ifdef CONFIG_TC_TIMEOUT_CLOCK_SOURCE_TOSC
# ifdef AS0 // Older mega have got the asynchronous TC on TC0
#  define TC_TIMEOUT_ASSR_MASK (1 << AS0)
# else
#  define TC_TIMEOUT_ASSR_MASK (1 << AS2)
# endif
#elif defined(CONFIG_TC_TIMEOUT_CLOCK_SOURCE_EXCLK)
# define TC_TIMEOUT_ASSR_MASK ((1 << EXCLK) | (1 << AS2))
#else
# define TC_TIMEOUT_ASSR_MASK 0
#endif

// Resolve which TIMSK register to use, timer interrupt mask register PvB
#ifdef TIMSK // Older mega have got a common TIMSK register
# define TC_TIMEOUT_TIMSK TIMSK
#else
# define TC_TIMEOUT_TIMSK TIMSK2
#endif

// Resolve which TC registers to use
#ifdef AS0 // Older mega have got the asynchronous TC on TC0
# define TC_TIMEOUT_OCR OCR0
# define TC_TIMEOUT_OCIE OCIE0
# define TC_TIMEOUT_TCCRA TCCR0
# define TC_TIMEOUT_COMP_vect TIMER0_COMP_vect
#elif defined(OCR2) // A bit newer mega got single compare on TC
# define TC_TIMEOUT_OCR OCR2
# define TC_TIMEOUT_OCIE OCIE2
# define TC_TIMEOUT_TCCRA TCCR2
# define TC_TIMEOUT_COMP_vect TIMER2_COMP_vect
#elif !defined(OCR2B) // LCD mega got single compare on TC called COMPA
# define TC_TIMEOUT_OCR OCR2A
# define TC_TIMEOUT_OCIE OCIE2A
# define TC_TIMEOUT_TCCRA TCCR2A
# define TC_TIMEOUT_COMP_vect TIMER2_COMP_vect
#else
# define TC_TIMEOUT_OCR OCR2A
# define TC_TIMEOUT_OCIE OCIE2A
# define TC_TIMEOUT_TCCRA TCCR2A
# define TC_TIMEOUT_TCCRB TCCR2B
# define TC_TIMEOUT_COMP_vect TIMER2_COMPA_vect
#endif

//! \brief Interrupt handler for TC compare
/*ISR(TC_TIMEOUT_COMP_vect)
{
	uint8_t i;  //modify 

	for (i = 0; i < CONFIG_TC_TIMEOUT_COUNT; i++) 
	{
		if (!(tc_timeout_active & (1 << i)))
			continue;   //timer not active, skip this one
		tc_timeout_array[i].count--;  //decrement count, an higher order counter? PvB
		if (tc_timeout_array[i].count)
			continue;  //count not zero so keep on counting..
		tc_timeout_expired |= 1 << i;  //signal this timer is expired
		if (tc_timeout_array[i].period)
			tc_timeout_array[i].count = tc_timeout_array[i].period;//if period not zero, load into count
		else
			tc_timeout_active &= ~(1 << i);  //clear counter active?   why?  seems period is never zeroed? PvB
	}
}*/

/** @} */
extern uint32_t irTobj;
void tc_timeout_init(void)
{
	// Set up clock source according to configuration
	ASSR = 0x40;   //b6 0x40 external clock input, ClkI/O rate for timer2 DEBUG use external clock for stk500.  TC_TIMEOUT_ASSR_MASK;  //set timer2 status register(0x20)

	// Set compare to value for desired tick rate
	OCR2A = 0xc2;  //=194 calculated defines dont work something is overiding them. TC_TIMEOUT_COMP;  //(0xb3 = 0x1f = 31 OCR2A )
	OCR2B = 0xff;  //set to max so it never compares, if interrupt actually cancels tcnt
	// Configure Timer/Counter to CTC mode, and set desired prescaler in control registers
	//b7,b6 Com2ra=0 A output pin disconnected; B5,B4 Com2rB normal, output pin disconnected
	TC_TIMEOUT_TCCRA = 1 << WGM21;  //0x2 clear timer on compare match(CTC) mode, counter resets with OCR2A, TOV flag on reset.
	TC_TIMEOUT_TCCRB = 0x7;// WGM22 = 0, TC_TIMEOUT_PRESCALER_MASK 2e6/1024= 976Hz; //0xb1=(0x01)  Clkt2s/8 prescaler = 1 prescale 1
	// Enable interrupt for compare match
	TIFR2 = 0x02;  //clear pending interrupt if any.
	TCNT2= 0;   //clearcount to start process
	TC_TIMEOUT_TIMSK = 1 << TC_TIMEOUT_OCIE;  //0x70=(0x02) enable timer compare match interrupt
}

void tc_timeout_start_offset(tc_timeout_id_t id, uint16_t period,
		uint16_t offset)
{
	irqflags_t flags;

	flags = cpu_irq_save();
	tc_timeout_array[id].count = offset;
	tc_timeout_array[id].period = period;
	tc_timeout_active |= 1 << id;
	// Clear any pending expired in case of timeout restart
	tc_timeout_expired &= ~(1 << id);
	cpu_irq_restore(flags);
}

void tc_timeout_start_singleshot(tc_timeout_id_t id, uint16_t timeout)
{
	tc_timeout_start_offset(id, 0, timeout);
}

void tc_timeout_start_periodic(tc_timeout_id_t id, uint16_t period)
{
	tc_timeout_start_offset(id, period, period);
}

bool tc_timeout_test_and_clear_expired(tc_timeout_id_t id)
{
	irqflags_t flags;

	if (tc_timeout_expired & (1 << id)) {
		// Clear expired flag safely if it's set
		flags = cpu_irq_save();
		tc_timeout_expired &= ~(1 << id);  //clear timer expired flag and restart
		cpu_irq_restore(flags);
		return true;
	}

	return false;
}

void tc_timeout_stop(tc_timeout_id_t id)
{
	irqflags_t flags;

	flags = cpu_irq_save();
	tc_timeout_active &= ~(1 << id);
	cpu_irq_restore(flags);
}

