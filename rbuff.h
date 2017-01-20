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
/**  
 *  \file   rbuff.h  
 *  \brief  Ring buffer   
 *  \author John Myers
 *  \version $Revision$
 */
#ifndef RING_BUFFER_DAKARA_H
#define RING_BUFFER_DAKARA_H

#include <stdint.h>
/*  
 *  Enumerations and Structure declarations
 */
/// Ring buffer pointer structure
struct rb_fifo_struct_type
{
    uint8_t * dptr;  ///< Always points to start of buffer
    uint8_t * head;  ///< Manipulated by Application FW (i.e you)
    uint8_t * tail;  ///< Manipulated by ISR
};
/// Ring buffer pointer
typedef struct rb_fifo_struct_type rbuff_t;

/*
 *  Global functions and variables
 */

/*================================================================*
                        ring_buff_init ()
 *================================================================*/
/// \param[in] *rb \b rbuff_t pointer to Initialize
/// \param[out] *ring \b *rb points to this buffer array location 
void ring_buff_init(volatile rbuff_t *const rb, uint8_t *const ring);
/*inline void ring_buff_init (volatile rbuff_t *const rb, uint8_t *const ring)
{
    //-Point to array buffer
    rb->dptr = ring;  
    rb->head = ring;
    rb->tail = ring;
}*/

#endif // RING_BUFFER_DAKARA_H
