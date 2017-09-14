/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: port.h,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#ifndef _PORT_H
#define _PORT_H

#include <assert.h>
#include <inttypes.h>

#include <bsp.h>

#define	INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

rtems_interrupt_level level;
#define ENTER_CRITICAL_SECTION( ) rtems_interrupt_disable(level)
#define EXIT_CRITICAL_SECTION( ) rtems_interrupt_enable(level) 

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

BOOL xMBPortSerialPoll(  );


/* Debug LED ******************/
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3)) 
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3)) 
#define GPIO_SET *(gpio+7)   // sets   bits which are 1 ignores bits which are 0 
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_SET_EXT *(gpio+8)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR_EXT *(gpio+11) // clears bits which are 1 ignores bits which are 0
#define LED_INIT()  do { unsigned int *gpio = (unsigned int *)BCM2835_GPIO_REGS_BASE; OUT_GPIO(47);} while(0)
#define LED_ON()  do { unsigned int *gpio = (unsigned int *)BCM2835_GPIO_REGS_BASE; GPIO_CLR_EXT = 1 << (47 % 32);} while(0)
#define LED_OFF()  do { unsigned int *gpio = (unsigned int *)BCM2835_GPIO_REGS_BASE; GPIO_SET_EXT = 1 << (47 % 32);} while(0)
/**************************************/

#endif
