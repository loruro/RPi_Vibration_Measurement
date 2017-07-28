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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */
/* ----------------------- RTEMS includes --------------------------------*/
#include <bsp/irq.h>
/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/
static uint32_t useconds;

/* ----------------------- Static functions ---------------------------------*/
static void prvvTIMERExpiredISR(void *arg);

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    rtems_status_code status;
    EXIT_CRITICAL_SECTION();
    status = rtems_interrupt_handler_install(
        BCM2835_IRQ_ID_GPU_TIMER_M1,
        "ModbusTimer",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) prvvTIMERExpiredISR,
        NULL
    );
    ENTER_CRITICAL_SECTION();

    if (status == RTEMS_SUCCESSFUL) {
        useconds = (uint32_t)usTim1Timerout50us * 50;
        return TRUE;
    } else {
        return FALSE;
    }
}


inline void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    uint32_t next_cmp = BCM2835_REG(BCM2835_GPU_TIMER_CLO);
    next_cmp += useconds; // Timer runs at 1 MHz.
    BCM2835_REG(BCM2835_GPU_TIMER_C1) = next_cmp;
    BCM2835_REG(BCM2835_GPU_TIMER_CS) = BCM2835_GPU_TIMER_CS_M1;
}

inline void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
    BCM2835_REG(BCM2835_GPU_TIMER_C1) = BCM2835_REG(BCM2835_GPU_TIMER_CLO) - 1;
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR(void *arg)
{
    BCM2835_REG(BCM2835_GPU_TIMER_CS) = BCM2835_GPU_TIMER_CS_M1;
    ( void )pxMBPortCBTimerExpired(  );
}
