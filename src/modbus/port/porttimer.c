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
/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/
rtems_id timerId;
rtems_interval ticks;

/* ----------------------- Static functions ---------------------------------*/
rtems_timer_service_routine prvvTIMERExpiredISR(rtems_id timer_id, void *user_data);

/* ----------------------- Start implementation -----------------------------*/
inline uint32_t integerDivision(uint32_t dividend, uint32_t divisor) {
    return (dividend + (divisor / 2)) / divisor;
}

BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    rtems_status_code status;
    rtems_name timerName = rtems_build_name( 'M', 'B', 'T', 'M' );

    // xMBPortTimersInit() function is called inside critical section.
    // This causes rtems_timer_create() to hang. To fix it, we exit
    // critical section just for handler installation.
    EXIT_CRITICAL_SECTION();
    status = rtems_timer_create(timerName, &timerId);
    ENTER_CRITICAL_SECTION();

    if (status == RTEMS_SUCCESSFUL) {
        uint32_t a = rtems_clock_get_ticks_per_second() * usTim1Timerout50us;
        ticks = integerDivision(a, 1000000 / 50);
        return TRUE;
    } else {
        return FALSE;
    }
}


inline void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    rtems_timer_fire_after(timerId, ticks, prvvTIMERExpiredISR, NULL);
}

inline void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
    rtems_timer_cancel(timerId);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
rtems_timer_service_routine prvvTIMERExpiredISR(rtems_id timer_id, void *user_data)
{
    ( void )pxMBPortCBTimerExpired(  );
}
