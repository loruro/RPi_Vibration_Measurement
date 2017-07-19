/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * HD44780 LCD display driver
 *
 * $Id: hd44780.h 2002 2009-06-25 20:21:16Z joerg_wunsch $
 */

/**
  ******************************************************************************
  * @file    hd44780.h
  * @author  Karol Leszczyński
  * @version V1.0.0
  * @date    03-October-2016
  * @brief   Header file of HD44780 driver.
  ******************************************************************************
  * @attention
  *
  * Changes made to the original file are part of RPi_Vibration_Measurement.
  * Original: www.nongnu.org/avr-libc/examples/stdiodemo/hd44780.h
  *
  * RPi_Vibration_Measurement is free software: you can redistribute it
  * and/or modify it under the terms of the GNU General Public License as
  * published by the Free Software Foundation, either version 3 of the License,
  * or (at your option) any later version.
  *
  * RPi_Vibration_Measurement is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with RPi_Vibration_Measurement.
  * If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  */

#include <stdbool.h>
#include <stdint.h>

/*
 * Send byte b to the LCD.  rs is the RS signal (register select), 0
 * selects instruction register, 1 selects the data register.
 */
void hd44780_outbyte(uint8_t b, uint8_t rs);

/*
 * Wait for the busy flag to clear.
 */
void hd44780_wait_ready(bool islong);

/*
 * Initialize the LCD controller hardware.
 */
void hd44780_init(void);

/*
 * Prepare the LCD controller pins for powerdown.
 */
void hd44780_powerdown(void);

/* Send a command to the LCD controller. */
#define hd44780_outcmd(n) hd44780_outbyte((n), 0)

/* Send a data byte to the LCD controller. */
#define hd44780_outdata(n) hd44780_outbyte((n), 1)

/* Clear LCD display command. */
#define HD44780_CLR 0x01

/* Home cursor command. */
#define HD44780_HOME 0x02

/*
 * Select the entry mode.  inc determines whether the address counter
 * auto-increments, shift selects an automatic display shift.
 */
#define HD44780_ENTMODE(inc, shift) (0x04 | ((inc)? 0x02: 0) | ((shift)? 1: 0))

/*
 * Selects disp[lay] on/off, cursor on/off, cursor blink[ing]
 * on/off.
 */
#define HD44780_DISPCTL(disp, cursor, blink) \
        (0x08 | ((disp)? 0x04: 0) | ((cursor)? 0x02: 0) | ((blink)? 1: 0))

/*
 * With shift = 1, shift display right or left.
 * With shift = 0, move cursor right or left.
 */
#define HD44780_SHIFT(shift, right) \
        (0x10 | ((shift)? 0x08: 0) | ((right)? 0x04: 0))

/*
 * Function set.  if8bit selects an 8-bit data path, twoline arranges
 * for a two-line display, font5x10 selects the 5x10 dot font (5x8
 * dots if clear).
 */
#define HD44780_FNSET(if8bit, twoline, font5x10) \
        (0x20 | ((if8bit)? 0x10: 0) | ((twoline)? 0x08: 0) | \
        ((font5x10)? 0x04: 0))

/*
 * Set the next character generator address to addr.
 */
#define HD44780_CGADDR(addr) (0x40 | ((addr) & 0x3f))

/*
 * Set the next display address to addr.
 */
#define HD44780_DDADDR(addr) (0x80 | ((addr) & 0x7f))
