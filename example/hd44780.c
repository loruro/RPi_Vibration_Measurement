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
 * The LCD controller is used in 4-bit mode with a full bi-directional
 * interface (i.e. R/~W is connected) so the busy flag can be read.
 *
 * $Id: hd44780.c 2002 2009-06-25 20:21:16Z joerg_wunsch $
 */

/**
  ******************************************************************************
  * @file    hd44780.c
  * @author  Karol Leszczyński
  * @version V1.0.0
  * @date    03-October-2016
  * @brief   HD44780 driver.
  ******************************************************************************
  * @attention
  *
  * Changes made to the original file are part of RPi_Vibration_Measurement.
  * Original: www.nongnu.org/avr-libc/examples/stdiodemo/hd44780.c
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

#include "hd44780.h"

#include "bsp/gpio.h"

#include "defines.h"

#define TICKS_PER_MICROSECOND 300
void microsecond_delay(uint32_t usec) {
  uint32_t cycles = usec * TICKS_PER_MICROSECOND;
  for (uint32_t i = 0; i < cycles; i++) {
    asm volatile("nop");
  }
}

/*
 * Send one pulse to the E signal (enable).  Mind the timing
 * constraints.  If readback is set to true, read the HD44780 data
 * pins right before the falling edge of E, and return that value.
 */
static inline void hd44780_pulse_e();

static inline void hd44780_pulse_e() {
  rtems_gpio_set(HD44780_E);
  microsecond_delay(1);
  rtems_gpio_clear(HD44780_E);
}

/*
 * Send one nibble out to the LCD controller.
 */
static void hd44780_outnibble(uint8_t n, uint8_t rs) {
  if (rs)
    rtems_gpio_set(HD44780_RS);
  else
    rtems_gpio_clear(HD44780_RS);

  (n >> 0 & 0x01) == 1 ? rtems_gpio_set(HD44780_D4) : rtems_gpio_clear(HD44780_D4);
  (n >> 1 & 0x01) == 1 ? rtems_gpio_set(HD44780_D5) : rtems_gpio_clear(HD44780_D5);
  (n >> 2 & 0x01) == 1 ? rtems_gpio_set(HD44780_D6) : rtems_gpio_clear(HD44780_D6);
  (n >> 3 & 0x01) == 1 ? rtems_gpio_set(HD44780_D7) : rtems_gpio_clear(HD44780_D7);

  (void)hd44780_pulse_e();
}

/*
 * Send one byte to the LCD controller.  As we are in 4-bit mode, we
 * have to send two nibbles.
 */
void hd44780_outbyte(uint8_t b, uint8_t rs) {
  hd44780_outnibble(b >> 4, rs);
  hd44780_outnibble(b & 0xf, rs);
}

/*
 * Wait until the busy flag is cleared.
 */
void hd44780_wait_ready(bool longwait) {
  if (longwait)
    microsecond_delay(1520);
  else
    microsecond_delay(37);
}

/*
 * Initialize the LCD controller.
 *
 * The initialization sequence has a mandatory timing so the
 * controller can safely recognize the type of interface desired.
 * This is the only area where timed waits are really needed as
 * the busy flag cannot be probed initially.
 */
void hd44780_init(void) {
  rtems_gpio_request_pin(HD44780_RS, DIGITAL_OUTPUT, false, false, NULL);
  rtems_gpio_request_pin(HD44780_E, DIGITAL_OUTPUT, false, false, NULL);
  rtems_gpio_request_pin(HD44780_D4, DIGITAL_OUTPUT, false, false, NULL);
  rtems_gpio_request_pin(HD44780_D5, DIGITAL_OUTPUT, false, false, NULL);
  rtems_gpio_request_pin(HD44780_D6, DIGITAL_OUTPUT, false, false, NULL);
  rtems_gpio_request_pin(HD44780_D7, DIGITAL_OUTPUT, false, false, NULL);

  microsecond_delay(15000);
  hd44780_outnibble(HD44780_FNSET(1, 0, 0) >> 4, 0);
  microsecond_delay(4100);
  hd44780_outnibble(HD44780_FNSET(1, 0, 0) >> 4, 0);
  microsecond_delay(100);
  hd44780_outnibble(HD44780_FNSET(1, 0, 0) >> 4, 0);
  microsecond_delay(37);

  hd44780_outnibble(HD44780_FNSET(0, 1, 0) >> 4, 0);
  hd44780_wait_ready(false);
  hd44780_outcmd(HD44780_FNSET(0, 1, 0));
  hd44780_wait_ready(false);
  hd44780_outcmd(HD44780_DISPCTL(0, 0, 0));
  hd44780_wait_ready(false);
}

/*
 * Prepare the LCD controller pins for powerdown.
 */
void hd44780_powerdown(void) {
  rtems_gpio_clear(HD44780_D4);
  rtems_gpio_clear(HD44780_D5);
  rtems_gpio_clear(HD44780_D6);
  rtems_gpio_clear(HD44780_D7);
  rtems_gpio_clear(HD44780_RS);
  rtems_gpio_clear(HD44780_E);
}
