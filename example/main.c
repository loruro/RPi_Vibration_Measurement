/**
  ******************************************************************************
  * @file    main.c
  * @author  Karol Leszczyński
  * @version V1.0.0
  * @date    03-October-2016
  * @brief   Main.
  ******************************************************************************
  * @attention
  *
  * This file is part of RPi_Vibration_Measurement.
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

#include <stdlib.h>
#include <pthread.h>

#include "bsp/gpio.h"

#include "hd44780.h"

pthread_mutex_t test_mutex;
pthread_cond_t test_condition;

void *lcd_display(void *arg) {
  hd44780_init();
  hd44780_outcmd(HD44780_CLR);
  hd44780_wait_ready(1);
  hd44780_outcmd(HD44780_ENTMODE(1, 0));
  hd44780_wait_ready(1);
  hd44780_outcmd(HD44780_DISPCTL(1, 0, 0));
  hd44780_wait_ready(1);
  hd44780_outdata('0');
  hd44780_wait_ready(1);

  pthread_mutex_lock(&test_mutex);

  int count = 0;
  while (1) {
    pthread_cond_wait(&test_condition, &test_mutex);
    count++;
    hd44780_outcmd(HD44780_DDADDR(0));
    hd44780_wait_ready(1);
    if ((count % 2) == 0)
      hd44780_outdata('0');
    else
      hd44780_outdata('1');
    hd44780_wait_ready(1);
  }

  return NULL;
}

void *POSIX_Init(void *argument) {
  pthread_t child_1;
  rtems_status_code rtems_status;

  rtems_status = rtems_gpio_initialize();

  pthread_mutex_init(&test_mutex, NULL);
  pthread_cond_init(&test_condition, NULL);
  pthread_create(&child_1, NULL, lcd_display, NULL);

  rtems_gpio_request_pin(26, DIGITAL_INPUT, false, false, NULL);
  rtems_gpio_resistor_mode(26, PULL_UP);

  while (1) {
    static uint8_t pressed_old = 1;
    uint8_t pressed = rtems_gpio_get_value(26);

    if (pressed_old != pressed) {
      pressed_old = pressed;
      pthread_cond_signal(&test_condition);
    }

    rtems_task_wake_after(rtems_clock_get_ticks_per_second() / 100);
  }
  exit(0);
}

/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

//#define CONFIGURE_MAXIMUM_TASKS             1

//#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_POSIX_INIT_THREAD_TABLE

#define CONFIGURE_MAXIMUM_POSIX_THREADS 10
#define CONFIGURE_MAXIMUM_POSIX_MUTEXES 10
#define CONFIGURE_MAXIMUM_POSIX_CONDITION_VARIABLES 10

#define CONFIGURE_INIT
#include <rtems/confdefs.h>

/****************  END OF CONFIGURATION INFORMATION  ****************/
