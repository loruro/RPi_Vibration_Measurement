/**
  ******************************************************************************
  * @file    main.c
  * @author  Karol Leszczyński
  * @version V1.0.0
  * @date    06-July-2017
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>
#include <bsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "bsp/i2c.h"
#include <rtems/status-checks.h>

#include "adxl345.h"
#include "mcp9808.h"

rtems_task Task_Read_MCP9808(
  rtems_task_argument unused
)
{
  printf("*** I2C -- task wake after ***\n");
  printf("printf Test\n");

  int rv = 0;
  int fd;

  rv = i2c_dev_register_mcp9808(
         "/dev/i2c",
         "/dev/i2c.mcp9808",
         MCP9808_ADDR
       );
  RTEMS_CHECK_RV(rv, "i2c_dev_register_mcp9808");

  /* Open the mcp9808 device file */
  fd = open("/dev/i2c.mcp9808", O_RDWR);
  RTEMS_CHECK_RV(rv, "Open /dev/i2c.mcp9808");

  while (1) {
    float temp;
    rv = ioctl(fd, MCP9808_READ_TEMP,(void*)&temp);
    printf("Temp: %f\n", temp); /***************/
    RTEMS_CHECK_RV(rv, "mcp9808 gpio set output");
    (void) rtems_task_wake_after( rtems_clock_get_ticks_per_second() / 4 );
  }

  rv = close(fd);
  RTEMS_CHECK_RV(rv, "Close /dev/i2c.mcp23008");
}

rtems_task Task_Read_ADXL345(
  rtems_task_argument unused
)
{
  printf("*** I2C -- task wake after ***\n");
  printf("printf Test\n");

  int rv = 0;
  int fd;

  rv = i2c_dev_register_adxl345(
         "/dev/i2c",
         "/dev/i2c.adxl345",
         ADXL345_ADDR
       );
  RTEMS_CHECK_RV(rv, "i2c_dev_register_adxl345");

  /* Open the mcp9808 device file */
  fd = open("/dev/i2c.adxl345", O_RDWR);
  RTEMS_CHECK_RV(rv, "Open /dev/i2c.adxl345");

  rv = ioctl(fd, ADXL345_START_MEASURE, NULL);
  RTEMS_CHECK_RV(rv, "adxl345 start measure");

  while (1) {
    float data[3];
    rv = ioctl(fd, ADXL345_READ_DATA_ALL, (void*)data);
    printf("DataX: %f\n", data[0]); /***************/
    printf("DataY: %f\n", data[1]); /***************/
    printf("DataZ: %f\n", data[2]); /***************/
    RTEMS_CHECK_RV(rv, "adxl345 read data");
    (void) rtems_task_wake_after( rtems_clock_get_ticks_per_second() / 100 );
  }

  rv = close(fd);
  RTEMS_CHECK_RV(rv, "Close /dev/i2c.adxl345");
}

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  printf("*** Task init ***\n");

  int rv = 0;
  
  rv = rpi_setup_i2c_bus();
  RTEMS_CHECK_RV(rv, "rpi_setup_i2c_bus");

  rtems_id id1,id2;
  rtems_task_create(
    rtems_build_name( 'T', 'A', '1', ' ' ), 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES, &id1
  );

  rtems_task_create(
    rtems_build_name( 'T', 'A', '2', ' ' ), 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES, &id2
  );

  rtems_task_start( id1, Task_Read_MCP9808, 1 );
  rtems_task_start( id2, Task_Read_ADXL345, 2 );

  status = rtems_task_delete( RTEMS_SELF );
}

/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

//#define CONFIGURE_MAXIMUM_TASKS             1

//#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_MAXIMUM_DRIVERS 10

#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM

#define CONFIGURE_MAXIMUM_SEMAPHORES 10

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 30

#define CONFIGURE_MAXIMUM_TASKS 20

#define CONFIGURE_INIT_TASK_STACK_SIZE (32 * 1024)


#define CONFIGURE_INIT
#include <rtems/confdefs.h>

/****************  END OF CONFIGURATION INFORMATION  ****************/