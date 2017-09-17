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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include "bsp/i2c.h"
#include <rtems/status-checks.h>
#include <rtems/printer.h>

#include "mb.h"

#include "adxl345.h"
#include "mcp9808.h"

/* Global variables ***************************/
// #define FREQ 100
#define FIFO_SIZE 480000 // 800 Hz for 10 minutes.
#define FIFO_PROCESSED_SIZE 6000 // 10 Hz for 10 minutes.
// #define BUFFER_LENGTH 20
#define REG_INPUT_START 1
#define REG_INPUT_NREGS (2+120+120+24+2)

#define REG_INPUT_INDEX_TIME 0
#define REG_INPUT_INDEX_X 2
#define REG_INPUT_INDEX_Y 42
#define REG_INPUT_INDEX_Z 82
#define REG_INPUT_INDEX_VELOCITY_X 122
#define REG_INPUT_INDEX_VELOCITY_Y 162
#define REG_INPUT_INDEX_VELOCITY_Z 202
#define REG_INPUT_INDEX_RMS_X 242
#define REG_INPUT_INDEX_RMS_Y 244
#define REG_INPUT_INDEX_RMS_Z 246
#define REG_INPUT_INDEX_VRMS_X 248
#define REG_INPUT_INDEX_VRMS_Y 250
#define REG_INPUT_INDEX_VRMS_Z 252
#define REG_INPUT_INDEX_PP_X 254
#define REG_INPUT_INDEX_PP_Y 256
#define REG_INPUT_INDEX_PP_Z 258
#define REG_INPUT_INDEX_KURT_X 260
#define REG_INPUT_INDEX_KURT_Y 262
#define REG_INPUT_INDEX_KURT_Z 264
#define REG_INPUT_INDEX_TEMP 266
// #define REG_DISCRETE_START 1
// #define REG_DISCRETE_NREGS 1
// static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];

uint16_t frequency = 800;
uint32_t processingStep = 100;

float fifoX[FIFO_SIZE];
float fifoY[FIFO_SIZE];
float fifoZ[FIFO_SIZE];
float fifoRMSX[FIFO_PROCESSED_SIZE];
float fifoRMSY[FIFO_PROCESSED_SIZE];
float fifoRMSZ[FIFO_PROCESSED_SIZE];
float fifoVRMSX[FIFO_PROCESSED_SIZE] = {0};
float fifoVRMSY[FIFO_PROCESSED_SIZE] = {0};
float fifoVRMSZ[FIFO_PROCESSED_SIZE] = {0};
float fifoPPX[FIFO_PROCESSED_SIZE] = {0};
float fifoPPY[FIFO_PROCESSED_SIZE] = {0};
float fifoPPZ[FIFO_PROCESSED_SIZE] = {0};
float fifoKurtX[FIFO_PROCESSED_SIZE] = {0};
float fifoKurtY[FIFO_PROCESSED_SIZE] = {0};
float fifoKurtZ[FIFO_PROCESSED_SIZE] = {0};

// uint32_t fifoUsedSize = frequency; // RAW Live
// uint32_t fifoUsedSize = processingStep * frequency / 200; // Processed Live
uint32_t fifoUsedSize  = 400; // TESTING
uint32_t fifoStored = 0;
uint32_t fifoWriteIndex = 0;
uint32_t fifoReadIndex = 0;

uint32_t fifoProcessedUsedSize  = 5;
uint32_t fifoProcessedStored = 0;
uint32_t fifoProcessedWriteIndex = 0;
uint32_t fifoProcessedReadIndex = 0;

float temperature;

/* Semaphore ***************************/
rtems_id sem_id;
/**************************************/

/* Test ******************************/
uint16_t fifoOverrunRpi = 0;
uint16_t fifoOverrunAdxl = 0;
uint16_t fifoOverrunProcessed = 0;
/**************************************/

rtems_task Task_Read_MCP9808(
  rtems_task_argument unused
)
{
  printf("*** I2C -- task wake after ***\n");
  printf("printf Test\n");
  
  rtems_name name;
  rtems_id period;
  rtems_status_code status;
  name = rtems_build_name( 'P', 'E', 'R', '2' );
  status = rtems_rate_monotonic_create( name, &period );
  if ( status != RTEMS_SUCCESSFUL ) {
    printf( "rtems_monotonic_create failed with status of %d.\n", status );
  }

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
    rtems_rate_monotonic_period( period, rtems_clock_get_ticks_per_second() / 4 );
    rv = ioctl(fd, MCP9808_READ_TEMP,(void*)&temperature);
    // printf("Temp: %f\n", temp); /***************/
    RTEMS_CHECK_RV(rv, "mcp9808 gpio set output");
    // rtems_printer printer;
    // rtems_print_printer_printf( &printer );
    // rtems_rate_monotonic_report_statistics_with_plugin( &printer );
    // rtems_cpu_usage_report(&printer);
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

  rtems_name name;
  rtems_id period;
  rtems_status_code status;
  name = rtems_build_name( 'P', 'E', 'R', '1' );
  status = rtems_rate_monotonic_create( name, &period );
  if ( status != RTEMS_SUCCESSFUL ) {
    printf( "rtems_monotonic_create failed with status of %d.\n", status );
  }

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

  uint8_t frequencyCode;
  switch (frequency) {
    case 100:
      frequencyCode = 0xA;
      break;

    case 800:
      frequencyCode = 0xD;
      break;

    default:
      frequencyCode = 0xD;
      printf("Frequency value error!\n");
      break;
  }
  rv = ioctl(fd, ADXL345_SET_FREQUENCY, (void*)&frequencyCode);
  RTEMS_CHECK_RV(rv, "adxl345 set frequency");

  uint8_t range = 3; // 16 g.
  rv = ioctl(fd, ADXL345_SET_RANGE, (void*)&range);
  RTEMS_CHECK_RV(rv, "adxl345 set range");

  rv = ioctl(fd, ADXL345_ENABLE_FIFO_STREAM, NULL);
  RTEMS_CHECK_RV(rv, "adxl345 enable fifo stream");

  rv = ioctl(fd, ADXL345_START_MEASURE, NULL);
  RTEMS_CHECK_RV(rv, "adxl345 start measure");

  // uint8_t bufferSample = 0;
  while (1) {
    rtems_rate_monotonic_period( period, rtems_clock_get_ticks_per_second() / (frequency / 25) );
    float data[3];
    uint8_t fifoEntries;
    rv = ioctl(fd, ADXL345_READ_FIFO_ENTRIES, (void*)&fifoEntries);
    if (fifoEntries >= 32) {
      fifoOverrunAdxl++;
    }

    for (uint8_t i = 0; i < fifoEntries; ++i) {
      rv = ioctl(fd, ADXL345_READ_DATA_ALL, (void*)data);
      // printf("DataX: %f\n", data[0]); /***************/
      // printf("DataY: %f\n", data[1]); /***************/
      // printf("DataZ: %f\n", data[2]); /***************/
      RTEMS_CHECK_RV(rv, "adxl345 read data");

      fifoX[fifoWriteIndex] = data[0];
      fifoY[fifoWriteIndex] = data[1];
      fifoZ[fifoWriteIndex] = data[2];
      if (fifoStored < fifoUsedSize) {
        fifoStored++;
      } else {
        fifoOverrunRpi++;
      }
      fifoWriteIndex++;
      if (fifoWriteIndex >= fifoUsedSize) {
        fifoWriteIndex = 0;
      }
    }

    // USHORT *dataBuffer;
    // if (!bufferNumber) {
    //   dataBuffer = usRegInputBuf2;
    // } else {
    //   dataBuffer = usRegInputBuf1;
    // }
    // // rtems_semaphore_obtain(sem_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    // for (uint8_t i = 0; i < 6; i++) {
    //   dataBuffer[i + bufferSample * 6] = *((uint16_t *)data + i);
    // }
    // // rtems_semaphore_release(sem_id);
    // bufferSample++;
    // if (bufferSample >= BUFFER_LENGTH) {
    //   bufferSample = 0;
    //   dataReady = 1;
    //   bufferNumber ^= 1;
    // }
  }

  rv = close(fd);
  RTEMS_CHECK_RV(rv, "Close /dev/i2c.adxl345");
}

rtems_task Task_Processing(
  rtems_task_argument unused
)
{
  uint32_t samplesAmount = processingStep * frequency / 1000;
  while (1) {
    float rms[3] = {0};
    float ppMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
    float ppMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
    for(uint32_t i = 0; i < samplesAmount; ++i) {
      while (fifoStored == 0) { // Possibility of infinite loop!
        // Wait for the rest of fifo samples.
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() / frequency);
      }
      float sample[3];
      sample[0] = fifoX[fifoReadIndex];
      sample[1] = fifoY[fifoReadIndex];
      sample[2] = fifoZ[fifoReadIndex];

      // RMS
      rms[0] += sample[0] * sample[0];
      rms[1] += sample[1] * sample[1];
      rms[2] += sample[2] * sample[2];

      // Peak-to-peak
      if (sample[0] < ppMin[0]) {ppMin[0] = sample[0];}
      if (sample[1] < ppMin[1]) {ppMin[1] = sample[1];}
      if (sample[2] < ppMin[2]) {ppMin[2] = sample[2];}
      if (sample[0] > ppMax[0]) {ppMax[0] = sample[0];}
      if (sample[1] > ppMax[1]) {ppMax[1] = sample[1];}
      if (sample[2] > ppMax[2]) {ppMax[2] = sample[2];}

      fifoReadIndex++;
      if (fifoReadIndex >= fifoUsedSize) {
        fifoReadIndex = 0;
      }
      fifoStored--;
    }
    // RMS
    rms[0] = sqrt(rms[0] / samplesAmount);
    rms[1] = sqrt(rms[1] / samplesAmount);
    rms[2] = sqrt(rms[2] / samplesAmount);

    fifoRMSX[fifoProcessedWriteIndex] = rms[0];
    fifoRMSY[fifoProcessedWriteIndex] = rms[1];
    fifoRMSZ[fifoProcessedWriteIndex] = rms[2];
    fifoPPX[fifoProcessedWriteIndex] = ppMax[0] - ppMin[0];
    fifoPPY[fifoProcessedWriteIndex] = ppMax[1] - ppMin[1];
    fifoPPZ[fifoProcessedWriteIndex] = ppMax[2] - ppMin[2];
    if (fifoProcessedStored < fifoProcessedUsedSize) {
      fifoProcessedStored++;
    } else {
      fifoOverrunProcessed++;
    }
    fifoProcessedWriteIndex++;
    if (fifoProcessedWriteIndex >= fifoProcessedUsedSize) {
      fifoProcessedWriteIndex = 0;
    }
  }
}

rtems_task Task_Modbus(
  rtems_task_argument unused
)
{
  for( ;; )
  {
      /* Call the main polling loop of the Modbus protocol stack. Internally
        * the polling loop waits for a new event by calling the port 
        * dependent function xMBPortEventGet(  ). In the FreeRTOS port the
        * event layer is built with queues.
        */
    ( void )eMBPoll(  );
  }
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  int             iRegIndex;

  if( ( usAddress >= REG_INPUT_START )
    && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
  {
    iRegIndex = ( int )( usAddress - REG_INPUT_START );

    // rtems_semaphore_obtain(sem_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

    // Check if requested registers overlap registers with acceleration data.
    if (iRegIndex <= REG_INPUT_INDEX_Z + 19 && REG_INPUT_INDEX_X <= iRegIndex + usNRegs) {
      while (fifoStored < 20) { // Possibility of infinite loop!
        // Wait for the rest of fifo samples.
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() * (20 - fifoStored) / frequency);
      }
      // TODO: Try without memcpy
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_X], &fifoX[fifoReadIndex], sizeof(USHORT) * 40);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_Y], &fifoY[fifoReadIndex], sizeof(USHORT) * 40);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_Z], &fifoZ[fifoReadIndex], sizeof(USHORT) * 40);
      fifoReadIndex += 20;
      if (fifoReadIndex >= fifoUsedSize) {
        fifoReadIndex = 0;
      }
      fifoStored -= 20;
    }
    // Check if requested registers overlap registers with temperature data.
    if (iRegIndex <= REG_INPUT_INDEX_TEMP + 1 && REG_INPUT_INDEX_TEMP <= iRegIndex + usNRegs) {
      // TODO: Try without memcpy
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_TEMP], &temperature, sizeof(USHORT) * 2);
    }
    // Check if requested registers overlap registers with processed data.
    if (iRegIndex <= REG_INPUT_INDEX_KURT_Z + 1 && REG_INPUT_INDEX_RMS_X <= iRegIndex + usNRegs) {
      while (fifoProcessedStored < 1) { // Possibility of infinite loop!
        // Wait for the rest of fifo samples.
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() * processingStep / 1000);
      }
      // TODO: Try without memcpy
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_RMS_X], &fifoRMSX[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_RMS_Y], &fifoRMSY[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_RMS_Z], &fifoRMSZ[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_VRMS_X], &fifoVRMSX[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_VRMS_Y], &fifoVRMSY[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_VRMS_Z], &fifoVRMSZ[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_PP_X], &fifoPPX[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_PP_Y], &fifoPPY[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_PP_Z], &fifoPPZ[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_KURT_X], &fifoKurtX[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_KURT_Y], &fifoKurtY[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      memcpy(&usRegInputBuf[REG_INPUT_INDEX_KURT_Z], &fifoKurtZ[fifoProcessedReadIndex], sizeof(USHORT) * 2);
      fifoProcessedReadIndex++;
      if (fifoProcessedReadIndex >= fifoProcessedUsedSize) {
        fifoProcessedReadIndex = 0;
      }
      fifoProcessedStored--;
    }

    while( usNRegs > 0 )
    {
      *pucRegBuffer++ =
          ( UCHAR )( usRegInputBuf[iRegIndex] >> 8 );
      *pucRegBuffer++ =
          ( UCHAR )( usRegInputBuf[iRegIndex] & 0xFF );
      iRegIndex++;
      usNRegs--;
    }
    // rtems_semaphore_release(sem_id);
  }
  else if (usAddress == 1000) // Test
  {
    *pucRegBuffer++ =
          ( UCHAR )( fifoOverrunRpi >> 8 );
    *pucRegBuffer++ =
          ( UCHAR )( fifoOverrunRpi & 0xFF );
    *pucRegBuffer++ =
          ( UCHAR )( fifoOverrunAdxl >> 8 );
    *pucRegBuffer++ =
          ( UCHAR )( fifoOverrunAdxl & 0xFF );
    *pucRegBuffer++ =
          ( UCHAR )( fifoOverrunProcessed >> 8 );
    *pucRegBuffer++ =
          ( UCHAR )( fifoOverrunProcessed & 0xFF );
  }
  else
  {
    eStatus = MB_ENOREG;
  }

  return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
  return MB_ENOREG;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
  return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
  return MB_ENOREG;
  // eMBErrorCode    eStatus = MB_ENOERR;
  // int             iRegIndex;

  // if( ( usAddress >= REG_DISCRETE_START )
  //   && ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_NREGS ) )
  // {
  //   iRegIndex = ( int )( usAddress - REG_DISCRETE_START );
  //   while( usNDiscrete > 0 )
  //   {
  //     *pucRegBuffer++ =
  //       ( UCHAR )( dataReady );
  //     iRegIndex++;
  //     usNDiscrete--;
  //   }
  // }
  // else
  // {
  //   eStatus = MB_ENOREG;
  // }

  // return eStatus;
}

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  printf("*** Task init ***\n");

  int rv = 0;
  
  rpi_i2c_init();
  rv = rpi_i2c_register_bus("/dev/i2c", 400000);
  RTEMS_CHECK_RV(rv, "rpi_setup_i2c_bus");

  rtems_id idMCP9808, idADXL345, idModbus, idProcessing;
  rtems_task_create(
    rtems_build_name( 'T', 'A', '1', ' ' ), 2, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &idMCP9808
  );

  rtems_task_create(
    rtems_build_name( 'T', 'A', '2', ' ' ), 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &idADXL345
  );
  
  rtems_task_create(
    rtems_build_name( 'T', 'A', '3', ' ' ), 3, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES, &idModbus
  );

  rtems_task_create(
    rtems_build_name( 'T', 'A', '4', ' ' ), 4, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
    RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &idProcessing
  );

  /* Select either ASCII or RTU Mode. */
  eMBErrorCode eStatus = eMBInit( MB_RTU, 0x0A, 0, 115200, MB_PAR_NONE );
  assert( eStatus == MB_ENOERR );

  /* Enable the Modbus Protocol Stack. */
  eStatus = eMBEnable(  );

  status = rtems_semaphore_create(
    rtems_build_name( 'S', 'E', 'M', '1' ),
    1,
    RTEMS_BINARY_SEMAPHORE,
    0,
    &sem_id
  );
  assert( status == RTEMS_SUCCESSFUL );

  rtems_task_start( idMCP9808, Task_Read_MCP9808, 0 );
  rtems_task_start( idADXL345, Task_Read_ADXL345, 0 );
  rtems_task_start( idModbus, Task_Modbus, 0 );
  rtems_task_start( idProcessing, Task_Processing, 0 );
  LED_INIT(); // Debug LED

  status = rtems_task_delete( RTEMS_SELF );
}

/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MICROSECONDS_PER_TICK  1000

//#define CONFIGURE_MAXIMUM_TASKS             1

//#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_MAXIMUM_DRIVERS 10

#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM

#define CONFIGURE_MAXIMUM_SEMAPHORES 10

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 30

#define CONFIGURE_MAXIMUM_TASKS 20

#define CONFIGURE_MAXIMUM_TIMERS 10

#define CONFIGURE_MAXIMUM_PERIODS 10

#define CONFIGURE_INIT_TASK_STACK_SIZE (32 * 1024)


#define CONFIGURE_INIT
#include <rtems/confdefs.h>

/****************  END OF CONFIGURATION INFORMATION  ****************/
