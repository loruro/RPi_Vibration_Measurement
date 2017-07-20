/**
  ******************************************************************************
  * @file    adxl345.h
  * @author  Karol Leszczyński
  * @version V1.0.0
  * @date    08-July-2017
  * @brief   Header file of ADXL345 sensor driver.
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

#ifndef LIBI2C_ADXL345_H
#define LIBI2C_ADXL345_H

#include <rtems.h>
#include <dev/i2c/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL345_ADDR 0x53

typedef enum
{
	ADXL345_READ_DEVID,
  ADXL345_START_MEASURE,
  ADXL345_READ_DATA_X,
  ADXL345_READ_DATA_Y,
  ADXL345_READ_DATA_Z,
  ADXL345_READ_DATA_ALL
} adxl345_cmd;

int i2c_dev_register_adxl345(
  const char *bus_path,
  const char *dev_path,
  uint16_t address
);

#ifdef __cplusplus
}
#endif

#endif /* LIBI2C_ADXL345_H */
