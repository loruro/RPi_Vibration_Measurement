/**
  ******************************************************************************
  * @file    adxl345.c
  * @author  Karol Leszczyński
  * @version V1.0.0
  * @date    08-July-2017
  * @brief   ADXL345 sensor driver.
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

#include "adxl345.h"

#define ADXL345_REG_DEVID 0x00
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_REG_DATAX1 0x33
#define ADXL345_REG_DATAY0 0x34
#define ADXL345_REG_DATAY1 0x35
#define ADXL345_REG_DATAZ0 0x36
#define ADXL345_REG_DATAZ1 0x37
#define ADXL345_REG_FIFO_CTL 0x38
#define ADXL345_REG_FIFO_STATUS 0x39

static int i2c_adxl345_write_register(i2c_dev *dev, uint8_t reg, uint8_t val)
{
  uint8_t content[2];

  content[0] = reg;
  content[1] = val;

  i2c_msg msg = {
    .addr = dev->address,
    .flags = 0,
    .len = 2,
    .buf = content
  };

  return i2c_bus_transfer(dev->bus, &msg, 1);
}

static int i2c_adxl345_read_register(
  i2c_dev *dev,
  uint8_t reg,
  uint8_t *reg_content
) {
  i2c_msg msg[2] = {
    {
    .addr = dev->address,
    .flags = 0,
    .len = 1,
    .buf = &reg
    },
    {
    .addr = dev->address,
    .flags = I2C_M_RD,
    .len = 1,
    .buf = (uint8_t*)reg_content
    }
  };

  return i2c_bus_transfer(dev->bus, msg, 2);
}

static int i2c_adxl345_read_multiple_registers(
  i2c_dev *dev,
  uint8_t reg,
  uint8_t *reg_content,
  uint8_t count
) {
  i2c_msg msg[2] = {
    {
    .addr = dev->address,
    .flags = 0,
    .len = 1,
    .buf = &reg
    },
    {
    .addr = dev->address,
    .flags = I2C_M_RD,
    .len = count,
    .buf = (uint8_t*)reg_content
    }
  };

  return i2c_bus_transfer(dev->bus, msg, 2);
}

static int i2c_adxl345_linux_ioctl(
  i2c_dev *dev,
  ioctl_command_t command,
  void *arg
) {
  uint8_t reg_content[6];
  int rv = 0;
  int16_t data;

  switch ( command ) {
    case ADXL345_READ_DEVID:
      rv = i2c_adxl345_read_register(dev, ADXL345_REG_DEVID, reg_content);
      *(uint8_t*)arg = reg_content[0];
      break;

    case ADXL345_START_MEASURE:
      rv = i2c_adxl345_read_register(dev, ADXL345_REG_POWER_CTL, reg_content);
      reg_content[0] |= 0x08;
      rv = i2c_adxl345_write_register(dev, ADXL345_REG_POWER_CTL, reg_content[0]);
      break;

    case ADXL345_SET_RANGE: ;
      uint8_t range = *(uint8_t*)arg;
      if (range >= 0 && range <= 3) {
        rv = i2c_adxl345_read_register(dev, ADXL345_REG_DATA_FORMAT, reg_content);
        reg_content[0] |= 0x08; // Set FULL_RES.
        reg_content[0] &= 0xFC; // Zero range bits.
        switch (range) {
          case 0:
            break;
          
          case 1:
            reg_content[0] |= 0x01;
            break;

          case 2:
            reg_content[0] |= 0x02;
            break;

          case 3:
            reg_content[0] |= 0x03;
            break;
        }
        rv = i2c_adxl345_write_register(dev, ADXL345_REG_DATA_FORMAT, reg_content[0]);
      } else {
        rv = -1;
      }
      break;

    case ADXL345_ENABLE_FIFO_STREAM:
      rv = i2c_adxl345_read_register(dev, ADXL345_REG_FIFO_CTL, reg_content);
      reg_content[0] &= 0x63;
      reg_content[0] |= 0x80;
      rv = i2c_adxl345_write_register(dev, ADXL345_REG_FIFO_CTL, reg_content[0]);
      break;

    case ADXL345_READ_FIFO_ENTRIES:
      rv = i2c_adxl345_read_register(dev, ADXL345_REG_FIFO_STATUS, reg_content);
      *(uint8_t*)arg = reg_content[0] & 0x3F;
      break;

    case ADXL345_READ_DATA_X:
      rv = i2c_adxl345_read_multiple_registers(dev, ADXL345_REG_DATAX0, reg_content, 2);
      data = (int16_t)reg_content[0];
      data |= (int16_t)reg_content[1] << 8;
      *(float*)arg = data * 0.00390625;
      break;

    case ADXL345_READ_DATA_Y:
      rv = i2c_adxl345_read_multiple_registers(dev, ADXL345_REG_DATAY0, reg_content, 2);
      data = (int16_t)reg_content[0];
      data |= (int16_t)reg_content[1] << 8;
      *(float*)arg = data * 0.00390625;
      break;

    case ADXL345_READ_DATA_Z:
      rv = i2c_adxl345_read_multiple_registers(dev, ADXL345_REG_DATAZ0, reg_content, 2);
      data = (int16_t)reg_content[0];
      data |= (int16_t)reg_content[1] << 8;
      *(float*)arg = data * 0.00390625;
      break;

    case ADXL345_READ_DATA_ALL:
      rv = i2c_adxl345_read_multiple_registers(dev, ADXL345_REG_DATAX0, reg_content, 6);
      data = (int16_t)reg_content[0];
      data |= (int16_t)reg_content[1] << 8;
      *((float*)arg + 0) = data * 0.00390625;
      data = (int16_t)reg_content[2];
      data |= (int16_t)reg_content[3] << 8;
      *((float*)arg + 1) = data * 0.00390625;
      data = (int16_t)reg_content[4];
      data |= (int16_t)reg_content[5] << 8;
      *((float*)arg + 2) = data * 0.00390625;
      break;

    default:
      rv = -1;
  }

  return rv;
}

int i2c_dev_register_adxl345(
  const char *bus_path,
  const char *dev_path,
  uint16_t address
) {
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, address);
  if (dev == NULL) {
    // perror();

    return -1;
  }

  dev->ioctl = i2c_adxl345_linux_ioctl;

  return i2c_dev_register(dev, dev_path);
}
