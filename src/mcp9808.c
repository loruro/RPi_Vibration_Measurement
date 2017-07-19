/**
  ******************************************************************************
  * @file    mcp9808.c
  * @author  Karol Leszczyński
  * @version V1.0.0
  * @date    06-July-2017
  * @brief   MCP9808 sensor driver.
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

#include "mcp9808.h"

#define MCP9808_REG_TA 0x05

static int i2c_mcp9808_read_register(
  i2c_dev *dev,
  uint8_t reg,
  uint16_t *reg_content
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
    .len = 2,
    .buf = (uint8_t*)reg_content
    }
  };

  int rv = i2c_bus_transfer(dev->bus, msg, 2);
  *reg_content = (*reg_content >> 8) | (*reg_content << 8); // Swap bytes
  return rv;
}

static int i2c_mcp9808_linux_ioctl(
  i2c_dev *dev,
  ioctl_command_t command,
  void *arg
) {
  uint16_t reg_content;
  int rv = 0;

  switch ( command ) {
    case MCP9808_READ_TEMP:
      rv = i2c_mcp9808_read_register(dev, MCP9808_REG_TA, &reg_content);
      reg_content &= 0x1FFF; // Clear flag bits
      float temperature;
      if ((reg_content & 0x1000) == 0x1000) {
        reg_content &= 0x0FFF; // Clear Sign bits
        temperature = 256 - ((float)reg_content / 16);
      } else {
        temperature = (float)reg_content / 16;
      }
      *(float*)arg = temperature;
      break;

    default:
      rv = -1;
  }

  return rv;
}

int i2c_dev_register_mcp9808(
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

  dev->ioctl = i2c_mcp9808_linux_ioctl;

  return i2c_dev_register(dev, dev_path);
}
