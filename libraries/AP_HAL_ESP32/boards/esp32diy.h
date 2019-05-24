/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#define HAL_INS_DEFAULT HAL_INS_ICM20XXX_I2C
#define HAL_INS_ICM20XXX_I2C_BUS 0
#define HAL_INS_ICM20XXX_I2C_ADDR (0x68)

//#define HAL_BARO_DEFAULT HAL_BARO_BMP280_I2C
#define HAL_BARO_DEFAULT HAL_BARO_ANALOG
//GPIO 34
#define HAL_BARO_ANALOG_PIN (6)
#define HAL_BARO_BMP280_BUS 0
#define HAL_BARO_BMP280_I2C_ADDR (0x77)

#define HAL_COMPASS_AK09916_I2C_BUS 0

#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK09916_ICM20948_I2C
#define HAL_COMPASS_ICM20948_I2C_ADDR (0x68)
#define HAL_COMPASS_AK09916_I2C_ADDR (0x0C)

#define HAL_ESP32_WIFI 1

//TODO RCOUT config
#define HAL_ESP32_RCOUT {GPIO_NUM_27, GPIO_NUM_13, GPIO_NUM_22, GPIO_NUM_21}

#define HAL_ESP32_SPI_BUSES \
    {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_19, .sclk=GPIO_NUM_18}

#define HAL_ESP32_SPI_DEVICES {}

#define HAL_ESP32_I2C_BUSES {}

#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_3 , .tx=GPIO_NUM_1 }



