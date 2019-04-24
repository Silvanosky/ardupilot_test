#pragma once

#define HAL_BOARD_NAME "ESP32"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150

#define HAL_STORAGE_SIZE (16384)

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_DIY

#define HAL_INS_DEFAULT HAL_INS_ICM20XXX_I2C
#define HAL_INS_ICM20XXX_I2C_BUS 0
#define HAL_INS_ICM20XXX_I2C_ADDR (0x68)

#define HAL_BARO_DEFAULT HAL_BARO_BMP280_I2C
#define HAL_BARO_BMP280_BUS 0
#define HAL_BARO_BMP280_I2C_ADDR (0x77)

//#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK09916_IMC20948_I2C
//#define HAL_COMPASS_AK09916_I2C_BUS 0

#define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#define HAL_COMPASS_ICM20948_I2C_ADDR (0x68)
#define HAL_COMPASS_AK09916_I2C_ADDR (0x0C)

//#define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE

//#define ALLOW_ARM_NO_COMPASS

#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_ICARUS

//#define HAL_BARO_ALLOW_INIT_NO_BARO
#define HAL_BARO_DEFAULT HAL_BARO_BMP280_I2C
#define HAL_BARO_BMP280_BUS 0
#define HAL_BARO_BMP280_I2C_ADDR  (0x76)
#define HAL_INS_DEFAULT HAL_INS_MPU60XX_SPI
#define HAL_INS_MPU60x0_NAME "MPU6000"
#define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
#define ALLOW_ARM_NO_COMPASS

#define HAL_OS_POSIX_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"
#define O_CLOEXEC 0

#endif

#define HAL_WITH_UAVCAN 0
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0

// allow for static semaphores
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore
#define HAL_Semaphore_Recursive ESP32::Semaphore_Recursive
