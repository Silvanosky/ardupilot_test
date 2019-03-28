#pragma once

#define HAL_BOARD_NAME "ESP32"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150

#ifndef CONFIG_HAL_BOARD_SUBTYPE
// allow for generic boards
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_GENERIC
#endif

#define HAL_STORAGE_SIZE (16384)
#define HAL_BARO_ALLOW_INIT_NO_BARO
#define HAL_I2C_INTERNAL_MASK 0

//# no built-in compass, but probe the i2c bus for all possible
//# external compass types
#define ALLOW_ARM_NO_COMPASS
//#define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
//#define HAL_PROBE_EXTERNAL_I2C_COMPASSES
#define HAL_COMPASS_HMC5843_I2C_BUS 0
#define HAL_COMPASS_HMC5843_I2C_ADDR (0x1E)

#define HAL_PARAM_DEFAULTS_PATH nullptr

#define HAL_INS_DEFAULT HAL_INS_MPU60XX_I2C
#define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
//#define HAL_COMPASS_HMC5843_I2C_BUS 1
//#define HAL_COMPASS_HMC5843_I2C_ADDR (0x1E)
#define HAL_INS_MPU60x0_I2C_BUS 0
#define HAL_INS_MPU60x0_I2C_ADDR 0x68

#define HAL_WITH_UAVCAN 0
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0

// allow for static semaphores
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore
#define HAL_Semaphore_Recursive ESP32::Semaphore_Recursive
