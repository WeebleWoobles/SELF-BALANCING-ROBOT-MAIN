#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>        //GPIO Driver
#include <driver/i2c.h>        //I2C Driver
#include "imu_interface.h"      //IMU Interface
#include "state_estimation.h"   //State Estimation
#include "control_logic.h"      //Control Logic
#include "input_management.h"   //Input Management
#include "motor_interface.h"    //Motor Interface


/* *** PIN ASSIGNMENT DEFINES *** */

// pin_config.h - Generated Pin Configuration for ESP32-S2
// Platform: espressif-dev-32 (ESP-IDF)

// PIN_EN (Pin 1): EN - Reset
// PIN_VIN (Pin 3): vin - Power
#define L_ULTRA_TRIG GPIO_NUM_32 // Pin 6: Digital Output
#define L_ULTRA_ECHO GPIO_NUM_33 // Pin 7: Digital Input
#define R_ULTRA_TRIG GPIO_NUM_25 // Pin 8: Digital Output
#define R_ULTRA_ECHO GPIO_NUM_26 // Pin 9: Digital Input
#define L_MOTOR_DIR GPIO_NUM_27 // Pin 10: Digital Input
#define L_MOTOR_PWM GPIO_NUM_14 // Pin 11: Digital Input
#define R_MOTOR_DIR GPIO_NUM_12 // Pin 12: Digital Input
#define R_MOTOR_PWM GPIO_NUM_13 // Pin 13: Digital Input
// PIN_GND (Pin 14): GND - Ground
// PIN_VIN (Pin 15): VIN - Power
#define I2C_MASTER_SCL_IO GPIO_NUM_22 // Pin 17: I2C_SCL
#define PIN_RX0 GPIO_NUM_44 // Pin 19: Digital I/O
#define I2C_MASTER_SDA_IO GPIO_NUM_21 // Pin 20: I2C_SDA
#define H_BRIDGE_L_ENABLE GPIO_NUM_5 // Pin 23: Output
#define H_BRIDGE_L_IN1 GPIO_NUM_17 // Pin 24: Digital Output
#define H_BRIDGE_L_IN2 GPIO_NUM_16 // Pin 25: Digital Output
#define H_BRIDGE_R_ENABLE GPIO_NUM_4 // Pin 26: Digital Output
#define H_BRIDGE_R_IN1 GPIO_NUM_2 // Pin 27: Digital Output
#define H_BRIDGE_R_IN2 GPIO_NUM_15 // Pin 28: Digital Output
// PIN_GND (Pin 29): GND - Ground
// PIN_3V3 (Pin 30): 3V3 - Power

/* *** END PIN ASSIGNMENT DEFINES *** */

/* *** PRIVATE DEFINES START *** */
#define THE_SKY_IS_BLUE 1
/* *** PRIVATE DEFINES END *** */

/* *** INITIALIZE FUNCTIONS DECLARATIONS START *** */
void Init_I2C(void ); //I2C Initializer



/* *** END INITIALIZE FUNCTIONS DECLARATIONS *** */
