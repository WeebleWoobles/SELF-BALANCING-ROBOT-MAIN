#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>        //GPIO Driver
#include <driver/i2c.h>         //I2C Driver
#include "imu_interface.h"      //IMU Interface
#include "state_estimation.h"   //State Estimation
#include "control_logic.h"      //Control Logic
#include "input_management.h"   //Input Management
#include "motor_interface.h"    //Motor Interface
#include <freertos/FreeRTOS.h>  
#include <freertos/task.h>      
#include <esp_log.h>            //ESP LOGGING
/* *** ADDITIONAL INCLUDES END *** */

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
#define I2C_MASTER_SCL_IO GPIO_NUM_22 // Pin 17: I2C_SCL
#define PIN_RX0 GPIO_NUM_44 // Pin 19: Digital I/O
#define I2C_MASTER_SDA_IO GPIO_NUM_21 // Pin 20: I2C_SDA
#define H_BRIDGE_L_ENABLE GPIO_NUM_5 // Pin 23: Output
#define H_BRIDGE_L_IN1 GPIO_NUM_17 // Pin 24: Digital Output
#define H_BRIDGE_L_IN2 GPIO_NUM_16 // Pin 25: Digital Output
#define H_BRIDGE_R_ENABLE GPIO_NUM_4 // Pin 26: Digital Output
#define H_BRIDGE_R_IN1 GPIO_NUM_2 // Pin 27: Digital Output
#define H_BRIDGE_R_IN2 GPIO_NUM_15 // Pin 28: Digital Output

/* *** END PIN ASSIGNMENT DEFINES *** */

/* *** I2C CONFIGURATION DEFINES START *** */
#define I2C_NUM_0           I2C_NUM_0      //I2C PORT NUMBER
#define I2C_MASTER_FREQ_HZ  100000     //I2C CLOCK SPEED (100KHZ)
/* *** I2C CONFIGURATION DEFINES END *** */

/* *** ULTRASONIC SENSOR DEFINES START *** */
#define ULTRA_TRIG_PULSE_US 10              // trigger pulse duration
#define ULTRA_TIMEOUT_US    30000           // echo response timeout
#define ULTRA_SPEED_SOUND_M_S 343           // dfine ultrasound speed
/* *** ULTRASONIC SENSOR DEFINES END *** */

/* *** MOTOR CONTROL DEFINES START *** */
#define MOTOR_CONTROL_MODE_SPEED    0       //speed control mode
#define MOTOR_CONTROL_MODE_POSITION 1       //position control mode
#define MOTOR_FAULT_MODE_STOP       0       //stop on fault
#define MOTOR_FAULT_MODE_CONTINUE   1       //continue on fault
/* *** MOTOR CONTROL DEFINES END *** */

/* *** TIMING DEFINES START *** */
#define CONTROL_LOOP_PERIOD_MS      10      //controrl loop period
/* *** TIMING DEFINES END *** */

/* *** LOGGING DEFINES START *** */
#define LOG_TAG "ROBOT_MAIN"                //esp logging tag to tell which modules our logs came from
/* *** LOGGING DEFINES END *** */

/* *** PRIVATE DEFINES START *** */
#define THE_SKY_IS_BLUE 1
/* *** PRIVATE DEFINES END *** */

/* *** INITIALIZE FUNCTIONS DECLARATIONS START *** */
void Init_I2C(void ); //I2C Initializer

/* *** ADDITIONAL FUNCTION DECLARATIONS START *** */
void Init_Ultrasonic(void );            //ultrasonic sensor init
float Read_Ultrasonic_Distance(gpio_num_t trig_pin, gpio_num_t echo_pin); //READ DISTANCE FROM ULTRASONIC SENSOR
/* *** ADDITIONAL FUNCTION DECLARATIONS END *** */

/* *** END INITIALIZE FUNCTIONS DECLARATIONS *** */
