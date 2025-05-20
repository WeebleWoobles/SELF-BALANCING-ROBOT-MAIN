#include "main.h"
//pin assignments look at main.h



void app_main() {
    //Initialize all things
    Init_I2C();



    // Infinate while loop
    while(THE_SKY_IS_BLUE){

    } ;
}




/* *** INITIALIZE FUNCTIONS DEFINITIONS START *** */

   
//This function initializes i2c protocal with the defined pin in main.h
void Init_I2C(void) {
     i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}


// Sensor Interface Module (IMU Data Acquisition)
//
// This module will be responsible for interfacing with the IMU sensor
// and acquiring the necessary data for the robot's balancing algorithm.
// It will handle the I2C communication and data processing.
// The module will include functions for initializing the sensor, reading
// data, and converting raw data into usable formats.
//
//  HAL/Scheduler



// State Estimation and Sensor Fusion (Tilt, Angular Velocity, etc.)
//
// This module will be responsible for estimating the robot's state
// using the data acquired from the IMU sensor. It will implement
// sensor fusion algorithms (e.g., Kalman filter, complementary filter)
// to combine data from multiple sensors and provide accurate estimates
// of tilt angles, angular velocities, and other relevant parameters.
//
//  Scheduler



// Main Control Logic (Balancing and Movement - PID Control)
//
// This module will implement the main control logic for the robot.
// It will use the estimated state from the previous module to determine
// the necessary motor commands for balancing and movement. The control
// logic will include PID control algorithms to adjust motor speeds
// based on the robot's tilt angle and angular velocity. It will also
// handle user inputs (e.g., from a remote control or Bluetooth) to
// control the robot's movement and direction.
//
//  Scheduler/Config



// Motor Driver Interface (Actuator Control)
//
// This module will be responsible for controlling the motors of the robot.
// It will interface with the motor driver hardware and implement
// functions for setting motor speeds and directions. The module will
// receive commands from the main control logic and translate them into
// appropriate signals for the motor driver. It will also include
// functions for monitoring motor status and handling any errors or
// faults that may occur during operation.
//
//  HAL/Scheduler



// Input Management Module (Bluetooth Xbox Controller Interface)
//
// This module will handle the input from the Xbox controller.
// It will interface with the controller hardware and implement
// functions for reading button presses, joystick movements, and other
// inputs. The module will convert the raw input data into usable
// formats and provide it to the main control logic. It will also
// handle any necessary debouncing or filtering of the input signals.
//
//  HAL/Scheduler



/* *** INITIALIZE FUNCTIONS DEFINITIONS END *** */
