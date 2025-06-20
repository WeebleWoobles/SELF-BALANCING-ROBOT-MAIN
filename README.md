# SELF Balancing Robot

MCU (Microcontroller Unit): ESP-WROOM-32 
- chip type... ESP32
- Chip is ESP32-D0WD-V3 (revision v3.1)
- Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
- Crystal is 40MHz
- MAC: 14:33:5c:37:79:e8

IMU (Inertial Measurment Unit): GY-521 MPU-6050

Motor Driver: L298N
- Features: Dual H-Bridge, 2A per channel, 46V max voltage, 25W max power, 2.5A peak current
- Pinout: 
  - IN1: GPIO 32
  - IN2: GPIO 33
  - IN3: GPIO 25
  - IN4: GPIO 26
  - ENA: GPIO 27
  - ENB: GPIO 14
- Power: 5V to 35V, 2A to 46V, 25W max power
- Power Supply: 5V to 35V, 2A to 46V, 25W max power

Two Motors:
- Type: 30:1 Metal DC Geared-Down Motor 
- Brand: CQRobot
- Model: CQR37D12V64EN-D

## Design

### Pin Out

[Google Sheet Pin Out Diagram](https://docs.google.com/spreadsheets/d/1lCuJJPs_-ZYu7kVZD1LAmiCTTDlawlaXdf8EDxlrphQ/edit?usp=sharing)


### Software Diagram
![Software Block Diagram](./Software_Block_Diagram-Copy_of_Page-1.drawio.png)

### Hardware Diagram
_link// insert here_



## Steps to Write Code

### Balance the Robot
1. **imu_interface.c** Make sure inputs and outputs are correct.
2. **state_estimation.c** Use the data imu_interface.c to get the angles and angular velocities.
3. **control_logic.c** Use the angles and angular velocities to calculate the control signals.
4. **motor_interface.c** Use the control signals to control the motors to balance the robot.
5. **main.c** Test the robot by running the code from all source files and checking if it balances and moves correctly.

### Move the Robot
1. **input_management.c** Get the inputs from the user via Bluetooth Xbox controller.
2. **control_logic.c** Use the inputs to calculate the control signals.
3. **motor_interface.c** Use the control signals to control the motors to move forward, backward, and turn.
4. **main.c** Test the robot by running the code from all source files and checking if it balances and moves correctly.
