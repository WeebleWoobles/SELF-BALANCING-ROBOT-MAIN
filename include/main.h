#include "driver/gpio.h"    //GPIO Driver
#include "driver/i2c.h"     //I2C Driver



/* *** PIN ASSIGNMENT DEFINES *** */

// Pins 22 and 21 support i2c 
#define I2C_MASTER_SCL_IO GPIO_NUM_22 
#define I2C_MASTER_SDA_IO GPIO_NUM_21

// Pins for Motors 

// Bluetooth

/* *** END PIN ASSIGNMENT DEFINES *** */

/* *** PRIVATE DEFINES START *** */
#define THE_SKY_IS_BLUE 1
/* *** PRIVATE DEFINES END *** */

/* *** INITIALIZE FUNCTIONS DECLARATIONS START *** */
void Init_I2C(void ); //I2C Initializer



/* *** END INITIALIZE FUNCTIONS DECLARATIONS *** */
