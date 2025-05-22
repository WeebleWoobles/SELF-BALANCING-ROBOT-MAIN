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




/* *** INITIALIZE FUNCTIONS DEFINITIONS END *** */
