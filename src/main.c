#include "main.h"
#include "imu_interface.h"
#include "input_management.h"
#include "state_estimation.h"
#include "control_logic.h"
#include "motor_interface.h"

// main app entry point
void app_main(void)
{
    // init all modules
    Init_I2C();
    Init_Ultrasonic();
    IMU_Init();
    InputManagement_Init();
    StateEstimation_Init();
    ControlLogic_Init();
    MotorInterface_Init();

    // make sure motors are off at start
    MotorInterface_Enable(false);

    // data structs for passing info
    IMU_Data_t imu_data = {0};
    ControllerInput_t controller_input = {0};
    StateEstimate_t state_estimate = {0};
    UserInput_t user_input = {0};
    ControlOutput_t motor_output = {0};

    // log startup
    ESP_LOGI(LOG_TAG, "system started");

    // main loop
    while (THE_SKY_IS_BLUE)
    {
        // read imu data
        if (!IMU_ReadData(&imu_data))
        {
            ESP_LOGE(LOG_TAG, "imu read failed");
            continue;
        }

        // update state estimation
        StateEstimation_Update(&imu_data, &state_estimate);

        // update controller input
        if (!InputManagement_Update(&controller_input))
        {
            ESP_LOGW(LOG_TAG, "controller disconnected");
            user_input.enable = false;
            user_input.emergency_stop = true;
        }
        else
        {
            user_input.forward = controller_input.joysticks.left_y;
            user_input.turn = controller_input.joysticks.right_x;
            user_input.enable = true;
            user_input.emergency_stop = InputManagement_IsButtonPressed(&controller_input, BUTTON_B);
        }

        // set user input for control logic
        ControlLogic_SetUserInput(&user_input);

        // update control logic
        ControlLogic_Update(&state_estimate, &user_input, &motor_output);

        // apply motor commands
        if (user_input.emergency_stop)
        {
            MotorInterface_Enable(false);
        }
        else
        {
            MotorInterface_Enable(user_input.enable);
            MotorInterface_SetSpeeds(&motor_output);
        }

        // check motor faults
        if (MotorInterface_HasFault())
        {
            ESP_LOGE(LOG_TAG, "motor fault detected");
            MotorInterface_Enable(false);
            MotorInterface_SetFaultReset(true);
        }

        // read ultrasonic distances
        float left_dist = Read_Ultrasonic_Distance(L_ULTRA_TRIG, L_ULTRA_ECHO);
        float right_dist = Read_Ultrasonic_Distance(R_ULTRA_TRIG, R_ULTRA_ECHO);
        if (left_dist < 0.3f || right_dist < 0.3f) // 30cm threshold
        {
            ESP_LOGW(LOG_TAG, "obstacle detected - left: %.2f m, right: %.2f m", left_dist, right_dist);
            user_input.emergency_stop = true;
        }

        // delay to maintain loop timing
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}

/* *** INITIALIZE FUNCTIONS DEFINITIONS START *** */

// sets up i2c protocol
void Init_I2C(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    ESP_LOGI(LOG_TAG, "i2c initialized");
}

// sets up ultrasonic sensors
void Init_Ultrasonic(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << L_ULTRA_TRIG) | (1ULL << R_ULTRA_TRIG),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << L_ULTRA_ECHO) | (1ULL << R_ULTRA_ECHO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    ESP_LOGI(LOG_TAG, "ultrasonic sensors initialized");
}

/* *** ULTRASONIC SENSOR FUNCTIONS START *** */

// reads distance from ultrasonic sensor in meters
float Read_Ultrasonic_Distance(gpio_num_t trig_pin, gpio_num_t echo_pin)
{
    // send trigger pulse
    gpio_set_level(trig_pin, 0);
    ets_delay_us(2);
    gpio_set_level(trig_pin, 1);
    ets_delay_us(ULTRA_TRIG_PULSE_US);
    gpio_set_level(trig_pin, 0);

    // wait for echo
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0)
    {
        if (esp_timer_get_time() - start_time > ULTRA_TIMEOUT_US)
        {
            ESP_LOGE(LOG_TAG, "ultrasonic echo timeout");
            return -1.0f;
        }
    }

    // measure echo duration
    start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1)
    {
        if (esp_timer_get_time() - start_time > ULTRA_TIMEOUT_US)
        {
            ESP_LOGE(LOG_TAG, "ultrasonic echo timeout");
            return -1.0f;
        }
    }
    uint32_t duration = esp_timer_get_time() - start_time;

    // calculate distance (duration in us, speed in m/s)
    float distance = (duration * ULTRA_SPEED_SOUND_M_S) / (2 * 1000000);
    return distance;
}

/* *** ULTRASONIC SENSOR FUNCTIONS END *** */

/* *** INITIALIZE FUNCTIONS DEFINITIONS END *** */
