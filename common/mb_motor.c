/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect(!!(x), 0)

// global initialized flag
static int init_flag = 0;
static int disable_flag = 0;

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init()
{

    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz)
{

    // There are three subsystems in total [0, 1, 2] and each subsystem has
    // a pair of channels, A and B. Each subsystem shares the same period
    // but can have different duty cycles.

    // Both motors are on subsystem 1
    if (rc_pwm_init(1, pwm_freq_hz))
        return -1;

    if (rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT))
        return -1;
    if (rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT))
        return -1;
    if (rc_gpio_init(MOT_BRAKE_EN_PIN, GPIOHANDLE_REQUEST_OUTPUT))
        return -1;
    if (rc_adc_init())
        return -1;

    init_flag = 1;
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup()
{

    if (unlikely(!init_flag))
    {
        fprintf(stderr, "ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }

    // Not strictly necessary since linux takes care of cleanup
    //rc_gpio_cleanup(MDIR1_CHIP, MDIR1_PIN);
    //rc_gpio_cleanup(MDIR2_CHIP, MDIR2_PIN);

    if (rc_adc_cleanup())
        return -1;
    if (rc_pwm_cleanup(1))
        return -1;

    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en)
{

    if (unlikely(!init_flag))
    {
        fprintf(stderr, "ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }

    return rc_gpio_set_value(MOT_BRAKE_EN_PIN, brake_en);
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable()
{

    if (unlikely(!init_flag))
    {
        fprintf(stderr, "ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }

    if (rc_pwm_set_duty(1, 'A', 0))
        return -1;
    if (rc_pwm_set_duty(1, 'B', 0))
        return -1;
    disable_flag = 1;

    return 0;
}

/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty)
{

    if (unlikely(!init_flag))
    {
        fprintf(stderr, "ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    if (disable_flag != 1)
    {
        if (motor == RIGHT_MOTOR)
        {
            int direction = (duty < 0) ? FORWARD : REVERSE;
            //printf("Duty: %f\n", duty);
            //printf("Direction: %d\n", direction);
            if (rc_gpio_set_value(MDIR2_CHIP, MDIR2_PIN, direction))
                return -1;
            if (rc_pwm_set_duty(1, 'B', fabs(duty)))
                return -1;
        }
        else
        {
            //printf("Adjusting Left Wheel\n");
            //printf("Duty: %f\n", duty);
            int direction = (duty < 0) ? REVERSE : FORWARD;
            //printf("Direction: %d\n", direction);
            if (rc_gpio_set_value(MDIR1_CHIP, MDIR1_PIN, direction))
                return -1;
            if (rc_pwm_set_duty(1, 'A', fabs(duty)))
                return -1;
        }
    }

    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty)
{

    if (unlikely(!init_flag))
    {
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    if (disable_flag != 1)
    {
        int direction_R = (duty < 0) ? FORWARD : REVERSE;
        int direction_L = (duty < 0) ? REVERSE : FORWARD;

        if (rc_gpio_set_value(MDIR1_CHIP, MDIR1_PIN, direction_L))
            return -1;
        if (rc_gpio_set_value(MDIR2_CHIP, MDIR2_PIN, direction_R))
            return -1;
        if (rc_pwm_set_duty(1, 'A', fabs(duty)))
            return -1;
        if (rc_pwm_set_duty(1, 'B', fabs(duty)))
            return -1;
    }

    return 0;
}

/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor)
{
    //DRV8801 driver board CS pin puts out 500mV/A

    double current;

    if (motor == RIGHT_MOTOR)
    {
        current = rc_adc_read_volt(MOT_2_CS) * CS_PIN_V2I_RATIO;
    }
    else
    {
        current = rc_adc_read_volt(MOT_1_CS) * CS_PIN_V2I_RATIO;
    }

    return current;
}
