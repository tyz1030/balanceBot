#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "mb_odometry.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_init(rc_filter_t **PIDS, pid_gains_t *pid_gains)
{
    mb_controller_load_config(pid_gains);

    // set up D1 balance (theta) controller
    if (rc_filter_pid(PIDS[BALANCE],
                      pid_gains->innerloop[0],
                      pid_gains->innerloop[1],
                      pid_gains->innerloop[2],
                      4 * DT, DT))
    {
        fprintf(stderr, "ERROR in rc_balance, failed to make balance controller\n");
        return -1;
    }
    rc_filter_enable_saturation(PIDS[BALANCE], -1.0, 1.0);
    rc_filter_enable_soft_start(PIDS[BALANCE], SOFT_START_SEC);

    // set up D2 position (phi) controller
    if (rc_filter_pid(PIDS[POSITION],
                      pid_gains->philoop[0],
                      pid_gains->philoop[1],
                      pid_gains->philoop[2],
                      4 * DT, DT))
    {
        fprintf(stderr, "ERROR in rc_balance, failed to make position controller\n");
        return -1;
    }
    rc_filter_enable_saturation(PIDS[POSITION], -MAX_THETA, MAX_THETA);
    rc_filter_enable_soft_start(PIDS[POSITION], SOFT_START_SEC);

    // set up D3 steering (yaw) controller
    if (rc_filter_pid(PIDS[STEERING],
                      pid_gains->yawloop[0],
                      pid_gains->yawloop[1],
                      pid_gains->yawloop[2],
                      4 * DT, DT))
    {
        fprintf(stderr, "ERROR in rc_balance, failed to make steering controller\n");
        return -1;
    }
    rc_filter_enable_saturation(PIDS[STEERING], -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

    // set up D4 distance controller
    if (rc_filter_pid(PIDS[DISTANCE],
                      pid_gains->distloop[0],
                      pid_gains->distloop[1],
                      pid_gains->distloop[2],
                      4 * DT, DT))
    {
        fprintf(stderr, "ERROR in rc_balance, failed to make distance controller\n");
        return -1;
    }
    rc_filter_enable_saturation(PIDS[DISTANCE], -FWD_VELOCITY_MAX, FWD_VELOCITY_MAX);

    // set up D5 distance controller
    if (rc_filter_pid(PIDS[HEADING],
                      pid_gains->headingloop[0],
                      pid_gains->headingloop[1],
                      pid_gains->headingloop[2],
                      4 * DT, DT))
    {
        fprintf(stderr, "ERROR in rc_balance, failed to make heading controller\n");
        return -1;
    }
    rc_filter_enable_saturation(PIDS[HEADING], -TURN_VELOCITY_MAX, TURN_VELOCITY_MAX);

    //printf("PID Balance Paramters: %7.3f, %7.3f, %7.3f\n", pid_gains->innerloop[0], pid_gains->innerloop[1], pid_gains->innerloop[2]);
    //printf("PID Position Paramters: %7.3f, %7.3f, %7.3f\n", pid_gains->philoop[0], pid_gains->philoop[1], pid_gains->philoop[2]);
    //printf("PID Steering Paramters: %7.3f, %7.3f, %7.3f\n", pid_gains->yawloop[0], pid_gains->yawloop[1], pid_gains->yawloop[2]);
    //printf("PID Distance Paramters: %7.3f, %7.3f, %7.3f\n", pid_gains->distloop[0], pid_gains->distloop[1], pid_gains->distloop[2]);
    //printf("PID Heading Paramters: %7.3f, %7.3f, %7.3f\n", pid_gains->headingloop[0], pid_gains->headingloop[1], pid_gains->headingloop[2]);

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_load_config(pid_gains_t *pid_gains)
{
    FILE *file = fopen(CFG_PATH, "r");
    if (file == NULL)
    {
        printf("Error opening %s\n", CFG_PATH);
    }

    fscanf(file, "%f %f %f", &pid_gains->innerloop[0], &pid_gains->innerloop[1], &pid_gains->innerloop[2]);
    fscanf(file, "%f %f %f", &pid_gains->philoop[0], &pid_gains->philoop[1], &pid_gains->philoop[2]);
    fscanf(file, "%f %f %f", &pid_gains->yawloop[0], &pid_gains->yawloop[1], &pid_gains->yawloop[2]);
    fscanf(file, "%f %f %f", &pid_gains->distloop[0], &pid_gains->distloop[1], &pid_gains->distloop[2]);
    fscanf(file, "%f %f %f", &pid_gains->headingloop[0], &pid_gains->headingloop[1], &pid_gains->headingloop[2]);

    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
*
*
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t *mb_state, mb_setpoints_t *mb_setpoints, rc_filter_t **PIDS)
{
    /************************************************************
	* MIDDLE LOOP ANGLE Position controller D2
	* Input to D1 is position error
	*************************************************************/
    double theta = rc_filter_march(PIDS[POSITION], (mb_setpoints->phi - mb_state->phi));

    /************************************************************
	* INNER LOOP ANGLE Theta controller D1
	* Input to D1 is theta error.
	*************************************************************/
    mb_state->pwmForward = rc_filter_march(PIDS[BALANCE], (theta - mb_state->theta));
    //mb_state->left_cmd = mb_state->right_cmd;

    /**********************************************************
	* gama (steering) controller D3
	* move the setpoint gamma based on user input like phi
	***********************************************************/
    mb_state->pwmHeadingR = rc_filter_march(PIDS[STEERING], mb_setpoints->gamma - mb_state->gamma);
    mb_state->pwmHeadingL = -mb_state->pwmHeadingR;

    mb_state->right_cmd = mb_state->pwmHeadingR + mb_state->pwmForward;
    mb_state->left_cmd = mb_state->pwmHeadingL + mb_state->pwmForward;
    /**********************************************************
	* Send signal to motors
	***********************************************************/
    mb_motor_set(RIGHT_MOTOR, mb_state->right_cmd);
    mb_motor_set(LEFT_MOTOR, mb_state->left_cmd);

    return 0;
}

/*******************************************************************************
* int mb_controller_cleanup()
*
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup()
{
    return 0;
}
