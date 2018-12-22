#ifndef BB_H
#define BB_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <string.h>
#include <math.h> // for M_PI
#include <signal.h>
#include <pthread.h>
#include <rc/mpu.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motor.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"
#include "../optitrack/common/serial.h"
#include <sys/ioctl.h>
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_gate_t.h"
#include "../lcmtypes/balancebot_msg_t.h"

// global variables
rc_mpu_data_t mpu_data;
pthread_mutex_t state_mutex;
pthread_mutex_t setpoint_mutex;
pthread_mutex_t optitrack_mutex;
pthread_mutex_t odometry_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;
mb_optitrack_data_t mb_optitrack_data;
pid_gains_t pid_gains;

// functions
void balancebot_controller();
void getData(balancebot_msg_t* BBmsg, int fd);
int setAutonomousPlan(int plan);

//threads
void *motion_capture_loop(void *ptr);
void *setpoint_control_loop(void *ptr);
void *printf_loop(void *ptr);

// stack functionality
int push(plan_t* plan,mb_waypoint_t wp);
int pop(plan_t* plan);
void display(plan_t* plan); 
#endif
