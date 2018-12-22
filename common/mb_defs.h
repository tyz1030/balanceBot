/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000   // period of motor drive pwm
#define LEFT_MOTOR              1       // id of left motor
#define RIGHT_MOTOR             2       // id of right motor
#define MDIR1_CHIP              1
#define MDIR1_PIN               28      // gpio1.28  P9.12
#define MDIR2_CHIP              1
#define MDIR2_PIN               16      // gpio1.16  P9.15
#define MOT_BRAKE_EN            0,20    // gpio0.20  P9.41
#define MOT_1_POL               1       // polarity of motor 1
#define MOT_2_POL               -1      // polarity of motor 2
#define ENC_1_POL               1       // polarity of encoder 1
#define ENC_2_POL               -1      // polarity of encoder 2
#define MOT_1_CS                0       // analog in of motor 1 current sense
#define MOT_2_CS                1       // analog in of motor 2 current sense
#define GEAR_RATIO              20.4    // gear ratio of motor
#define ENCODER_RES             48.0    // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER          0.08    // diameter of wheel in meters
#define WHEEL_BASE              0.208   // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     0.1     // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    0.1     // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ          100     // main filter and control loop speed
#define DT                      0.01    // 1/sample_rate
#define PRINTF_HZ               10      // rate of print loop
#define RC_CTL_HZ               25      // rate of RC data update
#define FORWARD			        1       // DIR Pin is high
#define REVERSE			        0       // DIR Pin is low
#define Encoder2phiR            0.006417 //count to rad
#define Encoder2phiL            -0.006417 //count to rad
#define CS_PIN_V2I_RATIO        2       // 500 mV is 1 A of current
#define SOFT_START_SEC		    0.7
#define MAX_THETA               0.33    // maximum body tilt allowed when moving
#define STEERING_INPUT_MAX	    0.5     // saturation paramter for steering pid controller

#define BALANCE                 0       // ID for selection of inner loop in filter array
#define POSITION                1       // ID for selection of position loop in filter array
#define STEERING                2       // ID for selection in steering loop in filter array
#define DISTANCE                3       // ID for selection in distance loop in filter array
#define HEADING                 4       // ID for selection in heading loop in filter array

#define DSM_DRIVE_CH            1  
#define DSM_TURN_CH             2
#define DSM_MODE_CH             6
#define DSM_KILL_CH             5

#define DSM_DEAD_ZONE		    0.04
#define DRIVE_RATE	            16
#define TURN_RATE	            6

#define LOG_RESPONSE

#define STACK_SIZE              100
#define MOVE                    0
#define TURN                    1
#define DRIVE_SQUARE_SIDE       0.2    // side lenght of the square is 0.2m
#define TURN90                  1.5708 // PI/2
#define DRIVE_20_CM             0.2
#define METERS2RADIANS          1 / (WHEEL_DIAMETER / 2)

#define FWD_VELOCITY_MAX        1 * DRIVE_RATE
#define TURN_VELOCITY_MAX       1 * TURN_RATE
#define DIST_TOL                 0.03 // 3 cm
#define GAMMA_TOL               0.05 // 3 deg           

#define GYRO_ODOMETRY_TOL		0.2  //a threshold about 12 degree
#endif