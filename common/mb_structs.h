#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_gate_t.h"
#include "mb_defs.h"

typedef struct mb_state mb_state_t;
struct mb_state
{
    // raw sensor inputs
    float theta;       // body angle (rad)
    float phi;         // average wheel angle (rad)
    float gamma;       // heading angle
    float gyro_heading // gyro heading angle read

    int left_encoder;  // left encoder counts since last reading
    int right_encoder; // right encoder counts since last reading

    float wheelAngleR;
    float wheelAngleL;

    //output
    float left_cmd;  //left wheel command [-1..1]
    float right_cmd; //right wheel command [-1..1]

    float dtheta; // velocity of body angle (rad)
    float dphiL;  // velocity of wheel angle (rad) left
    float dphiR;  // velocity of wheel angel (rad) right

    float pwmHeadingL; //turning cmd left pwm
    float pwmHeadingR; //turning cmd right pwm

    float pwmForward;  //forward cmd pwm

};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints
{
    double phi;   // wheel position (rad)
    double gamma;   // heading degree

    float fwd_velocity;  // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry
{

    float x;   // x position from initialization in m
    float y;   // y position from initialization in m
    float alfa;//change of heading angle in one step in rad
    float gamma; // orientation from initialization in rad
};

typedef struct pid_gains pid_gains_t;
struct pid_gains
{
    float innerloop[3]; // inner loop gains
    float philoop[3];   // position loop gains
    float yawloop[3];   // heading loop gains
    float distloop[3];  // distance loop gains
    float headingloop[3]; // heading loop gains
};

typedef struct mb_optitrack_data mb_optitrack_data_t;
struct mb_optitrack_data
{
    int64_t utime;

    // Pose data
    pose_xyt_t pose;

    balancebot_gate_t* gates;
};

typedef struct mb_waypoint mb_waypoint_t;
struct mb_waypoint
{
    int     type;       // 0 equals move, 1 equals turn
    double  dist;       // distance to move to new waypoint
    double  heading;    // heading to turn to new waypoint
};

typedef struct plan plan_t;
struct plan
{
    int             top;
    mb_waypoint_t   stack[STACK_SIZE];
    int             size;
};

typedef struct target target_t;
struct target
{
    double          x;
    double          y;
    double          gamma;
};
#endif
