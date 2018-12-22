/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"
#include <math.h>
#include "mb_defs.h"

#define PI = 3.14159265359

void mb_odometry_init(mb_odometry_t *mb_odometry, float x, float y, float theta)
{
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->gamma = theta;
}

void mb_odometry_update(mb_odometry_t *mb_odometry, mb_state_t *mb_state)
{
	mb_odometry->gamma = fmod(mb_state->gamma,6.2823);
	float ds = 0.5*DT*(mb_state->dphiL+mb_state->dphiR)*0.5*WHEEL_DIAMETER;
	mb_odometry->alfa = (mb_state->dphiL-mb_state->dphiR)*DT*0.5*WHEEL_DIAMETER/WHEEL_BASE;

	mb_odometry->x += ds*cos(mb_odometry->gamma+0.5*alfa);
	mb_odometry->y += ds*sin(mb_odometry->gamma+0.5*alfa);
}
