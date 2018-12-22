/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

FILE* f1;

static int loopCount = 0;
static int rotations = 10;
static double l_pwm = 1.0;
static double r_pwm = 1.0;
static char* filename = "log.csv";

int64_t utime_now (void){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // initialize motors
    if(mb_motor_init()<0) {
        fprintf(stderr, "ERROR: failed to initialize mb_motors\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

    f1 = fopen(filename, "w+");

    rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING) {

        if (loopCount > 50000) {
            break;
        }

    	if(rc_get_state()==RUNNING) {
            mb_motor_brake(1);
            
            int64_t time = utime_now();
            int l_qeCount = rc_encoder_read(LEFT_MOTOR);
            double l_current = mb_motor_read_current(LEFT_MOTOR);
            int r_qeCount = rc_encoder_read(RIGHT_MOTOR);
            double r_current = mb_motor_read_current(RIGHT_MOTOR);
           
            if (rotations * GEAR_RATIO * ENCODER_RES < r_qeCount) {
                mb_motor_brake(0);
                r_pwm = 0.0;
                l_pwm = 0.0;
            }    
             
            fprintf(f1, "%lld, %d, %f, %f\n", time, l_qeCount, l_current, l_pwm);
            fprintf(f1, "%lld, %d, %f, %f\n", time, r_qeCount, r_current, r_pwm);
            
            /*  
            if(loopCount > 200) {
                break;
            }
            if (r_pwm < .999 && l_pwm < .999) {
                r_pwm += 0.001;
                l_pwm += 0.001;

            } else {
                loopCount++;
            }
            */

            mb_motor_set(RIGHT_MOTOR, r_pwm);
            mb_motor_set(LEFT_MOTOR, l_pwm);

        }

        rc_nanosleep(1E5);
        loopCount++;
        printf("%d\n", loopCount); 
    }

	// exit cleanly
	rc_encoder_eqep_cleanup();
    mb_motor_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}

