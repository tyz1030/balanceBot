/*******************************************************************************

*
* Main template code for the BalanceBot Project
* based on rc_balance
*
*******************************************************************************/

#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "math.h"
#include "balancebot.h"
#include <stdio.h>
#include <stdlib.h>

rc_filter_t *PIDS[5];
rc_filter_t D1 = RC_FILTER_INITIALIZER;
rc_filter_t D2 = RC_FILTER_INITIALIZER;
rc_filter_t D3 = RC_FILTER_INITIALIZER;
rc_filter_t D4 = RC_FILTER_INITIALIZER;
rc_filter_t D5 = RC_FILTER_INITIALIZER;

// Optitrack global variables
const int baudRate = 57600;
const char port[] = "/dev/ttyO1";
const int num_gates = 4;
int fd;
int bytes_avail = 0;

/* Variables related to navigation */
int autonomousPlan = 0;
plan_t instructions;
int initializeAutonomousPlan = 0;
target_t target;
/* Variables related to navigation */

#ifdef LOG_RESPONSE
static char *filename = "log.csv";
#endif

/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(int argc, char **argv)
{
	// Read in command line arguments
	// Any arguments provided will be interpreted as the autonomous plan to choose
	if (argc == 2)
	{
		//printf("%d\n", atoi(argv[1]));
		autonomousPlan = atoi(argv[1]);
	}

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if (rc_kill_existing_process(2.0) < -2)
		return -1;

	// start signal handler so we can exit cleanly
	if (rc_enable_signal_handler() == -1)
	{
		fprintf(stderr, "ERROR: failed to start signal handler\n");
		return -1;
	}

	if (rc_cpu_set_governor(RC_GOV_PERFORMANCE) < 0)
	{
		fprintf(stderr, "Failed to set governor to PERFORMANCE\n");
		return -1;
	}

	// initialize enocders
	if (rc_encoder_eqep_init() == -1)
	{
		fprintf(stderr, "ERROR: failed to initialize eqep encoders\n");
		return -1;
	}

	// initialize adc
	if (rc_adc_init() == -1)
	{
		fprintf(stderr, "ERROR: failed to initialize adc\n");
		return -1;
	}

	// initialize receiver for remote control communicaiton
	if (rc_dsm_init() == -1)
	{
		fprintf(stderr, "failed to start initialize DSM\n");
		return -1;
	}

	//open serial port non-blocking for optitrack system
    fd = serial_open(port,baudRate,0);
    if(fd == -1){
        printf("Failed to open Serial Port: %s", port);
        return -1;
    }

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	mb_setpoints.phi = 0;
	mb_setpoints.gamma = 0;
	mb_setpoints.manual_ctl = 1;

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void *)NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void *)NULL, SCHED_FIFO, 50);

	// start motio capture thread
	// Priority of 49 (which means thread has lower priority than start control thread)
	printf("starting motion capture thread... \n");
	pthread_t motion_capture_thread;
	rc_pthread_create(&motion_capture_thread, motion_capture_loop, (void *)NULL, SCHED_FIFO, 50);

	// set up IMU configuration
	printf("initializing imu... \n");

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_config))
	{
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
	pthread_mutex_init(&state_mutex, NULL);
	pthread_mutex_init(&setpoint_mutex, NULL);
	pthread_mutex_init(&optitrack_mutex, NULL);
	pthread_mutex_init(&odometry_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	PIDS[BALANCE] = &D1;
	PIDS[POSITION] = &D2;
	PIDS[STEERING] = &D3;
	PIDS[DISTANCE] = &D4;
	PIDS[HEADING] = &D5;

	mb_controller_init(PIDS, &pid_gains);

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0, 0.0, 0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	// Keep looping until state changes to EXITING
	while (rc_get_state() != EXITING)
	{

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}

	// exit cleanly
	serial_close(fd);
	rc_filter_free(PIDS[BALANCE]);
	rc_filter_free(PIDS[POSITION]);
	rc_filter_free(PIDS[STEERING]);
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST
	return 0;
}

/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
*
*
*******************************************************************************/
void balancebot_controller()
{
	//lock state mutex
	pthread_mutex_lock(&state_mutex);

	// Read IMU
	float lastTheta = mb_state.theta;
	mb_state.theta = -mpu_data.dmp_TaitBryan[TB_PITCH_X];

	// Read encoders
	float lastLeftEncoder = mb_state.left_encoder;
	float lastRightEncoder = mb_state.right_encoder;
	mb_state.left_encoder = rc_encoder_eqep_read(1);
	mb_state.right_encoder = rc_encoder_eqep_read(2);

	mb_state.dtheta = (mb_state.theta - lastTheta) / (DT);
	mb_state.dphiL = (mb_state.left_encoder - lastLeftEncoder) * Encoder2phiL / (DT);
	mb_state.dphiR = (mb_state.right_encoder - lastRightEncoder) * Encoder2phiR / (DT);

	// Calculate controller outputs
	mb_state.wheelAngleL = mb_state.left_encoder * Encoder2phiL;
	mb_state.wheelAngleR = mb_state.right_encoder * Encoder2phiR;
	mb_state.phi = (mb_state.wheelAngleL + mb_state.wheelAngleR) / 2 + mb_state.theta;

	// steering angle gamma estimate
	mb_state.gamma = (mb_state.wheelAngleR-mb_state.wheelAngleL) * (0.5*WHEEL_DIAMETER/WHEEL_BASE);
	//mb_state.gamma = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	mb_odometry_update(mb_odometry, mb_state);

	if (mb_setpoints.manual_ctl == 2)
	{
		double remainingDist = sqrt(pow(target.x - mb_odometry.x, 2) + pow(target.y - mb_odometry.y, 2));
		mb_setpoints.fwd_velocity = rc_filter_march(PIDS[DISTANCE], remainingDist * METERS2RADIANS);
		mb_setpoints.turn_velocity = rc_filter_march(PIDS[HEADING], (target.gamma - mb_odometry.gamma));
	}
	if (fabs(mb_setpoints.fwd_velocity) > 0.001)
    {
        mb_setpoints.phi += mb_setpoints.fwd_velocity * DT;
    }
    if (fabs(mb_setpoints.turn_velocity) > 0.0001)
    {
        mb_setpoints.gamma += mb_setpoints.turn_velocity * DT;
    }
	//printf("Forward Velocity: %f\n", mb_setpoints.fwd_velocity);

#ifdef LOG_RESPONSE
    FILE *f1 = fopen(filename, "a");
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t pwm_time = (int64_t)(tv.tv_sec * 1000000 + tv.tv_usec) / 1000000;
    fprintf(f1, "%lld, %f, %f, %f\n", pwm_time, mb_odometry-> x, mb_odometry-> y, mb_odometry->gamma);
    fclose(f1);
#endif

	// steering angle gamma estimate
	//mb_state.gamma = (mb_state.wheelAngleR-mb_state.wheelAngleL) * (0.5*WHEEL_DIAMETER/WHEEL_BASE);
	//mb_state.gamma = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	//fusion of gyro and encoder heanding
	//if abs(delta gyro angle - delta encoder heading angle) > threshod, use gyro, else, use encoder
	float last_gyro_heading = mb_state.gyro_heading;
	mb_state.gyro_heading = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	mb_state.gyro_alfa = mb_state.gyro_heading - last_gyro_heading;
	if (mb_state.gyro_alfa > 3.1416)
	{
		mb_state.gyro_alfa = 6.2832 - (mb_state.gyro_heading - last_gyro_heading);
	}
	else if (mb_state.gyro_alfa < 3.1415926)
	{
		mb_state.gyro_alfa = 6.2832 + (mb_state.gyro_heading - last_gyro_heading);
	}


	if(fabs(mb_odometry.alfa-mb_state.gyro_alfa)>GYRO_ODOMETRY_TOL){
		mb_state.gamma += mb_state.gyro_alfa;
	}
	else{
		mb_state.gamma += mb_odometry.alfa;
	}

	mb_controller_update(&mb_state, &mb_setpoints, PIDS);

	//unlock state mutex
	pthread_mutex_unlock(&state_mutex);
}

void* motion_capture_loop(void* ptr)
{
	//construct message for storage
    balancebot_msg_t BBmsg;
    pose_xyt_t BBpose;
    balancebot_gate_t BBgates[num_gates];
    BBmsg.pose = BBpose;
    BBmsg.num_gates = num_gates;
    BBmsg.gates = BBgates;
    int packetLength = balancebot_msg_t_encoded_size(&BBmsg)+2;
    //printf("packetLength: %d\n", packetLength);

	while(1)
    {
		//printf("Inside mocap loop\n");
        //check bytes in serial buffer
        ioctl(fd, FIONREAD, &bytes_avail);
        //printf("bytes: %d\n",bytes_avail);
        if(bytes_avail >= packetLength){
            getData(&BBmsg, fd);

            pthread_mutex_lock(&optitrack_mutex);
            mb_optitrack_data.pose = BBmsg.pose;
            pthread_mutex_unlock(&optitrack_mutex);
        }
		rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}

/*******************************************************************************/
void *setpoint_control_loop(void *ptr)
{
	while (1)
	{
		// Read information from the receiver
		if (rc_dsm_is_new_data())
		{
			// Read normalized (+-1) inputs from RC radio stick
			double turn_stick = rc_dsm_ch_normalized(DSM_TURN_CH);
			double drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH);
			double mode_switch = rc_dsm_ch_normalized(DSM_MODE_CH);
			double kill_switch = rc_dsm_ch_normalized(DSM_KILL_CH);

			// Kill switch can be either 1 or 0.
			if (kill_switch > 0.9)
			{
				//printf("Kill switch enabled\n");
				mb_motor_disable();
			}

			//printf("Manual mode switch: %d\n", mb_setpoints.manual_ctl);

			// use a small deadzone to prevent slow drifts in position
			if (fabs(drive_stick) < DSM_DEAD_ZONE)
				drive_stick = 0.0;
			if (fabs(turn_stick) < DSM_DEAD_ZONE)
				turn_stick = 0.0;

			// translate normalized user input to real setpoint values
			pthread_mutex_lock(&setpoint_mutex);
			if (mode_switch  > 0.9 && mb_setpoints.manual_ctl != 2)
			{
				mb_setpoints.manual_ctl = 2; // autonomous control
				initializeAutonomousPlan = 1;
			}
			else if (mode_switch < -0.9)
			{
				mb_setpoints.manual_ctl = 0; // manual control
			}

			if (mb_setpoints.manual_ctl == 0)
			{
				mb_setpoints.fwd_velocity = DRIVE_RATE * drive_stick;
				mb_setpoints.turn_velocity = TURN_RATE * turn_stick;
			}
			pthread_mutex_unlock(&setpoint_mutex);

			//printf("Drive Stick: %7.3f   |   Turn Stick:   %7.3f\n", drive_stick, turn_stick);
		}
		else if (rc_dsm_is_connection_active() == 0)
		{
			//pthread_mutex_lock(&setpoint_mutex);
			//mb_setpoints.phi = 0;
			//pthread_mutex_unlock(&setpoint_mutex);
		}

		if (mb_setpoints.manual_ctl == 2)
		{
			if (initializeAutonomousPlan == 1)
			{
				printf("Starting Autonomous plan\n");
				if (setAutonomousPlan(autonomousPlan) == -1)
				{
					printf("Please increase autonomous plan buffer");
				}

				pthread_mutex_lock(&state_mutex);
				mb_state.phi = 0;
				mb_state.gamma = 0;
				pthread_mutex_unlock(&state_mutex);

				target.x = instructions.stack[instructions.top].dist * cos(target.gamma);
				target.y = instructions.stack[instructions.top].dist * sin(target.gamma);
				target.gamma = instructions.stack[instructions.top].heading;
				pop(&instructions);

				initializeAutonomousPlan = 0;
			}

			pthread_mutex_lock(&odometry_mutex);
			double remainingDist = sqrt(pow(target.x - mb_odometry.x, 2) + pow(target.y - mb_odometry.y, 2));
			if (remainingDist < DIST_TOL && fabs(mb_odometry.gamma - target.gamma) < GAMMA_TOL)
			{
				target.x += instructions.stack[instructions.top].dist * cos(target.gamma);
				target.y += instructions.stack[instructions.top].dist * sin(target.gamma);
				target.gamma += instructions.stack[instructions.top].heading;
				pop(&instructions);
			}
			pthread_mutex_unlock(&odometry_mutex);
		}

		rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}

/*******************************************************************************
* printf_loop()
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void *printf_loop(void *ptr)
{
	rc_state_t last_state, new_state; // keep track of last state
	while (rc_get_state() != EXITING)
	{
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if (new_state == RUNNING && last_state != RUNNING)
		{
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                         	      SENSORS                                  |");
			printf("\n");
			printf("    θ    |");
			printf("   dθ    |");
			printf("    φ    |");
			printf("   dφl   |");
			printf("   dφr   |");
			//printf("    L    |");
			//printf("    R    |");
			//printf("  L PWM  |");
			//printf("  R PWM  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			//printf("   D2θ   |")
			//printf(" Drive V |");

			printf("\n");
		}
		else if (new_state == PAUSED && last_state != PAUSED)
		{
			printf("\nPAUSED\n");
		}
		last_state = new_state;

		if (new_state == RUNNING)
		{
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.dtheta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7.3f  |", mb_state.dphiL);
			printf("%7.3f  |", mb_state.dphiR);
			pthread_mutex_unlock(&state_mutex);

			pthread_mutex_lock(&optitrack_mutex);
			printf("%7.3f  |", mb_optitrack_data.pose.x);
			printf("%7.3f  |", mb_optitrack_data.pose.y);
			printf("%7.3f  |", mb_optitrack_data.pose.theta);
			pthread_mutex_unlock(&optitrack_mutex);

			//printf("%7d  |", mb_state.left_encoder);
			//printf("%7d  |", mb_state.right_encoder);
			//printf("%7.3f  |", mb_state.left_cmd);
			//printf("%7.3f  |", mb_state.right_cmd);
			//printf("    0    |");
			fflush(stdout);
		}
		rc_nanosleep(1E9 / PRINTF_HZ);
	}
	return NULL;
}

void getData(balancebot_msg_t* BBmsg, int fd){
    char *ptr;
    int packetLength = balancebot_msg_t_encoded_size(BBmsg)+2;
    char *dataPacket = (char*) malloc (packetLength);
    int bytes_avail = 0;

    const char headByte = 0x1B;
    const char tailByte = 0xFF;
    int err_counter = 0;

    ptr = dataPacket;
    while(read(fd, ptr, 1) > 0){
        // if the first Byte is wrong keep looking
        if((ptr == dataPacket)&&(*ptr != headByte)){
            continue;
        }
        ptr++;
        // Once we have all of the Bytes check to make sure first and last are good
        if((ptr-dataPacket) == packetLength){
            if((dataPacket[0] != headByte) || (dataPacket[packetLength-1] != tailByte)){
                err_counter += 1;
            }
            else{
                //packet is good, decode it into BBmsg
                int status = balancebot_msg_t_decode(dataPacket, 1, packetLength-2, BBmsg);
                if (status < 0) {
                    fprintf (stderr, "error %d decoding balancebot_msg_t!!!\n", status);;
                }
                // if we have less than a full message in the serial buffer
                // we are done, we'll get the next one next time
                ioctl(fd, FIONREAD, &bytes_avail);
                if(bytes_avail < packetLength){
                    break;
                }
            }
            //keep reading until buffer is almost empty
            ptr = dataPacket;
        }
    }
}

int pop(plan_t* plan)
{
	if (plan->top <= -1)
	{
		printf("The stack has no elements to pop!\n");
		return -1;
	}
	else
	{
		plan->top--;
	}
	return 0;
}

int push(plan_t* plan, mb_waypoint_t wp)
{
	if (plan->top >= plan->size-1)
	{
		printf("There is no more room on the stack!\n");
		return -1;
	}
	else
	{
		plan->top++;
		plan->stack[plan->top] = wp;
	}
	return 0;
}

void display(plan_t* plan)
{
	if(plan->top < 0)
	{
		printf("The stack is empty!\n");
	}
	else
	{
		printf("Items in stack: %d\n\n", plan->top + 1);
		for (int i=plan->top; i >= 0; i--)
		{
			printf("--- Item %d---\n", i);
			printf("Type: %d\n", plan->stack[i].type);
			printf("Dist: %f\n", plan->stack[i].dist);
			printf("Heading: %f\n", plan->stack[i].heading);
			printf("--------------\n");
		}
	}
}

int setAutonomousPlan(int plan)
{
	// Initialize plans
	if (plan == 1)
	{
		instructions.size = STACK_SIZE;
		instructions.top = -1;

		// Push onto the stack in reverse order
		for (int i = 0; i < 4; ++i)
		{
			int ret;
			mb_waypoint_t turn;
			turn.type = TURN;
			turn.heading = TURN90; // 90 degree turn
			turn.dist = 0;
			ret = push(&instructions, turn);
			if (ret == -1)
			{
				return -1;
			}

			mb_waypoint_t move;
			move.type = MOVE;
			move.dist = 1;
			move.heading = 0;
			ret = push(&instructions, move);
			if (ret == 1)
			{
				return -1;
			}
		}
		//display(&instructions);

	}

	if (plan == 2)
	{
		instructions.size = STACK_SIZE;
		instructions.top = -1;

		int ret;
		mb_waypoint_t move;
		move.type = MOVE;
		move.dist = 10; // Drive 3 meter
		move.heading = 0;
		ret = push(&instructions, move);
		if (ret == -1)
		{
			return -1;
		}
	}

	if (plan == 3)
	{
		instructions.size = STACK_SIZE;
		instructions.top = -1;

		int ret;
		mb_waypoint_t turn;
		turn.type = TURN;
		turn.heading = TURN90;
		turn.dist = 0;
		ret = push(&instructions, turn);
		if (ret == -1)
		{
			return -1;
		}
	}
	return 0;
}

