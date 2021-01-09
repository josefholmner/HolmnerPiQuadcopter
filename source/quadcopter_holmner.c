#include "quadcopter_holmner.h"


/* Private prototypes */
static void quadcopter_holmner_execute();
static void quadcopter_holmner_control_loop_delay();
static void quadcopter_holmner_performance_measure();


QUADCOPTER_HOLMNER quadcopter_holmner;

int quadcopter_holmner_init()
{
	int status = -1;

	printf("quadcopter_holmner: initializing...\n");

	//Init io_master
	status = io_master_init(&quadcopter_holmner.io_master);

	if(status >= 0)
	{
		//Init motor_control
		motor_control_init(&quadcopter_holmner.motor_control);

		printf("quadcopter_holmner: init successful.\n");
	}


	return status;
}

void quadcopter_holmner_fly()
{
	printf("quadcopter_holmner: fly starting...\n");

	//Arm ESC's
	motor_control_arm_esc_all();

	//set first loop time in microseconds
	quadcopter_holmner.last_loop_time_us = micros();

	//Software performance measuring stuff
	quadcopter_holmner.performance_ctrl_loop_counter = 0;
	quadcopter_holmner.performance_ctrl_start_time_us = micros();

	//Run main control loop
	while(GLOBAL_ABORT_FLAG == 0)
	{
		quadcopter_holmner_execute();

		//Execute debug performance measuring
		quadcopter_holmner_performance_measure();
	}
}

static void quadcopter_holmner_execute()
{
	//Execute IO_MASTER
	io_master_execute(&quadcopter_holmner.io_master);

	//Execute motor_control
	motor_control_execute(&quadcopter_holmner.motor_control,
						  &quadcopter_holmner.io_master);

	//Execute control loop delay
	quadcopter_holmner_control_loop_delay();
}

static void quadcopter_holmner_control_loop_delay()
{
	unsigned int loop_time_us;

	//Insert controlled delay so that CTRL_LOOP_DELTA_TIME_US is true
	loop_time_us = micros()-quadcopter_holmner.last_loop_time_us;

	if(loop_time_us < CTRL_LOOP_DELTA_TIME_US)
	{
		delayMicroseconds(CTRL_LOOP_DELTA_TIME_US-loop_time_us);
	}

	quadcopter_holmner.last_loop_time_us = micros();
}

static void quadcopter_holmner_performance_measure()
{
	quadcopter_holmner.performance_ctrl_loop_counter++;

	if(HIDE_CONTROL_LOOP_TOO_SLOW_WARNING == 0 && quadcopter_holmner.performance_ctrl_loop_counter == 100)
	{
		unsigned int ctrl_loop_delta_time = micros()-quadcopter_holmner.performance_ctrl_start_time_us;

		//Show warning if the program is running too slowly and does not achieve the CTRL_LOOP_DELTA_TIME_US demand
		if(ctrl_loop_delta_time > 1.1*CTRL_LOOP_DELTA_TIME_US*100)
		{
			printf("**************************************************\n");
			printf("WARNING: QUADCOPTER SOFTWARE IS RUNNING TOO SLOWLY!\n");
			printf("Last 100 cycles took %u μs \n", ctrl_loop_delta_time);
			printf("Wanted time for 100 cycles = 100*CTRL_LOOP_DELTA_TIME_US = %d μs\n", 100*CTRL_LOOP_DELTA_TIME_US);
			printf("Increase CTRL_LOOP_DELTA_TIME_US in settings area in main(),\n");
			printf("**************************************************\n");
		}

		quadcopter_holmner.performance_ctrl_loop_counter = 0;
		quadcopter_holmner.performance_ctrl_start_time_us = micros();
	}
}

void quadcopter_holmner_calibrate_acc_gyro()
{

	//Init acc/gyro calibration
	acc_gyro_calibration_init(&quadcopter_holmner.acc_gyro_calibration);

	while(GLOBAL_ABORT_FLAG == 0)
	{
		//Read acc/gyro data
		io_master_read_accelerometer_and_gyro(&quadcopter_holmner.io_master);

		//Execute acc/gyro calibration
		acc_gyro_calibration_execute(&quadcopter_holmner.acc_gyro_calibration,
									 quadcopter_holmner.io_master.acc_gyro_mpu6050_device.angle_x_deg_unfilt,
									 quadcopter_holmner.io_master.acc_gyro_mpu6050_device.angle_y_deg_unfilt,
									 quadcopter_holmner.io_master.acc_gyro_mpu6050_device.gyr_x_deg_unfilt,
									 quadcopter_holmner.io_master.acc_gyro_mpu6050_device.gyr_y_deg_unfilt,
									 quadcopter_holmner.io_master.acc_gyro_mpu6050_device.gyr_z_deg_unfilt);

		//Execute loop delay
		quadcopter_holmner_control_loop_delay();
	}
}

void quadcopter_holmner_show_inputs()
{
	while(GLOBAL_ABORT_FLAG == 0)
	{
		//Execute IO_MASTER
		io_master_execute(&quadcopter_holmner.io_master);

		//Execute show inputs mode
		io_master_show_inputs_execute(&quadcopter_holmner.io_master);

		//Execute loop delay
		quadcopter_holmner_control_loop_delay();
	}
}

void quadcopter_holmner_exit_nicely()
{
	printf("quadcopter_holmner: exiting...");

	//Close pigpio stuff
	gpioTerminate();
}
