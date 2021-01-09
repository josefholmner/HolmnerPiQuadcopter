

/*
 * 			----------HOW TO USE----------
 * 1) Download and install the wiringPi library
 * 2) Download and install the pigpio library
 * 3) Make sure that I2C is activated on your Raspberry Pi (just google how)
 * 4) To increase the I2C baud rate to 400kHz (recommended), add
 *    dtparam=i2c1_baudrate=400000 to /boot/config.txt
 * 5) Go through the SETTINGS area in the main() function to setup GPIO pins etc.
 * 6) Depending of what sensors you are using, go through the defines in global_settings.h
 * 7) Build settings: gcc -Wall -pthread -o "Quadcopter_Holmner_run" *.c -lwiringPi -lpigpio -lm
 *
 *
 *			  ----------NOTE----------
 * The quadcopter is flown in X-configuration (google it if you dont know
 * what it is).
 * In this software, Roll is defined as X, Pitch is Y and Yaw is Z.
 * This software was developed on the Raspberry Pi 3 Model B.
 * The linux distribution used was: Raspbian GNU/LINUX 8
 * The sofware used for writing and compiling the code was Geany 1.24.1
 *
 *
 * 			 ----------HARDWARE----------
 * The following hardware was used for this project (using other hardware
 * may demand changing the code, especially for communication interfaces).
 *
 * Raspberry Pi 3 Model B
 * Accelerometer / Gyro: MPU6050
 * Radio transmitter: Hobby King 2.4Ghz 6Ch Tx & Rx V2 (Mode 2)
 * Motors: 4x FC 28-22 (1200kv)
 * ESC: 4x Hobbyking SS Series 15-18A ESC
 * Barometer BMP280 (optional)
 *
 *
 * 		      ----------CREDITS----------
 * Code written by Josef HOlmner 2018. It is free to use. Happy flying :)
 */


#include <stdlib.h>
#include "quadcopter_holmner.h"
#include "global_settings.h"


enum
{
	EXECUTION_MODE_FLY, EXECUTION_MODE_CALIBRATE_ACCEL_GYRO, EXECUTION_MODE_SHOW_INPUTS
};


int main(void)
{
	int status = -1;
	int EXECUTION_MODE;

	atexit(quadcopter_holmner_exit_nicely);


	/*****************************************************/
	/* -------------------SETUP START------------------- */
	/*****************************************************/


	//Setup pin assignments (GPIO pin numbering)
	PIN_THROTTLE_STICK_INPUT = 5;
	PIN_X_STICK_INPUT = 6;
	PIN_Y_STICK_INPUT = 13;
	PIN_Z_STICK_INPUT = 19;
	PIN_MOTOR_TOP_LEFT = 17;
	PIN_MOTOR_TOP_RIGHT = 27;
	PIN_MOTOR_BOTTOM_LEFT = 22;
	PIN_MOTOR_BOTTOM_RIGHT = 23;

	//Use barometer sensor (BMP280)
	/* Is set in global_settings.h !!
	 * Comment out #define USE_BAROMETER_SENSOR to disable */

	//Setup barometer sensor (BMP280) adress
	BAROMETER_ADDRESS = 0x77;

	//Setup accelerometer/gyro i2c adress
	ACC_GYRO_ADRESS = 0x68;

	/* Setup radio controller stick input signal widths when
	   at the "zero" position (microseconds)*/
	ZERO_PULSE_WIDTH_X_STICK_INPUT_US = 1500; //HERE: X => Roll
	ZERO_PULSE_WIDTH_Y_STICK_INPUT_US = 1500; //HERE: Y => Pitch
	ZERO_PULSE_WIDTH_Z_STICK_INPUT_US = 1500; //HERE: X => Yaw

	//Max radio controller stick input pulse width (microseconds)
	STICK_INPUT_MAX_PULSE_WIDTH_US = 2000;

	/* Set angle reference max (degrees) (will effect how aggressive
	   turns you can take) */
	ANGLE_X_REF_DEG_MAX = 40;
	ANGLE_Y_REF_DEG_MAX = 40;
	ANGLE_Z_REF_DEG_MAX = 40;

	//Select execution mode (fly, calibrate accelerometer/gyro etc)
	EXECUTION_MODE = EXECUTION_MODE_SHOW_INPUTS;

	/* Setup accelerometer/gyro calibration offsets
	   (use execution mode EXECUTION_MODE_CALIBRATE_ACCEL_GYRO) */
	ANGLE_X_DEG_CAL_OFFSET = 0;
	ANGLE_Y_DEG_CAL_OFFSET = 0;
	GYR_X_DEG_CAL_OFFSET = -0.015;
	GYR_Y_DEG_CAL_OFFSET = -0.015;
	GYR_Z_DEG_CAL_OFFSET = -0.015;

	//Setup PID parameters (for motor control)
	PID_XY_KP = 2.4;
	PID_XY_KI = 3.7;
	PID_XY_KD = 0.7;
	PID_Z_KP = 2;
	PID_Z_KI = 0.8;
	PID_Z_KD = 0.3;

	//Setup main control loop iteration time in microseconds (1 / frequency)
	CTRL_LOOP_DELTA_TIME_US = 2000;

	//Setup ECS's arming pwm pulse width (microseconds)
	ESC_ARM_PULSE_WIDTH_US = 800;


	/*****************************************************/
	/* --------------------SETUP END-------------------- */
	/*****************************************************/



	//Set global abort flag to 0
	GLOBAL_ABORT_FLAG = 0;

	//Show/hide control loop too slow warning
	HIDE_CONTROL_LOOP_TOO_SLOW_WARNING = 0;

	//Init
	status = quadcopter_holmner_init();

	//Execute
	if(status == 0)
	{
		switch(EXECUTION_MODE)
		{
		    case EXECUTION_MODE_FLY:
				quadcopter_holmner_fly();
		    break;

		    case EXECUTION_MODE_CALIBRATE_ACCEL_GYRO:
				HIDE_CONTROL_LOOP_TOO_SLOW_WARNING = 1;
				quadcopter_holmner_calibrate_acc_gyro();
		    break;

		    case EXECUTION_MODE_SHOW_INPUTS:
				HIDE_CONTROL_LOOP_TOO_SLOW_WARNING = 1;
				quadcopter_holmner_show_inputs();
		    break;

			default:
			printf("Unkown execution mode selected! Program will exit.");
			status = -1;
			break;
		}
	}
	else
	{
		printf("Error: quadcopter_holmner_init returned %d\n", status);
	}


	return status;
}





