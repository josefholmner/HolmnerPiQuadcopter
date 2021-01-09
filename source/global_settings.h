#ifndef GLOBAL_SETTINGS_INCLUDED
#define GLOBAL_SETTINGS_INCLUDED


/*
* Here, global settings, such as pin assignment
* can be set.
*/


/* Stick input pins from radio controller (GPIO pin number) */
int PIN_THROTTLE_STICK_INPUT;
int PIN_X_STICK_INPUT; //HERE: X => Roll
int PIN_Y_STICK_INPUT; //HERE: Y => Pitch
int PIN_Z_STICK_INPUT; //HERE: X => Yaw

int PIN_MOTOR_TOP_LEFT; //Quad in X-configuration, seen from above
int PIN_MOTOR_TOP_RIGHT; //Quad in X-configuration, seen from above
int PIN_MOTOR_BOTTOM_LEFT; //Quad in X-configuration, seen from above
int PIN_MOTOR_BOTTOM_RIGHT; //Quad in X-configuration, seen from above

/* Radio controller stick inputs zero pulse width (microseconds) */
int ZERO_PULSE_WIDTH_X_STICK_INPUT_US; //HERE: X => Roll
int ZERO_PULSE_WIDTH_Y_STICK_INPUT_US; //HERE: Y => Pitch
int ZERO_PULSE_WIDTH_Z_STICK_INPUT_US; //HERE: X => Yaw

/* Radio controller stick input maximum pulse width (microseconds) */
int STICK_INPUT_MAX_PULSE_WIDTH_US;

/* Angle reference max (degrees) (will effect how aggressive turns you can take) */
float ANGLE_X_REF_DEG_MAX;
float ANGLE_Y_REF_DEG_MAX;
float ANGLE_Z_REF_DEG_MAX;

/* Acclerometer/gyro i2c adress (MPU6050) */
int ACC_GYRO_ADRESS;

/* GLOBAL_ABORT_FLAG (set to 1 to abort execution) */
int GLOBAL_ABORT_FLAG;

/* Main control loop delay time in micro seconds (1/frequency) */
unsigned int CTRL_LOOP_DELTA_TIME_US;

/* Accelerometer/gyro calibration offsets (use EXECUTION_MODE_CALIBRATE_ACCEL_GYRO to calibrate) */
float ANGLE_X_DEG_CAL_OFFSET;
float ANGLE_Y_DEG_CAL_OFFSET;
float GYR_X_DEG_CAL_OFFSET;
float GYR_Y_DEG_CAL_OFFSET;
float GYR_Z_DEG_CAL_OFFSET;

/* PID parameters (for motor control) */
float PID_XY_KP;
float PID_XY_KI;
float PID_XY_KD;
float PID_Z_KP;
float PID_Z_KI;
float PID_Z_KD;

/* Arm ECS's pwm pulse width (microseconds) */
int ESC_ARM_PULSE_WIDTH_US;

/* Hide control loop too slow warning (default: 0) */
int HIDE_CONTROL_LOOP_TOO_SLOW_WARNING;

/* Use barometer sensor (BMP280) */
#define USE_BAROMETER_SENSOR

/* Barometer sensor adress (BMP280) */
int BAROMETER_ADDRESS;


#endif
