#ifndef MOTOR_CONTROL_INCLUDED
#define MOTOR_CONTROL_INCLUDED

#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include "global_settings.h"
#include "io_master.h"
#include "acc_gyro_mpu6050.h"


typedef struct MOTOR_CONTROL
{
	unsigned int last_loop_time_pid_us;
	unsigned int last_loop_time_write_motors_us;

	//References ("wanted" values)
	float angle_x_deg_ref;
	float angle_y_deg_ref;
	float angle_z_deg_ref;

	//PID values
	float p_x; //Proportional term for x
	float p_y; //Proportional term for y
	float p_z; //Proportional term for z
	float i_x; //Integral term for x
	float i_y; //Integral term for y
	float i_z; //Integral term for z
	float d_x; //Derivate term for x
	float d_y; //Derivate term for y
	float d_z; //Derivate term for z

	//Motor ppm out signal
	int ppm_out_motor_top_left; //Quad in X-configuration, seen from above
	int ppm_out_motor_top_right; //Quad in X-configuration, seen from above
	int ppm_out_motor_bottom_left; //Quad in X-configuration, seen from above
	int ppm_out_motor_bottom_right; //Quad in X-configuration, seen from above

}MOTOR_CONTROL;

void motor_control_init(MOTOR_CONTROL *p);
void motor_control_execute(MOTOR_CONTROL *p, const IO_MASTER *io_master_ptr);
void motor_control_arm_esc_all();


#endif
