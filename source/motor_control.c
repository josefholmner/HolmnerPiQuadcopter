#include "motor_control.h"

/* Private prototypes */
static void motor_control_generate_references(MOTOR_CONTROL *p,const IO_MASTER *io_master_ptr);
static void motor_control_do_PID_control_and_generate_motor_sign(MOTOR_CONTROL *p,const IO_MASTER *io_master_ptr);
static void motor_control_write_to_motors(MOTOR_CONTROL *p, const IO_MASTER *io_master_ptr);
static void motor_control_set_servo_pwm_pulse(int servo_pin, int pulse_length_us);
static float motor_control_map_value(float result_low_lim, float result_high_lim, float input_low_lim, float input_high_lim, float input_val);


#define MOTOR_CONTROL_SERVO_WRITE_INTERVAL_US 6000



void motor_control_init(MOTOR_CONTROL *p)
{

	//Init data
	p->angle_x_deg_ref = 0;
	p->angle_y_deg_ref = 0;
	p->angle_z_deg_ref = 0;

	p->p_x = 0;
	p->i_x = 0;
	p->d_x = 0;
	p->p_y = 0;
	p->i_y = 0;
	p->d_y = 0;
	p->p_z = 0;
	p->i_z = 0;
	p->d_z = 0;

	p->ppm_out_motor_top_left = 0;
	p->ppm_out_motor_top_right = 0;
	p->ppm_out_motor_bottom_left = 0;
	p->ppm_out_motor_bottom_right = 0;

	p->last_loop_time_pid_us = micros();
	p->last_loop_time_write_motors_us = micros();
}

void motor_control_execute(MOTOR_CONTROL *p,const IO_MASTER *io_master_ptr)
{
	//Generate references ("wanted" values)
	motor_control_generate_references(p, io_master_ptr);

	//Do PID control
	motor_control_do_PID_control_and_generate_motor_sign(p, io_master_ptr);

	//Write to motors
	motor_control_write_to_motors(p, io_master_ptr);
}

static void motor_control_generate_references(MOTOR_CONTROL *p,const IO_MASTER *io_master_ptr)
{
	//Generate references based on inputs
	float stick_input_x_mapped_to_deg = motor_control_map_value(-ANGLE_X_REF_DEG_MAX, ANGLE_X_REF_DEG_MAX, STICK_INPUT_MAX_PULSE_WIDTH_US - 2 * ZERO_PULSE_WIDTH_X_STICK_INPUT_US, STICK_INPUT_MAX_PULSE_WIDTH_US, io_master_ptr->stick_inp_x - ZERO_PULSE_WIDTH_X_STICK_INPUT_US);
	float stick_input_y_mapped_to_deg = motor_control_map_value(-ANGLE_Y_REF_DEG_MAX, ANGLE_Y_REF_DEG_MAX, STICK_INPUT_MAX_PULSE_WIDTH_US - 2 * ZERO_PULSE_WIDTH_Y_STICK_INPUT_US, STICK_INPUT_MAX_PULSE_WIDTH_US, io_master_ptr->stick_inp_y - ZERO_PULSE_WIDTH_Y_STICK_INPUT_US);
	float stick_input_z_mapped_to_deg = motor_control_map_value(-ANGLE_Z_REF_DEG_MAX, ANGLE_Z_REF_DEG_MAX, STICK_INPUT_MAX_PULSE_WIDTH_US - 2 * ZERO_PULSE_WIDTH_Z_STICK_INPUT_US, STICK_INPUT_MAX_PULSE_WIDTH_US, io_master_ptr->stick_inp_z - ZERO_PULSE_WIDTH_Z_STICK_INPUT_US);

	//Set references ("wanted" values)
	p->angle_x_deg_ref = stick_input_x_mapped_to_deg;
	p->angle_y_deg_ref = stick_input_y_mapped_to_deg;
	p->angle_z_deg_ref = stick_input_z_mapped_to_deg;
}

static void motor_control_do_PID_control_and_generate_motor_sign(MOTOR_CONTROL *p,const IO_MASTER *io_master_ptr)
{
	float delta_time_us = (micros() - p->last_loop_time_pid_us);

	//Do PID control
	p->p_x = PID_XY_KP * (p->angle_x_deg_ref - io_master_ptr->acc_gyro_mpu6050_device.angle_x_deg_filtered); //Proportional term for x
	p->p_y = PID_XY_KP * (p->angle_y_deg_ref - io_master_ptr->acc_gyro_mpu6050_device.angle_y_deg_filtered); //Proportional term for y
	p->p_z = PID_Z_KP * (p->angle_z_deg_ref - io_master_ptr->acc_gyro_mpu6050_device.angle_z_deg_filtered); //Proportional term for z

	p->i_x += PID_XY_KI * (p->angle_x_deg_ref - io_master_ptr->acc_gyro_mpu6050_device.angle_x_deg_filtered) * delta_time_us/1000000; //Integral term for x
	p->i_y += PID_XY_KI * (p->angle_y_deg_ref - io_master_ptr->acc_gyro_mpu6050_device.angle_y_deg_filtered) * delta_time_us/1000000; //Integral term for y
	p->i_z += PID_Z_KI * (p->angle_z_deg_ref - io_master_ptr->acc_gyro_mpu6050_device.angle_z_deg_filtered) * delta_time_us/1000000; //Integral term for z

	p->d_x = PID_XY_KD * io_master_ptr->acc_gyro_mpu6050_device.gyr_x_deg_unfilt; //Derivate term for x
	p->d_y = PID_XY_KD * io_master_ptr->acc_gyro_mpu6050_device.gyr_y_deg_unfilt; //Derivate term for y
	p->d_z = PID_Z_KD * io_master_ptr->acc_gyro_mpu6050_device.gyr_z_deg_unfilt; //Derivate term for z

	//Set main power level for all motors (baseline power)
	float motors_main_power = io_master_ptr->stick_inp_trottle;

	//Generate motor ppm pulses based on PID and main power
	p->ppm_out_motor_top_left = 	(int)(motors_main_power + (p->p_x + p->i_x + p->d_x) - (p->p_y + p->i_y + p->d_y) + (p->p_z + p->i_z + p->d_z));
	p->ppm_out_motor_top_right = 	(int)(motors_main_power - (p->p_x + p->i_x + p->d_x) - (p->p_y + p->i_y + p->d_y) - (p->p_z + p->i_z + p->d_z));
	p->ppm_out_motor_bottom_left = 	(int)(motors_main_power + (p->p_x + p->i_x + p->d_x) + (p->p_y + p->i_y + p->d_y) - (p->p_z + p->i_z + p->d_z));
	p->ppm_out_motor_bottom_right = (int)(motors_main_power - (p->p_x + p->i_x + p->d_x) + (p->p_y + p->i_y + p->d_y) + (p->p_z + p->i_z + p->d_z));

	p->last_loop_time_pid_us = micros();
}

static void motor_control_write_to_motors(MOTOR_CONTROL *p, const IO_MASTER *io_master_ptr)
{
	float delta_time_us = micros() - p->last_loop_time_write_motors_us;

	//Only write to motors every MOTOR_CONTROL_SERVO_WRITE_INTERVAL_US
	if(delta_time_us >= MOTOR_CONTROL_SERVO_WRITE_INTERVAL_US)
	{
		//Write pwm pulses to motors
		motor_control_set_servo_pwm_pulse(PIN_MOTOR_TOP_LEFT, p->ppm_out_motor_top_left);
		motor_control_set_servo_pwm_pulse(PIN_MOTOR_TOP_RIGHT, p->ppm_out_motor_top_right);
		motor_control_set_servo_pwm_pulse(PIN_MOTOR_BOTTOM_LEFT, p->ppm_out_motor_bottom_left);
		motor_control_set_servo_pwm_pulse(PIN_MOTOR_BOTTOM_RIGHT, p->ppm_out_motor_bottom_right);

		p->last_loop_time_write_motors_us = micros();
	}
}

/* Sets a pwm pulse to specified pin */
static void motor_control_set_servo_pwm_pulse(int servo_pin, int pulse_length_us)
{
	//Only allow 0-2000 microsecond pulse width
	if(pulse_length_us >= 0 && pulse_length_us <= 2000)
	{
		(void)gpioServo(servo_pin, pulse_length_us);
	}
	else
	{
		//printf("motor_control_set_servo_pwm_pulse: Warning, bad pulse width: %d\n", pulse_length_us);
	}
}

void motor_control_arm_esc_all()
{
	printf("motor_control: Arming ESC's...\n");

	//Arm all ESC's
	motor_control_set_servo_pwm_pulse(PIN_MOTOR_TOP_LEFT, ESC_ARM_PULSE_WIDTH_US);
	motor_control_set_servo_pwm_pulse(PIN_MOTOR_TOP_RIGHT, ESC_ARM_PULSE_WIDTH_US);
	motor_control_set_servo_pwm_pulse(PIN_MOTOR_BOTTOM_LEFT, ESC_ARM_PULSE_WIDTH_US);
	motor_control_set_servo_pwm_pulse(PIN_MOTOR_BOTTOM_RIGHT, ESC_ARM_PULSE_WIDTH_US);

	delay(2000);
	printf("motor_control: ESC's armed.\n");
}

static float motor_control_map_value(float result_low_lim, float result_high_lim, float input_low_lim, float input_high_lim, float input_val)
{
	float return_val = 0;
	float epsilon = 0.0001;

	if(fabs(result_high_lim - result_low_lim) > epsilon && fabs(input_high_lim - input_low_lim) > epsilon)
	{
		return_val = result_low_lim + ((input_val-input_low_lim)/(input_high_lim - input_low_lim)) * (result_high_lim - result_low_lim);
	}


	return return_val;
}
