#include "acc_gyro_mpu6050.h"


/* Private prototypes */
static int acc_gyro_mpu6050_setup(int device_id);



#define RADTODEG 57.2957795131
#define DEGTORAD 0.01745329251

#define ACC_GYRO_COMPLEMENTARY_FILTER_FACTOR 0.99

/*
MPU6050 accelerometer/gyro data registers.
H denotes high byte, L denotes low byte
*  (from datasheet)
*/
#define REG_ACC_GYRO_ACC_X_H 0x3B
#define REG_ACC_GYRO_ACC_Y_H 0x3D
#define REG_ACC_GYRO_ACC_Z_H 0x3F
#define REG_ACC_GYRO_GYR_X_H 0x43
#define REG_ACC_GYRO_GYR_Y_H 0x45
#define REG_ACC_GYRO_GYR_Z_H 0x47

#define REG_ACC_GYRO_WHO_AM_I 0x75
#define REG_ACC_GYRO_SLEEP 0x6B
#define REG_ACC_GYRO_GYRO_CONFIG 0x1B
#define REG_ACC_GYRO_ACC_CONFIG 0x1C
#define REG_ACC_GYRO_FILTER_CONFIG 0x1A


int acc_gyro_mpu6050_init(ACC_GYRO_MPU6050 *p)
{
	int status = -1;

	//Init data
	p->last_loop_time_us = micros();

	p->angle_x_deg_filtered = 0;
	p->angle_y_deg_filtered = 0;
	p->angle_z_deg_filtered = 0;
	p->angle_x_deg_unfilt = 0;
	p->angle_y_deg_unfilt = 0;

	p->gyr_x_deg_unfilt = 0;
	p->gyr_y_deg_unfilt = 0;
	p->gyr_z_deg_unfilt = 0;

	//Setup wiringPi i2c communication
	status = wiringPiI2CSetup(ACC_GYRO_ADRESS);
	p->acc_gyro_device_id = status;

	if (status < 0)
	{
		printf("io_master_init: could not setup i2c for wiringPi library for acc_gyro_mpu6050! status = %d.\n", status);
		return status;
	}


	//Check that accelerometer/gyro device can be found
	status = io_master_check_i2c_device(p->acc_gyro_device_id, REG_ACC_GYRO_WHO_AM_I, ACC_GYRO_ADRESS, ACC_GYRO_ADRESS);
	if(status < 0)
	{
		printf("io_master_init: could not find accelerometer/gyro! status = %d.\n", status);
		return status;
	}
	else
	{
		printf("io_master_init: Accelerometer/gyro sensor found.\n");
	}

	//Init accelerometer/gyro
	status = acc_gyro_mpu6050_setup(p->acc_gyro_device_id);
	if(status < 0)
	{
		printf("io_master_init: could not init accelerometer/gyro! status = %d.\n", status);
		return status;
	}

	return status;
}

static int acc_gyro_mpu6050_setup(int device_id)
{
	int status;

	//Wake accelerometer/gyro
	status = io_master_write_i2c_one_byte(device_id, REG_ACC_GYRO_SLEEP, 0);

	//Set gyro sensitivity 500deg/s
	if(status >= 0)
	{
		//0x08 = 500deg/s (from datasheet)
		status = io_master_write_i2c_one_byte(device_id, REG_ACC_GYRO_GYRO_CONFIG, 0x08);
	}

	//Set accelerometer sensitivity +-4g
	if(status >= 0)
	{
		//0x08 = +-4g (from datasheet)
		status = io_master_write_i2c_one_byte(device_id, REG_ACC_GYRO_ACC_CONFIG, 0x08);
	}

	//Activate low pass filter
	if(status >= 0)
	{
		//0x03 from datasheet
		status = io_master_write_i2c_one_byte(device_id, REG_ACC_GYRO_FILTER_CONFIG, 0x03);
	}


	return status;
}

void acc_gyro_mpu6050_read_acc_gyr(ACC_GYRO_MPU6050 *p)
{
	int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;

	io_master_read_i2c_16_bit_swapped_siged(p->acc_gyro_device_id, REG_ACC_GYRO_ACC_Y_H, &acc_x); //x and y switched to match acc/gyro drawing
	io_master_read_i2c_16_bit_swapped_siged(p->acc_gyro_device_id, REG_ACC_GYRO_ACC_X_H, &acc_y); //x and y switched to match acc/gyro drawing
	io_master_read_i2c_16_bit_swapped_siged(p->acc_gyro_device_id, REG_ACC_GYRO_ACC_Z_H, &acc_z);
	io_master_read_i2c_16_bit_swapped_siged(p->acc_gyro_device_id, REG_ACC_GYRO_GYR_X_H, &gyr_x);
	io_master_read_i2c_16_bit_swapped_siged(p->acc_gyro_device_id, REG_ACC_GYRO_GYR_Y_H, &gyr_y);
	io_master_read_i2c_16_bit_swapped_siged(p->acc_gyro_device_id, REG_ACC_GYRO_GYR_Z_H, &gyr_z);

	float acc_x_deg_unfiltered, acc_y_deg_unfiltered;
	float gyr_x_deg_s_unfiltered, gyr_y_deg_s_unfiltered, gyr_z_deg_s_unfiltered;

	acc_x_deg_unfiltered = atan2((float)acc_x, sqrt((float)acc_y*(float)acc_y + (float)acc_z*(float)acc_z))*RADTODEG - ANGLE_X_DEG_CAL_OFFSET;
	acc_y_deg_unfiltered = -atan2((float)acc_y, sqrt((float)acc_x*(float)acc_x + (float)acc_z*(float)acc_z))*RADTODEG - ANGLE_Y_DEG_CAL_OFFSET;

	gyr_x_deg_s_unfiltered = (float)gyr_x/65.5 - GYR_X_DEG_CAL_OFFSET;
	gyr_y_deg_s_unfiltered = (float)gyr_y/65.5 - GYR_Y_DEG_CAL_OFFSET;
	gyr_z_deg_s_unfiltered = (float)gyr_z/65.5 - GYR_Z_DEG_CAL_OFFSET;

	//Keep track of loop time (used for complementary filter further below)
	float loop_time = (float)(micros() - p->last_loop_time_us)/1000000;
	if(loop_time > 2*(float)(CTRL_LOOP_DELTA_TIME_US)/1000000)
	{
		loop_time = (float)(CTRL_LOOP_DELTA_TIME_US)/1000000;
	}

	p->last_loop_time_us = micros();
	//Perform complementary filter and store result
	p->angle_x_deg_filtered = ACC_GYRO_COMPLEMENTARY_FILTER_FACTOR * (p->angle_x_deg_filtered + gyr_x_deg_s_unfiltered * loop_time) + (1-ACC_GYRO_COMPLEMENTARY_FILTER_FACTOR) * acc_x_deg_unfiltered;
	p->angle_y_deg_filtered = ACC_GYRO_COMPLEMENTARY_FILTER_FACTOR * (p->angle_y_deg_filtered + gyr_y_deg_s_unfiltered * loop_time) + (1-ACC_GYRO_COMPLEMENTARY_FILTER_FACTOR) * acc_y_deg_unfiltered;
	//Z angle has to be integrated from gyro data. Magnetometer would be nice :)
	p->angle_z_deg_filtered = p->angle_z_deg_filtered + gyr_z_deg_s_unfiltered * loop_time;

	p->angle_x_deg_unfilt = acc_x_deg_unfiltered;
	p->angle_y_deg_unfilt = acc_y_deg_unfiltered;
	p->gyr_x_deg_unfilt = gyr_x_deg_s_unfiltered;
	p->gyr_y_deg_unfilt = gyr_y_deg_s_unfiltered;
	p->gyr_z_deg_unfilt = gyr_z_deg_s_unfiltered;
}
