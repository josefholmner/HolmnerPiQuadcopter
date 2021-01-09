#ifndef ACC_GYRO_MPU6050_INCLUDED
#define ACC_GYRO_MPU6050_INCLUDED

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <math.h>
#include "global_settings.h"


typedef struct ACC_GYRO_MPU6050
{
	int acc_gyro_device_id;

	unsigned int last_loop_time_us;

	//Roll (x), pitch(y) yaw (z) angles from acc/gyro
    float angle_x_deg_filtered;
    float angle_y_deg_filtered;
    float angle_z_deg_filtered;

    //Acc/gyro infiltered (used for acc/gyro calibration etc.)
    float angle_x_deg_unfilt;
    float angle_y_deg_unfilt;
    float gyr_x_deg_unfilt;
    float gyr_y_deg_unfilt;
    float gyr_z_deg_unfilt;

} ACC_GYRO_MPU6050;
#include "io_master.h"

int acc_gyro_mpu6050_init(ACC_GYRO_MPU6050 *p);
void acc_gyro_mpu6050_read_acc_gyr(ACC_GYRO_MPU6050 *p);



#endif
