#ifndef ACC_GYRO_CALIBRATION_INCLUDED
#define ACC_GYRO_CALIBRATION_INCLUDED

#include <stdio.h>
#include <wiringPi.h>

typedef struct ACC_GYRO_CALIBRATION
{
	float x_ang_deg_low_pass_filt;
	float y_ang_deg_low_pass_filt;

	float x_gyr_deg_low_pass_filt;
	float y_gyr_deg_low_pass_filt;
	float z_gyr_deg_low_pass_filt;

	unsigned int last_print_time;
}ACC_GYRO_CALIBRATION;

void acc_gyro_calibration_init(ACC_GYRO_CALIBRATION *p);
void acc_gyro_calibration_execute(ACC_GYRO_CALIBRATION *p,
								  const float ang_x_deg_unfilt,
								  const float ang_y_deg_unfilt,
								  const float gyr_x_deg_unfilt,
								  const float gyr_y_deg_unfilt,
								  const float gyr_z_deg_unfilt);

#endif
