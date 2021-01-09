#include "acc_gyro_calibration.h"


/* PRIVATE PROTOTYPES */
static void acc_gyro_calibration_print_calib_info(ACC_GYRO_CALIBRATION *p);


#define ACC_GYRO_CALIBRATION_PRINT_DELAY_TIME_MS 1000


static const float low_pass_filter_factor = 0.99f;

void acc_gyro_calibration_init(ACC_GYRO_CALIBRATION *p)
{
	//Init data
	p->x_ang_deg_low_pass_filt = 0;
	p->y_ang_deg_low_pass_filt = 0;

	p->x_gyr_deg_low_pass_filt = 0;
	p->y_gyr_deg_low_pass_filt = 0;
	p->z_gyr_deg_low_pass_filt = 0;
	p->last_print_time = millis();
}

void acc_gyro_calibration_execute(ACC_GYRO_CALIBRATION *p,
								  const float ang_x_deg_unfilt,
								  const float ang_y_deg_unfilt,
								  const float gyr_x_deg_unfilt,
								  const float gyr_y_deg_unfilt,
								  const float gyr_z_deg_unfilt)
{
	//Perform low pass filtering
	p->x_ang_deg_low_pass_filt = low_pass_filter_factor * p->x_ang_deg_low_pass_filt + (1-low_pass_filter_factor) * ang_x_deg_unfilt;
	p->y_ang_deg_low_pass_filt = low_pass_filter_factor * p->y_ang_deg_low_pass_filt + (1-low_pass_filter_factor) * ang_y_deg_unfilt;
	p->x_gyr_deg_low_pass_filt = low_pass_filter_factor * p->x_gyr_deg_low_pass_filt + (1-low_pass_filter_factor) * gyr_x_deg_unfilt;
	p->y_gyr_deg_low_pass_filt = low_pass_filter_factor * p->y_gyr_deg_low_pass_filt + (1-low_pass_filter_factor) * gyr_y_deg_unfilt;
	p->z_gyr_deg_low_pass_filt = low_pass_filter_factor * p->z_gyr_deg_low_pass_filt + (1-low_pass_filter_factor) * gyr_z_deg_unfilt;

	//Print calibration information if it is time
	unsigned int delta_time = millis() - p->last_print_time;
	if(delta_time > ACC_GYRO_CALIBRATION_PRINT_DELAY_TIME_MS)
	{
		acc_gyro_calibration_print_calib_info(p);
		p->last_print_time = millis();
	}
}

static void acc_gyro_calibration_print_calib_info(ACC_GYRO_CALIBRATION *p)
{
	int i;
	//Print blanks to make cmd scroll
	for(i = 0; i < 100; i++)
	{
		printf("\n");
	}

	printf("**********ACC/GYRO CALIBRATION********\n");
	printf("\n");
	printf("1) Level your quadcopter and let it stand still.\n");
	printf("2) If this is the first time calibrating: ensure all\n");
	printf("calibration offsets are set to 0 in the settings area in main().\n");
	printf("3) Go to the settings area in main()\n");
	printf("4) Note these values and set the variables accordingly;\n");
	printf("\n");
	printf("ANGLE_X_DEG_CAL_OFFSET = %f;\n", p->x_ang_deg_low_pass_filt);
	printf("ANGLE_Y_DEG_CAL_OFFSET = %f;\n", p->y_ang_deg_low_pass_filt);
	printf("GYR_X_DEG_CAL_OFFSET  = %f;\n", p->x_gyr_deg_low_pass_filt);
	printf("GYR_Y_DEG_CAL_OFFSET  = %f;\n", p->y_gyr_deg_low_pass_filt);
	printf("GYR_Z_DEG_CAL_OFFSET  = %f;\n", p->z_gyr_deg_low_pass_filt);
	printf("\n");
	printf("Tip: After doing the calibration, run this again to make\n");
	printf("sure that the offsets are correct. They should be close to 0.\n");
	printf("\n");
	printf("**************************************\n");
}
