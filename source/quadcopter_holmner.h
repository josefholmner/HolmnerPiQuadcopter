#ifndef QUADCOPTER_HOLMNER_INCLUDED
#define QUADCOPTER_HOLMNER_INCLUDED

#include "io_master.h"
#include "acc_gyro_calibration.h"
#include "motor_control.h"

typedef struct QUADCOPTER_HOLMNER
{
	IO_MASTER io_master;
	ACC_GYRO_CALIBRATION acc_gyro_calibration;
	MOTOR_CONTROL motor_control;

	unsigned int last_loop_time_us;

	//Software performance measurement
	unsigned int performance_ctrl_start_time_us;
	int performance_ctrl_loop_counter;
} QUADCOPTER_HOLMNER;


int quadcopter_holmner_init();
void quadcopter_holmner_fly();
void quadcopter_holmner_calibrate_acc_gyro();
void quadcopter_holmner_show_inputs();
void quadcopter_holmner_exit_nicely();

#endif
