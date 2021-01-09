#ifndef IO_MASTER_INCLUDED
#define IO_MASTER_INCLUDED



#include <stdio.h>
#include <pigpio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include "global_settings.h"
#include "acc_gyro_mpu6050.h"
#include "barometer_bmp280.h"
#include "debugging_functions.h"


typedef struct IO_MASTER
{

	unsigned int last_loop_time_us;

	//Accelerometer/gyro device
    ACC_GYRO_MPU6050 acc_gyro_mpu6050_device;

    //Barometer device
    BAROMETER_BMP280 barometer_bmp280_device;

    //Radio controller stick inputs
    float stick_inp_trottle;
    float stick_inp_x;
    float stick_inp_y;
    float stick_inp_z;

    //Show inputs mode stuff
    unsigned int last_show_inputs_loop_time_ms;

} IO_MASTER;


int io_master_init(IO_MASTER *p);
void io_master_execute(IO_MASTER *p);
void io_master_read_accelerometer_and_gyro(IO_MASTER *p);
void io_master_show_inputs_execute(IO_MASTER *p);
int io_master_read_i2c_one_byte(const int device_id, const int start_reg, uint8_t *data_out);
int io_master_write_i2c_one_byte(const int device_id, const int start_reg, const uint8_t data_in);
int io_master_check_i2c_device(const int device_id, const int reg_who_am_i, const int device_adress, const int who_am_i_value);
void io_master_read_i2c_16_bit_swapped_unsiged(const int device_id, const int start_reg, uint16_t *data_out);
void io_master_read_i2c_16_bit_swapped_siged(const int device_id, const int start_reg, int16_t *data_out);
void io_master_read_i2c_24_bit_unsiged(const int device_id, const int start_reg, uint32_t *data_out);
void io_master_read_i2c_16_bit_unsiged(const int device_id, const int start_reg, uint16_t *data_out);
void io_master_read_i2c_16_bit_siged(const int device_id, const int start_reg, int16_t *data_out);

#endif
