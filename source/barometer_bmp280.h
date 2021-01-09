#ifndef BAROMETER_BMP280_INCLUDED
#define BAROMETER_BMP280_INCLUDED

#include <pigpio.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>

typedef struct BAROMETER_COMENSATION_DATA
{
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;

	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;

} BAROMETER_COMENSATION_DATA;

typedef struct BAROMETER_BMP280
{
	//Compensation data
	BAROMETER_COMENSATION_DATA barometer_compensation_data;

	int barometer_device_id;

    //Current altitude (meters) from barometer sensor
    float current_altitude_m;

    //Starting altitude (meters) from barometer sensor
    float starting_altitude_m;

    //Barometer read last time ms
    unsigned int last_barometer_read_time_ms;

    //Temperature (used in temp. compensation algorithm: see datasheet)
	int t_fine;

	//Uncompensated temperature
	uint32_t uncomp_temp;

	//Compensated temperature
	int comp_temp;

	//Uncompensated pressure
	uint32_t uncomp_pressure;

	//Compensated pressure
	float comp_pressure;

} BAROMETER_BMP280;
#include "io_master.h"

int barometer_bmp280_init(BAROMETER_BMP280 *p);
int barometer_bmp280_read_barometer(BAROMETER_BMP280 *p);

#endif
