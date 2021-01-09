#include "barometer_bmp280.h"


/* Private prototypes */
static int barometer_bmp280_setup(const int device_id);
static void barometer_bmp280_get_calib_param(BAROMETER_BMP280 *p);
static void barometer_bmp280_read_uncompensated_temperature(BAROMETER_BMP280 *p);
static void barometer_bmp280_calc_compensated_temperature(BAROMETER_BMP280 *p);
static void barometer_bmp280_read_uncompensated_pressure(BAROMETER_BMP280 *p);
static void barometer_bmp280_calc_compensated_pressure(BAROMETER_BMP280 *p);
static void barometer_bmp280_calc_current_altitude_m(BAROMETER_BMP280 *p);


/*
 BMP280 registers
 (from datasheet)
 */
#define REG_BAROMETER_IIR_FILTER_SETTING 0xF5
#define REG_BAROMETER_POWER_MODE_OSRP_OSRST_SETTING 0xF4
#define REG_BAROMETER_FILTER_AND_STANDBY_SETTING 0xF5
#define REG_BAROMETER_WHO_AM_I 0xD0
#define REG_BAROMETER_PREASSURE_H 0xF7
#define REG_BAROMETER_PRESSURE_L_LAST4 0xF9
#define REG_BAROMETER_TEMPDATA_H 0xFA

#define REG_BAROMETER_DIG_T1 0x88
#define REG_BAROMETER_DIG_T2 0x8A
#define REG_BAROMETER_DIG_T3 0x8C
#define REG_BAROMETER_DIG_P1 0x8E
#define REG_BAROMETER_DIG_P2 0x90
#define REG_BAROMETER_DIG_P3 0x92
#define REG_BAROMETER_DIG_P4 0x94
#define REG_BAROMETER_DIG_P5 0x96
#define REG_BAROMETER_DIG_P6 0x98
#define REG_BAROMETER_DIG_P7 0x9A
#define REG_BAROMETER_DIG_P8 0x9C
#define REG_BAROMETER_DIG_P9 0x9E

 #define BAROMETER_WHO_AM_I_VALUE 0x58

 #define BAROMETER_READ_DELAY_MS 37



int barometer_bmp280_init(BAROMETER_BMP280 *p)
{
	int status = -1;


	//Init data
	p->last_barometer_read_time_ms = millis();

	status = wiringPiI2CSetup(BAROMETER_ADDRESS);
	p->barometer_device_id = status;
	if (status < 0)
	{
		printf("io_master_init: could not setup i2c for wiringPi library for barometer_bmp280! status = %d.\n", status);
		return status;
	}

	//Check that barometer sensor is connected
	status = io_master_check_i2c_device(p->barometer_device_id, REG_BAROMETER_WHO_AM_I, BAROMETER_ADDRESS, BAROMETER_WHO_AM_I_VALUE);
	if(status < 0)
	{
		printf("io_master_init: could not find barometer sensor! status = %d.\n", status);
		return status;
	}
	else
	{
		printf("io_master_init: Barometer sensor found.\n");
	}

	//Setup barometer sensor
	status = barometer_bmp280_setup(p->barometer_device_id);
	if(status < 0)
	{
		printf("io_master_init: could not init barometer sensor! status = %d.\n", status);
		return status;
	}

	//Get compensation parameters
	barometer_bmp280_get_calib_param(p);

	//Get starting altitude (average of 20 readings)
	int number_of_samples = 0;
	float start_alt = 0;
	while(number_of_samples < 20)
	{
		if(barometer_bmp280_read_barometer(p) == 0)
		{
			start_alt += p->current_altitude_m;
			number_of_samples++;
		}
		delay(BAROMETER_READ_DELAY_MS);
	}
	start_alt /= 20;
	p->starting_altitude_m = start_alt;

	return status;
}

static int barometer_bmp280_setup(const int device_id)
{
	int status;

	uint8_t power_mode_osrp_osrts_byte = 0;

	//Set power mode to normal
	power_mode_osrp_osrts_byte |= 0x03;

	//Set osrs_p to x16
	power_mode_osrp_osrts_byte |= (0x05<<2);

	//Set osrs_t to x2
	power_mode_osrp_osrts_byte |= (0x02<<5);

	//Write REG_BAROMETER_POWER_MODE_OSRP_OSRST_SETTING register
	status = io_master_write_i2c_one_byte(device_id, REG_BAROMETER_POWER_MODE_OSRP_OSRST_SETTING, power_mode_osrp_osrts_byte);

	//Set filter and standby settings
	if(status >= 0)
	{
		uint8_t filter_and_standby_byte = 0;

		//Set iir filter coeficient to 16
		filter_and_standby_byte |= (0x04<<2);

		//Set standby time to 0.5 ms
		filter_and_standby_byte |= (0<<5);

		//Write REG_BAROMETER_FILTER_AND_STANDBY_SETTING register
		status = io_master_write_i2c_one_byte(device_id, REG_BAROMETER_FILTER_AND_STANDBY_SETTING, filter_and_standby_byte);
	}

	return status;
}

/* Returns 0 if reading was done, otherwise -1 */
int barometer_bmp280_read_barometer(BAROMETER_BMP280 *p)
{
	int return_val = -1;
	unsigned int delta_time = millis() - p->last_barometer_read_time_ms;

	//Only read barometer avery BAROMETER_READ_DELAY_MS millisecond
	if(delta_time >= BAROMETER_READ_DELAY_MS)
	{
		//Read uncompensated temperature
		barometer_bmp280_read_uncompensated_temperature(p);

		//Calculate compensated temperature
		barometer_bmp280_calc_compensated_temperature(p);

		//Read uncompensated pressure
		barometer_bmp280_read_uncompensated_pressure(p);

		//Calculate compensated pressure
		barometer_bmp280_calc_compensated_pressure(p);

		//Calculate current altitude (meters)
		barometer_bmp280_calc_current_altitude_m(p);

		//Update last_barometer_read_time_ms to current millis
		p->last_barometer_read_time_ms = millis();

		return_val = 0;
	}

	return return_val;
}

static void barometer_bmp280_calc_current_altitude_m(BAROMETER_BMP280 *p)
{
	float altitude = 0;

	float pressure = (float)p->comp_pressure/256; // in Si units for Pascal
	pressure /= 100;

	altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));

	p->current_altitude_m = altitude;
}

static void barometer_bmp280_read_uncompensated_pressure(BAROMETER_BMP280 *p)
{
	io_master_read_i2c_24_bit_unsiged(p->barometer_device_id, REG_BAROMETER_PREASSURE_H, &p->uncomp_pressure);
}

static void barometer_bmp280_calc_compensated_pressure(BAROMETER_BMP280 *p)
{
	int64_t var1, var2, p_;

	int32_t adc_P = (int32_t)p->uncomp_pressure;

	var1 = ((int64_t)p->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)p->barometer_compensation_data.dig_p6;
	var2 = var2 + ((var1*(int64_t)p->barometer_compensation_data.dig_p5)<<17);
	var2 = var2 + (((int64_t)p->barometer_compensation_data.dig_p4)<<35);
	var1 = ((var1 * var1 * (int64_t)p->barometer_compensation_data.dig_p3)>>8) +
	((var1 * (int64_t)p->barometer_compensation_data.dig_p2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)p->barometer_compensation_data.dig_p1)>>33;

	if (var1 == 0) {
	return;  // avoid exception caused by division by zero
	}
	p_ = 1048576 - adc_P;
	p_ = (((p_<<31) - var2)*3125) / var1;
	var1 = (((int64_t)p->barometer_compensation_data.dig_p9) * (p_>>13) * (p_>>13)) >> 25;
	var2 = (((int64_t)p->barometer_compensation_data.dig_p8) * p_) >> 19;

	p_ = ((p_ + var1 + var2) >> 8) + (((int64_t)p->barometer_compensation_data.dig_p7)<<4);
	p->comp_pressure = (float)p_;
}

static void barometer_bmp280_read_uncompensated_temperature(BAROMETER_BMP280 *p)
{
	io_master_read_i2c_24_bit_unsiged(p->barometer_device_id, REG_BAROMETER_TEMPDATA_H, &p->uncomp_temp);
}

static void barometer_bmp280_calc_compensated_temperature(BAROMETER_BMP280 *p)
{
	int32_t var1;
	int32_t var2;
	int32_t temperature = 0;


	var1 = ((((p->uncomp_temp >> 3) - ((int32_t) p->barometer_compensation_data.dig_t1 << 1)))
	* ((int32_t) p->barometer_compensation_data.dig_t2)) >> 11;
	var2 = (((((p->uncomp_temp >> 4) - ((int32_t) p->barometer_compensation_data.dig_t1))
	* ((p->uncomp_temp >> 4) - ((int32_t) p->barometer_compensation_data.dig_t1))) >> 12)
	* ((int32_t) p->barometer_compensation_data.dig_t3)) >> 14;

	p->t_fine = var1 + var2;
	temperature = (p->t_fine * 5 + 128) >> 8;

	p->comp_temp = temperature;
}

/* Read calibration data from barometer (used in temperature compensation algorith: see datasheet) */
static void barometer_bmp280_get_calib_param(BAROMETER_BMP280 *p)
{
	io_master_read_i2c_16_bit_unsiged(p->barometer_device_id, REG_BAROMETER_DIG_T1, &p->barometer_compensation_data.dig_t1);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_T2, &p->barometer_compensation_data.dig_t2);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_T3, &p->barometer_compensation_data.dig_t3);

	io_master_read_i2c_16_bit_unsiged(p->barometer_device_id, REG_BAROMETER_DIG_P1, &p->barometer_compensation_data.dig_p1);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P2, &p->barometer_compensation_data.dig_p2);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P3, &p->barometer_compensation_data.dig_p3);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P4, &p->barometer_compensation_data.dig_p4);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P5, &p->barometer_compensation_data.dig_p5);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P6, &p->barometer_compensation_data.dig_p6);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P7, &p->barometer_compensation_data.dig_p7);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P8, &p->barometer_compensation_data.dig_p8);
	io_master_read_i2c_16_bit_siged(p->barometer_device_id, REG_BAROMETER_DIG_P9, &p->barometer_compensation_data.dig_p9);
}

