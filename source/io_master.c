#include "io_master.h"

/* Private prototypes */
static void io_master_pin_interrupt_stick_input_throttle();
static void io_master_pin_interrupt_stick_input_x();
static void io_master_pin_interrupt_stick_input_y();
static void io_master_pin_interrupt_stick_input_z();
static void io_master_read_barometer(IO_MASTER *p);
static void io_master_set_pwm_stick_input_vals(IO_MASTER *p);


typedef struct IO_MASTER_INTERRUPT
{
	unsigned int last_rising_edge_us;
	float pwm_width_us;
} IO_MASTER_INTERRUPT;

static IO_MASTER_INTERRUPT interrupt_throttle_stick_input;
static IO_MASTER_INTERRUPT interrupt_x_stick_input;
static IO_MASTER_INTERRUPT interrupt_y_stick_input;
static IO_MASTER_INTERRUPT interrupt_z_stick_input;


#define SHOW_INPUTS_PRINT_INTERVAL_MS 100


int io_master_init(IO_MASTER *p)
{
	int status = -1;


	p->stick_inp_trottle = 0;
	p->stick_inp_x = 0;
    p->stick_inp_y = 0;
    p->stick_inp_z = 0;

	p->last_show_inputs_loop_time_ms = millis();

	interrupt_throttle_stick_input.last_rising_edge_us = 0;
	interrupt_throttle_stick_input.pwm_width_us = 0;

	//Init gpio
	status = gpioInitialise();
	if (status < 0)
	{
		printf("io_master_init: could not initialize pigpio library! status = %d.\n", status);
		return status;
	}

	//Init wiringpi
	status = wiringPiSetupGpio();
	if (status < 0)
	{
		printf("io_master_init: could not initialize wiringPi library! status = %d.\n", status);
		return status;
	}

	status = acc_gyro_mpu6050_init(&p->acc_gyro_mpu6050_device);
	if(status < 0)
	{
		return status;
	}


#ifdef USE_BAROMETER_SENSOR
	barometer_bmp280_init(&p->barometer_bmp280_device);
#endif


	//Setup interrupts for radio controller input pins
	pinMode(PIN_THROTTLE_STICK_INPUT, INPUT);
	pinMode(PIN_X_STICK_INPUT, INPUT);
	pinMode(PIN_Y_STICK_INPUT, INPUT);
	pinMode(PIN_Z_STICK_INPUT, INPUT);
	wiringPiISR (PIN_THROTTLE_STICK_INPUT, INT_EDGE_BOTH, io_master_pin_interrupt_stick_input_throttle);
	wiringPiISR (PIN_X_STICK_INPUT, INT_EDGE_BOTH, io_master_pin_interrupt_stick_input_x);
	wiringPiISR (PIN_Y_STICK_INPUT, INT_EDGE_BOTH, io_master_pin_interrupt_stick_input_y);
	wiringPiISR (PIN_Z_STICK_INPUT, INT_EDGE_BOTH, io_master_pin_interrupt_stick_input_z);


	return 0;
}

void io_master_execute(IO_MASTER *p)
{
	//Read accelerometer and gyro
	io_master_read_accelerometer_and_gyro(p);

#ifdef USE_BAROMETER_SENSOR
	//Read barometer sensor
	(void)io_master_read_barometer(p);
#endif

	//Set stick input pwm widths
	io_master_set_pwm_stick_input_vals(p);
}


void io_master_read_accelerometer_and_gyro(IO_MASTER *p)
{
	acc_gyro_mpu6050_read_acc_gyr(&p->acc_gyro_mpu6050_device);
}

static void io_master_read_barometer(IO_MASTER *p)
{
	barometer_bmp280_read_barometer(&p->barometer_bmp280_device);
}

/* This function is called for each rising/falling edge of PIN_THROTTLE_STICK_INPUT pin (interrupt) */
static void io_master_pin_interrupt_stick_input_throttle()
{

		if(digitalRead(PIN_THROTTLE_STICK_INPUT) == 1)
		{
			//Rising edge detected
			interrupt_throttle_stick_input.last_rising_edge_us = micros();
		}
		else
		{
			//Falling edge detected
			unsigned int delta_time = micros() - interrupt_throttle_stick_input.last_rising_edge_us;

			//Only accept pwm width of 0-STICK_INPUT_MAX_PULSE_WIDTH_US us
			if(delta_time > 0 && delta_time < STICK_INPUT_MAX_PULSE_WIDTH_US)
			{
				interrupt_throttle_stick_input.pwm_width_us = delta_time;
			}
		}
}

/* This function is called for each rising/falling edge of PIN_X_STICK_INPUT pin (interrupt) */
static void io_master_pin_interrupt_stick_input_x()
{

		if(digitalRead(PIN_X_STICK_INPUT) == 1)
		{
			//Rising edge detected
			interrupt_x_stick_input.last_rising_edge_us = micros();
		}
		else
		{
			//Falling edge detected
			unsigned int delta_time = micros() - interrupt_x_stick_input.last_rising_edge_us;

			//Only accept pwm width of 0-STICK_INPUT_MAX_PULSE_WIDTH_US us
			if(delta_time > 0 && delta_time < STICK_INPUT_MAX_PULSE_WIDTH_US)
			{
				interrupt_x_stick_input.pwm_width_us = delta_time;
			}
		}
}

/* This function is called for each rising/falling edge of PIN_Y_STICK_INPUT pin (interrupt) */
static void io_master_pin_interrupt_stick_input_y()
{

		if(digitalRead(PIN_Y_STICK_INPUT) == 1)
		{
			//Rising edge detected
			interrupt_y_stick_input.last_rising_edge_us = micros();
		}
		else
		{
			//Falling edge detected
			unsigned int delta_time = micros() - interrupt_y_stick_input.last_rising_edge_us;

			//Only accept pwm width of 0-STICK_INPUT_MAX_PULSE_WIDTH_US us
			if(delta_time > 0 && delta_time < STICK_INPUT_MAX_PULSE_WIDTH_US)
			{
				interrupt_y_stick_input.pwm_width_us = delta_time;
			}
		}
}

/* This function is called for each rising/falling edge of PIN_Z_STICK_INPUT pin (interrupt) */
static void io_master_pin_interrupt_stick_input_z()
{

		if(digitalRead(PIN_Z_STICK_INPUT) == 1)
		{
			//Rising edge detected
			interrupt_z_stick_input.last_rising_edge_us = micros();
		}
		else
		{
			//Falling edge detected
			unsigned int delta_time = micros() - interrupt_z_stick_input.last_rising_edge_us;

			//Only accept pwm width of 0-STICK_INPUT_MAX_PULSE_WIDTH_US us
			if(delta_time > 0 && delta_time < STICK_INPUT_MAX_PULSE_WIDTH_US)
			{
				interrupt_z_stick_input.pwm_width_us = delta_time;
			}
		}
}

/* Read data from i2c device starting from register start_reg */
int io_master_read_i2c_one_byte(const int device_id, const int start_reg, uint8_t *data_out)
{
	int status = 0;

	//Read one byte of data
	*data_out = wiringPiI2CReadReg8 (device_id, start_reg);

	return status;
}

/* Read 16 bit unsigned int and swap high and low byte */
void io_master_read_i2c_16_bit_swapped_unsiged(const int device_id, const int start_reg, uint16_t *data_out)
{
	uint16_t temp = (uint16_t)wiringPiI2CReadReg16(device_id, start_reg);

	*data_out = ((temp << 8) | ((temp >> 8) & 0xFF));
}

/* Read 16 bit signed int and swap high and low byte */
void io_master_read_i2c_16_bit_swapped_siged(const int device_id, const int start_reg, int16_t *data_out)
{
	uint16_t temp;
	io_master_read_i2c_16_bit_swapped_unsiged(device_id, start_reg, &temp);

	*data_out = (int16_t)temp;
}

/* Read 16 bit unsigned int */
void io_master_read_i2c_16_bit_unsiged(const int device_id, const int start_reg, uint16_t *data_out)
{
	*data_out = (uint16_t)wiringPiI2CReadReg16(device_id, start_reg);
}

/* Read 16 bit unsigned int */
void io_master_read_i2c_16_bit_siged(const int device_id, const int start_reg, int16_t *data_out)
{
	*data_out = (int16_t)wiringPiI2CReadReg16(device_id, start_reg);
}

/* Read 24 bit unsigned int */
void io_master_read_i2c_24_bit_unsiged(const int device_id, const int start_reg, uint32_t *data_out)
{
	uint8_t temp1, temp2, temp3;

	io_master_read_i2c_one_byte(device_id, start_reg, &temp1);
	io_master_read_i2c_one_byte(device_id, start_reg + 1, &temp2);
	io_master_read_i2c_one_byte(device_id, start_reg + 2, &temp3);


	*data_out = ((temp1 << 12) | (temp2 << 4) | ((temp3 >> 4) & 0xF));
}


/* Write data to i2c device starting from register start_reg */
int io_master_write_i2c_one_byte(const int device_id, const int start_reg, const uint8_t data_in)
{
	int status = 0;

	//Write one byte of data
	status = wiringPiI2CWriteReg8(device_id, start_reg, data_in);

	return status;
}

int io_master_check_i2c_device(const int device_id, const int reg_who_am_i, const int device_adress, const int who_am_i_value)
{
	int status;
	uint8_t data;

	status = io_master_read_i2c_one_byte(device_id, reg_who_am_i, &data);
	if(status >= 0)
	{
		if(data == who_am_i_value)
		{
			//The i2c device was found, return OK
			status = 0;
		}
		else
		{
			status = -1;
		}
	}
	return status;
}

static void io_master_set_pwm_stick_input_vals(IO_MASTER *p)
{
    //Throttle stick input
    p->stick_inp_trottle = interrupt_throttle_stick_input.pwm_width_us;
    p->stick_inp_x = interrupt_x_stick_input.pwm_width_us;
    p->stick_inp_y = interrupt_y_stick_input.pwm_width_us;
    p->stick_inp_z = interrupt_z_stick_input.pwm_width_us;
}

void io_master_show_inputs_execute(IO_MASTER *p)
{
	//Print calibration information if it is time
	unsigned int delta_time = millis() - p->last_show_inputs_loop_time_ms;
	if(delta_time > SHOW_INPUTS_PRINT_INTERVAL_MS)
	{

		int i;
		//Print blanks to make cmd scroll
		for(i = 0; i < 100; i++)
		{
			printf("\n");
		}

		printf("**************SHOW INPUTS************\n");
		printf("\n");
		printf("angle_x_deg_filtered: %f\n",p->acc_gyro_mpu6050_device.angle_x_deg_filtered);
		printf("angle_y_deg_filtered: %f\n",p->acc_gyro_mpu6050_device.angle_y_deg_filtered);
		printf("angle_z_deg_filtered: %f\n",p->acc_gyro_mpu6050_device.angle_z_deg_filtered);
		printf("gyr_x_deg_unfilt: %f\n",p->acc_gyro_mpu6050_device.gyr_x_deg_unfilt);
		printf("gyr_y_deg_unfilt: %f\n",p->acc_gyro_mpu6050_device.gyr_y_deg_unfilt);
		printf("gyr_z_deg_unfilt: %f\n",p->acc_gyro_mpu6050_device.gyr_z_deg_unfilt);
		printf("stick_inp_trottle: %f\n",p->stick_inp_trottle);
		printf("stick_inp_x: %f\n",p->stick_inp_x);
		printf("stick_inp_y: %f\n",p->stick_inp_y);
		printf("stick_inp_z: %f\n",p->stick_inp_z);
		printf("Barometer sensor temperature (degC): %f\n", (float)p->barometer_bmp280_device.t_fine/5120);
		printf("Barometer sensor pressure (kPa): %f\n", (float)p->barometer_bmp280_device.comp_pressure/256000);
		printf("Barometer sensor altitude from start (m): %f\n", p->barometer_bmp280_device.current_altitude_m - p->barometer_bmp280_device.starting_altitude_m);
		printf("\n");
		printf("**************************************\n");

		p->last_show_inputs_loop_time_ms = millis();
	}
}



