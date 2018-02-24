#include "bmp085.h"

static uint16_t callibrations[BMP085_CALLIBRATIONS_ARR_LEN];

// to avoid index calculation I've created 2 arrays
static byte calibration_addresses_msb[BMP085_CALLIBRATIONS_ARR_LEN];
static byte calibration_addresses_lsb[BMP085_CALLIBRATIONS_ARR_LEN];

static long b3, b5, b6;
static unsigned long b4, b7;


/*initialize data used in communication with bmp085*/
void bmp085_init(void)
{
	byte i;
	
	// clear calibrations array
	for(i = 0; i < BMP085_CALLIBRATIONS_ARR_LEN; i++)
		callibrations[i] = 0;
	
	bmp085_init_calibration_addresses();
	
	twi_init(BMP085_TWCR_INIT, BMP085_BITRATE_PRESCALER);
}

/* clears arrays where bmp085 data would be held */
static void bmp085_init_calibration_addresses(void)
{
	byte reg_value = BMP085_INIT_CALLEBRATION_ADDR;
	byte i;
	
	for(i = 0; i < BMP085_CALLIBRATIONS_ARR_LEN; i++)
	{
		calibration_addresses_msb[i] = reg_value;
		reg_value++;
		calibration_addresses_lsb[i] = reg_value;
		reg_value++;
	}
}

/* reads calibrations from bmp085 sensor */
void bmp085_read_callibrations(void)
{
	byte i;
	uint16_t calibration_addr16;
	uint8_t err;
	
	for(i = 0; i < BMP085_CALLIBRATIONS_ARR_LEN; i++)
	{
		calibration_addr16 = calibration_addresses_msb[i] << 8 | calibration_addresses_lsb[i];
		
		err = bmp085_read_data(calibration_addresses_msb[i], calibration_addr16, &callibrations[i]);
	}
}

/* read temperature data from bmp085 sensor 
returns calculated value of temperature 1 = 0.1 degree */  
short bmp085_get_temperature(void)
{
	uint16_t sensor_data;
	uint16_t temp_addr16;
	uint8_t err = 0;
	
	err = bmp085_request_sensor_data(BMP085_TEMP_CTRL_ADDR);
	
	if(!err)
	{
		_delay_ms(BMP085_TEMP_DELAY_MS);
		
		temp_addr16 = BMP085_CONV_RESULT_MSB << 8 | BMP085_CONV_RESULT_LSB;
		
		err = bmp085_read_data(BMP085_CONV_RESULT_MSB, temp_addr16, &sensor_data);
		
		if(!err)
			return bmp085_calculate_temperature(sensor_data);
	}
	
	return 0;
}

/* read pressure std data from bmp085 sensor 
returns calculated value of pressure in Pa */
long bmp085_get_pressure(void)
{
	uint16_t sensor_data;
	uint16_t press_addr16;
	uint8_t err = 0;
	
	err = bmp085_request_sensor_data(BMP085_PRES0_CTRL_ADDR);
	
	if(!err)
	{
		_delay_ms(BMP085_PRESS_DELAY_STD_MS);
		
		press_addr16 = BMP085_CONV_RESULT_MSB << 8 | BMP085_CONV_RESULT_LSB;
		
		err = bmp085_read_data(BMP085_CONV_RESULT_MSB, press_addr16, &sensor_data);
		
		if(!err)
			return bmp085_calculate_pressure(sensor_data);
	}
	
	return 0;
}

// Requests data from specified bmp085 sensor address
// _sensor_reg_address - bmp085 sensor register address
// On success returns 0, otherwise status code
uint8_t bmp085_request_sensor_data(uint8_t _sensor_reg_address)
{
	uint8_t status;
	uint8_t error = 0;			// for some reason compiler think, that I\m not using this variable and optimize it away...So I've declared it as volatile 
	
	// send start condition to sensor
	twi_send_start_condition();
	
	status = TWI_GET_STATUS();
	
	if(status != TWI_START_CONDITION && status != TWI_RSTART_CONDITION)
	{
		error = 1;
	}
	
	// send bmp085 write address
	if(!error)
	{
		twi_send_byte(BMP085_MODULE_ADDR_W);

		status = TWI_GET_STATUS();
		
		if(status != TWI_MT_SLA_W_ACK)
			error = 1;
	}
	
	// send bmp085 register address to read
	if(!error)
	{
		twi_send_byte(BMP085_MEASURE_REG_ADDR);

		status = TWI_GET_STATUS();
		
		if(status != TWI_MT_DATA_W_ACK)
			error = 1;
	}
	
	if(!error)
	{
		twi_send_byte(_sensor_reg_address);

		status = TWI_GET_STATUS();
		
		if(status != TWI_MT_DATA_W_ACK)
			error = 1;
	}
	
	twi_send_stop_condition();
	
	return !error ? 0 : status;
}

// read data from bmp085 sensor
// _reg_addr_to_read - 8bit bmp085 register address to read 
//		this should be MSB address in case of calibrations or BMP085_CONV_RESULT_MSB in case of sensor data
// _reg_addr_internal - 16bit bmp085 internal register address to read
//		if reading calibrations MSB of this value should be calibration MSB and LSB - calibration LSB
// _buffer - buffer where read data will be saved
// on success return 0, on failure error code
static uint8_t bmp085_read_data(uint8_t _reg_addr_to_read, uint16_t _reg_addr_internal, uint16_t* _buffer)
{
	uint8_t status;
	uint8_t error = 0;
	
	// send start condition to sensor
	twi_send_start_condition();
	
	status = TWI_GET_STATUS();
	
	if(status != TWI_START_CONDITION)
		error = 1;
	
	// send bmp085 write address
	if(!error)
	{
		twi_send_byte(BMP085_MODULE_ADDR_W);

		status = TWI_GET_STATUS();
		
		if(status != TWI_MT_SLA_W_ACK)
			error = 1;
	}
	
	// send bmp085 register address to read
	if(!error)
	{
		twi_send_byte(_reg_addr_to_read);

		status = TWI_GET_STATUS();
		
		if(status != TWI_MT_DATA_W_ACK)
			error = 1;
	}
	
	if(!error)
	{
		// read data from sensor
		twi_send_start_condition();
		status = TWI_GET_STATUS();
	
		if(status != TWI_RSTART_CONDITION)
			error = 1;
	}
	
	if(!error)
	{
		// send bmp085 read address
		twi_send_byte(BMP085_MODULE_ADDR_R);

		status = TWI_GET_STATUS();
	
		if(status != TWI_MR_SLA_R_ACK)
			error = 1;
	}
		
	if(!error)
	{
		twi_send_byte((uint8_t)_reg_addr_internal >> 8);
		
		// read value from BMP085_CONV_RESULT_MSB (Atmega328 is Little endian)
		*_buffer = ((uint16_t)twi_read_data()) << 8;
		
		status = TWI_GET_STATUS();
		
		if(status != TWI_MR_DATA_R_ACK)
			error = 1;
	}
	
	if(!error)
	{
		TWI_ACK_DISABLE();
		
		// send LSB of conversion result register address
		twi_send_byte((uint8_t)_reg_addr_internal);
		
		// read value from BMP085_CONV_RESULT_LSB (Atmega328 is Little endian)
		*_buffer |= ((uint16_t)twi_read_data()); 
		
		status = TWI_GET_STATUS();
		
		if(status != TWI_MR_DATA_R_NACK)
			error = 1;
	}
	
	twi_send_stop_condition();
	
	return !error ? 0 : status;
}

// int from_sensor - temperature value from sensor
// returns real temperature value
static short bmp085_calculate_temperature(unsigned int from_sensor)
{
	long x1, x2;
	uint8_t tmp_uint;
	//long ret = 0;
	
	x1 = (((long)from_sensor - (long)callibrations[5])*(long)callibrations[4]) >> 15;

	// callibrations[9] should be signed
	x2 = ((long)((short)callibrations[9]) << 11)/(x1 + (long)callibrations[10]);

	b5 = x1 + x2;
	
	return (b5 + 8) >> 4;	
}

// int from_sensor - temperature value from sensor
// returns pressure value in Pa
static long bmp085_calculate_pressure(unsigned int from_sensor)
{
	long x1, x2, x3, p;
/*#ifdef debug
	b5 = 2399;
	callibrations[0] = 408; 	// ac1
	callibrations[1] = -72; 	// ac2
	callibrations[2] = -14383;	// ac3
	callibrations[3] = 32741;	// ac4
	callibrations[6] = 6190;	// b1
	callibrations[7] = 4; 		// b2
#endif*/
	
	b6 = b5 - 4000;
	
	x1 = ((long)((short)callibrations[7]) * (b6 * b6 >> 12)) >> 11;	
	x2 = (long)((short)callibrations[1]) * b6 >> 11;
	x3 = x1 + x2;	
	b3 = ((long)((short)callibrations[0]) * 4 + x3) >> 2;
	
	x1 = (long)((short)callibrations[2]) * b6 >> 13;
	x2 = ((long)((short)callibrations[6]) * (b6 * b6 >> 12)) >> 16;	
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (long)callibrations[3] * (unsigned long)(x3 + 32768) >> 15;
	
	b7 = ((unsigned long)23843 - b3) * 50000;

	if(b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;
	
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	
	p = p + ((x1 + x2 + 3791) >> 4);
	
	return p;
}