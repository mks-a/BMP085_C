#include "bmp085.h"

static uint16_t callibrations[BMP085_CALLIBRATIONS_ARR_LEN];

// to avoid index calculation I've created 2 arrays
static byte calibration_addresses_msb[BMP085_CALLIBRATIONS_ARR_LEN];
static byte calibration_addresses_lsb[BMP085_CALLIBRATIONS_ARR_LEN];

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
		
/*#ifdef debug
	usart_dbg_transmit_hex((uint8_t)callibrations[i] >> 8);
	usart_dbg_transmit_hex((uint8_t)callibrations[i]);
	
	usart_dbg_transmit_hex(0xFF);
#endif*/
		
/*#ifdef debug
		// 0x0000 or 0xFFFF will be received in case of error
		if(err || callibrations[i] == 0x0000 || callibrations[i] == 0xFFFF)
		{
			usart_dbg_transmit_hex(0xFF);
			usart_dbg_transmit_hex(0xFF);
		}
#endif*/
	}
}

/* read temperature data from bmp085 sensor 
returns calculated value of temperature 1 = 0.1 degree */  
int16_t bmp085_get_temperature(void)
{
	uint16_t sensor_data;
	uint16_t calibration_addr16;
	uint8_t err = 0;
	
	err = bmp085_request_sensor_data(BMP085_TEMP_CTRL_ADDR);
	
	if(!err)
	{
		_delay_ms(BMP085_TEMP_DELAY_MS);
		
		calibration_addr16 = BMP085_CONV_RESULT_MSB << 8 | BMP085_CONV_RESULT_LSB;
		
		err = bmp085_read_data(BMP085_CONV_RESULT_MSB, calibration_addr16, &sensor_data);
		
		if(!err)
			return bmp085_calculate_temperature(sensor_data);
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
	uint16_t tmp_data;
	
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
		
		// read value from BMP085_CONV_RESULT_MSB
		tmp_data = twi_read_data() << 8;
		
		status = TWI_GET_STATUS();
		
		if(status != TWI_MR_DATA_R_ACK)
			error = 1;
	}
	
	if(!error)
	{
		TWI_ACK_DISABLE();
		
		// send LSB of conversion result register address
		twi_send_byte((uint8_t)_reg_addr_internal);
		
		// read value from BMP085_CONV_RESULT_LSB
		tmp_data |= twi_read_data();
		
		status = TWI_GET_STATUS();
		
		if(status != TWI_MR_DATA_R_NACK)
			error = 1;
	}
	
	*_buffer = tmp_data;

#ifdef debug
	usart_dbg_transmit_hex((uint8_t)tmp_data >> 8);
	usart_dbg_transmit_hex((uint8_t)tmp_data);
	
	usart_dbg_transmit_hex(0xFF);
#endif
	
	twi_send_stop_condition();
	
	return !error ? 0 : status;
}

// int from_sensor - temperature value from sensor
// returns real temperature value
static int16_t bmp085_calculate_temperature(uint16_t from_sensor)
{
	int32_t x1, x2, b5;
	int32_t tmp_data;
	int16_t ret = 0;

/*#ifdef debug
	usart_dbg_transmit_hex((uint8_t)callibrations[4] >> 8);
	usart_dbg_transmit_hex((uint8_t)callibrations[4]);
	
	usart_dbg_transmit_hex(0xFF);
#endif*/
	// x1 calculation
	tmp_data = callibrations[4] >> 15;	
/*#ifdef debug
	usart_dbg_transmit_hex((uint8_t)tmp_data >> 24);
	usart_dbg_transmit_hex((uint8_t)tmp_data >> 16);
	usart_dbg_transmit_hex((uint8_t)tmp_data >> 8);
	usart_dbg_transmit_hex((uint8_t)tmp_data);
	
	usart_dbg_transmit_hex(0xFF);
#endif*/
	x1 = (from_sensor - callibrations[5]) * tmp_data;
	
/*#ifdef debug
	usart_dbg_transmit_hex((uint8_t)x1 >> 24);
	usart_dbg_transmit_hex((uint8_t)x1 >> 16);
	usart_dbg_transmit_hex((uint8_t)x1 >> 8);
	usart_dbg_transmit_hex((uint8_t)x1);
	
	usart_dbg_transmit_hex(0xFF);
#endif*/
	
	// x2 calculation
	tmp_data = callibrations[9] << 11;
	x2 = tmp_data / (x1 + callibrations[10]);
	
/*#ifdef debug
	usart_dbg_transmit_hex((uint8_t)x2 >> 8);
	usart_dbg_transmit_hex((uint8_t)x2);
#endif*/
	
	// b5 calculation
	b5 = x1 + x2;
	
/*#ifdef debug
	usart_dbg_transmit_hex((uint8_t)b5 >> 8);
	usart_dbg_transmit_hex((uint8_t)b5);
#endif*/
	
	// real value calculation
	ret = (b5 + 8) >> 4;
	
/*#ifdef debug
	usart_dbg_transmit_hex((uint8_t)ret >> 8);
	usart_dbg_transmit_hex((uint8_t)ret);
#endif*/
	
	return ret;
}