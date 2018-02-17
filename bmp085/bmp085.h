#ifndef bmp085_h

#define bmp085_h

typedef unsigned char byte;

#ifndef F_CPU          		// if F_CPU was not defined in Project -> Properties
#define F_CPU 16000000UL    // define it now as 16 MHz unsigned long
#endif

#include <util/delay.h>
#include "../twi/twi.h"

#define debug

#ifdef debug
#include "../usart_dbg/usart_dbg.h"
#endif

#define BMP085_MODULE_ADDR_W 0xEE
#define BMP085_MODULE_ADDR_R 0xEF
#define BMP085_TEMP_CTRL_ADDR 0x2E
#define BMP085_PRES0_CTRL_ADDR 0x34
#define BMP085_PRES1_CTRL_ADDR 0x74
#define BMP085_PRES2_CTRL_ADDR 0xB4
#define BMP085_PRES3_CTRL_ADDR 0xF4
#define BMP085_MEASURE_REG_ADDR 0xF4
#define BMP085_BITRATE_PRESCALER 0x48 	// prescaler for Atmega328p

// 16 bit bmp085 register address where sensor value will be stored before sending to master device (that address should be send in read request)
#define BMP085_CONV_RESULT_MSB 0xF6
#define BMP085_CONV_RESULT_LSB 0xF7

#define BMP085_CALLIBRATIONS_ARR_LEN 11
#define BMP085_INIT_CALLEBRATION_ADDR 0xAA		// initial address of callebration registers in BMP085

#define BMP085_TWCR_INIT 0x44			// init value from TWCR register

#define BMP085_TEMP_DELAY_MS 5

void bmp085_init(void);
static void bmp085_init_calibration_addresses(void);
void bmp085_read_callibrations(void);
int16_t bmp085_get_temperature(void);
static int16_t bmp085_calculate_temperature(uint16_t from_sensor);
static uint8_t bmp085_read_data(uint8_t _reg_addr_to_read, uint16_t _reg_addr, uint16_t* _buffer);
uint8_t bmp085_request_sensor_data(uint8_t _sensor_reg_address);

#endif