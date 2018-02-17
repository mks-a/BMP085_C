// put LED on PC5
#include <avr/io.h>       	// this is always included in AVR programs
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "../bmp085/bmp085.h"
#include "../timer1/timer1.h"

//#define debug

#ifdef debug
#include "../usart_dbg/usart_dbg.h"
#endif

int16_t temperature;

int main(void) 
{	
	init();
	bmp085_read_callibrations();
  
	while (1) 
	{
		temperature = bmp085_get_temperature();
		
/*#ifdef debug
		usart_dbg_transmit_hex(temperature >> 8);
		usart_dbg_transmit_hex(temperature);
#endif*/
		
		sleep_enable();
		sleep_cpu();		
		sleep_disable();
	}
  
	return(0);          	// should never get here, this is to prevent a compiler warning
}

void init(void)
{
	byte i;
	
	DDRB = 0x20;     		// set pin 13 as output, others as input

#ifdef debug	
	usart_dbg_init();
#endif

	timer1_init();
	
	bmp085_init();
	
	sei();
}

ISR(TIMER1_OVF_vect)
{
	/* this ISR purpose is wake up MCU and indicate of it with LED */
	PORTB ^= 0x20;
}

ISR(BADISR_vect) 
{
}