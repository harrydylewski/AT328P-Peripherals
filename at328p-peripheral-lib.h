#ifndef AT328P_PERIPHERAL_LIB_H
#define AT328P_PERIPHERAL_LIB_H

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <stdbool.h>

#include "at328p-peripheral-config.h"
#include "at328p-peripheral-defines.h"
	
#define PINS(LOW,HIGH)  (unsigned char)((1<<(HIGH-LOW+1))-1)<<LOW

//Setup Defines

/*
ISR (WDT_vect)
{
	
}
*/
static inline void AVR_PERIPHERAL_INIT(void)
{
	
	wdt_disable();
	
	UART_INIT();
	SPI_INIT();
	I2C_INIT();
	
	ADC_INIT();
	AC_INIT();
	
	TIMER0_INIT();
	TIMER1_INIT();
	TIMER2_INIT();
	#if     (I2C_DISABLE_CLK<<PRTWI)     | (SPI_DISABLE_CLK<<PRSPI)     | (USART_DISABLE_CLK<<PRUSART0)
	      | (TIMER2_DISABLE_CLK<<PRTIM2) | (TIMER1_DISABLE_CLK<<PRTIM1) | (TIMER0_DISABLE_CLK<<PRTIM0)
	      | (ADC_DISABLE_CLK<<PRADC)  != _DEFAULT_REGISTER_VALUE 
		  
	PRR =   (I2C_DISABLE_CLK<<PRTWI)     | (SPI_DISABLE_CLK<<PRSPI)     | (USART_DISABLE_CLK<<PRUSART0)
	      | (TIMER2_DISABLE_CLK<<PRTIM2) | (TIMER1_DISABLE_CLK<<PRTIM1) | (TIMER0_DISABLE_CLK<<PRTIM0)
	      | (ADC_DISABLE_CLK<<PRADC);
	#endif
}

inline void AVR_PIN_INIT(void)
{
	UART_INIT_PINS();
	SPI_INIT_PINS();
	GPIO_INT_INIT();
	
	TIMER0_PIN_INIT();
	TIMER1_PIN_INIT();
	TIMER2_PIN_INIT();
}
//------------------------------------------------------------------------------------

enum FLAGS
{
	SPI_FLAG,
	UART_RX_FLAG,
	UART_TX_FLAG,
	UART_UDRE_FLAG,
	I2C_FLAG,
	ADC_FLAG
};

#define GLOBAL_FLAG_REG GPIOR0
/*
ISR(TWI_vect, ISR_NAKED)
{
	GLOBAL_FLAG_REG &=~ (1<<I2C_FLAG);
	reti();
}

ISR(SPI_STC_vect, ISR_NAKED)
{
	GLOBAL_FLAG_REG &=~ (1<<SPI_FLAG);
	reti();
}

ISR(USART_RX_vect, ISR_NAKED)
{
	GLOBAL_FLAG_REG &=~ (1<<UART_RX_FLAG);
	reti();
}

ISR(USART_TX_vect, ISR_NAKED)
{
	GLOBAL_FLAG_REG &=~ (1<<UART_TX_FLAG);
	reti();
}

ISR(USART_UDRE_vect, ISR_NAKED)
{
	GLOBAL_FLAG_REG &=~ (1<<UART_UDRE_FLAG);
	reti();
}

ISR(ADC_vect, ISR_NAKED)
{
	GLOBAL_FLAG_REG &=~ (1<<ADC_FLAG);
	reti();
}
*/
inline void clear_interrupt_flags(void)
{
	GLOBAL_FLAG_REG = 0;
}
//------------------------------------------------

inline unsigned int adc_readWithInt(unsigned char channel)
{
	ADMUX  = (ADMUX&0xF0) | channel;
	
	GLOBAL_FLAG_REG |= ADC_FLAG;
	set_sleep_mode(SLEEP_MODE_ADC);
	
	while (GLOBAL_FLAG_REG&ADC_FLAG)
	{
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	};
	
	return ADC;
}

//--------------------------------------------------
inline void spi_writeWithInt(unsigned char data)
{

	SPDR = data;
	
	GLOBAL_FLAG_REG |= SPI_FLAG;
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	while (GLOBAL_FLAG_REG&SPI_FLAG)
	{
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	};
}

inline unsigned char spi_readWithInt(void)
{
	SPDR = 0;
	
	GLOBAL_FLAG_REG |= SPI_FLAG;
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	while (GLOBAL_FLAG_REG&SPI_FLAG)
	{
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	};

	return(SPDR);
}

//
inline void i2c_writeWithInt(unsigned char data)
{
	TWDR = data;
	TWCR |= (1<<TWINT);
	
	GLOBAL_FLAG_REG |= I2C_FLAG;
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	while (GLOBAL_FLAG_REG&I2C_FLAG)
	{
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	};
	
}

inline unsigned char i2c_readAckWithInt(void)
{
	TWCR |= (1<<TWINT)|(1<<TWEA);
	
	
	GLOBAL_FLAG_REG|= I2C_FLAG;
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	while (GLOBAL_FLAG_REG&I2C_FLAG)
	{
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	};
	
	return TWDR;
}

inline unsigned char i2c_readNackWithInt(void)
{
	TWCR |= (1<<TWINT);
	
	GLOBAL_FLAG_REG |= I2C_FLAG;
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	while(GLOBAL_FLAG_REG&I2C_FLAG)
	{
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	};
	
	return TWDR;
}
//-----------------------------------------------------------

#endif