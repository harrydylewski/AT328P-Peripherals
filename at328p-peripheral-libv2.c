

#include "at328p-peripheral-libv2.h"

/**************************************************
Macros
**************************************************/

#define ADC_START_CONVERSION()      ADCSRA |= (1<<ADSC)
#define ADC_WAIT_CONVERSION()       while(!(ADCSRA&(1<<ADIF)))

#define ADC_RIGHTADJUST()            ADMUX &=~ (0<<ADLAR)
#define ADC_LEFTADJUST()             ADMUX &=~ (1<<ADLAR)
#define ADC_STOP_CONVERSION()        ADCSRA &=~ (1<<ADEN)
#define ADC_DISABLE()                ADCSRA = 0

/**************************************************
Pin Init Functions
**************************************************/
void SPI_INIT_PINS(void)
{
	#if SPI_ENABLE == 1
		#if SPI_MASTER_SETTING   == SPI_MASTER
		SPI_DDR |= SPI_MASTER_MASK;
		SPI_DDR &=~MISO;
	
		#elif SPI_MASTER_SETTING   == SPI_SLAVE
		SPI_DDR |= MISO;
		SPI_DDR &=~SPI_SLAVE_MASK;
		#endif
	#endif
}

void UART_INIT_PINS(void)
{
	#if UART_TX_SETTING != 0
	UART_CONFIG_TX_PIN();
	#endif
	
	#if UART_TX_SETTING != 0
	UART_CONFIG_RX_PIN();  
	UART_PORT &=~ UART_RX_PIN; //disable any pull up resistors
	#endif
}

void TIMER0_PIN_INIT(void)
{
	#if  (TIMER0_COMA != 0 && (TIMER0_MODE != 2 || TIMER0_MODE != 5 || TIMER0_MODE != 7)) && TIMER0_COMB != 0
	TIMER0_DDR |= (1<<OC0A) |(1<<OC0B);
	#elif TIMER0_COMB != 0 && TIMER0_COMA == 0
	TIMER0_DDR |= (1<<OC0B);
	#elif (TIMER0_COMA != 0 && (TIMER0_MODE != 2 || TIMER0_MODE != 5 || TIMER0_MODE != 7)) && TIMER1_COMB == 0
	TIMER0_DDR |= (1<<OC0A);
	#endif
}

void TIMER1_PIN_INIT(void)
{
	
	#if  (TIMER1_COMA > 0 && (TIMER1_MODE != 4 || TIMER1_MODE != 9 || TIMER1_MODE != 11|| TIMER1_MODE != 15)) && TIMER1_COMB > 0
	TIMER1_DDR |= (1<<OC1A) |(1<<OC1B);
	
	#elif TIMER1_COMB > 0 && TIMER1_COMA == 0
	TIMER1_DDR |= (1<<OC1B);
	
	#elif (TIMER1_COMA > 0 && (TIMER1_MODE != 4 && TIMER1_MODE != 9 && TIMER1_MODE != 1 && TIMER1_MODE != 15)) && TIMER1_COMB == 0
	TIMER1_DDR |= (1<<OC1A) ;
	#endif
}

void TIMER2_PIN_INIT(void)
{
	#if  (TIMER2_COMA != 0 && (TIMER2_MODE != 2 || TIMER2_MODE != 5 || TIMER2_MODE != 7)) && TIMER2_COMB != 0
	DDRB |= (1<<OC2A);
	DDRD |= (1<<OC2B);
	#elif TIMER2_COMB != 0 && TIMER2_COMA == 0
	DDRD |= (1<<OC2B);
	#elif (TIMER2_COMA != 0 && (TIMER2_MODE != 2 || TIMER2_MODE != 5 || TIMER2_MODE != 7)) && TIMER2_COMB == 0
	DDRB |= (1<<OC2A);
	#endif

}

/**************************************************
Peripheral Init Functions
**************************************************/
inline void AC_INIT(void)
{
	#if ( ((AC_MULTIPLEXER_ENABLE<<ACME) | AC_MULTIPLEXER_SOURCE) != _DEFAULT_REGISTER_VALUE)
	ADCSRB = (AC_MULTIPLEXER_ENABLE<<ACME) | AC_MULTIPLEXER_SOURCE;
	#endif
	
	#if ( ASC_SR_SETTING != _DEFAULT_REGISTER_VALUE)
	ACSR    =   ASC_SR_SETTING;
	#endif
}

void ADC_INIT(void)
{
	#if ((ADC_REFERENCE&3)<<6 != _DEFAULT_REGISTER_VALUE)
	ADMUX =  (ADC_REFERENCE&3)<<6;
	#endif
	#if ( ((ADC_ENABLE&1)<<ADEN) | ((ADC_AUTOTRIGGER_ENABLE&1)<<ADATE) | ((ADC_INTERRUPT_ENABLE&1)<<ADIE) | (ADC_PRESCALER&7) != _DEFAULT_REGISTER_VALUE)
	ADCSRA = ((ADC_ENABLE&1)<<ADEN) | ((ADC_AUTOTRIGGER_ENABLE&1)<<ADATE) | ((ADC_INTERRUPT_ENABLE&1)<<ADIE) | (ADC_PRESCALER&7);
	#endif
	#if (ADC_AUTOTRIGGER_SOURCE&7 != _DEFAULT_REGISTER_VALUE)
	ADCSRB = (ADC_AUTOTRIGGER_SOURCE&7);
	#endif
}

void GPIO_INT_INIT(void)
{
	#if (((INTERRUPT1_MODE&2)<<2)   | (INTERRUPT0_MODE&2) != _DEFAULT_REGISTER_VALUE)
	EICRA  = ((INTERRUPT1_MODE&2)<<2)   | (INTERRUPT0_MODE&2);
	#endif
	
	#if ( ((INTERRUPT1_ENABLE&1)<<1) | (INTERRUPT0_ENABLE&1) != _DEFAULT_REGISTER_VALUE )
	EIMSK  = ((INTERRUPT1_ENABLE&1)<<1) | (INTERRUPT0_ENABLE&1);
	#endif
	#if (PCINT_PORTD!=_DEFAULT_REGISTER_VALUE)
	PCMSK2 = PCINT_PORTD;
	#endif
	#if (PCINT_PORTC != _DEFAULT_REGISTER_VALUE)
	PCMSK1 = PCINT_PORTC;
	#endif
	#if (PCINT_PORTB != _DEFAULT_REGISTER_VALUE)
	PCMSK0 = PCINT_PORTB;
	#endif
	#if (((PCINT_PORTD&1)<<2) | ((PCINT_PORTC&1)<<1) | (PCINT_PORTB&1) != _DEFAULT_REGISTER_VALUE )
	PCICR  = ((PCINT_PORTD&1)<<2) | ((PCINT_PORTC&1)<<1) | (PCINT_PORTB&1);
	#endif
}

void SPI_INIT(void)
{

	#if ((SPI_ENABLE<<SPE) | (SPI_MASTER_SETTING<<4) | (SPI_PRESCALER_SETTING) != _DEFAULT_REGISTER_VALUE )
	SPCR = ((SPI_ENABLE<<SPE) | (SPI_MASTER_SETTING<<4) | (SPI_PRESCALER_SETTING));
	#endif
	
	#if (SPI_PRESCALER_SETTING==SPI_PRESCALER2 || SPI_PRESCALER_SETTING==SPI_PRESCALER8 || SPI_PRESCALER_SETTING==SPI_PRESCALER32)
	SPSR = 0x01; //turn on double speed bit
	#endif

}

void I2C_INIT(void)
{
	#if (I2C_PRESCALER_SETTING != _DEFAULT_REGISTER_VALUE)
	TWSR = I2C_PRESCALER_SETTING;
	#endif
	#if (I2C_FREQUENCY_SETTING != _DEFAULT_REGISTER_VALUE && I2C_ENABLE)
	TWBR = I2C_FREQUENCY_SETTING;
	#endif
	#if ( ((I2C_ENABLE&1)<<TWEN) | (I2C_INT_ENABLE&1) != _DEFAULT_REGISTER_VALUE)
	TWCR = ((I2C_ENABLE&1)<<TWEN) | (I2C_INT_ENABLE&1);
	#endif
}

void UART_INIT(void)
{
	#if( (UART_DOUBLE_BAUDRATE != _DEFAULT_REGISTER_VALUE) && ((UART_RX_ENABLE) && (UART_TX_ENABLE)) )
	UBRR0  = UART_DOUBLE_BAUDRATE; //Set Baud Rate
	UCSR0A =  (1<<U2X0);            //Enable Double Baud Rate
	#endif
	
	#if( ((UART_RX_ENABLE&1)<<RXEN0)      | ((UART_TX_ENABLE&1)<<TXEN0)         | (UART_CHAR_SIZE_SETTING&4) != _DEFAULT_REGISTER_VALUE )
	UCSR0B = ((UART_RX_ENABLE&1)<<RXEN0)      | ((UART_TX_ENABLE&1)<<TXEN0)         | (UART_CHAR_SIZE_SETTING&4);
	#endif
	
	#if ( (( ((UART_PARITY_SETTING&2)<<UPM00)  | ((UART_STOP_SIZE_SETTING&1)<<USBS0)  | (((UART_CHAR_SIZE_SETTING-5)&3)<<1) )  != _DEFAULT_REGISTER_VALUE) && ((UART_RX_ENABLE) && (UART_TX_ENABLE))   )
	UCSR0C = ((UART_PARITY_SETTING&2)<<UPM00)  | ((UART_STOP_SIZE_SETTING&1)<<USBS0)  | (((UART_CHAR_SIZE_SETTING-5)&3)<<1);
	#endif
}

void  TIMER0_INIT(void)
{
	#if ( ((TIMER0_COMA&3)<<COMA_OFFSET)  |  ((TIMER0_COMB&3)<<COMB_OFFSET)  |  (TIMER0_MODE&TIMER_MASK_MODE_LOW) != _DEFAULT_REGISTER_VALUE)
	TCCR0A = ((TIMER0_COMA&3)<<COMA_OFFSET)  |  ((TIMER0_COMB&3)<<COMB_OFFSET)  |  (TIMER0_MODE&TIMER_MASK_MODE_LOW);
	#endif
	#if (  ((TIMER0_MODE&TIMER_MASK_MODE_HIGH)<<MODE_OFFSET)  | (TIMER0_PRESCALER&7) != _DEFAULT_REGISTER_VALUE )
	TCCR0B = ((TIMER0_MODE&TIMER_MASK_MODE_HIGH)<<MODE_OFFSET)  | (TIMER0_PRESCALER&7) ;
	#endif
	#if( ((TIMER0_COMB_INT&1)<<COMB_INT_OFFSET) | ((TIMER0_COMA_INT&1)<<COMA_INT_OFFSET) | (TIMER0_OVF_INT&1) != _DEFAULT_REGISTER_VALUE)
	TIMSK0 = ((TIMER0_COMB_INT&1)<<COMB_INT_OFFSET) | ((TIMER0_COMA_INT&1)<<COMA_INT_OFFSET) | (TIMER0_OVF_INT&1);
	#endif
}

void TIMER1_INIT(void)
{
	#if ((TIMER1_COMA<<COMA_OFFSET) |  (TIMER1_COMB<<COMB_OFFSET) | (TIMER1_MODE&TIMER_MASK_MODE_LOW) != _DEFAULT_REGISTER_VALUE)
	TCCR1A = (TIMER1_COMA<<COMA_OFFSET) |  (TIMER1_COMB<<COMB_OFFSET) | (TIMER1_MODE&TIMER_MASK_MODE_LOW);
	#endif
	#if ((TIMER1_PRESCALER) |  ((TIMER1_MODE&TIMER_MASK_MODE_HIGH)<<MODE_OFFSET) != _DEFAULT_REGISTER_VALUE )
	TCCR1B = (TIMER1_PRESCALER) |  ((TIMER1_MODE&TIMER_MASK_MODE_HIGH)<<MODE_OFFSET);
	#endif
	#if  ( (TIMER1_COMB_INT<<COMB_INT_OFFSET) | (TIMER1_COMA_INT<<COMA_INT_OFFSET) | (TIMER1_OVF_INT) != _DEFAULT_REGISTER_VALUE ) 	
	TIMSK1 = (TIMER1_COMB_INT<<COMB_INT_OFFSET) | (TIMER1_COMA_INT<<COMA_INT_OFFSET) | (TIMER1_OVF_INT);
	#endif
}


void  TIMER2_INIT(void)
{
	#if ( ((TIMER2_COMA&3)<<COMA_OFFSET)  |  ((TIMER2_COMB&3)<<COMB_OFFSET)  |  (TIMER2_MODE&TIMER_MASK_MODE_LOW) != _DEFAULT_REGISTER_VALUE )
	TCCR2A = ((TIMER2_COMA&3)<<COMA_OFFSET)  |  ((TIMER2_COMB&3)<<COMB_OFFSET)  |  (TIMER2_MODE&TIMER_MASK_MODE_LOW);
	#endif
	#if ( ((TIMER2_MODE&TIMER_MASK_MODE_HIGH)<<MODE_OFFSET)  | (TIMER2_PRESCALER&7) != _DEFAULT_REGISTER_VALUE )
	TCCR2B = ((TIMER2_MODE&TIMER_MASK_MODE_HIGH)<<MODE_OFFSET)  | (TIMER2_PRESCALER&7) ;
	#endif
	#if ( ((TIMER2_COMB_INT&1)<<COMB_INT_OFFSET) | ((TIMER2_COMA_INT&1)<<COMA_INT_OFFSET) | (TIMER2_OVF_INT&1) != _DEFAULT_REGISTER_VALUE)	
	TIMSK2 = ((TIMER2_COMB_INT&1)<<COMB_INT_OFFSET) | ((TIMER2_COMA_INT&1)<<COMA_INT_OFFSET) | (TIMER2_OVF_INT&1);
	#endif
	
}
/**************************************************
Read/Write Functions
**************************************************/

//ADC Functions--------------------------------------------------------------


unsigned int adc_read(unsigned char channel)
{
	
	ADMUX  = (ADMUX&0xF0) | channel;
	ADC_START_CONVERSION();
	ADC_WAIT_CONVERSION();
	
	return ADC;
}

//SPI Functions-----------------------------------------

void spi_write(unsigned char data)
{

	SPDR = data;

	while(!(SPSR & (1<<SPIF)))
	;
	
}

unsigned char spi_read(void)
{
	SPDR = 0;

	while(!(SPSR & (1<<SPIF)))
	;

	return(SPDR);
}

//I2C Functions ----------------------------------------------------


void i2c_start(void)
{
	TWCR = 0;
	TWCR |= (1<<TWINT)|(1<<TWSTA);
	while (!(TWCR & (1<<TWINT)));
}

void i2c_stop(void)
{
	TWCR |= (1<<TWINT)|(1<<TWSTO);
}

void i2c_write(unsigned char data)
{
	TWDR = data;
	TWCR |= (1<<TWINT);
	while (!(TWCR & (1<<TWINT)));
}

unsigned char i2c_readAck(void)
{
	TWCR |= (1<<TWINT)|(1<<TWEA);
	while( !(TWCR & (1<<TWINT)) );
	return TWDR;
}

unsigned char i2c_readNack(void)
{
	TWCR |= (1<<TWINT);
	while( !(TWCR & (1<<TWINT)) );
	return TWDR;
}

//UART Functions-----------------------------------------------


unsigned char uart_read(void)
{
	while(!( UCSR0A&(1<<RXC0) ) )
	;
	return UDR0;
}

void uart_write(unsigned char data)
{
	UDR0=data;
	
	while(!( UCSR0A&(1<<UDRE0) ) )
	;
}

/**************************************************
Call all Init Functions
**************************************************/

void AVR_PIN_INIT(void)
{
	UART_INIT_PINS();
	SPI_INIT_PINS();
	GPIO_INT_INIT();
	
	TIMER0_PIN_INIT();
	TIMER1_PIN_INIT();
	TIMER2_PIN_INIT();
}

void AVR_PERIPHERAL_INIT(void)
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


