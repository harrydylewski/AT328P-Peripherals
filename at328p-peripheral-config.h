
#ifndef AT328P_PERIPHERAL_CONFIG_H
#define AT328P_PERIPHERAL_CONFIG_H

#include "at328p-peripheral-defines.h"

// Anything value with 0 will automatically be skipped to avoid overhead

/**************************************************
Setting Defines
**************************************************/



//Enable these bits to disable the respective peripheral
#define I2C_DISABLE_CLK     DISABLE
#define SPI_DISABLE_CLK     DISABLE
#define USART_DISABLE_CLK   DISABLE

#define TIMER2_DISABLE_CLK  DISABLE
#define TIMER0_DISABLE_CLK  DISABLE
#define TIMER1_DISABLE_CLK  DISABLE

#define ADC_DISABLE_CLK     DISABLE

//Analog Comparator Defines-----------------------------------------------

#define AC_MULTIPLEXER_ENABLE   DISABLE //0 disable, 1 enable
#define AC_MULTIPLEXER_SOURCE   AC_AIN1 //AC_NEG_INPUT_ENUM ADC_INT_MODE_ENUM

#define AC_DISABLE				DISABLE
#define AC_BANDGAP_ENABLE       DISABLE
#define AC_COMP_OUTPUT          DISABLE
#define AC_INT_ENABLE           DISABLE
#define AC_INPUT_CAPT_ENABLE    DISABLE
#define AC_INT_MODE             AC_TOGGLE //ADC_INT_MODE_ENUM

//ADC Defines--------------------------------------------------

#define ADC_ENABLE                   DISABLE
#define ADC_AUTOTRIGGER_ENABLE       DISABLE  //0 disable, 1 enable
#define ADC_AUTOTRIGGER_SOURCE       ADC_FREERUN
#define ADC_PRESCALER                ADC_PRESCALER0 // for 16MHZ, you must set it to 128
#define ADC_REFERENCE                ADC_REF_AREF
#define ADC_INTERRUPT_ENABLE         DISABLE

//GPIO 

#define INTERRUPT0_ENABLE DISABLE
#define INTERRUPT1_ENABLE DISABLE

#define INTERRUPT0_MODE GPIO_LOW //enum INT_TYPE
#define INTERRUPT1_MODE GPIO_LOW

#define PCINT_PORTD DISABLE  //PCMSK2
#define PCINT_PORTC DISABLE  //PCMSK1
#define PCINT_PORTB DISABLE  //PCMSK0

#define PCINT_PIND PINS(0,0)
#define PCINT_PINC PINS(0,0)
#define PCINT_PINB PINS(0,0)

//I2C Settings-----------------------------------------------------

#define I2C_ENABLE      DISABLE
#define I2C_FREQUENCY   400000UL //4000000UL or 100000UL
#define I2C_PRESCALER   1        //1, 4, 16, 64
#define I2C_INT_ENABLE  DISABLE  //WIP, do not use      

//SPI----------------------------------------------------------------------
#define SPI_ENABLE                 DISABLE
#define SPI_PRESCALER_SETTING      SPI_PRESCALER128 //choose from 2,4,8,16,32,64,128
#define SPI_MASTER_SETTING         SPI_SLAVE     //master or slave
//if SPI is not used set to slave and set prescaler to 128

//Uart Settings-------------------------------------------------------

#define UART_BAUDRATE 111520

#define UART_NORMAL_BAUDRATE (F_CPU / (16*UART_BAUDRATE)) - 1
#define UART_DOUBLE_BAUDRATE (F_CPU / ( 8*UART_BAUDRATE)) - 1

#define UART_INIT_BAUDRATE(BAUDRATE) UBRR0 = BAUDRATE

#define UART_INIT_DOUBLE_SPEED() UCSR0A = (1<<U2X0)

#define UART_CHAR_SIZE_SETTING   8  //5-9, 9 is not actively supported
#define UART_PARITY_SETTING      UART_PARITY_DISABLE

#define UART_STOP_SIZE_SETTING  UART_STOP_1BIT
#define UART_TX_ENABLE          DISABLE
#define UART_RX_ENABLE          DISABLE
#define UART_INT_ENABLE			DISABLE

//Timer Settings-------------------------------------------------------

#define TIMER0_MODE       T0_MODE_NORMAL
#define TIMER0_PRESCALER  T_PRESCALER0
#define TIMER0_COMA       T_COM_DISCONNECT
#define TIMER0_COMB       T_COM_DISCONNECT
#define TIMER0_OVF_INT    DISABLE
#define TIMER0_COMA_INT   DISABLE
#define TIMER0_COMB_INT   DISABLE

#define TIMER1_MODE       T1_MODE_NORMAL
#define TIMER1_PRESCALER  T_PRESCALER0
#define TIMER1_COMA       T_COM_DISCONNECT
#define TIMER1_COMB       T_COM_DISCONNECT
#define TIMER1_OVF_INT    DISABLE
#define TIMER1_COMA_INT   DISABLE
#define TIMER1_COMB_INT   DISABLE

#define TIMER2_MODE       T2_MODE_NORMAL
#define TIMER2_PRESCALER  T_PRESCALER0
#define TIMER2_COMA       T_COM_DISCONNECT
#define TIMER2_COMB       T_COM_DISCONNECT
#define TIMER2_OVF_INT    DISABLE
#define TIMER2_COMA_INT   DISABLE
#define TIMER2_COMB_INT   DISABLE


/**************************************************
Redefines and Error Messages
**************************************************/

//Spi Errors
#if SPI_ENABLE !=0 && SPI_ENABLE !=1
#error "SPI_ENABLE Misconfigued"
#endif

#if SPI_PRESCALER_SETTING<0 && SPI_PRESCALER_SETTING>3
#error "SPI Prescaler Misconfigured"
#endif

#if SPI_MASTER != 0 && SPI_MASTER !=1
#error "SPI Master Misconfigured"
#endif

//I2C Errors

#if I2C_ENABLE!=0 && I2C_ENABLE!=1
#error "I2C_ENABLE Misconfigued"
#endif

#define I2C_FREQUENCY_SETTING ( ((F_CPU/I2C_FREQUENCY)-16) / (2*I2C_PRESCALER) ) 

#if     I2C_PRESCALER == 1
#define I2C_PRESCALER_SETTING 0
#elif   I2C_PRESCALER == 4
#define I2C_PRESCALER_SETTING 1
#elif   I2C_PRESCALER == 16
#define I2C_PRESCALER_SETTING 2
#elif   I2C_PRESCALER == 64
#define I2C_PRESCALER_SETTING 3
#else
#define I2C_PRESCALER_SETTING 0
#pragma  "I2C Prescaler Setting Defaulted to 1"
#endif

//UART Errors

#if UART_CHAR_SIZE_SETTING<5 || UART_CHAR_SIZE_SETTING>8
#error "Invalid Character Size for UART"
//DO NOT DELETE
/*
#elif  UART_CHAR_SIZE_SETTING == 9
#undef UART_CHAR_SIZE_SETTING
#define UART_CHAR_SIZE_SETTING 12
*/
#endif

#if UART_PARITY_SETTING>3
#error "UART Parity Misconfigured"
#endif

#if UART_STOP_SIZE_SETTING !=0 && UART_STOP_SIZE_SETTING!=1
#error "UART Stop Size Misconfigured"
#endif

#if UART_TX_SETTING !=0 && UART_TX_SETTING !=1
#error "UART TX Pin Misconfigured"
#endif

#if UART_RX_SETTING !=0 && UART_RX_SETTING !=1
#error "UART TX Pin Misconfigured"
#endif

#define ASC_SR_SETTING (AC_DISABLE&1<<ACD)              | (AC_BANDGAP_ENABLE&1<<ACBG) \
                       | (AC_COMP_OUTPUT&1<<ACO)        | (AC_INT_ENABLE&1<<ACIE)     \
                       | (AC_INPUT_CAPT_ENABLE&1<<ACIC) | (AC_INT_MODE)
/**************************************************
Functions
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

//ADC Functions--------------------------------------------------------------

inline void ADC_INIT(void)
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

inline unsigned int adc_readWithPoll(unsigned char channel)
{
	
	ADMUX  = (ADMUX&0xF0) | channel;
	ADC_START_CONVERSION();
	ADC_WAIT_CONVERSION();
	
	return ADC;
}

//GPIO Interrupts -----------------------------------------------

inline void GPIO_INT_INIT(void)
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

//SPI Functions-----------------------------------------
inline void SPI_INIT_PINS(void)
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

inline void SPI_INIT(void)
{

	#if ((SPI_ENABLE<<SPE) | (SPI_MASTER_SETTING<<4) | (SPI_PRESCALER_SETTING) != _DEFAULT_REGISTER_VALUE )
	SPCR = ((SPI_ENABLE<<SPE) | (SPI_MASTER_SETTING<<4) | (SPI_PRESCALER_SETTING));
	#endif
	
	#if (SPI_PRESCALER_SETTING==SPI_PRESCALER2 || SPI_PRESCALER_SETTING==SPI_PRESCALER8 || SPI_PRESCALER_SETTING==SPI_PRESCALER32)
	SPSR = 0x01; //turn on double speed bit
	#endif

}

inline void spi_writeWithPoll(unsigned char data)
{

	SPDR = data;

	while(!(SPSR & (1<<SPIF)))
	;
	
}

inline unsigned char spi_readWithPoll(void)
{
	SPDR = 0;

	while(!(SPSR & (1<<SPIF)))
	;

	return(SPDR);
}


//I2C Functions ----------------------------------------------------
inline void I2C_INIT(void)
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

inline void i2c_start(void)
{
	TWCR = 0;
	TWCR |= (1<<TWINT)|(1<<TWSTA);
	while (!(TWCR & (1<<TWINT)));
}

inline void i2c_stop(void)
{
	TWCR |= (1<<TWINT)|(1<<TWSTO);
}

inline void i2c_writeWithPoll(unsigned char data)
{
	TWDR = data;
	TWCR |= (1<<TWINT);
	while (!(TWCR & (1<<TWINT)));
}

inline unsigned char i2c_readAckWithPoll(void)
{
	TWCR |= (1<<TWINT)|(1<<TWEA);
	while( !(TWCR & (1<<TWINT)) );
	return TWDR;
}

inline unsigned char i2c_readNackWithPoll(void)
{
	TWCR |= (1<<TWINT);
	while( !(TWCR & (1<<TWINT)) );
	return TWDR;
}

//UART Functions-----------------------------------------------
inline void UART_INIT_PINS(void)
{
	#if UART_TX_SETTING != 0
	UART_CONFIG_TX_PIN();
	#endif
	
	#if UART_TX_SETTING != 0
	UART_CONFIG_RX_PIN();  
	UART_PORT &=~ UART_RX_PIN; //disable any pull up resistors
	#endif
}

inline void UART_INIT(void)
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

inline unsigned char uart_readWithPoll(void)
{
	while(!( UCSR0A&(1<<RXC0) ) )
	;
	return UDR0;
}

inline void uart_writeWithPoll(unsigned char data)
{
	UDR0=data;
	
	while(!( UCSR0A&(1<<UDRE0) ) )
	;
}

//Timer Functions ----------------------------------------------------

inline void TIMER0_PIN_INIT(void)
{
	#if  (TIMER0_COMA != 0 && (TIMER0_MODE != 2 || TIMER0_MODE != 5 || TIMER0_MODE != 7)) && TIMER0_COMB != 0
	TIMER0_DDR |= (1<<OC0A) |(1<<OC0B);
	#elif TIMER0_COMB != 0 && TIMER0_COMA == 0
	TIMER0_DDR |= (1<<OC0B);
	#elif (TIMER0_COMA != 0 && (TIMER0_MODE != 2 || TIMER0_MODE != 5 || TIMER0_MODE != 7)) && TIMER1_COMB == 0
	TIMER0_DDR |= (1<<OC0A);
	#endif
}

inline void  TIMER0_INIT(void)
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
//
inline void TIMER1_PIN_INIT(void)
{
	
	#if  (TIMER1_COMA > 0 && (TIMER1_MODE != 4 || TIMER1_MODE != 9 || TIMER1_MODE != 11|| TIMER1_MODE != 15)) && TIMER1_COMB > 0
	TIMER1_DDR |= (1<<OC1A) |(1<<OC1B);
	
	#elif TIMER1_COMB > 0 && TIMER1_COMA == 0
	TIMER1_DDR |= (1<<OC1B);
	
	#elif (TIMER1_COMA > 0 && (TIMER1_MODE != 4 && TIMER1_MODE != 9 && TIMER1_MODE != 1 && TIMER1_MODE != 15)) && TIMER1_COMB == 0
	TIMER1_DDR |= (1<<OC1A) ;
	#endif
}

inline void TIMER1_INIT(void)
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
//
inline void TIMER2_PIN_INIT(void)
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

inline void  TIMER2_INIT(void)
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

#define TIMER1_ENABLE_PRESCALER(SELECT_PRESCALER) TCCR1B |= SELECT_PRESCALER
#define TIMER1_DISABLE_PRESCALER() TCCR1B &=~ TIMER_MASK_PRESCALER

#endif