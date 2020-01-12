#include "at328p-peripheral-libv2.h"

#define PIN(X) (1<<X)
#define BIT(X) (1<<X_
#define PINS(LOW,HIGH)  (unsigned char)((1<<(HIGH-LOW+1))-1)<<LOW


/*****************************************************

*******************************************************/

typedef enum 
{
	DISABLE,ENABLE
}STATUS;


#define _DEFAULT_REGISTER_VALUE 0

/******************************************************
Analog Comparator
*******************************************************/

//Enums-----------------------------------------------
//typedef enum 
//{
#define	AC_AIN1   0
#define	AC_AIN01  1
#define	AC_ADC0   2
#define	AC_ADC1   3
#define	AC_ADC2   4
#define	AC_ADC3   5
#define	AC_ADC4   6
#define	AC_ADC5   7
#define	AC_ADC6   8
#define	AC_ADC7   9
//} AC_NEG_INPUT_ENUM;

//typedef enum AC_INT_MODE_ENUM
//{
#define	AC_TOGGLE     0
#define	AC_RESERVED   1
#define	AC_FALLING    2
#define	AC_RISING     3
//}AC_INT_MODE_ENUM;

/******************************************************
ADC
*******************************************************/

//Enum-----------------------------------------------------

//typedef enum 
//{
#define	ADC_REF_AREF         0
#define	ADC_REF_AVCC         1
#define	ADC_REF_RESERVED     2
#define	ADC_REF_INTERNAL11V  3
//}ADC_REFERENCE_ENUM;

//typedef enum 
//{
#define	ADC_CHANNEL0            0
#define	ADC_CHANNEL1			1
#define	ADC_CHANNEL2			2			  
#define	ADC_CHANNEL3			3
#define	ADC_CHANNEL4			4
#define	ADC_CHANNEL5			5
#define	ADC_CHANNEL6			6
#define	ADC_CHANNEL7			7
#define	ADC_CHANNEL_TEMPERATURE 8
#define	ADC_CHANNEL_RESERVED0   9
#define	ADC_CHANNEL_RESERVED1   10
#define	ADC_CHANNEL_RESERVED2	11
#define	ADC_CHANNEL_RESERVED3 	12
#define	ADC_CHANNEL_RESERVED4	13
#define	ADC_CHANNEL_11V			14
#define	ADC_CHANNEL_GND			15
//}ADC_CHANNELS_ENUM;

//typedef enum 
//{
#define	ADC_PRESCALER0		0
#define	ADC_PRESCALER2		1
#define	ADC_PRESCALER4		2
#define	ADC_PRESCALER8		3
#define	ADC_PRESCALER16		4
#define	ADC_PRESCALER32		5
#define	ADC_PRESCALER64		6
#define	ADC_PRESCALER128    7
//}ADC_PRESCALER_ENUM;

//typedef enum 
//{
#define	ADC_FREERUN			0
#define	ADC_ANALOG_COMP		1
#define	ADC_EXT_INT0		2
#define	ADC_T0_COMA			3
#define	ADC_T0_OVF			4
#define	ADC_T1_COMB			5
#define	ADC_T1_OVF			6
#define	ADC_T1_CAPTURE		7
//}ADC_AUTOTRIGGER_ENUM;

//Hardware Defines-----------------------------------------------------
#define ADC_PORT PORTC
#define ADC_DDR  DDRC

//Masking Defines-----------------------------------------------
#define ADC_AC_MASK 0x40

//Macros----------------------------------------------------------

#define ADC_START_CONVERSION()      ADCSRA |= (1<<ADSC)
#define ADC_WAIT_CONVERSION()       while(!(ADCSRA&(1<<ADIF)))

#define ADC_RIGHTADJUST()            ADMUX &=~ (0<<ADLAR)
#define ADC_LEFTADJUST()             ADMUX &=~ (1<<ADLAR)
#define ADC_STOP_CONVERSION()        ADCSRA &=~ (1<<ADEN)
#define ADC_DISABLE()                ADCSRA = 0

/******************************************************
GPIO Interrupts
*******************************************************/

//Enum------------------------------------------------------------
//typedef enum 
//{
#define	GPIO_LOW		0
#define	GPIO_CHANGE		1
#define	GPIO_FALLING	2
#define	GPIO_RISING		3
//}INT_TYPE;

/*******************************************************
Timers
*******************************************************/

//Enums-----------------------------------------------
//typedef enum  //com is short for compare output mode
//{
#define	T_COM_DISCONNECT	0
#define	T_COM_TGL_ON_MATCH	1
#define	T_COM_CLR_ON_MATCH	2
#define	T_COM_SET_ON_MATCH	3
//} T_COM;

//typedef enum 
//{
#define	T_PRESCALER0		0
#define	T_PRESCALER1		1
#define	T_PRESCALER8		2
#define	T_PRESCALER64		3
#define	T_PRESCALER256		4
#define	T_PRESCALER1024, 	5
#define	T_EXTERNAL_CLOCK_FALLING  6
#define	T_EXTERNAL_CLOCK_RISING	  7
//} T_PRESCALER;

//typedef enum 
//{
#define	T0_MODE_NORMAL        0
#define	T0_MODE_PHASE_CORRECT 1
#define	T0_MODE_CTC_OCR0A	  2
#define	T0_MODE_FAST_PWM	  3
#define	T0_MODE_RESERVED4	  4
#define	T0_MODE_PHASE_CORRECT_OCR0A  5
#define	T0_MODE_RESERVED6			 6
#define	T0_MODE_FAST_PWM_OCR0A   	 7
//} T0_MODES;

//typedef enum 
//{
#define	T2_MODE_NORMAL			0
#define	T2_MODE_PHASE_CORRECT	1
#define	T2_MODE_CTC_OCR2A		2
#define	T2_MODE_FAST_PWM		3
#define	T2_MODE_RESERVED4		4
#define	T2_MODE_PHASE_CORRECT_OCR2A		5
#define	T2_MODE_RESERVED6				6
#define	T2_MODE_FAST_PWM_OCR2A   		7
//} T2_MODES;

//typedef enum 
//{
#define	T1_MODE_NORMAL,        			0
#define	T1_MODE_PHASE_CORRECT_8BIT		1
#define	T1_MODE_PHASE_CORRECT_9BIT		2
#define	T1_MODE_PHASE_CORRECT_10BIT		3
#define	T1_MODE_CTC_OCR1A				4
#define	T1_MODE_FAST_PWM_8BIT			5
#define	T1_MODE_FAST_PWM_9BIT			6
#define	T1_MODE_FAST_PWM_10BIT			7
#define	T1_MODE_PHASE_FREQUENCY_CORRECT_ICR1	8
#define	T1_MODE_PHASE_FREQUENCY_CORRECT_OCR1A	9
#define	T1_MODE_PHASE_CORRECT_ICR1				10
#define	T1_MODE_PHASE_CORRECT_OCR1A				11
#define	T1_MODE_CTC_ICR1						12
#define	T1_MODE_TIMER1_RESERVED13				13
#define	T1_MODE_FAST_PWM_ICR1					14
#define	T1_MODE_FAST_PWM_OCR1A  				15
//} T1_MODES;

//Masking Defines --------------------------------------------------
#define COMA_OFFSET 6
#define COMB_OFFSET 4
#define MODE_OFFSET 1
#define COMA_INT_OFFSET 1
#define COMB_INT_OFFSET 2

#define TIMER_MASK_PRESCALER 0x07
#define TIMER_MASK_ENUM      0x03
#define TIMER_MASK_MODE      0x04
#define TIMER_MASK_MODE_MSB  0x04

#define TIMER_MASK_MODE_LOW   0x03
#define TIMER_MASK_MODE_HIGH  0x0C

//Hardware Defines -----------------------------------------------------

#define OC1A PB1
#define OC1B PB2

#define OC2A PB3
#define OC2B PD3

#define OC0A PD6
#define OC0B PD5

#define TIMER0_DDR  DDRD
#define TIMER0_PORT PORTD

#define TIMER1_DDR  DDRB
#define TIMER1_PORT PORTB


/**************************************************
UART Section
**************************************************/

//Enum-----------------------------------------------

//typedef enum 
//{
#define	UART_ASYNC			0
#define	UART_SYNC			1
#define	UART_MODE_RESERVED	2
#define	UART_MASTER_SPI		3
//} UART_MODE_ENUM;

//typedef enum 
//{
#define	UART_PARITY_DISABLE		0
#define	UART_PARITY_RESERVED	1
#define	UART_PARITY_EVEN		2
#define	UART_PARITY_ODD			3
//} UART_PARITY_ENUM;

//typedef enum 
//{
#define	UART_STOP_1BIT		0
#define	UART_STOP_2BIT		1
//} UART_STOP_ENUM;

//typedef enum
//{
#define	UART_CHAR_SIZE_5BIT		0
#define	UART_CHAR_SIZE_6BIT		1
#define	UART_CHAR_SIZE_7BIT		2
#define	UART_CHAR_SIZE_8BIT		3
#define	UART_CHAR_RESERVED0		4
#define	UART_CHAR_RESERVED1		5
#define	UART_CHAR_RESERVED2		6
#define	UART_CHAR_SIZE_9BIT		7
//} UART_CHAR_SIZE_ENUM;

//typedef enum
//{
#define	UART_RISING_EDGE	0 // for async always leave as zero/rising edge
#define	UART_FALL_EDGE		1
//} UART_CLOCK_POLARITY_ENUM;

//typedef enum 
//{
#define	UART_INT_DISABLE	0
#define	UART_INT_ENABLE		1
	
//} UART_INT_ENUM;

//Masking Defines-----------------------------------------

#define UART_CHAR_SIZE_MASKC  0x03
#define UART_CHAR_SIZE_OFFSETC   1

#define UART_CHAR_SIZE_MASKB  0x04
#define UART_CHAR_SIZE_OFFSETB   2

#define UART_RX_PIN_MASK 0x01
#define UART_TX_PIN_MASK 0x02

//Hardware Defines---------------------------------------------------

#define UART_DDR  DDRD
#define UART_PORT PORTD

#define UART_RX_PIN PD0
#define UART_TX_PIN PD1

#define UART_CONFIG_TX_PIN() UART_DDR |=  (1<<UART_TX_PIN)
#define UART_CONFIG_RX_PIN() UART_DDR &=~ (1<<UART_RX_PIN)


/**************************************************
SPI Section
**************************************************/

//Due to how the code is written, do not rewrite anything as an enum
//Hardware Defines-------------------------------------------
#define SPI_MASTER_MASK ( (1<<SCK) | (1<<MOSI) | (1<<SS) )
#define SPI_SLAVE_MASK  ( (1<<SCK) | (1<<MISO) | (1<<SS) )

#define SPI_SS_MASK  (1<<SS)
#define SPI_DDR      DDRB
#define SPI_PORT     PORTB

#define SCK  PB5
#define MISO PB4
#define MOSI PB3
#define SS   PB2

#define SPI_SLAVE 0
#define SPI_MASTER 1

#define SPI_PRESCALER4   0
#define	SPI_PRESCALER16  1
#define	SPI_PRESCALER64  2
#define	SPI_PRESCALER128 3
#define	SPI_PRESCALER2   0
#define	SPI_PRESCALER8   1
#define	SPI_PRESCALER32  2


/**************************************************
I2C Section
**************************************************/

/*
@8Mhz
Prescaler 1 | 4 | 16 | 64 
          2   8   32  128
@16MHz   
Prescaler 1 | 4 | 16 | 64 
          12  48  192   - 
*/

#endif