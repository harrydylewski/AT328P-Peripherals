
#ifndef AT328P_PERIPHERAL_DEFINES_H
#define AT328P_PERIPHERAL_DEFINES_H

/*****************************************************

*******************************************************/
/*
typedef enum 
{
	FALSE,TRUE
}BOOLEAN;
*/
typedef enum 
{
	DISABLE,ENABLE
}STATUS;


#define _DEFAULT_REGISTER_VALUE 0


/******************************************************
Analog Comparator
*******************************************************/

//Enums-----------------------------------------------
typedef enum 
{
	AC_AIN1,
	AC_AIN01,
	AC_ADC0,
	AC_ADC1,
	AC_ADC2,
	AC_ADC3,
	AC_ADC4,
	AC_ADC5,
	AC_ADC6,
	AC_ADC7
} AC_NEG_INPUT_ENUM;

typedef enum AC_INT_MODE_ENUM
{
	AC_TOGGLE,
	AC_RESERVED,
	AC_FALLING,
	AC_RISING
}AC_INT_MODE_ENUM;

/******************************************************
ADC
*******************************************************/

//Enum-----------------------------------------------------

typedef enum 
{
	ADC_REF_AREF,
	ADC_REF_AVCC,
	ADC_REF_RESERVED,
	ADC_REF_INTERNAL11V
}ADC_REFERENCE_ENUM;

typedef enum 
{
	ADC_CHANNEL0,
	ADC_CHANNEL1,
	ADC_CHANNEL3,
	ADC_CHANNEL4,
	ADC_CHANNEL5,
	ADC_CHANNEL6,
	ADC_CHANNEL7,
	ADC_CHANNEL_TEMPERATURE,
	ADC_CHANNEL_RESERVED0,
	ADC_CHANNEL_RESERVED1,
	ADC_CHANNEL_RESERVED2,
	ADC_CHANNEL_RESERVED3,
	ADC_CHANNEL_RESERVED4,
	ADC_CHANNEL_11V,
	ADC_CHANNEL_GND
}ADC_CHANNELS_ENUM;

typedef enum 
{
	ADC_PRESCALER0,
	ADC_PRESCALER2,
	ADC_PRESCALER4,
	ADC_PRESCALER8,
	ADC_PRESCALER16,
	ADC_PRESCALER32,
	ADC_PRESCALER64,
	ADC_PRESCALER128
}ADC_PRESCALER_ENUM;

typedef enum 
{
	ADC_FREERUN,
	ADC_ANALOG_COMP,
	ADC_EXT_INT0,
	ADC_T0_COMA,
	ADC_T0_OVF,
	ADC_T1_COMB,
	ADC_T1_OVF,
	ADC_T1_CAPTURE
}ADC_AUTOTRIGGER_ENUM;

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
typedef enum 
{
	GPIO_LOW,
	GPIO_CHANGE,
	GPIO_FALLING,
	GPIO_RISING
}INT_TYPE;

/*******************************************************
Timers
*******************************************************/

//Enums-----------------------------------------------
typedef enum  //com is short for compare output mode
{
	T_COM_DISCONNECT,
	T_COM_TGL_ON_MATCH,
	T_COM_CLR_ON_MATCH,
	T_COM_SET_ON_MATCH
} T_COM;

typedef enum 
{
	T_PRESCALER0, //0
	T_PRESCALER1,
	T_PRESCALER8,
	T_PRESCALER64,
	T_PRESCALER256,
	T_PRESCALER1024, //5
	T_EXTERNAL_CLOCK_FALLING,
	T_EXTERNAL_CLOCK_RISING
} T_PRESCALER;

typedef enum 
{
	T0_MODE_NORMAL,         //0
	T0_MODE_PHASE_CORRECT,
	T0_MODE_CTC_OCR0A,
	T0_MODE_FAST_PWM,
	T0_MODE_RESERVED4,
	T0_MODE_PHASE_CORRECT_OCR0A,
	T0_MODE_RESERVED6,
	T0_MODE_FAST_PWM_OCR0A   //7
} T0_MODES;

typedef enum 
{
	T2_MODE_NORMAL,         //0
	T2_MODE_PHASE_CORRECT,
	T2_MODE_CTC_OCR2A,
	T2_MODE_FAST_PWM,
	T2_MODE_RESERVED4,
	T2_MODE_PHASE_CORRECT_OCR2A,
	T2_MODE_RESERVED6,
	T2_MODE_FAST_PWM_OCR2A   //7
} T2_MODES;

typedef enum 
{
	T1_MODE_NORMAL,         //0
	T1_MODE_PHASE_CORRECT_8BIT,
	T1_MODE_PHASE_CORRECT_9BIT,
	T1_MODE_PHASE_CORRECT_10BIT,
	T1_MODE_CTC_OCR1A,
	T1_MODE_FAST_PWM_8BIT,
	T1_MODE_FAST_PWM_9BIT,
	T1_MODE_FAST_PWM_10BIT,
	T1_MODE_PHASE_FREQUENCY_CORRECT_ICR1,
	T1_MODE_PHASE_FREQUENCY_CORRECT_OCR1A,
	T1_MODE_PHASE_CORRECT_ICR1,
	T1_MODE_PHASE_CORRECT_OCR1A,
	T1_MODE_CTC_ICR1,
	T1_MODE_TIMER1_RESERVED13,
	T1_MODE_FAST_PWM_ICR1,
	T1_MODE_FAST_PWM_OCR1A  //15
} T1_MODES;

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

typedef enum 
{
	UART_ASYNC,
	UART_SYNC,
	UART_MODE_RESERVED,
	UART_MASTER_SPI
} UART_MODE_ENUM;

typedef enum 
{
	UART_PARITY_DISABLE,
	UART_PARITY_RESERVED,
	UART_PARITY_EVEN,
	UART_PARITY_ODD
} UART_PARITY_ENUM;

typedef enum 
{
	UART_STOP_1BIT,
	UART_STOP_2BIT
} UART_STOP_ENUM;

typedef enum
{
	UART_CHAR_SIZE_5BIT,
	UART_CHAR_SIZE_6BIT,
	UART_CHAR_SIZE_7BIT,
	UART_CHAR_SIZE_8BIT,
	UART_CHAR_RESERVED0,
	UART_CHAR_RESERVED1,
	UART_CHAR_RESERVED2,
	UART_CHAR_SIZE_9BIT
} UART_CHAR_SIZE_ENUM;

typedef enum
{
	UART_RISING_EDGE, // for async always leave as zero/rising edge
	UART_FALL_EDGE
} UART_CLOCK_POLARITY_ENUM;

typedef enum 
{
	UART_INT_DISABLE,
	UART_INT_ENABLE
	
} UART_INT_ENUM;

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