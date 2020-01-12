#ifndef AT328P_PERIPHERAL_LIBV2_H
#define AT328P_PERIPHERAL_LIBV2_H


#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <stdbool.h>


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


void SPI_INIT_PINS(void);
void UART_INIT_PINS(void);
void TIMER0_PIN_INIT(void);
void TIMER1_PIN_INIT(void);
void TIMER2_PIN_INIT(void);

void ADC_INIT(void);
void GPIO_INT_INIT(void);
void SPI_INIT(void);
void I2C_INIT(void);
void UART_INIT(void);
void TIMER0_INIT(void);
void TIMER1_INIT(void);
void TIMER2_INIT(void);

unsigned int adc_read(unsigned char channel);

void spi_write(unsigned char data);
unsigned char spi_read(void);

void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char data);
unsigned char i2c_readAck(void);
unsigned char i2c_readNack(void);

unsigned char uart_read(void);
void uart_write(unsigned char data);

void AVR_PIN_INIT(void);
void AVR_PERIPHERAL_INIT(void);

#endif