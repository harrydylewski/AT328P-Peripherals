# AT328P-Peripherals
API for use with ATMEGA328P

1) In the at328p-peripherals-lib.h file, every bit for every register is listed as a define which you 
will use to write your configuration to

2) The defines header contains defines and enums for what the configuration file is, it's listed seperately
so that the lib header can contain the configuration defines only

3) The term "INIT" is short for initialize, an initialize WILL OVERWRITE any current peripheral config,
so avoid using any functions with INIT in it during your main loop, the exception being any GPIO (DDR/PORT/PIN)
configs because of direct bit writing instructions SBI/CBI

4) For complete initialize you call AVR_PINS_INIT() which will configure the pins each peripheral uses as needed,
otherwise every other pin will be left alone as it's default floating input, and AVR_PERIPHERALS_INIT() which will
call every peripheral INIT function

5) Non configured peripherals should automatically be skipped over by the INIT functions
