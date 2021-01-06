/* =============================================================
EE101 Embedded Firmware Debugger
Firmware Library
Provided by EE101.com

This file is to be included in your embedded firmware project to 
send debug information to the EE101 Embedded Firmware Debugger.

It can operate in a 1-wire (UART) mode, or a 2-wire (GPIO) mode.

In 1-wire mode, you will use your onboard Async UART set to N,8,1 at any
baud rate up to 3MBaud.

In 2-wire mode, it uses 2 signals (a clock and data line).  
The EE101 Insight-Pro&trade; autoselects the correct polarity.  The 2 
signals are General Purpose I/O pins (GPIO) which are 
available on most microprocessors. Only output is required.
The output high voltage level can be anywhere from 1V to 5V.

You must modify this file in a few places to specify how to set 
and clear the GPIO and how to enable/disable interrupts (if you 
need to). 

ONLY MODIFY THE CODE BELOW BETWEEN THE FLAGS.

If you have any questions or issues, please email us at
support@ee101.com.
===============================================================*/

//************** MAKE YOUR CHANGES BELOW ONLY *************************
// These defines are an example using the Cypress PSoC5LP Microcontroller

#define EE101_DEBUG_ON // Comment this line out to turn off the EE101 Debug outputs

// Change #1 - Add any includes that you need for the below defines
//#include "project.h"          // PSoC Creator PSoC 4,5,6
//#include "cy_pdl.h"           // ModusToolbox PSoC 6
//#include "cyhal.h"            // ModusToolbox PSoC 6
//#include "cybsp.h"            // ModusToolbox PSoC 6
//#include "cy_retarget_io.h"   // ModusToolbox PSoC 6
//#include "resource_map.h"     // ModusToolbox PSoC 6
//extern cyhal_uart_t uart_obj; // ModusToolbox PSoC 6

#include <wiringPi.h>           // Raspberry Pi
#include <wiringSerial.h>       // Raspberry Pi
extern int fd;                  // Raspberry Pi

// CHANGE #2 - How to disable Interrupts. Only needed if you have debug 
// output in interrupts.  Make sure if you do disable interrupts that it
// is done in a way that does not lose the interrupts (keeps the interrupt
// flags active) so that when the interrupt is re-enabled the interrupt will 
// be serviced.  
#define EE101IntDisable     ;		// No Interrupt Debugging
#define EE101IntEnable      ;
//#define EE101IntDisable     __disable_irq();		    // ModusToolbox Interrupt Debugging
//#define EE101IntEnable      __enable_irq();           // ModusToolbox Interrupt Debugging
//#define EE101IntDisable     CyGlobalIntDisable;       // PSoC Creator Interrupt Debugging
//#define EE101IntEnable      CyGlobalIntEnable;        // PSoC Creator Interrupt Debugging

// CHANGE #3 - Choose if you're using 2 wire (GPIO) or 1 wire (UART) EE101 interface
#define EE101_ONE_WIRE          // Uncomment this line if you are using a single signal UART interface for EE101 debug data
//#define EE101_TWO_WIRE        // Uncomment this if you are using 2 wire sync interface for EE101 debug data

// CHANGE #4 - FOR 2-wire EE101 Mode:  Defines that set the GPIO pins to High and Low levels and Toggles 
// These should be as fast as possible, but not faster than 50ns (20MHz)
// These two GPIO are a Clock and a Data line.  They must be setup as outputs
// elsewhere in your firmware during system initialization.
// The Data line toggle must invert the current state of the GPIO line
//#define EE101ClockLow     EE101_CLOCK_DR &= ~(1 << EE101_CLOCK_SHIFT);                // PSoC Version
//#define EE101ClockHigh    EE101_CLOCK_DR |= (1 << EE101_CLOCK_SHIFT);                 // PSoC Version
//#define EE101DataLow      EE101_DATA_DR &= ~(1 << EE101_DATA_SHIFT);                  // PSoC Version
//#define EE101DataHigh     EE101_DATA_DR |= (1 << EE101_DATA_SHIFT);                   // PSoC Version
//#define EE101DataToggle   EE101_DATA_DR ^= (1 << EE101_DATA_SHIFT);                   // PSoC Version
//#define EE101ClockLow     cyhal_gpio_write(CYBSP_D10, 0u);          		            // ModusToolbox PSoC 6 Version P12.2[D13] and P12.3 [D10]
//#define EE101ClockHigh    cyhal_gpio_write(CYBSP_D10, 1u);           			        // ModusToolbox PSoC 6 Version
//#define EE101DataLow      cyhal_gpio_write(CYBSP_D13, 0u);            			    // ModusToolbox PSoC 6 Version
//#define EE101DataHigh     cyhal_gpio_write(CYBSP_D13, 1u);             			    // ModusToolbox PSoC 6 Version
//#define EE101DataToggle   cyhal_gpio_write(CYBSP_D13, ~cyhal_gpio_read(CYBSP_D13));   // ModusToolbox PSoC 6 Version

#define EE101ClockLow     digitalWrite(7, LOW);                                         // Raspberry Pi Version, using wiringPi library numbering
#define EE101ClockHigh    digitalWrite(7, HIGH);                                        // Raspberry Pi Version, using wiringPi library numbering
#define EE101DataLow      digitalWrite(0, LOW);                                         // Raspberry Pi Version, using wiringPi library numbering
#define EE101DataHigh     digitalWrite(0, HIGH);                                        // Raspberry Pi Version, using wiringPi library numbering
#define EE101DataToggle   digitalWrite(0, !digitalRead(0));                             // Raspberry Pi Version, using wiringPi library numbering

//#define EE101ClockLow     digitalWrite(10, LOW);                                // Arduino Version
//#define EE101ClockHigh    digitalWrite(10, HIGH);                               // Arduino Version
//#define EE101DataLow      digitalWrite(11, LOW);                                // Arduino Version
//#define EE101DataHigh     digitalWrite(11, HIGH);                               // Arduino Version
//#define EE101DataToggle   digitalWrite(11, !digitalRead(11));                   // Arduino Version
//#define EE101ClockLow     GPIO_PinOutClear(EE101_CLOCK_PORT, EE101_CLOCK_PIN)   // STM EFM32 Version
//#define EE101ClockHigh    GPIO_PinOutSet(EE101_CLOCK_PORT, EE101_CLOCK_PIN)     // STM EFM32 Version
//#define EE101DataLow      GPIO_PinOutClear(EE101_DATA_PORT, EE101_DATA_PIN)     // STM EFM32 Version
//#define EE101DataHigh     GPIO_PinOutSet(EE101_DATA_PORT, EE101_DATA_PIN)       // STM EFM32 Version
//#define EE101DataToggle   GPIO_PinOutToggle(EE101_DATA_PORT, EE101_DATA_PIN)    // STM EFM32 Version
//#define EE101ClockLow     PORTCbits.RC12 = 0;                                   // PIC X32 Version
//#define EE101ClockHigh    PORTCbits.RC12 = 1;                                   // PIC X32 Version
//#define EE101DataLow      PORTCbits.RC13 = 0;                                   // PIC X32 Version
//#define EE101DataHigh     PORTCbits.RC13 = 1;                                   // PIC X32 Version
//#define EE101DataToggle   PORTCbits.RC13 = !PORTCbits.RC13;                     // PIC X32 Version

// CHANGE #5 - FOR 1-wire EE101 Mode:  Defines your routine name to send a single byte to the UART 
// The UART must be configured and enabled elsewhere in your firmware.
//#define EE101UartTx(x)     UART_PutChar(x)                        // PSoC Creator
//#define EE101UartTx(x)     cyhal_uart_putc(&uart_obj, x) 		    // ModusToolBox PSoC 6 P12.1 [D12]
#define EE101UartTx(x)      serialPutchar(fd, x)                    // Raspberry Pi

// CHANGE #6 - Enable/Disable Variable Argument support for SendEE101printf
#define VARIABLE_ARGUMENT_SUPPORT // Comment out this line if your compiler does not
                                  // have va_list, va_start, vsprintf, and va_end support
                                  // as defined in stdarg.h

#define MAX_STRING_LENGTH 250 // How much RAM to use for the SendEE101printf buffer
                              // This must not be greater than 250
                              // This defines the maximum length of any debug text message

#ifdef VARIABLE_ARGUMENT_SUPPORT // Includes required by the va_list, va_start, vsprintf, and va_end 
#include <stdio.h>    // vsprintf
#include <stdarg.h>   // va_list, va_start, and va_end
#endif

// CHANGE #7 - Type defines for your platform
// Copy these defines and function prototypes to your header files to define the API
#define euint8   unsigned char    // unsigned 8 bit value
#define eint8    signed char      // signed 8 bit value
#define eint32   signed long      // signed 32 bit value
#define echar    char             // bytes within a string
void EE101Value( euint8 channel, eint32 value );            // Output a Value for this channel
void EE101Text( euint8 channel, echar *string );            // Output Text for this channel
void EE101ValueLabel( euint8 channel, echar *string );      // Set the label for this Value Channel (sent every 256 times)
void EE101TextLabel( euint8 channel, echar *string );       // Set the label for this Text Channel (sent every 256 times)
#ifdef VARIABLE_ARGUMENT_SUPPORT                            
void EE101printf( euint8 channel, echar *format, ... );     // printf-like function with variable argument list
#endif

//************** MAKE YOUR CHANGES ABOVE ONLY *************************

#define EE101_SYNC 0x50
#define EE101_VALUE_TYPE 0x80
#define EE101_TEXT_TYPE 0x00
#define EE101_LABEL 0x08

#ifdef EE101_ONE_WIRE   // This is a 1-wire interface

void SendEE101Byte( euint8 value )
{
    EE101UartTx( value );
} 

void EE101Value( euint8 channel, eint32 value )
{
#ifdef EE101_DEBUG_ON
    EE101IntDisable; 

    SendEE101Byte( (channel & 0x07) | EE101_VALUE_TYPE | EE101_SYNC);
    SendEE101Byte( value >> 24);
    SendEE101Byte( value >> 16);
    SendEE101Byte( value >> 8);
    SendEE101Byte( value );

    EE101IntEnable;
#endif
};

void EE101Text( euint8 channel, echar *string )
{
#ifdef EE101_DEBUG_ON
    euint8 bytes = 1;

    EE101IntDisable; 
    
    SendEE101Byte( (channel&0x07) | EE101_SYNC | EE101_TEXT_TYPE);
    while(*string)
    {
        if (bytes++ > MAX_STRING_LENGTH)
            break;
        SendEE101Byte( *string++ );
    }
    SendEE101Byte( 0 );

    EE101IntEnable;
#endif
};

void EE101TextLabel( euint8 channel, echar *string )
{
#ifdef EE101_DEBUG_ON
    static euint8 timeout[8] = {0,0,0,0,0,0,0,0};
    euint8 bytes = 1;

    channel &= 0x07;
    timeout[channel]++;
    if ( timeout[channel] != 1 ) return;
    EE101IntDisable; 
    SendEE101Byte( channel | EE101_SYNC | EE101_TEXT_TYPE | EE101_LABEL);
    while(*string)
    {
        if (bytes++ > MAX_STRING_LENGTH)
            break;
        SendEE101Byte( *string++ );
    }
    SendEE101Byte( 0 );
    EE101IntEnable;
#endif
};

void EE101ValueLabel( euint8 channel, echar *string )
{
#ifdef EE101_DEBUG_ON
    static euint8 timeout[8] = {0,0,0,0,0,0,0,0};
    euint8 bytes = 1;

    channel &= 0x07;
    timeout[channel]++;
    if ( timeout[channel] != 1 ) return;
    EE101IntDisable; 
    SendEE101Byte( channel | EE101_SYNC | EE101_VALUE_TYPE | EE101_LABEL);
    while(*string)
    {
        if (bytes++ > MAX_STRING_LENGTH)
            break;
        SendEE101Byte( *string++ );
    }
    SendEE101Byte( 0 );
    EE101IntEnable; 
#endif
};
    
#else                   // Otherwise it is a 2-wire interface
void SendEE101Byte( euint8 value )
{
    if (value & 0x80) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
    if (value & 0x40) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
    if (value & 0x20) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
    if (value & 0x10) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
    if (value & 0x08) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
    if (value & 0x04) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
    if (value & 0x02) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
    if (value & 0x01) {EE101DataHigh;} else {EE101DataLow;} EE101ClockHigh; EE101ClockLow; 
} 

void EE101Value( euint8 channel, eint32 value )
{
#ifdef EE101_DEBUG_ON
    EE101IntDisable; 
    SendEE101Byte( (channel & 0x07) | EE101_VALUE_TYPE | EE101_SYNC);
    if ((value > 32767L) || (value < -32768L))
    {
        SendEE101Byte( value >> 24);
        SendEE101Byte( value >> 16);
    }
    if ((value > 127L) || (value < -128L))
        SendEE101Byte( value >> 8);
    SendEE101Byte( value );
    EE101DataToggle;EE101DataToggle;
    EE101IntEnable;
#endif
};

void EE101Text( euint8 channel, echar *string )
{
#ifdef EE101_DEBUG_ON
    euint8 bytes = 1;

    EE101IntDisable; 
    SendEE101Byte( (channel&0x07) | EE101_SYNC | EE101_TEXT_TYPE);
    while(*string)
    {
        if (bytes++ > MAX_STRING_LENGTH)
            break;
        SendEE101Byte( *string++ );
    }
    EE101DataToggle;EE101DataToggle;
    EE101IntEnable;
#endif
};

void EE101TextLabel( euint8 channel, echar *string )
{
#ifdef EE101_DEBUG_ON
    static euint8 timeout[8] = {0,0,0,0,0,0,0,0};
    euint8 bytes = 1;

    channel &= 0x07;
    timeout[channel]++;
    if ( timeout[channel] != 1 ) return;
    EE101IntDisable; 
    SendEE101Byte( channel | EE101_SYNC | EE101_TEXT_TYPE | EE101_LABEL);
    while(*string)
    {
        if (bytes++ > MAX_STRING_LENGTH)
            break;
        SendEE101Byte( *string++ );
    }
    EE101DataToggle;EE101DataToggle;
    EE101IntEnable;
#endif
};

void EE101ValueLabel( euint8 channel, echar *string )
{
#ifdef EE101_DEBUG_ON
    static euint8 timeout[8] = {0,0,0,0,0,0,0,0};
    euint8 bytes = 1;

    channel &= 0x07;
    timeout[channel]++;
    if ( timeout[channel] != 1 ) return;
    EE101IntDisable; 
    SendEE101Byte( channel | EE101_SYNC | EE101_VALUE_TYPE | EE101_LABEL);
    while(*string)
    {
        if (bytes++ > MAX_STRING_LENGTH)
            break;
        SendEE101Byte( *string++ );
    }
    EE101DataToggle;EE101DataToggle;
    EE101IntEnable;
#endif
};

#endif


#ifdef VARIABLE_ARGUMENT_SUPPORT 
echar EE101str[MAX_STRING_LENGTH];
void EE101printf( euint8 channel, echar *format, ... )
{
#ifdef EE101_DEBUG_ON
    va_list arglist;
    va_start( arglist, format );
    vsprintf( EE101str, format, arglist );
    va_end( arglist );
    EE101Text( channel, EE101str );
#endif
};
#endif
