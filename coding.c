/*
-----------------------------------------------------------------------*/
#include "project-defs.h"
#include "coding.h"

#include <delay.h>
#include <gpio-hal.h>
// **********************************************************************


void rfsyncPulse(struct Protocol* protocol, GpioConfig* rfASK)
{
    // rf sync pulse
    //gpioWrite(rfASK, 1);
    P3_4 = 1;
    delay10us(protocol->sync_high);
    
    //gpioWrite(rfASK, 0);
    P3_4 = 0;
    delay1ms(protocol->sync_low_ms);
    delay10us(protocol->sync_low_ms);
}

// *********************************************************************/
void send(struct Protocol* protocol, const unsigned char byte, const unsigned int numBits, GpioConfig* rfASK)
{
    // set as volatile so it does not get optimized out
    // because compiler does not understand we are shifting out of hardware pin
    volatile unsigned int i;
    
    // byte for shifting
    volatile unsigned char toSend = byte;

    // Repeat until all bits sent
    for(i = 0; i < numBits; i++)
    {
        // Check bit value, process logic one
        if((toSend & 0x80) == 0x80)
        {
            gpioWrite(rfASK, 1);
            //P3_4 = 1;
            delay10us(protocol->high);
            gpioWrite(rfASK, 0);
            //P3_4 = 0;
            delay10us(protocol->low);
        }
        else
        {
            gpioWrite(rfASK, 1);
            //P3_4 = 1;
            delay10us(protocol->low);
            gpioWrite(rfASK, 0);
            //P3_4 = 0;
            delay10us(protocol->high);

        }
        
        toSend = toSend << 1;
    }
}