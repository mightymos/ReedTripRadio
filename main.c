/*
    Alternative firmware for STC15W104 processor + SYN115 radio transmitter door/window reed sensor(s)
    (see README)
 */
#include "project-defs.h"

#include "coding.h"
#include "flash_dump.h"

#include <delay.h>
#include <gpio-hal.h>
#include <power-hal.h>
#include <timer-hal.h>

#ifndef MCU_HAS_WAKE_UP_TIMER
    // Shouldn't happen, unless using an STC12.
    #error "The selected MCU doesn't have a power-down wake-up timer."
#endif // MCU_HAS_WAKE_UP_TIMER

#define RFCODE_LENGTH  3
#define REPEAT_TIMES   2
#define BITS_PER_PIECE 8

// milliseconds
#define STARTUP_TIME 120

// milliseconds
#define SLEEP_TIME 5000

// FIXME: read microcontroller UID to derive keys as original firmware did
const unsigned char open[RFCODE_LENGTH]   = {0x16, 0xE0, 0x0A};
const unsigned char closed[RFCODE_LENGTH] = {0x16, 0xE0, 0x0E};
const unsigned char tamper[RFCODE_LENGTH] = {0x16, 0xE0, 0x07};

// timings taken from RCSwitch project
// (no apparent need to match original firmware timings)
const struct Protocol protocols[2] = {
    {.sync_high = 35, .sync_low_ms = 10, .sync_low_us = 85, .low = 35, .high = 105, .id = 0},
    {.sync_high = 65, .sync_low_ms =  6, .sync_low_us = 50, .low = 65, .high = 130, .id = 0}    
};


// circuit is voltage divider and high side transistor with processor controlling divider
static GpioConfig radioVDD     = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN0, GPIO_OPEN_DRAIN_MODE);

// LED is attached to P3.1 for door sensor RF 433 MHz (STC15W101 and STC15W104 processor, SYN115 RF transmitter)
static GpioConfig ledPin       = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN1, GPIO_OPEN_DRAIN_MODE);

static GpioConfig reedSwitch   = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN2, GPIO_HIGH_IMPEDANCE_MODE);

static GpioConfig tamperSwitch = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN3, GPIO_PUSH_PULL_MODE);

// ASK modulation to RF chip
static GpioConfig radioASK     = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN4, GPIO_BIDIRECTIONAL_MODE);



volatile bool reedInterrupted = false;


/*! \brief Brief description.
 *         Brief description continued.
 *
 * Needed?
 */
unsigned char _sdcc_external_startup(void) __nonbanked
{
    //disable_interrupts();
    //clear_watchdog();
    //enable_xram();
    
    return 0;
}




//-----------------------------------------
void external_isr0(void) __interrupt 0
{
    reedInterrupted = true;
    //isFalling = INT0;
}

void enable_ext0(void)
{
    // default is that external interrup is triggered on falling and rising edge
    // TCON at IT0
    
    // enable external interrupt 0
    IE1 |= M_EX0;
    
    // enable global interrupts
    IE1 |= EA;
}


void main()
{
    INIT_EXTENDED_SFR();
    
    bool heartbeatEnabled = true;
    
    volatile unsigned char byteCount;
    volatile unsigned char repeatCount;
    
    unsigned char* rfcode = &open[0];

    unsigned char currentProtocol = 0;
    
    gpioConfigure(&radioVDD);
    gpioConfigure(&ledPin);
    gpioConfigure(&reedSwitch);
    gpioConfigure(&tamperSwitch);
    gpioConfigure(&radioASK);

    
//    UART_INIT();
    
    
    // pulse LED
    // (note: do not leave LED on, powering other radio pins may be exceeding port sink/source capability)
    gpioWrite(&ledPin, 0);
    delay1ms(750);
    gpioWrite(&ledPin, 1);
    delay1ms(20);
    
    // disable VDD to RF chip
    gpioWrite(&radioVDD, 1);
    
    // set ASK modulation pin to zero level
    gpioWrite(&radioASK, 0);
    
    enable_ext0();
    
    enablePowerDownWakeUpTimer(millisecondsToWakeUpCount(SLEEP_TIME));

    // Main loop -------------------------------------------------------
    while (1) {
        // FIXME: necessary?
        configureUnusedGpioPins(GPIO_PORT3, 0xE0);

        // see wake up time set above
        enterPowerDownMode();
        
        
        // pulse LED
        // (note: do not leave LED on, powering other radio pins may be exceeding port sink/source capability)
        gpioWrite(&ledPin, 0);
        delay1ms(500);
        gpioWrite(&ledPin, 1);
        delay1ms(20);
        
        // send different codes based on reed switch state
        if(gpioRead(&reedSwitch))
        {
            rfcode = &open[0];
        } else {
            rfcode = &closed[0];
        }
        
        
        // enable strong pull up temporarily (pulled down by tamper switch)
        gpioWrite(&tamperSwitch, 1);
        delay1ms(20);
        
        // FIXME: outside of case switch is open (in development), when in housing switch is closed
        // tamper switch always has higher priority than reed switch, so check last
        if(!gpioRead(&tamperSwitch))
        {
            rfcode = &tamper[0];
        }
        
        // disable strong pullup to avoid exceeding port capability/save power
        gpioWrite(&tamperSwitch, 0);
        
        // FIXME: not working
        if (reedInterrupted)
        {
            // double pulse LED
            gpioWrite(&ledPin, 0);
            delay1ms(500);
            gpioWrite(&ledPin, 1);
            delay1ms(100);
            gpioWrite(&ledPin, 0);
            delay1ms(500);
            gpioWrite(&ledPin, 1);
            delay1ms(20);
            
            reedInterrupted = false;
        }
        
        // send out periodic messages regardless of state changing
        if (heartbeatEnabled)
        {            
            // enable VDD to rf chip
            gpioWrite(&radioVDD, 0);
            
            // sec. 9, table datasheet output blanking VDD transition from low to high (500 microseconds)
            // oscillator startup time crystal HC49S (300 microseconds)
            // standby delay time (min, 30), typical(75), max(120) milliseconds
            delay1ms(STARTUP_TIME);
            
            for (repeatCount = 0; repeatCount < REPEAT_TIMES; repeatCount++)
            {
                rfsyncPulse(&protocols[currentProtocol], &radioASK);

                for (byteCount = 0; byteCount < RFCODE_LENGTH; byteCount++)
                {
                    // send rf key
                    send(&protocols[currentProtocol], rfcode[byteCount], BITS_PER_PIECE, &radioASK);
                }
                
            }
            
            // disable VDD to rf chip
            gpioWrite(&radioVDD, 1);
            
        }
 
    }
}
