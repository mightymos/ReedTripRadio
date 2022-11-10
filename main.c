/*
    Alternative firmware for STC15W104 processor + SYN115 radio transmitter door/window reed sensor(s)
    (see README)
 */
#include "project-defs.h"

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

// milliseconds
#define LED_HIGH_TIME 750
#define LED_LOW_TIME  100

// milliseconds
#define TAMPER_POWER_UP_DELAY 20

#define GUID_ADDR_RAM 0xF1
#define GUID_2ND_ADDR 0xF6
#define GUID_1ST_ADDR 0xF7

//4K MCU(eg. STC15F404AD, STC15F204EA,
//STC15F104EA)
#define ID_ADDR_ROM 0x0FF9
#define ID_2ND_BYTE 0x0FFE
#define ID_1ST_BYTE 0x0FFF


// read microcontroller global unique identification number to derive radio keys as original firmware did
// see sec. 1.12 global unique identification number STC15-English.pdf
// last byte is code indicating state (open, closed, tamper) used on original firmware
static unsigned char open[RFCODE_LENGTH]   = {0x00, 0x00, 0x0A};
static unsigned char closed[RFCODE_LENGTH] = {0x00, 0x00, 0x0E};
static unsigned char tamper[RFCODE_LENGTH] = {0x00, 0x00, 0x07};
static const unsigned char debug[RFCODE_LENGTH]  = {0x55, 0xAA, 0x55};

struct Protocol {
   unsigned int  sync_high;
   unsigned int  sync_low_ms;
   unsigned char sync_low_us;
   unsigned char low;
   unsigned char high;
   unsigned char id;
};  

// timings taken from RCSwitch project
// (author concluded there was no apparent need to match original firmware timings)
static const struct Protocol protocols[2] = {
    {.sync_high = 35, .sync_low_ms = 10, .sync_low_us = 85, .low = 35, .high = 105, .id = 0},
    {.sync_high = 65, .sync_low_ms =  6, .sync_low_us = 50, .low = 65, .high = 130, .id = 0}    
};


// LED is attached to P3.1 for door sensor RF 433 MHz (STC15W101 and STC15W104 processor, SYN115 RF transmitter)
static GpioConfig ledPin       = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN1, GPIO_OPEN_DRAIN_MODE);
static GpioConfig reedSwitch   = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN2, GPIO_HIGH_IMPEDANCE_MODE);
static GpioConfig tamperSwitch = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN3, GPIO_PUSH_PULL_MODE);

// WARNING: receive pin for in circuit programming shares this pin
// WANRING: and appears to require disconnecting from CP102x or similar usb to uart module to allow driving pin
// circuit is voltage divider and high side transistor with processor controlling divider
GpioConfig radioVDD = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN0, GPIO_OPEN_DRAIN_MODE);

// ASK modulation to RF chip
GpioConfig radioASK = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN4, GPIO_BIDIRECTIONAL_MODE);

// FIXME: tamper interrupt is not set up yet because it may be power inefficient to have pull up enabled all the time
struct Flags {
    volatile bool reedInterrupted;
    volatile bool tamperInterrupted;
};

struct Flags settings = {.reedInterrupted = false, .tamperInterrupted = false};


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
    settings.reedInterrupted = true;
}

//-----------------------------------------
void external_isr1(void) __interrupt 2
{
    settings.tamperInterrupted = true;
}

void enable_ext0(void)
{
    // set default such that external interrupt is triggered on falling and rising edges
    TCON |= IT0;
    
    // enable external interrupt 0
    IE1 |= M_EX0;
}

void enable_ext1(void)
{
    // set default such that external interrupt is triggered on falling and rising edges
    //TCON |= IT1;
    
    // enable external interrupt 0
    IE1 |= M_EX1;
}

void enable_global_interrupts(void)
{
    // enable global interrupts
    EA = 1;
}

/*! \brief Purpose is to not leave LED on because powering other radio pins may be exceeding port sink/source capability
 *         Brief description continued.
 *
 */
void pulseLED(void)
{
    gpioWrite(&ledPin, 0);
    delay1ms(LED_HIGH_TIME);
    gpioWrite(&ledPin, 1);
    delay1ms(LED_LOW_TIME);
}

void rfsyncPulse(struct Protocol* protocol)
{
    // rf sync pulse
    gpioWrite(&radioASK, 1);
    delay10us(protocol->sync_high);
    
    gpioWrite(&radioASK, 0);
    delay1ms(protocol->sync_low_ms);
    delay10us(protocol->sync_low_ms);
}

/*! \brief We apply power/pull up to read tamper switch momentarily, read switch, and finally disable pull up
 *         Need to measure current usage to compare to using interrupts.
 *
 */        
bool isTamperOpen(void)
{
    volatile bool tamperOpen = false;
    
    // enable strong pull up temporarily (pulled down by tamper switch)
    gpioWrite(&tamperSwitch, 1);
    delay1ms(TAMPER_POWER_UP_DELAY);
    
    // outside of housing switch is open, but while inside switch is closed
    if(gpioRead(&tamperSwitch))
    {
        tamperOpen = true;
    }
    
    // disable strong pullup to avoid exceeding port current capability/save power
    gpioWrite(&tamperSwitch, 0);
    
    return tamperOpen;
}

/*! \brief Description
 *         Brief description continued.
 *
 */  
void send(struct Protocol* protocol, const unsigned char byte, const unsigned int numBits)
{
    // set as volatile so it does not get optimized out
    // because compiler does not understand we are shifting out of hardware pin
    volatile unsigned int i;
    
    // byte for shifting
    volatile unsigned char toSend = byte;

    // Repeat until all bits sent
    for(i = 0; i < numBits; i++)
    {
        // FIXME: account for numBits left shift
        // Check bit value, process logic one
        if((toSend & 0x80) == 0x80)
        {
            gpioWrite(&radioASK, 1);
            delay10us(protocol->high);
            
            gpioWrite(&radioASK, 0);
            delay10us(protocol->low);
        }
        else
        {
            gpioWrite(&radioASK, 1);
            delay10us(protocol->low);
            
            gpioWrite(&radioASK, 0);
            delay10us(protocol->high);

        }
        
        toSend = toSend << 1;
    }
}

void sendRadioPacket(unsigned char* rfcode, unsigned char protocol)
{
    unsigned char byteCount;
    unsigned char repeatCount;
    
    // enable VDD to rf chip
    gpioWrite(&radioVDD, 0);
    
    // sec. 9, table datasheet output blanking VDD transition from low to high (500 microseconds)
    // oscillator startup time crystal HC49S (300 microseconds)
    // standby delay time (min, 30), typical(75), max(120) milliseconds
    delay1ms(STARTUP_TIME);
    
    for (repeatCount = 0; repeatCount < REPEAT_TIMES; repeatCount++)
    {
        rfsyncPulse(&protocols[protocol]);

        for (byteCount = 0; byteCount < RFCODE_LENGTH; byteCount++)
        {
            // send rf key
            send(&protocols[protocol], rfcode[byteCount], BITS_PER_PIECE);
        }
        
    }
    
    // disable VDD to rf chip
    gpioWrite(&radioVDD, 1);
}


void main()
{
    INIT_EXTENDED_SFR();
    
    // might allow human to control these later (with tamper switch presses?)
    const bool heartbeatForTamper = true;
    const bool heartbeatForReed   = false;
    
    // open/close assignment is arbitrary here, but probably better than an unassigned pointer
    unsigned char* rfcode = &open[0];

    unsigned char currentProtocol = 0;
    
    // code pointers for reading microcontroller unique id
    __idata unsigned char *iptr;
    __code unsigned char  *cptr;
    
    gpioConfigure(&radioVDD);
    gpioConfigure(&ledPin);
    gpioConfigure(&reedSwitch);
    gpioConfigure(&tamperSwitch);
    gpioConfigure(&radioASK);

        
    // double pulse LED at startup
    pulseLED();
    pulseLED();
    
    // disable VDD to RF chip
    gpioWrite(&radioVDD, 1);
    
    // set ASK modulation pin to zero level
    gpioWrite(&radioASK, 0);
    
    enable_ext0();
//    enable_ext1();
    enable_global_interrupts();
    
    // copy unigue ID to radio codes
    cptr = (__code unsigned char*) ID_2ND_BYTE;
    open[0]   = *cptr;
    closed[0] = *cptr;
    tamper[0] = *cptr;
    
    cptr = (__code unsigned char*) ID_1ST_BYTE;
    open[1]   = *cptr;
    closed[1] = *cptr;
    tamper[1] = *cptr;

    // Main loop -------------------------------------------------------
    while (1) {
        // FIXME: necessary?
        configureUnusedGpioPins(GPIO_PORT3, 0xE0);

        // only enable wake up timer if any heartbeat is enabled
        if (heartbeatForTamper || heartbeatForReed)
        {
            enablePowerDownWakeUpTimer(millisecondsToWakeUpCount(SLEEP_TIME));
        }
        
        // this will either wake up from timer (if enabled) or interrupt
        enterPowerDownMode();
        
        
        // force sending out periodic messages if tamper open
        if (heartbeatForTamper)
        {
            if (isTamperOpen())
            {
                pulseLED();
                pulseLED();
            
                sendRadioPacket(&tamper[0], currentProtocol);
            }
        }
        

        
        // send out periodic messages of reed switch state
        if (heartbeatForReed)
        {

            if(gpioRead(&reedSwitch))
            {
                rfcode = &open[0];
            } else {
                rfcode = &closed[0];
            }
            
            pulseLED();
            pulseLED();
            
            sendRadioPacket(&tamper[0], currentProtocol);
        }
        

        // send reed switch state on interrupt
        if (settings.reedInterrupted)
        {
            if(gpioRead(&reedSwitch))
            {
                rfcode = &open[0];
            } else {
                rfcode = &closed[0];
            }
            
            pulseLED();
            pulseLED();
            
            sendRadioPacket(rfcode, currentProtocol);
            
            // clear flag which should only be set in interrupt
            settings.reedInterrupted = false;
        }
 
    }
}
