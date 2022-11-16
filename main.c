/*
    Alternative firmware for STC15W104 processor + SYN115 radio transmitter door/window reed sensor(s)
    (see README)
 */
#include "project-defs.h"

#include "uart_software.h"

// NOTE: avoid gpio-hal to save RAM, and instead support software UART
#include <delay.h>
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
#define RADIO_STARTUP_TIME 120

// milliseconds
#define SLEEP_TIME 5000

// milliseconds
#define LED_HIGH_TIME 750
#define LED_LOW_TIME  100

// milliseconds
#define CONTROLLER_STARTUP_TIME 200

// unique ID location in RAM
#define GUID_ADDR_RAM 0xF1

// unique ID location in FLASH
//4K MCU(eg. STC15F404AD, STC15F204EA, STC15F104EA)
#define ID_ADDR_ROM 0x0FF9


// placeholder bytes (0x00) are filled with global unique identification number to derive radio messages as original firmware did
// see sec. 1.12 global unique identification number STC15-English.pdf
// last byte is a code indicating state (open, closed, tamper) as used on original firmware
static unsigned char open[RFCODE_LENGTH]         = {0x00, 0x00, 0x0A};
static unsigned char closed[RFCODE_LENGTH]       = {0x00, 0x00, 0x0E};
static unsigned char tamper[RFCODE_LENGTH]       = {0x00, 0x00, 0x07};
static const unsigned char debug[RFCODE_LENGTH]  = {0x55, 0xAA, 0x55};


// timings taken from rc-switch project
// (concluded here there is no need to match original firmware timings)
// https://github.com/sui77/rc-switch

/**
 * Description of a single pulse, which consists of a high signal
 * whose duration is "high" times the base pulse length, followed
 * by a low signal lasting "low" times the base pulse length.
 * Thus, the pulse overall lasts (high+low)*pulseLength
 */
struct HighLow {
    uint8_t high;
    uint8_t low;
};

/**
 * A "protocol" describes how zero and one bits are encoded into high/low
 * pulses.
 */
struct Protocol {
    /** base pulse length in microseconds, e.g. 350 */
    uint16_t pulseLength;

    struct HighLow syncFactor;
    struct HighLow zero;
    struct HighLow one;

    bool invertedSignal;
};

// FIXME: I do not think other protocols are working with Sonoff Bridge w/ Tasmota so need to investigate
// changed pulse lengths given in rc-switch project from microseconds to 10 microseconds units
// because available delay function is delay10us() with hardware abstraction layer
static const struct Protocol protocols[] = {

  { 35, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1
  { 65, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
  { 10, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
  { 38, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
  { 50, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
  { 45, { 23,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 6 (HT6P20B)
  { 15, {  2, 62 }, {  1,  6 }, {  6,  1 }, false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
  { 20, {  3, 130}, {  7, 16 }, {  3,  16}, false},     // protocol 8 Conrad RS-200 RX
  { 20, { 130, 7 }, {  16, 7 }, { 16,  3 }, true},      // protocol 9 Conrad RS-200 TX
  { 36, { 18,  1 }, {  3,  1 }, {  1,  3 }, true },     // protocol 10 (1ByOne Doorbell)
  { 27, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 11 (HT12E)
  { 32, { 36,  1 }, {  1,  2 }, {  2,  1 }, true }      // protocol 12 (SM5212)
};

enum {
   numProto = sizeof(protocols) / sizeof(protocols[0])
};

// circuit is voltage divider and high side transistor with processor controlling divider
#define RADIO_VDD     P3_0

// LED is attached to P3.1 for door sensor RF 433 MHz (STC15W101 and STC15W104 processor, SYN115 RF transmitter)
#define LED_PIN       P3_1
#define REED_SWITCH   P3_2
#define TAMPER_SWITCH P3_3

// ASK modulation to RF chip
#define RADIO_ASK     P3_4

// save switch states in interrupts for use in main loop
struct Flags {
    volatile bool reedInterrupted;
    volatile bool tamperInterrupted;
    volatile bool reedIsOpen;
    volatile bool tamperIsOpen;
    volatile unsigned char tamperIntCount;
    volatile unsigned char reedIntCount;
};

struct Flags flag = {.reedInterrupted = false, .tamperInterrupted = false, .reedIsOpen = false, .tamperIsOpen = false, .tamperIntCount = 0, .reedIntCount = 0};


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
    // disable this interrupt
    IE1 &= ~M_EX0;
    
    flag.reedIsOpen = REED_SWITCH;
    
    flag.reedIntCount++;
}

//-----------------------------------------
void external_isr1(void) __interrupt 2
{
    // disable this interrupt
    IE1 &= ~M_EX1;
    
    flag.tamperIsOpen = TAMPER_SWITCH;
    
    flag.tamperIntCount++;
}

// only enable power to radio when we are going to modulate ASK pin (i.e., send data)
void enable_radio_vdd(void)
{
    RADIO_VDD = 0;
}

// pin setting functions are more readable than direct pin setting
// and avoid making errors (e.g., "enabling" something is actually setting pin equal zero)
void disable_radio_vdd(void)
{
    RADIO_VDD = 1;
}

// TODO: are these functions inlined by compiler automatically?
void radio_ask_high(void)
{
    RADIO_ASK = 1;
}

void radio_ask_low(void)
{
    RADIO_ASK = 0;
}

// led is controlled by transistor which essentially inverts pin output
// (so low level turns transistor and then LED on)
void led_on(void)
{
    LED_PIN = 0;
}

void led_off(void)
{
    LED_PIN = 1;
}

void enable_ext0(void)
{
    // clear so that interrrupt triggers on falling and rising edges (should be default)
    IT0 = 0;
    
    // enable external interrupt 0
    IE1 |= M_EX0;
}

void enable_ext1(void)
{
    // set default such that external interrupt is triggered on falling and rising edges
    IT1 = 0;
    
    // enable external interrupt 0
    IE1 |= M_EX1;
}

void enable_global_interrupts(void)
{
    // enable global interrupts
    EA = 1;
}

// allows long delays with 10 microsecond function at the expense of accuracy
void delay10us_wrapper(unsigned int microseconds)
{
    const unsigned char step = 0xFF;
    
    while (microseconds > step)
    {
        delay10us(step);
        microseconds -= step;
    }
    
    delay10us(microseconds);
}

/*! \brief Purpose is to not leave LED on because powering other radio pins may be exceeding port sink/source capability
 *         Brief description continued.
 *
 */
void pulseLED(unsigned char repeat)
{
    unsigned char i;
    
    for (i = 0; i < repeat; i++)
    {
        led_on();
        delay1ms(LED_HIGH_TIME);
        
        led_off();
        delay1ms(LED_LOW_TIME);
    }
}

/*! \brief Pull up to pin must be enabled to read tamper switch.
 *         Need to measure current usage to compare to using interrupts.
 *
 */        
bool isTamperOpen(void)
{
    volatile bool pinState;
    
    pinState = TAMPER_SWITCH;
    
    return pinState;
}

bool isReedOpen(void)
{
    volatile bool pinState;
    
    pinState = REED_SWITCH;
    
    return pinState;
}

void rfsyncPulse(struct Protocol* protocol)
{
    // rf sync pulse
    radio_ask_high();
    delay10us_wrapper(protocol->pulseLength * protocol->syncFactor.high);
    
    radio_ask_low();
    delay10us_wrapper(protocol->pulseLength * protocol->syncFactor.low);
}

/*! \brief Description
 *         Brief description continued.
 *
 */  
void send(struct Protocol* protocol, const unsigned char byte)
{
    // set as volatile so it does not get optimized out
    // because compiler does not understand we are shifting out of hardware pin
    volatile unsigned int i;
    const unsigned char numBits = 8;
    
    // byte for shifting
    volatile unsigned char toSend = byte;

    // Repeat until all bits sent
    for(i = 0; i < numBits; i++)
    {
        // FIXME: account for numBits left shift
        // Check bit value, process logic one
        if((toSend & 0x80) == 0x80)
        {
            radio_ask_high();
            delay10us_wrapper(protocol->pulseLength * protocol->one.high);
            
            radio_ask_low();
            delay10us_wrapper(protocol->pulseLength * protocol->one.low);
        }
        else
        {
            radio_ask_high();
            delay10us_wrapper(protocol->pulseLength * protocol->zero.high);
            
            radio_ask_low();
            delay10us_wrapper(protocol->pulseLength * protocol->zero.low);

        }
        
        toSend = toSend << 1;
    }
}

void sendRadioPacket(unsigned char* rfcode, unsigned char protocol)
{
    unsigned char byteCount;
    unsigned char repeatCount;
    
    enable_radio_vdd();
    
    // sec. 9, table datasheet output blanking VDD transition from low to high (500 microseconds)
    // oscillator startup time crystal HC49S (300 microseconds)
    // standby delay time (min, 30), typical(75), max(120) milliseconds
    delay1ms(RADIO_STARTUP_TIME);
    
    for (repeatCount = 0; repeatCount < REPEAT_TIMES; repeatCount++)
    {
        rfsyncPulse(&protocols[protocol]);

        for (byteCount = 0; byteCount < RFCODE_LENGTH; byteCount++)
        {
            // send rf key
            send(&protocols[protocol], rfcode[byteCount]);
        }
        
    }
    
    disable_radio_vdd();
}


//TODO: push pull mode uses lots of current, so need to see if it is avoidable
//radioASK     GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN4, GPIO_BIDIRECTIONAL_MODE);
//tamperSwitch GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN3, GPIO_PUSH_PULL_MODE);
//reedSwitch   GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN2, GPIO_HIGH_IMPEDANCE_MODE);
//ledPin       GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN1, GPIO_OPEN_DRAIN_MODE);
//radioVDD     GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN0, GPIO_OPEN_DRAIN_MODE);
void configure_pin_modes(void)
{
    P3M1 &= ~0x10;
    P3M1 &= ~0x08;
    P3M1 |= 0x04;
    P3M1 |= 0x02;
    P3M1 |= 0x01;
    
    P3M0 &= ~0x10;
    P3M0 |= 0x08;
    P3M0 &= ~0x04;
    P3M0 |= 0x02;
    P3M0 |= 0x01;
}

void main()
{
    INIT_EXTENDED_SFR();
    
    // TODO: might allow human to control these later (with tamper switch presses?)
    volatile bool heartbeatForTamper = false;
    const volatile bool heartbeatForReed   = false;
    

    const unsigned char protocolIndex = 0;
    unsigned char rxByte;
    
    // code pointer for reading microcontroller unique id
    __code unsigned char  *cptr;
    
    
    configure_pin_modes();
    uart_init();


    // double pulse LED at startup
    pulseLED(2);
    
    // disable power to radio for power saving
    // and to disallow any transmission
    disable_radio_vdd();
    
    // datasheet warns against applying pulses to ASK pin while power is disabled
    radio_ask_low();
    
    // provide a pull up voltage to the tamper switch
    TAMPER_SWITCH = 1;
    
    // give the microcontroller time to stabilize
    delay1ms(CONTROLLER_STARTUP_TIME);


    // copy unigue ID to radio codes
    cptr = (__code unsigned char*) ID_ADDR_ROM;
    open[0]   = *(cptr + 5);
    closed[0] = *(cptr + 5);
    tamper[0] = *(cptr + 5);
    
    open[1]   = *(cptr + 6);
    closed[1] = *(cptr + 6);
    tamper[1] = *(cptr + 6);

    // enable interrupts
    enable_ext0();
    enable_ext1();
    enable_global_interrupts();

    // timer used for software serial UART
    enable_timer0();
    
    // demonstrate that software serial UART is working
    puts("Startup...");
    putc('\r');
    puts(__DATE__);
    putc('\r');
    puts(__TIME__);
    putc('\r');
    puts("Protocol: 0x");
    puthex2(protocolIndex);
    putc('\r');

    // Main loop -------------------------------------------------------
    while (1)
    {
        // only enable wake up timer if any heartbeat is enabled
        if (heartbeatForTamper || heartbeatForReed)
        {
            enablePowerDownWakeUpTimer(millisecondsToWakeUpCount(SLEEP_TIME));
        }
        
        

        // this will either wake up due to timer (if enabled) or interrupt
        enterPowerDownMode();

        // provided in software UART example
        // FIXME: implement receive with software serial UART
        
        
        // 
        // force sending out periodic messages if tamper open detected by interrupt routine
        if (heartbeatForTamper)
        {
            // keep checking if tamper is open, if not stop sending out periodic messages
            if (isTamperOpen())
            {            
                sendRadioPacket(&tamper[0], protocolIndex);
                
                // single pulse LED
                pulseLED(1);
                //putc('H');
            } else {
                heartbeatForTamper = false;
            }
        }
        

        // send reed switch state after count incremented by interrupt
        while (flag.reedIntCount > 0)
        {

            if(flag.reedIsOpen)
            {
                sendRadioPacket(&open[0], protocolIndex);
            } else {
                sendRadioPacket(&closed[0], protocolIndex);
            }
            
            // single pulse LED
            pulseLED(1);
            //putc('R');
            //puthex(flag.reedIntCount);
            
            //
            flag.reedIntCount--;
            
            // re-enable interrupts
            enable_ext0();
            
        }
 
        // FIXME: we may be missing push-release button presses if they are quick enough
        //        this does not necessarily matter if we just want to detect housing cover being opened
        //        but if we use button for user input switch debouncing etc. needs to be more robust
        while (flag.tamperIntCount > 0)
        {
            
            sendRadioPacket(&tamper[0], protocolIndex);
            
            // single pulse LED
            pulseLED(1);
            //putc('T');
            //puthex(flag.tamperIntCount);
            
            //
            flag.tamperIntCount--;
            
            // if tamper is open, begin waking up periodically and sending out tamper message
            if (flag.tamperIsOpen)
            {
                heartbeatForTamper = true;
            }
            
            // re-enable interrupts
            enable_ext1();
            
        }
    }
}
