/*
    Alternative firmware for STC15W104 processor + SYN115 radio transmitter door/window reed sensor(s)
    (see README)
 */
#include "project-defs.h"

#include <delay.h>
#include <eeprom-hal.h>
//#include <gpio-hal.h>
//#include <power-hal.h>
//#include <timer-hal.h>

#ifndef MCU_HAS_WAKE_UP_TIMER
    // Shouldn't happen, unless using an STC12.
    #error "The selected MCU doesn't have a power-down wake-up timer."
#endif // MCU_HAS_WAKE_UP_TIMER


// hardware pin definitions
// circuit is voltage divider and high side transistor with processor controlling divider
#define RADIO_VDD      P3_0
// LED is attached to P3.1 for door sensor RF 433 MHz (STC15W101 and STC15W104 processor, SYN115 RF transmitter)
#define LED_PIN        P3_1
#define REED_SWITCH    P3_2
#define TAMPER_SWITCH  P3_3
// ASK modulation to RF chip
#define RADIO_ASK      P3_4
// on my board when variable power supply (stand in for battery)
// reaches just above one volt from starting point of 1.5 volts
//this signal line goes from high (3.3V) to low (0V)
#define BATTERY_DETECT P3_5


// radio protocol requires sending packet twice so it is accepted at receiver
#define REPEAT_TIMES   2

// milliseconds
#define RADIO_STARTUP_TIME 120

// milliseconds
// maximum is 32768 as per sec. 7.8 power down wake-up special timer
// highest bit of register will be set to enable wake up timer
// maximum is 0x7FFF and then highest bit set

// about 16 seconds
#define SLEEP_TIME_0 32766


// milliseconds
#define LED_HIGH_TIME 750
#define LED_LOW_TIME  100

// milliseconds
#define CONTROLLER_STARTUP_TIME 200

// array size
#define SWITCH_HISTORY_SIZE 8

// unique ID location in FLASH
//4K MCU(eg. STC15F404AD, STC15F204EA, STC15F104EA)
#define ID_ADDR_ROM 0x0FF9

// unique ID location in RAM
//#define GUID_ADDR_RAM 0xF1

// placeholder bytes (0x00) are filled with global unique identification number to derive radio messages as original firmware did
// see sec. 1.12 global unique identification number STC15-English.pdf
// last byte is a code indicating state (open, closed, tamper) as used on original firmware
static unsigned char guid0 = 0x00;
static unsigned char guid1 = 0x00;

// codes used in original firmware
// (we leave upper bits available for possibly adding transmission count in future)
static const unsigned char reed_open    = 0x0A;
static const unsigned char reed_close   = 0x0E;
// low voltage code reported by https://community.home-assistant.io/t/budget-priced-wife-friendly-wireless-wall-switch-using-433mhz-and-mqtt/88281
// if sensor is being powered by header pins this will also show low apparently
static const unsigned char battery_low  = 0x06;
static const unsigned char tamper_open  = 0x07;
// added close to support tamper closed
// note: if we resend only a generic tamper code (e.g., 0x07) too quickly due to any switch change (e.g., a press and then a release)
//       I think the duplicate code sent may be discarded at receiver
static const unsigned char tamper_close = 0x08;


// rc-switch project timings (https://github.com/sui77/rc-switch)
// FIXME: Other protocols are not working on Sonoff Bridge w/ Tasmota so need to investigate
// static const struct Protocol protocols[] = {
  // { 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1
  // { 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
  // { 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
  // { 380, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
  // { 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
  // { 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 6 (HT6P20B)
  // { 150, {  2, 62 }, {  1,  6 }, {  6,  1 }, false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
  // { 200, {  3, 130}, {  7, 16 }, {  3,  16}, false},     // protocol 8 Conrad RS-200 RX
  // { 200, { 130, 7 }, {  16, 7 }, { 16,  3 }, true},      // protocol 9 Conrad RS-200 TX
  // { 365, { 18,  1 }, {  3,  1 }, {  1,  3 }, true },     // protocol 10 (1ByOne Doorbell)
  // { 270, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 11 (HT12E)
  // { 320, { 36,  1 }, {  1,  2 }, {  2,  1 }, true }      // protocol 12 (SM5212)
// };

// changed pulse lengths from microseconds to 10 microseconds units
// because hardware abstraction layer provides delay10us() 
// I avoid performing multiplies on processor by computing these as constants
const uint16_t gPulseHigh =   35;
const uint16_t gPulseLow  = 1085;
const uint16_t gZeroHigh  =   35;
const uint16_t gZeroLow   =  105;
const uint16_t gOneHigh   =  105;
const uint16_t gOneLow    =   35;

// it saves code space to just specify a single protocol and use define here
// uncomment only one line immediately below
#define PROTOCOL_INVERTED false
//#define PROTOCOL_INVERTED true


// save switch states checked in interrupts for use in main loop
struct Flags {
    volatile bool reedInterrupted;
    volatile bool tamperInterrupted;
    volatile bool tamperTripped;
    volatile bool reedIsOpen[SWITCH_HISTORY_SIZE];
    volatile bool tamperIsOpen[SWITCH_HISTORY_SIZE];
    volatile unsigned char tamperCount;
    volatile unsigned char reedCount;
};

// isOpen arrays are not initialized
// should be alright because we only read when count incremented
struct Flags flag = {
    .reedInterrupted   = false, 
    .tamperInterrupted = false,
    .tamperTripped     = false,
    .tamperCount = 0, 
    .reedCount   = 0
};

struct Settings {
    unsigned char eepromWritten;
    unsigned char protocol;
    uint16_t sleepTime;
    bool tamperHeartbeatEnabled;
    bool reedHeartbeatEnabled;
    bool tamperTripEnabled;
};

// default is to pick least frequent wake up time and disable radio heartbeats to save power
// convention with eeprom is often that uninitialized is 0xff
struct Settings setting = {
    .eepromWritten = 0xFF, 
    .protocol = 0, 
    .sleepTime = SLEEP_TIME_0, 
    .tamperHeartbeatEnabled = false, 
    .reedHeartbeatEnabled   = false,
    .tamperTripEnabled      = true
};


// only enable power to radio when we are going to modulate ASK pin (i.e., send data)
// low pin setting enables transistor which supplies power to radio chip
inline void enable_radio_vdd(void)
{
    RADIO_VDD = 0;
}

// pin setting functions are more readable than direct pin setting
// and avoid making errors (e.g., "enabling" something is actually setting pin equal zero)
inline void disable_radio_vdd(void)
{
    RADIO_VDD = 1;
}

// are these functions inlined by compiler automatically?
// apparently note, so specify inline
inline void radio_ask_high()
{
    #if PROTOCOL_INVERTED
        RADIO_ASK = 0;
    #else
        RADIO_ASK = 1;
    #endif
}

inline void radio_ask_low()
{
    #if PROTOCOL_INVERTED
        RADIO_ASK = 1;
    #else
        RADIO_ASK = 0;
    #endif
}

// led is controlled by transistor which essentially inverts pin output
// (so low level enables transistor and then LED is on)
inline void led_on(void)
{
    LED_PIN = 0;
}

inline void led_off(void)
{
    LED_PIN = 1;
}

// using pullup on mcu pin is apparently needed provide high level to tamper switch
inline void enable_tamper_pullup(void)
{
    TAMPER_SWITCH = 1;
}

inline void enable_ext0(void)
{
    // clear so that interrrupt triggers on falling and rising edges (should be default)
    IT0 = 0;
    
    // enable external interrupt 0
    IE1 |= M_EX0;
}

inline void enable_ext1(void)
{
    // set default such that external interrupt is triggered on falling and rising edges
    IT1 = 0;
    
    // enable external interrupt 0
    IE1 |= M_EX1;
}

inline void enable_global_interrupts(void)
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

/*! \brief Purpose of pulsing is to avoid leaving LED on because simultaneously powering other radio pins may be exceeding port sink/source capability
 *         Brief description continued.
 *
 */
void pulseLED(void)
{
    led_on();
    delay1ms(LED_HIGH_TIME);
    
    led_off();
    delay1ms(LED_LOW_TIME);
}

/*! \brief Pull up to pin must be enabled to read tamper switch.
 *         Need to measure current usage to compare to using interrupts.
 *
 */        
inline bool isTamperOpen(void)
{
    volatile bool pinState;
    
    pinState = TAMPER_SWITCH;
    
    return pinState;
}

inline bool isReedOpen(void)
{
    volatile bool pinState;
    
    pinState = REED_SWITCH;
    
    return pinState;
}

inline bool isBatteryLow(void)
{
    volatile bool pinState;
    
    pinState = BATTERY_DETECT;
    
    return !pinState;
}

void rfsyncPulse()
{
    // rf sync pulse
    radio_ask_high();
    delay10us_wrapper(gPulseHigh);
    
    radio_ask_low();
    delay10us_wrapper(gPulseLow);
}

/*! \brief Description
 *         Tips [http://ww1.microchip.com/downloads/en/AppNotes/Atmel-9164-Manchester-Coding-Basics_Application-Note.pdf]
 *
 */  
void send(const unsigned char byte)
{
    unsigned char i = 0;
    const unsigned char numBits = 8;
    
    // mask out highest bit
    const unsigned char mask = 1 << (numBits - 1);
    
    // byte for shifting
    unsigned char toSend = byte;
    
    // Repeat until all bits sent
    for(i = 0; i < numBits; i++)
    {
        // check left most bit value, and if equal to one then send one level
        if((toSend & mask) == mask)
        {
            radio_ask_high();
            delay10us_wrapper(gOneHigh);
            
            radio_ask_low();
            delay10us_wrapper(gOneLow);
        }
        else
        {
            radio_ask_high();
            delay10us_wrapper(gZeroHigh);
            
            radio_ask_low();
            delay10us_wrapper(gZeroLow);
        }
        
        toSend = toSend << 1;
    }
}

void sendRadioPacket(const unsigned char rfcode)
{
    unsigned char index;
    
    enable_radio_vdd();
    
    // sec. 9, table datasheet output blanking VDD transition from low to high (500 microseconds)
    // oscillator startup time crystal HC49S (300 microseconds)
    // standby delay time (min, 30), typical(75), max(120) milliseconds
    delay1ms(RADIO_STARTUP_TIME);
    
    // sonoff or tasmota or espurna seems to require sending twice to accept receipt
    for (index = 0; index < REPEAT_TIMES; index++)
    {
        rfsyncPulse();

        // send rf key
        send(guid0);
        send(guid1);
        send(rfcode);
    }
    
    disable_radio_vdd();
}

//FIXME: handle reentrancy?
//-----------------------------------------
void external_isr0(void) __interrupt 0
{
    if (flag.reedCount < SWITCH_HISTORY_SIZE)
    {
        flag.reedIsOpen[flag.reedCount] = REED_SWITCH;
        
        flag.reedCount++;
    }
}

//-----------------------------------------
void external_isr1(void) __interrupt 2
{
    const bool tamperState = TAMPER_SWITCH;
    
    if (flag.tamperCount < SWITCH_HISTORY_SIZE)
    {
        flag.tamperIsOpen[flag.tamperCount] = tamperState;
        
        if (setting.tamperTripEnabled)
        {
            if (tamperState)
            {
                flag.tamperTripped = true;
            }
        }
    
        flag.tamperCount++;
    }
}

//TODO: push pull mode uses lots of current, so need to see if it is avoidable
//batteryMonitor GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN5, GPIO_HIGH_IMPEDANCE_MODE);
//radioASK       GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN4, GPIO_BIDIRECTIONAL_MODE);
//tamperSwitch   GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN3, GPIO_PUSH_PULL_MODE);
//reedSwitch     GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN2, GPIO_HIGH_IMPEDANCE_MODE);
//ledPin         GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN1, GPIO_OPEN_DRAIN_MODE);
//radioVDD       GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN0, GPIO_OPEN_DRAIN_MODE);
inline void configure_pin_modes(void)
{
    P3M1 |= 0x20;
    P3M1 &= ~0x10;
    P3M1 &= ~0x08;
    P3M1 |= 0x04;
    P3M1 |= 0x02;
    P3M1 |= 0x01;
    
    P3M0 &= ~0x20;
    P3M0 &= ~0x10;
    P3M0 |= 0x08;
    P3M0 &= ~0x04;
    P3M0 |= 0x02;
    P3M0 |= 0x01;
}

void main(void)
{
    // as per HAL instructions
    INIT_EXTENDED_SFR();
    
    // allow possibility to flash multiple pulses
    unsigned char ledPulseCount = 0;
    
    // code pointer for reading microcontroller unique id
    __code unsigned char  *cptr;


    // setup gpio, strong pull up, input only, etc.
    configure_pin_modes();
    
    // double pulse LED at startup
    pulseLED();
    pulseLED();
    
    // disable power to radio for power saving and to disallow any transmission
    disable_radio_vdd();
    
    // datasheet warns against applying (high) pulses to ASK pin while power is disabled
    radio_ask_low();
    
    // provide a pull up voltage to the tamper switch
    enable_tamper_pullup();
    
    // give the microcontroller time to stabilize
    delay1ms(CONTROLLER_STARTUP_TIME);


    // copy unigue processor ID from flash to RAM
    // (sec. 1.12 global unique ID shows placement in ram, but addressing there does not seem to work)
    cptr = (__code unsigned char*) ID_ADDR_ROM;
    guid0   = *(cptr + 5);
    guid1   = *(cptr + 6);
    
    // does exactly what the function name says
    enable_global_interrupts();


    // enable tamper and reed interrupts
    enable_ext0();
    enable_ext1();

    // Main loop -------------------------------------------------------
    while (true)
    {
        // only enable wake up timer if any heartbeat is enabled
        if (setting.tamperHeartbeatEnabled || setting.reedHeartbeatEnabled || setting.tamperTripEnabled)
        {
            // FIXME: why does setting to 0x7FFF wake up every two seconds?
            // set wake up count
            WKTC = setting.sleepTime;
            
            // enable wake up timer
            WKTC |= 0x8000;
        }
        
        // do not go to sleep if unsent radio packets are available
        if ((flag.reedCount == 0) && (flag.tamperCount == 0))
        {
            // this will either wake up in the future due to timer (if enabled) or due to interrupt
            // datasheet and example HAL show providing nops() after power down
            PCON |= M_PD;
            NOP();
            NOP();
            
            // FIXME: need to disable wake up timer?
            WKTC &= ~0x8000;
        }
        

        // force sending out periodic messages to indicate tamper state
        if (setting.tamperHeartbeatEnabled)
        {
            // send different keys depending on reading push button
            if (isTamperOpen())
            {            
                sendRadioPacket(tamper_open);
            } else {
                sendRadioPacket(tamper_close);
            }
            
            // single pulse LED
            ledPulseCount = 1;
        }
        
        // similar period message but for reed switch
        if (setting.reedHeartbeatEnabled)
        {
            if(isReedOpen())
            {
                sendRadioPacket(reed_open);
            } else {
                sendRadioPacket(reed_close);
            }
            
            // notice that we do not increment count
            // so in effect, multiple radio packets could be sent and we just pulse once
            // and that is okay so that we save battery and avoid long unneeded delays
            ledPulseCount = 1;
        }
        
        // if trip setting enabled and one open tamper interrupt has occurred
        // keep sending out tamper radio message until tamper closes
        if (setting.tamperTripEnabled)
        {
            if (flag.tamperTripped)
            {
                sendRadioPacket(tamper_open);
                
                if (!isTamperOpen())
                {
                    flag.tamperTripped = false;
                }
            }
        }
        
        // FIXME: consider making interrupt based
        // FIXME: consider distinguising between battery going from low to high versus high to low
        // whenever microcontroller is awake just check for low battery
        if (isBatteryLow())
        {
            sendRadioPacket(battery_low);
            
            ledPulseCount = 1;
        }

        // send reed switch state if count is incremented by interrupt
        while (flag.reedCount > 0)
        {
            // track count because we might have multiple reed events in quick succession
            flag.reedCount--;
            
            if(flag.reedIsOpen[flag.reedCount])
            {
                sendRadioPacket(reed_open);
            } else {
                sendRadioPacket(reed_close);
            }
            
            // single pulse LED
            ledPulseCount = 1;
        }
 

        // difficult to capture tamper press releases, so need to track count to avoid misses
        while (flag.tamperCount > 0)
        {
            //
            flag.tamperCount--;
            
            if(flag.tamperIsOpen[flag.tamperCount])
            {
                sendRadioPacket(tamper_open);
            } else {
                sendRadioPacket(tamper_close);
            }
            
            // single pulse LED
            ledPulseCount = 1;
            

        }
        
        
        // finally blink LED here after sending out radio packets
        while (ledPulseCount > 0)
        {
            pulseLED();
            
            // decrementing (instead of zeroing) allows the potential
            // to pulse LED multiple times but usually we just pulse once
            ledPulseCount--;
        }
        

    }
}
