/*
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 * Copyright (c) 2022 Vincent DEFERT. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "project-defs.h"

#include "coding.h"
//#include "swuart.h"

#include <delay.h>
#include <gpio-hal.h>
#include <power-hal.h>

#ifndef MCU_HAS_WAKE_UP_TIMER
    // Shouldn't happen, unless using an STC12.
    #error "The selected MCU doesn't have a power-down wake-up timer."
#endif // MCU_HAS_WAKE_UP_TIMER

#define RFCODE_LENGTH 3
#define BITS_PER_PIECE 8

// milliseconds
#define STARTUP_TIME 120

// FIXME: read microcontroller UID to derive keys as original firmware did
const unsigned char open[RFCODE_LENGTH]   = {0x16, 0xE0, 0x0A};
const unsigned char closed[RFCODE_LENGTH] = {0x16, 0xE0, 0x0E};
const unsigned char tamper[RFCODE_LENGTH] = {0x16, 0xE0, 0x07};

// timings taken from RCSwitch project
// (no apparent need to match original firmware timings)
struct Protocol protocols[1] = {{.sync_high = 35, .sync_low_ms = 10, .sync_low_us = 85, .low = 35, .high = 105, .id = 0}};


// LED is attached to P3.1 for door sensor RF 433 MHz (STC15W101 and STC15W104 processor, SYN115 RF transmitter)
static GpioConfig ledPin = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN1, GPIO_OPEN_DRAIN_MODE);

// circuit is voltage divider and high side transistor with processor controlling divider
static GpioConfig rfVDD  = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN0, GPIO_OPEN_DRAIN_MODE);

// ASK modulation to RF chip
static GpioConfig rfASK  = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN4, GPIO_BIDIRECTIONAL_MODE);

static GpioConfig reedSwitch  = GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN2, GPIO_HIGH_IMPEDANCE_MODE);


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


void main()
{
    INIT_EXTENDED_SFR();
    
    volatile unsigned char byteCount;
    volatile unsigned char repeatKey;
    
    unsigned char* rfcode = &open[0];

    
    gpioConfigure(&ledPin);
    gpioConfigure(&rfVDD);
    gpioConfigure(&rfASK);
    
    // turn off LED
    gpioWrite(&ledPin, 1);
    
    // disable VDD to RF chip
    gpioWrite(&rfVDD, 1);
    
    // set ASK modulation pin to zero level
    gpioWrite(&rfASK, 0);

    
    enablePowerDownWakeUpTimer(millisecondsToWakeUpCount(5000));

    // Main loop -------------------------------------------------------
    while (1) {
        //configureUnusedGpioPins(GPIO_PORT3, M_ALL_PINS);
        //configureUnusedGpioPins(GPIO_PORT5, 0xef);
        //enterPowerDownMode();
        //gpioWrite(&ledPin, !gpioRead(&ledPin));
        
        enterPowerDownMode();
        delay1ms(5000);
        
        // toggle LED
        gpioToggle(&ledPin);
        
        if (gpioRead(&reedSwitch) == 0)
        {
            // enable VDD to rf chip
            gpioWrite(&rfVDD, 0);
            
            // sec. 9, table datasheet output blanking VDD transition from low to high (500 microseconds)
            // oscillator startup time crystal HC49S (300 microseconds)
            // standby delay time (min, 30), typical(75), max(120) milliseconds
            delay1ms(STARTUP_TIME);
            
            for (repeatKey = 0; repeatKey < 2; repeatKey++)
            {
                rfsyncPulse(&protocols[0], &rfASK);

                // FIXME: I do not think size of array can be obtained from pointer
                // FIXME: therefore this should not be working yet gives impression it's working
                for (byteCount = 0; byteCount < RFCODE_LENGTH; byteCount++)
                {
                    // send rf key
                    send(&protocols[0], rfcode[byteCount], BITS_PER_PIECE, &rfASK);
                }
                
            }
            
            // disable VDD to rf chip
            gpioWrite(&rfVDD, 1);
        }

    }
}
