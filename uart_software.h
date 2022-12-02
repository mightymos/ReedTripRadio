#ifndef _SOFTWARE_UART_H
#define _SOFTWARE_UART_H

#include <STC/15W10x/DIP8.h>

// FIXME: make this a macro computed from MCU_FREQ in Makefile
//define baudrate const
//BAUD = 65536 - FOSC/3/BAUDRATE/M (1T:M=1; 12T:M=12)
//NOTE: (FOSC/3/BAUDRATE) must be greater than 98, (RECOMMEND GREATER THAN 110)
//#define BAUD 0xF400 // 1200bps @ 11.0592MHz
//#define BAUD 0xFA00 // 2400bps @ 11.0592MHz
//#define BAUD 0xFD00 // 4800bps @ 11.0592MHz
//#define BAUD 0xFE80 // 9600bps @ 11.0592MHz
//#define BAUD 0xFF40 //19200bps @ 11.0592MHz
//#define BAUD 0xFFA0 //38400bps @ 11.0592MHz
//#define BAUD 0xEC00 // 1200bps @ 18.432MHz
//#define BAUD 0xF600 // 2400bps @ 18.432MHz
//#define BAUD 0xFB00 // 4800bps @ 18.432MHz
//#define BAUD 0xFD80 // 9600bps @ 18.432MHz
//#define BAUD 0xFEC0 //19200bps @ 18.432MHz
//#define BAUD 0xFF60 //38400bps @ 18.432MHz
//#define BAUD 0xE800 // 1200bps @ 22.1184MHz
//#define BAUD 0xF400 // 2400bps @ 22.1184MHz
//#define BAUD 0xFA00 // 4800bps @ 22.1184MHz
//#define BAUD 0xFD00 // 9600bps @ 22.1184MHz
//#define BAUD 0xFE80 //19200bps @ 22.1184MHz
//#define BAUD 0xFF40 //38400bps @ 22.1184MHz
//#define BAUD 0xFF80 //57600bps @ 22.1184MHz

//#define BAUD 0xFCC0 //2400bps @ 5.99MHz
//#define BAUD 0xFD10 //2400bps @ 5.414MHz
//#define BAUD 0xFA18 //2400bps @ 10.886MHz
#define BAUD (65536-MCU_FREQ/3/2400/1)


// pin definitions
#define RXB P3_0
#define TXB P3_1
#define BTN P3_2

extern unsigned char TBUF,RBUF;
extern __bit TING,RING;
extern __bit TEND,REND;
extern unsigned char t, r;

// when indexing take care to apply wrap around equivalent to buffer size (e.g., buf[r & 0x0F] for size 16)
extern unsigned char buf[8];


//-----------------------------------------
//Timer interrupt routine for UART
//void tm0() interrupt 1 using 1
void tm0() __interrupt 1 __using 1;

//-----------------------------------------
//initial UART module variable
void uart_init();
void enable_timer0(void);
void disable_timer0(void);

void putc(const char c);
void puts(const char *s);

void puthex(unsigned char v);
void puthex2(const unsigned char x);

#endif // _SOFTWARE_UART_H