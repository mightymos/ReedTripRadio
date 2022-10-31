#ifndef _FLASH_DUMP_H
#define _FLASH_DUMP_H

#include <STC/15W10x/DIP8.h>

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
#define BAUD 0xFA18 //2400bps @ 10.886MHz



//#define BAUD 2400



//sfr AUXR = 0x8E;
//sbit RXB = P3^0; //define UART TX/RX port
//sbit TXB = P3^1;

#define RXB P3_0
#define TXB P3_1
#define BTN P3_2

//typedef bit BOOL;
typedef __bit BOOL;
typedef unsigned char BYTE;
typedef unsigned int WORD;


//-----------------------------------------
//Timer interrupt routine for UART
//void tm0() interrupt 1 using 1
void tm0() __interrupt 1 __using 1;

//-----------------------------------------
//initial UART module variable
void UART_INIT();
void init_software_uart_timer0(void);

void putc(const char c);
void puts(const char *s);

void puthex(BYTE v);
void puthex2(const BYTE x);

void putihex(WORD addr, BYTE len, BYTE type);
void check_flash(uint16_t startAddress, uint16_t endAddress);

#endif // _FLASH_DUMP_H