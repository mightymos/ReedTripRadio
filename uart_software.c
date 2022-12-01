// based on test code posted by user PSLLSP
// https://github.com/grigorig/stcgal/issues/26

#include "uart_software.h"

#include <STC/15W10x/DIP8.h>

extern void pulseLED(unsigned char repeat);

unsigned char TBUF,RBUF;

__bit TING,RING;
__bit TEND,REND;
unsigned char t, r;
unsigned char buf[16];

// internal
unsigned char TDAT,RDAT;
unsigned char TCNT,RCNT;
unsigned char TBIT,RBIT;

//-----------------------------------------
//Timer interrupt routine for UART
void tm0() __interrupt 1 __using 1
{    
  if (RING) {
    if (--RCNT == 0) {
      RCNT = 3; //reset send baudrate counter
      if (--RBIT == 0) {
        RBUF = RDAT; //save the data to RBUF
        RING = 0; //stop receive
        REND = 1; //set receive completed flag
      }
      else {
        RDAT >>= 1;
        if (RXB) RDAT |= 0x80; //shift RX data to RX buffer
      }
    }
  }
  else if (!RXB) {
    RING = 1; //set start receive flag
    RCNT = 4; //initial receive baudrate counter
    RBIT = 9; //initial receive bit number (8 data bits + 1 stop bit)
  }

  if (--TCNT == 0) {
    TCNT = 3;    //reset send baudrate counter
    if (TING) {  //judge whether sending
      if (TBIT == 0) {
        TXB = 0; //send start bit
        TDAT = TBUF; //load data from TBUF to TDAT
        TBIT = 9; //initial send bit number (8 data bits + 1 stop bit)
      }
    else {
      TDAT >>= 1; //shift data to CY
      if (--TBIT == 0) {
        TXB = 1;
        TING = 0; //stop send
        TEND = 1; //set send completed flag
      }
      else {
        TXB = CY; //write CY to TX port
      }
    }
  }
}
}


void enable_timer0(void)
{
    //timer0 in 16-bit auto reload mode
    TMOD = 0x00;
    
    //timer0 working at 1T mode    
    AUXR = 0x80;
    
    //initial timer0 and set reload value
    T0L = BAUD & 0xff;
    T0H = BAUD >> 8;

    //timer0 start running
    T0R = 1;
    
    //enable timer0 interrupt
    IE1  |= M_ET0;
    
    //improve timer0 interrupt priority
    IP1H |= M_PT0;
}

void disable_timer0(void)
{
    //timer0 in 16-bit auto reload mode
    //TMOD = 0x00;
    //timer0 working at 12T mode    
    AUXR &= ~0x80;
    
    //timer0 stop running
    T0R = 0;
    
    // clear overflow flag
    T0IF = 0;
    
    //initial timer0 and set reload value
    T0L = 0x00;
    T0H = 0x00;
    
    //disable timer0 interrupt
    IE1  &= ~M_ET0;
    
    //clear timer0 interrupt priority
    IP1H &= ~M_PT0;
}

//-----------------------------------------
//initial UART module variable
void uart_init()
{
  TING = 0;
  RING = 0;
  TEND = 1;
  REND = 0;
  TCNT = 0;
  RCNT = 0;
}

//-----------------------------------------


void putc(const char c)
{
  while (TEND==0);
  TBUF = c;
  TEND = 0;
  TING = 1;
}

void puts(const char *s)
{
   while (*s) putc(*s++);
}

void puthex(unsigned char v)
{
   unsigned char c;
   v &= 0x0f;
   if (v<10) c = '0'+v;
   else c = 'A'-10+v;
   putc(c);
}

void puthex2(const unsigned char x)
{
   puthex(x >> 4);
   puthex(x);
}