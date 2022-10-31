#include "flash_dump.h"

#include <STC/15W10x/DIP8.h>

BYTE TBUF,RBUF;
BYTE TDAT,RDAT;
BYTE TCNT,RCNT;
BYTE TBIT,RBIT;
BOOL TING,RING;
BOOL TEND,REND;
void UART_INIT();
BYTE t, r;
BYTE buf[16];

//-----------------------------------------
//Timer interrupt routine for UART
//void tm0() interrupt 1 using 1
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


void init_software_uart_timer0(void)
{
    TMOD = 0x00;        //timer0 in 16-bit auto reload mode
    AUXR = 0x80;        //timer0 working at 1T mode
    T0L = BAUD & 0xff;
    T0H = BAUD>>8;      //initial timer0 and set reload value
    T0R = 1;            //tiemr0 start running
    IE1  |= M_ET0;      //enable timer0 interrupt
    IP1H |= M_PT0;      //improve timer0 interrupt priority
    EA = 1;             //open global interrupt switch
}

//-----------------------------------------
//initial UART module variable
void UART_INIT()
{
  TING = 0;
  RING = 0;
  TEND = 1;
  REND = 0;
  TCNT = 0;
  RCNT = 0;
}

//-----------------------------------------

void isp_firmware()
{
    //IAP_CONTR |= 1 << 6; // SWBS, activate ISP monitor
    //IAP_CONTR |= 1 << 5; // SWRST, trigger reset
    //IAP_CONTR = (1<<5) | (1<<6);
    IAP_CONTR = 0x60;  // IAP_CONTR is at address 0xC7
}


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

void puthex(BYTE v)
{
   BYTE c;
   v &= 0x0f;
   if (v<10) c = '0'+v;
   else c = 'A'-10+v;
   putc(c);
}

void puthex2(const BYTE x)
{
   puthex(x >> 4);
   puthex(x);
}

void putihex(WORD addr, BYTE len, BYTE type)
{
  BYTE v;
  BYTE s = 0;
  __code BYTE* ptr;

  putc(':');
  puthex2(len);       // length
  s -= len;
  puthex2(addr >> 8); // address
  puthex2(addr);
  s -= (addr >> 8) & 0xff;
  s -= addr & 0xff;
  puthex2(type);      // type - data
  s -= type;
  ptr = (__code BYTE*) addr;
  while (len--) {
    v = *ptr++;
    s -= v;
    puthex2(v);
  }
  puthex2(s);
  putc('\r');
  putc('\n');
}

///////////////////////////////////////////////////////////////////////////////////////////////
// CHECK FLASH
// print program memory to terminal for verification (probably used for debugging only)
//////////////////////////////////////////////////////////////////////////////////////////////
void check_flash(uint16_t startAddress, uint16_t endAddress)
{
    uint16_t i;
    __code uint16_t *codePtr;
    uint8_t flashByte;
        
    /* print program memory range to terminal
       FIXME: is stop condition correct?
    */
    for (i = startAddress; i < (startAddress + endAddress); i++)
    {
        codePtr   = (__code uint16_t *) i;
        flashByte = *codePtr;
        puthex2(flashByte);
    }
        
}