/*---------------------------------------------------------------------*/
#ifndef CODING_H__
#define CODING_H__

#include <gpio-hal.h>

struct Protocol {
   unsigned int  sync_high;
   unsigned int  sync_low_ms;
   unsigned char sync_low_us;
   unsigned char low;
   unsigned char high;
   unsigned char id;
};  

void rfsyncPulse(struct Protocol* protocol, GpioConfig* pinASK);
void send(struct Protocol* protocol, const unsigned char byte, const unsigned int numBits, GpioConfig* pinASK);


/*---------------------------------------------------------------------*/
#endif // CODING_H_