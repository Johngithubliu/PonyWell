#ifndef __FIFO_H


#define __FIFO_H

#define MAX_BUF 0x1000
/*
0x100 256
256*3s=768s/60s/min=13min
0x1000 4k
4k*3s=12ks 12ks/60s=0.2k hour=200mins 
0x400 1k =50mins
0x2kk

*/

#define DATA_LEN	(2+4+4)

struct FIFO   
{
  unsigned char *buf;
  unsigned char protect;
 
  unsigned int start;
  unsigned int end;
 
};




extern unsigned char is_empty(struct FIFO *s);
extern void  fifo_push(struct FIFO * s,unsigned char *pch);
extern unsigned char fifo_pop(struct FIFO * s,unsigned char *pch);
extern void fifo_init(struct FIFO *,unsigned char * pbuf);





#endif
