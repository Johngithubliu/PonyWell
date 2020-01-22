#include "fifo.h"
#include "ql_stdlib.h"



void lock(struct FIFO *s)
{	
	while(s->protect){Ql_Sleep(2);};
		
		s->protect=1; 
}
void unlock(struct FIFO *s)
{
	s->protect=0;

}


void fifo_init(struct FIFO *s,unsigned char * pbuf)
{
	s->start=0;
	s->end=0;
	s->protect=0;
	s->buf=pbuf;
	
}

unsigned char fifo_pop(struct FIFO * s,unsigned char *pch)
{ 
	
  lock(s);
  if(s->start==s->end) 
  {
    unlock(s);
    return  0;
  }
	
	//*pch=s->buf[s->start];
 	Ql_memcpy(pch,s->buf+(unsigned int )(DATA_LEN*(s->start)),DATA_LEN);
	s->start++;
	s->start&=(MAX_BUF-1);
		
	unlock(s);
		
  return 1;
}

void  fifo_push(struct FIFO * s,unsigned char *pch)
{
	
  lock(s);
	
  //s->buf[s->end]=*pch;
  if(((s->end+1)&(MAX_BUF-1))==s->start)
  	{unlock(s);return;}
   
	Ql_memcpy(s->buf+(unsigned int )(DATA_LEN*(s->end)),pch,DATA_LEN);
	s->end++;
  s->end&=(MAX_BUF-1);
 
  unlock(s);
}

unsigned char is_empty(struct FIFO *s)
{
    if(s->start==s->end)return 1;
    else return 0;
}
