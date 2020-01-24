
#ifdef __PONY_WELL__  
#include "custom_feature_def.h"
#include "ql_stdlib.h"
#include "ql_common.h"
#include "ql_type.h"
#include "ql_trace.h"
#include "ql_error.h"
#include "ql_uart.h"
#include "ql_timer.h"
#include "ril_network.h"
#include "ril_socket.h"
#include "ril.h"
#include "ql_time.h"
#include "ql_system.h"
#include "ql_gpio.h"

#include "def.h"
#include "io.h"
#include "uart0.h"


bool initialized=false;

void proc_main_task(s32 taskId)
{
    uart0init();
    ioInit();
  
    initialized=true;
    
    while(true)
    {
        APP_DEBUG("Good afternoon\r\n");
        ledStatusOn();
        sleep(50);
        ledStatusOff();
        sleep(3000);
    }
}
 
void proc_send2srv(s32 taskId)
{
    while(TRUE)
    {
        ledNetworkOn();
        sleep(1000);
        ledNetworkOff();
        sleep(2000);
        
    }
}

void proc_tcp_network(s32 taskId)
{
    while(TRUE)
    {
        sleep(1500);
    }

}


#endif // __PONY_WELL__


