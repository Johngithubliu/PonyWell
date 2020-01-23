
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

void proc_main_task(s32 taskId)
{
    // Initialize the GPIO pin (output high level, pull up)
    Ql_GPIO_Init(LED_UART, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);      
    Ql_GPIO_Init(LED_NET, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);
    Ql_GPIO_Init(SEND485, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);
    //Ql_GPIO_Init(contron, PINDIRECTION_OUT, PINLEVEL_HIGH, PINPULLSEL_PULLUP);
    
    while(TRUE)
    {
        Ql_Sleep(100);
        Ql_GPIO_SetLevel(LED_NET,PINLEVEL_LOW);
        Ql_GPIO_SetLevel(LED_UART,PINLEVEL_LOW);
        Ql_Sleep(500);
        Ql_GPIO_SetLevel(LED_NET,PINLEVEL_HIGH);
        Ql_GPIO_SetLevel(LED_UART,PINLEVEL_HIGH);
    }
}
 
void proc_send2srv(s32 taskId)
{
    while(TRUE)
    {
        Ql_Sleep(1000);
        //Ql_GPIO_SetLevel(LED_UART,PINLEVEL_LOW);
        Ql_Sleep(1000);
        //Ql_GPIO_SetLevel(LED_UART,PINLEVEL_HIGH);
    }
}

void proc_tcp_network(s32 taskId)
{
    while(TRUE)
    {
        Ql_Sleep(1500);
        //Ql_GPIO_SetLevel(SEND485,PINLEVEL_LOW);
        Ql_Sleep(1500);
        //Ql_GPIO_SetLevel(SEND485,PINLEVEL_HIGH);
    }

}


#endif // __PONY_WELL__


