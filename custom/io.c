#include "ql_stdlib.h"
#include "ql_common.h"
#include "ql_gpio.h"
#include "ql_type.h"



//GPIO define
Enum_PinName  LED_UART  =PINNAME_GPIO2;
Enum_PinName  LED_NET   =PINNAME_NETLIGHT;
Enum_PinName  SEND485   =PINNAME_GPIO1;
Enum_PinName  LED_STATUS =PINNAME_RI ;

void ioInit(void)
{
    // Initialize the GPIO pin (output high level, pull up)
    Ql_GPIO_Init(LED_UART, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);      
    Ql_GPIO_Init(LED_NET, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);
    Ql_GPIO_Init(SEND485, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);
    Ql_GPIO_Init(LED_STATUS, PINDIRECTION_OUT, PINLEVEL_HIGH, PINPULLSEL_PULLUP);    
}


void ledStatusOn(void) 
{
    Ql_GPIO_SetLevel(LED_NET,PINLEVEL_HIGH);
}

void ledStatusOff(void)
{
    Ql_GPIO_SetLevel(LED_NET,PINLEVEL_LOW);
}
void ledNetworkOn(void)
{
    Ql_GPIO_SetLevel(LED_STATUS,PINLEVEL_HIGH);
}
void ledNetworkOff(void)
{
    Ql_GPIO_SetLevel(LED_STATUS,PINLEVEL_LOW);
}

