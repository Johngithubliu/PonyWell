
#include "ql_uart.h"


void CallBack_UART0(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
}


void uart0init(void)
{
    s32 ret;
    ret=Ql_UART_Register(UART_PORT0, CallBack_UART0, NULL);
    ret=Ql_UART_Open(UART_PORT0, 115200, FC_NONE);
}

