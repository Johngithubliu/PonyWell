/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2013
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   example_tcpclient.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   This example demonstrates how to establish a TCP connection, when the module
 *   is used for the client.
 *
 * Usage:
 * ------
 *   Compile & Run:
 *
 *     Set "C_PREDEF=-D __EXAMPLE_TCPCLIENT__" in gcc_makefile file. And compile the 
 *     app using "make clean/new".
 *     Download image bin to module to run.
 * 
 *   Operation:
 *            set server parameter, which is you want to connect.
 *            Command:Set_Srv_Param=<srv ip>,<srv port>
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * 
 ****************************************************************************/
#ifdef __BC26__  
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


#include "fifo.h"



#define TIME_OUT_CON 15
#define MAX_FRAM 16
#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT0
#define DBG_BUF_LEN   512
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    if (UART_PORT2 == (DEBUG_PORT)) \
    {\
        Ql_Debug_Trace(DBG_BUFFER);\
    } else {\
        Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
    }\
}
#else
#define APP_DEBUG(FORMAT,...) 
#endif


/*****************************************************************
* define process state
******************************************************************/
typedef enum{
    STATE_NW_QUERY_STATE,
    STATE_SOC_OPEN,
    STATE_SOC_CONNECTED,
    STATE_SOC_SEND,
    STATE_SOC_SENDEX,
    STATE_SOC_READ,
    STATE_SOC_CLOSE,
    STATE_TOTAL_NUM
}Enum_TCPSTATE;
static u8 m_tcp_state = STATE_NW_QUERY_STATE;

/*****************************************************************
* UART Param
******************************************************************/
#define SERIAL_RX_BUFFER_LEN  64
static u8 m_RxBuf_Uart[SERIAL_RX_BUFFER_LEN];

/*****************************************************************
* timer param
******************************************************************/
#define TCP_TIMER_ID          TIMER_ID_USER_START
#define TCP_TIMER_PERIOD      1000

#define READ_TIMER_ID           TIMER_ID_USER_START+1
#define READ_TIMER_PERIOD      10000

#define NTP_TIMER_ID           TIMER_ID_USER_START+2
#define NTP_TIMER_PERIOD      10000

//#define MODBUS_ADD              0x02
#define MODBUS_WRITE            0x06


/*****************************************************************
* socket Param
******************************************************************/
static Enum_SerialPort m_myUartPort  = UART_PORT2;

#define SRVADDR_LEN  32

#define SEND_BUFFER_LEN     512
#define RECV_BUFFER_LEN     512
static u8 m_send_buf[SEND_BUFFER_LEN];
static u8 m_recv_buf[RECV_BUFFER_LEN];
static u8 string_buffer[200];
static u8 string_len=0;

static u8  m_SrvADDR_mast[SRVADDR_LEN] = "uhf.hopto.org\0";
static u32 m_SrvPort_mast = 8449;

static u8  m_mast_ip[SRVADDR_LEN]="65.49.220.94\0";

static u8  m_SrvADDR[SRVADDR_LEN] = "65.49.220.94\0";
static u32 m_SrvPort = 8449;

static s32 m_socketid = -1; 

static u8 send_buffer[256] = {'a','b'};     //string mode
static u8 send_buffer_hex[512]={};          //hex mode
//u8 fram_head[8]={0xcc,0xdd,0x00,0x00,0x00,0x00,0x00,0x00};
//u8 data_head[8]={0x01,0x01,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
u8 utc[4]={0x00,0x00,0x00,0x00};
ST_Socket_Param_t  socket_param_t = {0,0,0,NULL,0,0,0,0};
static u8 ip_saved[25];

Socket_Recv_Param_t* socket_recv_param_ptr = NULL;

s32 recv_actual_length = 0;
s32 recv_remain_length = 0;

static ST_Time time;
static ST_Time  timeLocal;
static ST_Time* pTime = NULL;
static u32 totalSeconds;
static s32 ntptime=0;
static u8 flag_send=0;
#define SEND_DATA_HEX

/*****************************************************************
* uart callback function
******************************************************************/
static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);

/*****************************************************************
* timer callback function
******************************************************************/
//static void Callback_Timer(u32 timerId, void* param);
//static void Callback_Read_Timer(u32 timerId, void* param);
static void Callback_Ntp_Timer(u32 timerId, void* param);
u8 tcp_network_enable=0;
/*****************************************************************
* other subroutines
******************************************************************/
extern s32 Analyse_Command(u8* src_str,s32 symbol_num,u8 symbol, u8* dest_buf);
static s32 ReadSerialPort(Enum_SerialPort port, /*[out]*/u8* pBuffer, /*[in]*/u32 bufLen);
static void proc_handle(u8 *pData,s32 len);

void data_buf2hex(u8* data_buf,u8* data_bufhex,u16 buf_len);
unsigned char crc8(unsigned char *str, unsigned char len);
u16 usMBCRC16( u8 * pucFrame, u16 usLen );
void save_ip();
void get_ip();
void save_parament();
void restore_parament();
void  imsi2bcd();
void  iccid2bcd();
Enum_PinName  sending =PINNAME_GPIO2 ;
//Enum_PinName  sending =PINNAME_RI ;
Enum_PinName  send485 =PINNAME_GPIO1 ;
Enum_PinName  contron =PINNAME_RI ;
Enum_PinName  net_ok =PINNAME_NETLIGHT ;

    // Define the initial level for GPIO pin
Enum_PinLevel gpioLvl = PINLEVEL_LOW;

static  struct FIFO *data_fifo;

unsigned char * pbuf;

static unsigned int time_out_con;
static unsigned char flag_resp;
static unsigned char command_read[8]={0x00,0x03,0x00,0x75,0x00,0x02,0x00,0x00};
static s32 totalBytes;

typedef struct{
    u8 modbus_add;
    
    u8 mast_ip[SRVADDR_LEN];
    u8 mast_dn[SRVADDR_LEN];
    u8 para1;
}PARAMENT;
PARAMENT parament={
    2,
    "65.49.220.94",
    "uhf.hopto.org",
    1


};
u8 command_len;
u8 command_data[32];
u8 proc_cmd(u8 ,u8 * ,u8 * );
u8 ack_len=0;
u8 ack_data[64];

u8 imsi[32];
u8 imsi_bcd[8];
u8 imei[32];
u8 iccid[32];
u8 iccid_bcd[8];
u8 rssi;
u8 timestr[64];
u8 first=1;

unsigned char insert_list(unsigned char *,unsigned char *);
unsigned char cmp6(unsigned char *in1,unsigned char *in2);
void exchange(unsigned char *in1,unsigned char *in2);



void proc_main_task(s32 taskId)
{
    ST_MSG msg;
    u8 m_recv_buf_hex[512];
    u16 modbus_read_crc;
	s32 ret;
    time_out_con=TIME_OUT_CON;
     
    
  
    // Initialize the GPIO pin (output high level, pull up)
    Ql_GPIO_Init(sending, PINDIRECTION_OUT, gpioLvl, PINPULLSEL_PULLUP);
    
    Ql_GPIO_Init(net_ok, PINDIRECTION_OUT, gpioLvl, PINPULLSEL_PULLUP);
    Ql_GPIO_Init(send485, PINDIRECTION_OUT, gpioLvl, PINPULLSEL_PULLUP);
    Ql_GPIO_Init(contron, PINDIRECTION_OUT, PINLEVEL_HIGH, PINPULLSEL_PULLUP);


    // Register & open UART port
    ret = Ql_UART_Register(m_myUartPort, CallBack_UART_Hdlr, NULL);
    ret = Ql_UART_Open(m_myUartPort, 1200, FC_NONE);
	ret = Ql_UART_Register(UART_PORT0, CallBack_UART_Hdlr, NULL);
    ret = Ql_UART_Open(UART_PORT0, 115200, FC_NONE);
    APP_DEBUG("\r\nOpenCPU for BC26,SDK version 1.3 ,Builde at %s %s by John\r\n",__DATE__,__TIME__);
    data_fifo=Ql_MEM_Alloc(sizeof(struct FIFO));
    pbuf=Ql_MEM_Alloc(MAX_BUF*DATA_LEN);
    fifo_init(data_fifo,pbuf);
    if(data_fifo->buf==NULL)
       { APP_DEBUG("\r\nFIFO init err!");}
    else APP_DEBUG("\r\nFIFO init ok!");

    restore_parament();
    get_ip();
    
    command_read[0]=parament.modbus_add;
    APP_DEBUG("\r\nModbus address: %d\r\n\
Setup server ip use:Set_Srv_Param=<server_ip>,<port>\r\n\
Setup default server nomain name use:Set_mast_DN=<xxx.xxx.xxx>\r\n\
Setup default server ip use:Set_mast_ip==<123.456.789>\r\n\
Setup modbus address use:Set_Para=<Modbus_address>\r\n",command_read[0]);
    modbus_read_crc=usMBCRC16(command_read,6);
    command_read[7]=modbus_read_crc>>8;
    command_read[6]=modbus_read_crc&0x0ff;


    //register & start timer 
  //  Ql_Timer_Register(TCP_TIMER_ID, Callback_Timer, NULL);
  //  Ql_Timer_Register(READ_TIMER_ID, Callback_Read_Timer, NULL);
  //  Ql_Timer_Start(READ_TIMER_ID, READ_TIMER_PERIOD, TRUE);

     Ql_Timer_Register(NTP_TIMER_ID, Callback_Ntp_Timer, NULL);
    Ql_Timer_Start(NTP_TIMER_ID, NTP_TIMER_PERIOD, TRUE);

    while(TRUE)
    {
        Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
#ifdef __OCPU_RIL_SUPPORT__
        case MSG_ID_RIL_READY:
            APP_DEBUG("<-- RIL is ready -->\r\n");
            Ql_RIL_Initialize();
            

            break;
#endif
		case MSG_ID_URC_INDICATION:
		{     
			switch (msg.param1)
            {
    		    case URC_SIM_CARD_STATE_IND:
    			APP_DEBUG("<-- SIM Card Status:%d -->\r\n", msg.param2);
				if(SIM_STAT_READY == msg.param2)
				{
                  // Ql_Timer_Start(TCP_TIMER_ID, TCP_TIMER_PERIOD, TRUE);
                    tcp_network_enable=1;
   
				}
                Ql_memset(imei,0,32);
                RIL_GetIMEI(imei);
                APP_DEBUG("\r\nIMEI=%s",imei);

                 Ql_memset(imsi,0,32);
                RIL_GetIMSI(imsi);
                APP_DEBUG("\r\nIMSI=%s",imsi);
                imsi2bcd();

                 Ql_memset(iccid,0,32);
                RIL_GetICCID(iccid);
                APP_DEBUG("\r\nICCID=%s",iccid);
                iccid2bcd();
                //for test only
                /*
                iccid_bcd[0]=0x02;
                iccid_bcd[1]=0x0B9;
                iccid_bcd[2]=0x26;
                iccid_bcd[3]=0x17;
                iccid_bcd[4]=0x71;
                iccid_bcd[5]=0x11;
                iccid_bcd[6]=0x88;
                iccid_bcd[7]=0x84;
                */
                //for test only
                ret = Ql_RIL_SendATCmd("AT+SM=LOCK\r\n",Ql_strlen("AT+SM=LOCK\r\n"),NULL,NULL,0);
                ret = Ql_RIL_SendATCmd("AT+QBAND=1,8\r\n",Ql_strlen("AT+QBAND=1,8\r\n"),NULL,NULL,0);

    			break;			  
    		    case URC_SOCKET_RECV_DATA:
					socket_recv_param_ptr = msg.param2;
    				if(socket_param_t.access_mode == SOCKET_ACCESS_MODE_BUFFER)
    				{
                        Ql_Sleep(10);
                        time_out_con=TIME_OUT_CON;
                        flag_resp=1;
						//APP_DEBUG("<recv_len:%d -->\r\n",socket_recv_param_ptr->recv_length);
	                    //m_tcp_state = STATE_SOC_READ;
                        ret = RIL_SOC_QIRD(m_socketid,RECV_BUFFER_LEN,&recv_actual_length,&recv_remain_length,m_recv_buf);
                        if (ret == 0)
                        {
                           if(recv_actual_length == 0)
                           {
                              APP_DEBUG("<--The buffer has been read empty\r\n");
                              break;
                           }
                           data_buf2hex(m_recv_buf,m_recv_buf_hex,recv_actual_length);
                           //APP_DEBUG("\r\nrec:%s,%d\r\n",m_recv_buf_hex,recv_actual_length);
                            if(recv_actual_length>=16) 
                                command_len=proc_cmd(recv_actual_length-8,m_recv_buf+8,command_data);

                        }
                        else
                        {
                            APP_DEBUG("<--read socket id failure,error=%d.-->\r\n",ret);
                        }

                       //Ql_UART_Write(UART_PORT2,m_recv_buf,recv_actual_length);
    				}
    				else if(socket_param_t.access_mode == SOCKET_ACCESS_MODE_DIRECT)
    				{
						  APP_DEBUG("<-- recv data,%d,%d,%s\r\n", socket_recv_param_ptr->connectID,\
							 	socket_recv_param_ptr->recv_length,socket_recv_param_ptr->recv_buffer);
    				}

					
    			    break;
    			case URC_SOCKET_CLOSE:
					APP_DEBUG("<-- close socket id(%d )-->\r\n", msg.param2);   
					ret = RIL_SOC_QICLOSE(msg.param2);
					if (ret == 0)
                    {
                        APP_DEBUG("<-- closed socket successfully\r\n");
                        m_tcp_state = STATE_TOTAL_NUM;
                        Ql_GPIO_SetLevel(net_ok, PINLEVEL_LOW);
                         // m_tcp_state = STATE_NW_QUERY_STATE;
                        // APP_DEBUG("<--Restart the TCP connection process.-->\r\n");
                        
                    }else
                    {
                        APP_DEBUG("<--closed socket id failure,error=%d.-->\r\n",ret);
                    }
        		    break;
                

		        default:
    		    //APP_DEBUG("<-- Other URC: type=%d\r\n", msg.param1);
    	        break;
			}
		}
		break;
	default:
         break;
        }
    }
}
 
void proc_send2srv(s32 taskId)
{
    static u32 count_test=0l;
    u16 crc_u16;
    u8 f_ack_cmd=0;
    u8 first_conect=0;
    

    ST_MSG msg;
    s32 ret;
    u8 count_wait=0;
    unsigned char data_fram[DATA_LEN];
    unsigned char data_mult_fram[20+DATA_LEN*MAX_FRAM];
    unsigned char count_fram;
    u16 temp_u16;
    flag_resp=1;
   // u8 i=0;
    
    Ql_GPIO_SetLevel(sending,PINLEVEL_LOW);
    Ql_Sleep(5000);

    Ql_memset(data_mult_fram,0,20+DATA_LEN*MAX_FRAM);

 
 

    while(first==1)
    {
        ret=RIL_GetNtpTime(timestr);
        if(ret==RIL_AT_SUCCESS)
           { 
                APP_DEBUG("GetNtpTime OK! %s\r\n",timestr);
                first=0;
                first_conect=1;
            }
        else 
            {
                APP_DEBUG("GetNtpTime Fail!\r\n");
                Ql_Sleep(2000);
            }
    }
    
    while(TRUE)
    {
        RIL_GetCSQ(&rssi);
        if(flag_resp)
        {
            if(ack_len==0)
            {
                Ql_memcpy(data_mult_fram+8,iccid_bcd,8);
                count_fram=0;
                if(!first_conect)
                {
                    while(TRUE)
                    {

                        while(fifo_pop(data_fifo,data_fram))
                        {
                            //APP_DEBUG("\r\nPop:%d ",data_fifo->start);
                          //  Ql_UART_Write(m_myUartPort, data_fram, DATA_LEN);
                            Ql_memcpy(data_mult_fram+16+count_fram*DATA_LEN,data_fram,DATA_LEN);
                            if(++count_fram>=MAX_FRAM){break;}
                           
                        }
                        if(count_fram)break;
                        if(string_len)break;
                        Ql_Sleep(1000);

                    }
                }

             //Ql_UART_Write(m_myUartPort, data_mult_fram, DATA_LEN*count_fram+19);

                if(count_fram)//have data from fifo;
                {
               // APP_DEBUG("\r\nmult_fram:");
                

               // Ql_UART_Write(m_myUartPort, data_mult_fram, DATA_LEN*count_fram+19);

               //make fram-------------------
                    temp_u16=DATA_LEN*count_fram+8+3;// insert csq 
                    data_mult_fram[16+count_fram*DATA_LEN]=0x01;//csq lenght=1;
                    data_mult_fram[16+count_fram*DATA_LEN+1]=0x02;//csq fram type =2;
                    data_mult_fram[16+count_fram*DATA_LEN+2]=rssi;//insert csq tu mult_fram;

                    //APP_DEBUG("temp_u16=%d:%04x\r\n",temp_u16,temp_u16);
                    data_mult_fram[0]=0x0cc;
                    data_mult_fram[1]=0x0dd;
                    data_mult_fram[2]=temp_u16>>8;
                    data_mult_fram[3]=temp_u16&0x0ff;
                    crc_u16=usMBCRC16(data_mult_fram+8, temp_u16 );
                    data_mult_fram[4]=crc_u16>>8;
                    data_mult_fram[5]=crc_u16&0x0ff;
                    data_mult_fram[6]=0x00;
                    data_mult_fram[7]=crc8(data_mult_fram,7);
                
                    data_buf2hex(data_mult_fram,send_buffer_hex,temp_u16+8);
                  //APP_DEBUG("\r\nno ack:%s\r\n",send_buffer_hex);

                //end make fram---------------
                }
                else 
                if(first_conect)//first conect ,
                {
                    data_mult_fram[0]=0x0cc;
                    data_mult_fram[1]=0x0dd;
                    data_mult_fram[2]=0;
                    data_mult_fram[3]=8;
                    temp_u16=usMBCRC16(data_mult_fram+8, 8 );
                    data_mult_fram[4]=temp_u16>>8;
                    data_mult_fram[5]=temp_u16&0x0ff;
                    data_mult_fram[6]=0x00;
                    data_mult_fram[7]=crc8(data_mult_fram,7);
                
                    data_buf2hex(data_mult_fram,send_buffer_hex,16);
                    APP_DEBUG("\r\nsend first NULL fram:%s\r\n",send_buffer_hex);
                    first_conect=0;
                    //------------first power on ,send version information.


                    if((Ql_GetLocalTime(&time)))
                    {
                    
                        totalSeconds= Ql_Mktime(&time);
                      
                                           
                    }
                    else
                    {
                        APP_DEBUG("\r\n<--failed !! Local time not determined -->\r\n");
                    }

                    Ql_sprintf(string_buffer+6,"BC26(%s %s)",__DATE__,__TIME__);
                    string_buffer[0]=Ql_strlen(string_buffer+6)+4;//length
                    string_buffer[1]=0x0ff;//data_type=0xff,is string.
                    string_buffer[2]=(totalSeconds>>24)&0x0ff;
                    string_buffer[3]=(totalSeconds>>16)&0x0ff;
                    string_buffer[4]=(totalSeconds>>8)&0x0ff;
                    string_buffer[5]=(totalSeconds)&0x0ff;//time ,4bytes.
               
                    string_len=string_buffer[0]+2;


                }
                else if(string_len)
                {
                    temp_u16=string_len+8;
                    Ql_memcpy(data_mult_fram+16,string_buffer,string_len);
                    data_mult_fram[temp_u16+8+0]=0x01;
                    data_mult_fram[temp_u16+8+1]=0x02;
                    data_mult_fram[temp_u16+8+2]=rssi;
                    temp_u16+=3;

                    data_mult_fram[0]=0x0cc;
                    data_mult_fram[1]=0x0dd;
                    data_mult_fram[2]=temp_u16>>8;
                    data_mult_fram[3]=temp_u16&0x0ff;
                    crc_u16=usMBCRC16(data_mult_fram+8, temp_u16 );
                    data_mult_fram[4]=crc_u16>>8;
                    data_mult_fram[5]=crc_u16&0x0ff;
                    data_mult_fram[6]=0x00;
                    data_mult_fram[7]=crc8(data_mult_fram,7);

                            
                    data_buf2hex(data_mult_fram,send_buffer_hex,temp_u16+8);
                    //APP_DEBUG("\r\nsend string fram:%s\r\n",send_buffer_hex);

                    string_len=0;
                }
            }
            else
            {
                temp_u16=16+ack_len;
                
                data_mult_fram[8]=0x0;
                data_mult_fram[9]=0x0;
                data_mult_fram[10]=0x0;
                data_mult_fram[11]=0x0;

                data_mult_fram[12]=0x0;
                data_mult_fram[13]=0x0;
                data_mult_fram[14]=0x0;
                data_mult_fram[15]=0x041;
                
                Ql_memcpy(data_mult_fram+16,iccid_bcd,8);
                Ql_memcpy(data_mult_fram+24,ack_data,ack_len);
                
                data_mult_fram[0]=0x0cc;
                data_mult_fram[1]=0x0dd;
                data_mult_fram[2]=temp_u16>>8;
                data_mult_fram[3]=temp_u16&0x0ff;
                temp_u16=usMBCRC16(data_mult_fram+8, temp_u16 );
                data_mult_fram[4]=temp_u16>>8;
                data_mult_fram[5]=temp_u16&0x0ff;
                data_mult_fram[6]=0x00;
                data_mult_fram[7]=crc8(data_mult_fram,7);

                f_ack_cmd=1;
            
                data_buf2hex(data_mult_fram,send_buffer_hex,ack_len+24);
                //APP_DEBUG("\r\nack:%s\r\n",send_buffer_hex);

            }

            
        }
        else 
        {

            if(++count_wait>5)
            {

                count_wait=0;
                m_tcp_state=STATE_SOC_CLOSE;
            }
        }
        //send--------------------
        
        while(m_tcp_state!=STATE_SOC_CONNECTED)
        {  

            if(m_tcp_state==STATE_TOTAL_NUM)
            {
                m_tcp_state=STATE_NW_QUERY_STATE;
               
               
            }
             APP_DEBUG(".");
             Ql_Sleep(2000);
        }
        Ql_GPIO_SetLevel(sending,PINLEVEL_HIGH);
        ret = RIL_SOC_QISENDEX(m_socketid,Ql_strlen(send_buffer_hex)/2,send_buffer_hex);
        
        //APP_DEBUG("\r\nsend:%s,ack_len=%d ",send_buffer_hex,ack_len);
        if(ack_len&&f_ack_cmd)
            {flag_resp=1;ack_len=0;f_ack_cmd=0;}
        else flag_resp=0;  

        if(ret==0)
        {
            Ql_GPIO_SetLevel(sending,PINLEVEL_LOW);
            time_out_con=TIME_OUT_CON;
            APP_DEBUG("ok.\r\n");
        }   
        else 
        {
            APP_DEBUG(" failure,close connection and retry.\r\n");
            m_tcp_state=URC_SOCKET_CLOSE;
            Ql_Sleep(2000);
            
            while(m_tcp_state!=STATE_SOC_CONNECTED)
            {  

                if(m_tcp_state==STATE_TOTAL_NUM)
                {
                    m_tcp_state=STATE_NW_QUERY_STATE;
                   
                   
                }
                 APP_DEBUG("*");
                 Ql_Sleep(2000);
            }
                //send again here...
            ret = RIL_SOC_QISENDEX(m_socketid,Ql_strlen(send_buffer_hex)/2,send_buffer_hex);
            Ql_GPIO_SetLevel(sending,PINLEVEL_LOW);
            APP_DEBUG("\r\nretry send ,ret=%d\r\n",ret);
                
        }

        //end send-----------------
     
        for(u8 i=0;i<50;i++)
        {

            Ql_Sleep(200);
            //APP_DEBUG("*");
            if(flag_resp) break;
        }
       
        Ql_Sleep(200);
        /*
        if(ntptime>10l)
        {
            
            ret=RIL_GetNtpTime(timestr);
            if(ret==RIL_AT_SUCCESS)
            {APP_DEBUG("GetNtpTime OK! %s\r\n",timestr);
                ntptime=0;

            }
            else APP_DEBUG("GetNtpTime Fail!\r\n");
        }*/
    }
    APP_DEBUG("task send2srv ended!\r\n");
 
}

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
    
    static long test_weight=0;
    //static long recev_count=0;
    static u32 pr_totalSeconds;
    switch (msg)
    {
    case EVENT_UART_READY_TO_READ:
        {

           //s32 totalBytes = ReadSerialPort(port, m_RxBuf_Uart+4, sizeof(m_RxBuf_Uart)-4);
           if(port==m_myUartPort)totalBytes = ReadSerialPort(port, m_RxBuf_Uart+6, 32);
           else totalBytes = ReadSerialPort(port, m_RxBuf_Uart, sizeof(m_RxBuf_Uart));
           
           if (totalBytes > 0)
           {
                if(port==m_myUartPort)
                {   
                    APP_DEBUG("i");
                    if(totalBytes!=9)    APP_DEBUG("\r\nrec %d B\r\n",totalBytes);
                    if((m_RxBuf_Uart[6]=='*')&&(m_RxBuf_Uart[7]=='*'))//come from pakge
                    {
                        if((Ql_GetLocalTime(&time)))
                            {
                            
                                totalSeconds= Ql_Mktime(&time);
                                //totalSeconds-=8*3600;
                                if(totalSeconds==pr_totalSeconds)
                                    totalSeconds++;
                                pr_totalSeconds=totalSeconds;
                                                   
                            }
                            else
                            {
                                APP_DEBUG("\r\n<--failed !! Local time not determined -->\r\n");
                            }
                            string_buffer[0]=totalBytes-2+4;//length
                            string_buffer[1]=0x0ff;//data_type=0xff,is string.
                            string_buffer[2]=(totalSeconds>>24)&0x0ff;
                            string_buffer[3]=(totalSeconds>>16)&0x0ff;
                            string_buffer[4]=(totalSeconds>>8)&0x0ff;
                            string_buffer[5]=(totalSeconds)&0x0ff;//time ,4bytes.
                       
                            Ql_memcpy(string_buffer+6,m_RxBuf_Uart+8,totalBytes-2);
                            string_len=string_buffer[0]+2;
                            APP_DEBUG("Ur:%s",m_RxBuf_Uart+8);

                    }
                    if(m_RxBuf_Uart[6]==parament.modbus_add)

                    {
                                           
                        if(m_RxBuf_Uart[9]||m_RxBuf_Uart[10]||m_RxBuf_Uart[11]||m_RxBuf_Uart[12])
                        {
                            if((Ql_GetLocalTime(&time)))
                            {
                             
                                totalSeconds= Ql_Mktime(&time);
                                //totalSeconds-=8*3600;
                                if(totalSeconds==pr_totalSeconds)
                                    totalSeconds++;
                                pr_totalSeconds=totalSeconds;

                                //APP_DEBUG("Mk:%ld\r\n", totalSeconds);
                                                   
                            }
                            else
                            {
                                APP_DEBUG("\r\n<--failed !! Local time not determined -->\r\n");
                            }
                        //02 03 0A 00 07 A0 BC 18 10 26 15 01 38 A8 23  ;49.99,18/10/26,15:01:38
                            m_RxBuf_Uart[0]=8;//length
                            m_RxBuf_Uart[1]=0x01;//data_type=1,is time+weight.
                            m_RxBuf_Uart[2]=(totalSeconds>>24)&0x0ff;
                            m_RxBuf_Uart[3]=(totalSeconds>>16)&0x0ff;
                            m_RxBuf_Uart[4]=(totalSeconds>>8)&0x0ff;
                            m_RxBuf_Uart[5]=(totalSeconds)&0x0ff;//time ,4bytes.
                       
                            m_RxBuf_Uart[6]=m_RxBuf_Uart[9];
                            m_RxBuf_Uart[7]=m_RxBuf_Uart[10];
                            m_RxBuf_Uart[8]=m_RxBuf_Uart[11];
                            m_RxBuf_Uart[9]=m_RxBuf_Uart[12];//weight,4bytes.The unit is 100mg.
                                             
                            fifo_push(data_fifo,m_RxBuf_Uart);
                          
                        }
                        else APP_DEBUG("rec:0\r\n");
                     
                    }
                   
                }
                else
                   //APP_DEBUG("receive at UART_PORT0:%s\r\n",m_RxBuf_Uart+4);
                   proc_handle(m_RxBuf_Uart,sizeof(m_RxBuf_Uart));
            }
           
           break;
        }
    case EVENT_UART_READY_TO_WRITE:
       /* if(port==m_myUartPort)
        {   
            Ql_GPIO_SetLevel(send485,PINLEVEL_LOW);

        }*/
        break;
    default:
        break;
    }
}


static s32 ReadSerialPort(Enum_SerialPort port, /*[out]*/u8* pBuffer, /*[in]*/u32 bufLen)
{
    s32 rdLen = 0;
    s32 rdTotalLen = 0;
    if (NULL == pBuffer || 0 == bufLen)
    {
        return -1;
    }
    Ql_memset(pBuffer, 0x0, bufLen);
    while (1)
    {
        rdLen = Ql_UART_Read(port, pBuffer + rdTotalLen, bufLen - rdTotalLen);
        if (rdLen <= 0)  // All data is read out, or Serial Port Error!
        {
            break;
        }
        rdTotalLen += rdLen;
        // Continue to read...
    }
    if (rdLen < 0) // Serial Port Error!
    {
        APP_DEBUG("<--Fail to read from port[%d]-->\r\n", port);
        return -99;
    }
    return rdTotalLen;
}

static void proc_handle(u8 *pData,s32 len)
{
    u8 srvport[10];
	u8 *p = NULL;
    //command: Set_Srv_Param=<srv ip>,<srv port>
    //APP_DEBUG("pData=%s\r\n",pData);
    p = Ql_strstr(pData,"Set_Srv_Param=");
    //APP_DEBUG("p=%s\r\n",p);
    if (p)
    {
        Ql_memset(m_SrvADDR, 0, SRVADDR_LEN);
        if (Analyse_Command(pData, 1, '>', m_SrvADDR))
        {
            APP_DEBUG("<--Server Address Parameter Error.add=%s,p=%s-->\r\n",m_SrvADDR,p);
            return;
        }
        Ql_memset(srvport, 0, 10);
        if (Analyse_Command(pData, 2, '>', srvport))
        {
            APP_DEBUG("<--Server Port Parameter Error.-->\r\n");
            return;
        }
        m_SrvPort = Ql_atoi(srvport);
        APP_DEBUG("<--Set TCP Server Parameter Successfully<%s>,<%d>.-->\r\n",m_SrvADDR,m_SrvPort);
        save_ip();
        ip_saved[24]++;
        if((ip_saved[24])>3)   ip_saved[24]=0;
        save_ip();

        ip_saved[24]++;
        if((ip_saved[24])>3)   ip_saved[24]=0;
        save_ip();

        ip_saved[24]++;
        if((ip_saved[24])>3)   ip_saved[24]=0;
        save_ip();
        m_tcp_state = STATE_SOC_CLOSE;
       // APP_DEBUG("<--Restart the TCP connection process.-->\r\n");
      
        return;
    }
    p = Ql_strstr(pData,"Set_mast_DN=");//Set_mast_DN=<mast domain name>
    //APP_DEBUG("p=%s\r\n",p);
    if (p)
    {
        Ql_memset(parament.mast_dn, 0, SRVADDR_LEN);
        if (Analyse_Command(pData, 1, '>', parament.mast_dn))
        {
            APP_DEBUG("<--Server mast domain name Error->");
            return;
        }
       
        APP_DEBUG("<--Set mast_ip Successfully<%s>\r\n",parament.mast_ip);
        save_parament();
        return;
    }

    p = Ql_strstr(pData,"Set_mast_ip=");//Set_mast_DN=<mast domain name>
    //APP_DEBUG("p=%s\r\n",p);
    
    if (p)
    {
        Ql_memset(parament.mast_ip, 0, SRVADDR_LEN);
        if (Analyse_Command(pData, 1, '>', parament.mast_ip))
        {
            APP_DEBUG("<--Server mast domain name Error->");
            return;
        }
       
        APP_DEBUG("<--Set mast domain name Successfully<%s>\r\n",parament.mast_dn);
        save_parament();
        return;
    }
    

//Set_Para=<2>,<test>
    p= Ql_strstr(pData,"Set_Para=");
    if(p)
    {
        u8 temp[4];
        u8 index;

        Ql_memset(temp, 0, 4);
        if (Analyse_Command(pData, 1, '>', temp))
        {
            APP_DEBUG("<--Parameter Error.-->\r\n");
            return;
        }
        
        parament.modbus_add = Ql_atoi(temp);
         APP_DEBUG("<para.modbus_add=%d>\r\n", parament.modbus_add);
        /*
        if (Analyse_Command(pData, 2, '>', parament.data))
        {
            APP_DEBUG("<--Parameter Error.-->\r\n");
            return;
        }
        APP_DEBUG("<parament.data=%s>\r\n",parament.data)  ;     
        */
        save_parament();
          
        return;
    }


    
}
/*
static void Callback_Read_Timer(u32 timerId, void* param)
{
    s32 ret;
   
    if(timerId==READ_TIMER_ID)
    {
      //  APP_DEBUG("o->");
     //   Ql_GPIO_SetLevel(send485,PINLEVEL_HIGH);

        //if(command_len)
        {
            Ql_UART_Write(m_myUartPort, command_data, command_len);
           // command_len=0;
           // APP_DEBUG("cmd send to device");
        }
    //    else 
          
    //    Ql_UART_Write(m_myUartPort, command_read, 8);
      
      
    //    Ql_Sleep(20);
     //   Ql_GPIO_SetLevel(send485,PINLEVEL_LOW);
        
 
        
    }

}*/
static void Callback_Ntp_Timer(u32 timerId, void* param)
{
    s32 ret;
    static u8 count_1000s=0;
   
    if(timerId==NTP_TIMER_ID)
    {
        //command_len=8;
        Ql_UART_Write(m_myUartPort, command_data, 8);
        APP_DEBUG("Au10\r\n");
        if(++count_1000s>100)
        {   
            count_1000s=0;
            ret=RIL_GetNtpTime(timestr);
            if(ret==RIL_AT_SUCCESS)
            {
                APP_DEBUG("GetNtpTime OK! %s\r\n",timestr);
            }
            else APP_DEBUG("GetNtpTime Fail!\r\n");    
        }         
        
        
    }

}


//static void Callback_Timer(u32 timerId, void* param)
void proc_tcp_network(s32 taskId)
{
    static int count_conect=0;
    static unsigned char ip_err_count=0;
    static unsigned char chang_ip_count=0;
    static unsigned char cereg_count=0;
	s32 ret;

   //while(tcp_network_enable==0){};
    Ql_Sleep(6000);

   APP_DEBUG("proc_tcp_network started!\r\n");
    
    //if (TCP_TIMER_ID == timerId)
    while (1)
    {
        //APP_DEBUG("<--...........m_tcp_state=%d..................-->\r\n",m_tcp_state);
        Ql_Sleep(1000);
    
        switch (m_tcp_state)
        {        
            case STATE_NW_QUERY_STATE:
            {
                s32 cgreg = 0;
                Ql_GPIO_SetLevel(net_ok, PINLEVEL_HIGH);
                Ql_Sleep(20);
                Ql_GPIO_SetLevel(net_ok, PINLEVEL_LOW);
                ret=RIL_GetCSQ(&rssi);
                APP_DEBUG("CSQ=%d\r\n",rssi);
                ret = RIL_NW_GetEGPRSState(&cgreg);
                APP_DEBUG("<--Network State:cgreg=%d-->\r\n",cgreg);
                if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
                {
                    if(first==0)
                    {
                        m_tcp_state = STATE_SOC_OPEN;
                        cereg_count=0;
                       // Ql_GPIO_SetLevel(contron,PINLEVEL_LOW);
                    }
                }

                else
                {
                    if(++cereg_count>50)
                    {
                        cereg_count=0;
                        reboot_rf();

                    }

                } 
              
                break;
            }

            case STATE_SOC_OPEN:
            {
				socket_param_t.contextID = 1;
				socket_param_t.connectID = 0;
				socket_param_t.access_mode = SOCKET_ACCESS_MODE_BUFFER; 
                //if(chang_ip_count>3)m_SrvPort=m_SrvPort_mast;
				socket_param_t.remote_port = m_SrvPort;
                if(socket_param_t.remote_port==0)
                    socket_param_t.remote_port=m_SrvPort_mast;
				socket_param_t.local_port = 0;
				socket_param_t.service_type =SOCKET_SERVICE_TYPE_TCP;
				socket_param_t.protocol_type = SOCKET_PROTOCOL_TYPE_IPV4;
				
				socket_param_t.address = Ql_MEM_Alloc(sizeof(u8)*SRVADDR_LEN);

				if(socket_param_t.address!=NULL)
				{
					Ql_memset(socket_param_t.address,0,SRVADDR_LEN);
                    if(chang_ip_count>3)Ql_memcpy(m_SrvADDR,parament.mast_dn,sizeof(parament.mast_dn));
                    Ql_memcpy(socket_param_t.address,m_SrvADDR,sizeof(m_SrvADDR));
				}
                else APP_DEBUG("socket_recv_param_ptr.address get memmory failure\r\n");
				              
                APP_DEBUG("open %s:%d \r\n",socket_param_t.address,socket_param_t.remote_port);
                
                ret = RIL_SOC_QIOPEN(&socket_param_t);
				if(ret>=0 && ret<=4)
                {
					m_socketid = ret;
                    APP_DEBUG("<socketid=%d.>\r\n",m_socketid);
				    Ql_GPIO_SetLevel(net_ok, PINLEVEL_HIGH);
                    m_tcp_state=STATE_SOC_CONNECTED;
                    time_out_con=TIME_OUT_CON;
                    ip_err_count=0;
                    chang_ip_count=0;
                                       
                }
				else
                {
                    APP_DEBUG("<--Open server failure,error=%d,%d times>\r\n",ret,ip_err_count);
                    /*if(ret==-1)
                    {
                        reboot_rf();
                    
					}*/
                    ret = RIL_SOC_QICLOSE(0);
                    APP_DEBUG("close socket ,ret=%d\r\n",ret);
                    //m_tcp_state = STATE_SOC_CLOSE;
                    Ql_GPIO_SetLevel(net_ok, PINLEVEL_LOW);

                    if((++ip_err_count%2)==0)
                    {
                       
                        APP_DEBUG("change ip %d\r\n",chang_ip_count);
                        if(chang_ip_count<4) 
                        {
                            chang_ip_count++;
                            change_next_ip();
                        }
                        if(ip_err_count>15)
                        {
                            ip_err_count=0;
                            reboot_rf();    
                        }
                        
                    }
                    
                   
                }
			     Ql_MEM_Free(socket_param_t.address);
				 socket_param_t.address = NULL;
                break;
            }

		
            case STATE_SOC_CLOSE:
            {
            
                ret = RIL_SOC_QICLOSE(m_socketid);
                if (ret == 0)
                {
                    APP_DEBUG("<closed>\r\n");
					

                }else
                {
                    APP_DEBUG("<--closed socketid failure,error=%d.-->\r\n",ret);
                }
                m_tcp_state = STATE_TOTAL_NUM;
                Ql_GPIO_SetLevel(net_ok, PINLEVEL_LOW);
               // APP_DEBUG("<--Restart the TCP connection process.-->\r\n");
               
                break;
            }
            case STATE_SOC_CONNECTED:
            {
                //APP_DEBUG("STATE_SOC_CONNECTED\r\n");
                if(time_out_con)
                {
                    time_out_con--;

                }
                else m_tcp_state=STATE_SOC_CLOSE;
            
                break;
            }
            case STATE_TOTAL_NUM:
            {
                ++count_conect;
                break;
            }
            default:
                break;
        }
    }
}
void   reboot_rf()

{
    u32 ret; 
   // Ql_Timer_Stop(TCP_TIMER_ID);
    APP_DEBUG("reboot rf,cfun=0\r\n");
    ret = Ql_RIL_SendATCmd("AT+CFUN=0\r\n",Ql_strlen("AT+CFUN=0\r\n"),NULL,NULL,0);

    Ql_Sleep(5000);
    ret = Ql_RIL_SendATCmd("AT+CFUN=1\r\n",Ql_strlen("AT+CFUN=1\r\n"),NULL,NULL,0);
    Ql_Sleep(1000);
    APP_DEBUG("cfun=1\r\n");
    m_tcp_state = STATE_NW_QUERY_STATE;
    first=1;
   // Ql_Timer_Start(TCP_TIMER_ID, TCP_TIMER_PERIOD, TRUE);

}
void data_buf2hex(u8* data_buf,u8* data_bufhex,u16 buf_len)
{
    u16 i;
    for(i=0;i<buf_len;i++)
    {
        Ql_sprintf(data_bufhex+i+i,"%02x",data_buf[i] );
    }
    data_bufhex[i*2]=0;
}

unsigned char crc8(unsigned char *str, unsigned char len) {
    unsigned char crc = 0;
    unsigned char i, j;
    for (i = 0; i < len; i++) {
        crc ^= str[i];
        for (j = 8; j > 0; j--) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc = (crc << 1);
        }
    }
    crc &= 0x0ff;
    return crc;
}
void get_ip()
{
    Ql_SecureData_Read(1, ip_saved,25);
    //Ql_memset(ip_saved,0,25);//for test only
    print_list(ip_saved);
    
    if((ip_saved[ip_saved[24]*6]==0)&&(ip_saved[ip_saved[24]*6+1]==0)&&(ip_saved[ip_saved[24]*6+2]==0)&&(ip_saved[ip_saved[24]*6+3]==0))
        Ql_memcpy(m_SrvADDR,parament.mast_ip,Ql_strlen(parament.mast_ip));
    else {  
        Ql_sprintf(m_SrvADDR,"%d.%d.%d.%d",ip_saved[ip_saved[24]*6],ip_saved[ip_saved[24]*6+1],ip_saved[ip_saved[24]*6+2],ip_saved[ip_saved[24]*6+3]);
        Ql_sprintf(m_SrvADDR,"%d.%d.%d.%d",ip_saved[ip_saved[24]*6],ip_saved[ip_saved[24]*6+1],ip_saved[ip_saved[24]*6+2],ip_saved[ip_saved[24]*6+3]);

}
    m_SrvPort=(ip_saved[ip_saved[24]*6+4]<<8)+ip_saved[ip_saved[24]*6+5];
    
    APP_DEBUG("\r\nget_ip:%s,port:%d\r\n",m_SrvADDR,m_SrvPort);

}
//u8 *p :The string of server IP,for example ,m_SrvADDR
//u16 port:the port
//u8* ip_int:output ip and port ,4byte+2byte
void get_ip_from_string(u8 *p,u16 port,u8 * ip_int)
{
  
    ip_int[0]=Ql_atoi(p);
    p=Ql_strstr(p,".");
    p++;
    ip_int[1]=Ql_atoi(p);
    p=Ql_strstr(p,".");
    p++;
    ip_int[2]=Ql_atoi(p);
    p=Ql_strstr(p,".");
    p++;
    ip_int[3]=Ql_atoi(p);

    ip_int[4]=port>>8;
    ip_int[5]=port&0x0ff;
    //APP_DEBUG("get ip from string:%d.%d.%d.%d:%d\r\n",ip_int[0],ip_int[1],ip_int[2],ip_int[3],ip_int[4]*256+ip_int[5]);
   

}
void save_ip()
{
    u8 ip_int[6];
    get_ip_from_string(m_SrvADDR,m_SrvPort,ip_int);
    Ql_memcpy(ip_saved+ip_saved[24]*6,ip_int,6);
   
    Ql_SecureData_Store(1, ip_saved,25);
}


void change_next_ip()
{
    ip_saved[24]++;
    if((ip_saved[24])>3)   ip_saved[24]=0;

    Ql_SecureData_Store(1, ip_saved,25);
    get_ip();
    //APP_DEBUG("change to next ip:%s,port:%d,position:%d\r\n",m_SrvADDR,m_SrvPort,ip_saved[24]);
    //print_list(ip_saved);
}
void init_nvram()
{
    Ql_memset(ip_saved,0,25);
    Ql_SecureData_Store(1, ip_saved,25);

}
void save_parament()
{
    Ql_SecureData_Store(2, (u8*)&parament,sizeof(parament));
}
void restore_parament()
{
    u16 crc;
    Ql_SecureData_Read(2, (u8*)&parament,sizeof(parament));
    if(Ql_strlen(parament.mast_dn)==0) 
        {
            Ql_memcpy(parament.mast_dn,m_SrvADDR_mast,Ql_strlen(m_SrvADDR_mast));

        }
    if(Ql_strlen(parament.mast_ip)==0) 
        {
            Ql_memcpy(parament.mast_ip,m_mast_ip,Ql_strlen(m_mast_ip));

        }
    
        Ql_GPIO_SetLevel(contron,parament.para1);
        command_data[0]=parament.modbus_add;
        command_data[1]=MODBUS_WRITE;
        //0x06
        command_data[2]=0;
        command_data[3]=0x01;
        command_data[4]=0;
        command_data[5]=parament.para1;

        crc=usMBCRC16(command_data,6);
        command_data[7]=crc>>8;
        command_data[6]=crc&0x0ff;
        command_len=8;
      

    APP_DEBUG("\r\nModbus add =%d,mast_ip=%s,mast_dn=%s\r\n",parament.modbus_add,parament.mast_ip,parament.mast_dn);
   
}


u16 usMBCRC16( u8 * buf,u16 len)

{

    int i,j;
    u16 crc=0x0ffff;
    u8 ch;
    for(i=0;i<len;i++)
    {

        crc^=buf[i]&0x0ff;
        for(j=0;j<8;j++)
        {
            ch=crc&0x01;
            crc>>=1;
            if(ch)
            {
                crc^=0x0a001;
            }
        }
      //  APP_DEBUG("%02d %02x %04x|",i,buf[i],crc);
    }
    
    return crc;
}
u8 proc_cmd(u8 len,u8 * inbuf,u8 * outbuf)
{
    u8 temp_len=0;
    u8 i;
    u8 pakg_type;
    u8 ip_int[6];
    u8 temp_outbuf[128];
    u8 current_len;
    u8 * p;
    u16 port;
    u8 next_ip;
    u32 order=0;
    u16 crc;

    for(i=0;i<len;i++)
    {
        current_len=inbuf[i++];
        pakg_type=inbuf[i++];
        if((current_len==0x0cc)&&(pakg_type==0x0dd))
        {
            i+=5;
            continue;
        }
        if(current_len==0) break;
        //APP_DEBUG("pakg_len:%02x,pakg_type:%02x\r\n",current_len,pakg_type);
        switch(pakg_type)
        {
            case 0x81://time
                Ql_memcpy(temp_outbuf,inbuf+i,current_len);
                i+=(current_len-1);
                //time no used
            break;
            case 0x82://ip
                Ql_memcpy(temp_outbuf,inbuf+i,current_len);
                i+=(current_len-1);
                temp_outbuf[current_len]=0;
                p=Ql_strstr(temp_outbuf,":");
                Ql_Sleep(20);
                if(p)
                {
                    p++;
                    port=Ql_atoi(p);
                    p=Ql_strstr(temp_outbuf,":");
                    *p=0;
                    get_ip_from_string(temp_outbuf,port,ip_int);

                    insert_list(ip_saved,ip_int);
              

                }


            break;


            case 0x83://command
            
                Ql_memcpy(ack_data,inbuf+i,current_len);
                i+=(current_len-1);
                ack_len=current_len;//for ack
                order=ack_data[ack_len-4];
                order<<=8;
                order+=ack_data[ack_len-3];
                order<<=8;
                order+=ack_data[ack_len-2];
               // order<<=8;
               // order+=ack_data[ack_len-1];


                switch(order)
                {

                    case 0x05aaaaa://for modbus address
                    //is for translat to device
                        parament.modbus_add=ack_data[ack_len-1];
                        APP_DEBUG("para.modbus_add=%d\r\n", parament.modbus_add);
                        save_parament();
                    break;
                    case 0x05ccccc://for control relay
                        parament.para1=ack_data[ack_len-1];
                        save_parament();
                       
                        Ql_GPIO_SetLevel(contron,parament.para1);
                        command_data[0]=parament.modbus_add;
                        command_data[1]=MODBUS_WRITE;
                        //0x06
                        command_data[2]=0;
                        command_data[3]=0x01;
                        command_data[4]=0;
                        command_data[5]=parament.para1;

                        crc=usMBCRC16(command_data,6);
                        command_data[7]=crc>>8;
                        command_data[6]=crc&0x0ff;
                        command_len=8;
                        
                        temp_len=8;
                       
                    break;
                    case 0xddddddd:
                       /* parament.para1=0;
                        save_parament();
                        Ql_GPIO_SetLevel(contron,PINLEVEL_HIGH);
                        command_data[0]=parament.modbus_add;
                        command_data[1]=MODBUS_WRITE;
                        //0x06
                        command_data[2]=0;
                        command_data[3]=0xaa;
                        command_data[4]=0;
                        command_data[5]=0x01;

                        crc=usMBCRC16(command_data,6);
                        command_data[7]=crc>>8;
                        command_data[6]=crc&0x0ff;
                        command_len=8;
                        
                        temp_len=8;
                        */
                    break;
                    default:
                    break;


                }
                

            break;

            default:
            break;


        }

    }
    //APP_DEBUG("temp_len=%d",temp_len);
    return temp_len;
}
void print_list(u8 * ip_list)
{
    for(int i=0;i<4;i++)
    {
        APP_DEBUG("\r\nList[%d]:%d.%d.%d.%d:%d",i,ip_list[i*6],ip_list[i*6+1],ip_list[i*6+2],\
ip_list[i*6+3],ip_list[i*6+4]*256+ip_list[i*6+5]);
        if(i==ip_list[24]) APP_DEBUG(" *");
    }
    APP_DEBUG("current postion=%d\r\n",ip_list[24]);

}
unsigned char  insert_list(unsigned char *list,unsigned char *ip_new)
{

    static unsigned int count_save_ip=0;
    unsigned char i,a,b,c;
    i=list[24];
    
    if(cmp6(list+i*6,ip_new))   return 0;

    i++;
    if(i>3)i=0;
    a=i;
    if(cmp6(list+i*6,ip_new))   return 0;
    
    i++;
    if(i>3)i=0;
    b=i;
    if(cmp6(list+i*6,ip_new))   {exchange(list+i*6,list+a*6);return 0;}
    
    
    i++;
    if(i>3)i=0;
    c=i;
    if(cmp6(list+i*6,ip_new))   {exchange(list+i*6,list+b*6);return 0;}
    
    exchange(list+c*6,list+b*6);
    exchange(list+a*6,list+b*6);
    Ql_memcpy(list+a*6,ip_new,6);
    
    if(++count_save_ip>=2) {  
        Ql_SecureData_Store(1, list,25);count_save_ip=0;
        APP_DEBUG("ip_saved!\r\n")
    }
    else APP_DEBUG("find new ip %d times\r\n",count_save_ip);
    return 1;
}
unsigned char cmp6(unsigned char *in1,unsigned char *in2)
{
    unsigned char i;
    for(i=0;i<6;i++)
    {
        if((*in1)!=(*in2)) return 0;//in1!=in2
        in1++;
        in2++;
    }
    return 1;//in1=in2
}
void exchange(unsigned char *in1,unsigned char *in2)
{
    unsigned char temp[6];

    Ql_memcpy(temp,in2,6);
    Ql_memcpy(in2,in1,6);
    Ql_memcpy(in1,temp,6);
    
}
void  imsi2bcd()
{
    unsigned char bcd;
    bcd=Ql_atoi(imsi+13);
    //APP_DEBUG("imsi+13=%s,bcd=%d\r\n",imsi+13,bcd);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[7]=bcd;

    imsi[13]=0;
    bcd=Ql_atoi(imsi+11);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[6]=bcd;

      imsi[11]=0;
    bcd=Ql_atoi(imsi+9);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[5]=bcd;

      imsi[9]=0;
    bcd=Ql_atoi(imsi+7);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[4]=bcd;

      imsi[7]=0;
    bcd=Ql_atoi(imsi+5);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[3]=bcd;

      imsi[5]=0;
    bcd=Ql_atoi(imsi+3);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[2]=bcd;

      imsi[3]=0;
    bcd=Ql_atoi(imsi+1);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[1]=bcd;

      imsi[1]=0;
    bcd=Ql_atoi(imsi);
    bcd=(bcd/10)*16+bcd%10;
    imsi_bcd[0]=bcd;


    
}

void  iccid2bcd()
{

  // sscanf(iccid,"%2X%2X%2X%2X%2X%2X%2X%2X",iccid_bcd,iccid_bcd+1,iccid_bcd+2,iccid_bcd+3,iccid_bcd+4,iccid_bcd+5,iccid_bcd+6,iccid_bcd+7);
    u8 i;
    for(i=0;i<16;i++)
    //sscanf(iccid,"%2x",iccid_bcd);
    {
       // APP_DEBUG(":%x ",iccid[i]);
        if((iccid[i]>='0')&&(iccid[i]<='9'))
        {
            iccid_bcd[i/2]=(iccid[i]-'0')<<4;
        }
        else if((iccid[i]>='A')&&(iccid[i]<='F'))
        {
            iccid_bcd[i/2]=(iccid[i]-'A'+0x0a)<<4;
           // APP_DEBUG("?%x ?%x ",iccid[i],iccid_bcd[i/2]);
        }
        i++;
       // APP_DEBUG("%x ",iccid[i]);
         if((iccid[i]>='0')&&(iccid[i]<='9'))
        {
            iccid_bcd[i/2]+=(iccid[i]-'0');
        }
        else if((iccid[i]>='A')&&(iccid[i]<='F'))
        {
            iccid_bcd[i/2]+=(iccid[i]-'A'+0x0a);
        }

    }
}
#endif // __EXAMPLE_TCPCLIENT__


