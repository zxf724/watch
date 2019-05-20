#ifndef __XYMODEM_H_ 
#define __XYMODEM_H_ 

#include <Arduino.h> 
#include <inttypes.h>

#define MODEM_MAX_RETRIES   50
#define MODEM_CRC_RETRIES   51
#define MODEM_CAN_COUNT     3       //Wait for 3 times CAN before quiting 
#define MODEM_EOT_COUNT     1 
 
#define MODEM_SOH  0x01     //packet start char
#define MODEM_STX  0x02 
#define MODEM_EOT  0x04 
#define MODEM_ACK  0x06 
#define MODEM_NAK  0x15 
#define MODEM_CAN  0x18 
#define MODEM_C    0x43 

typedef struct{ 
    int           ok;
    int           modemtype;  //1: Xmodem    2:Ymodem
    int           crc_mode; 
    int           nxt_num;    //next packet number
    int           cur_num;    //current packet number
    int           len; 
    int           rec_err;
    unsigned char buf[128];  //data
    unsigned int  filelen;    //Ymodem's file name and len
    unsigned char filename[32]; 
}modem_struct; 


class xyModemClass
{
    private:
    unsigned int buf_filelen(unsigned char *ptr);

    public:
    modem_struct modem;

    void Nak()
    {
       printf("NAK\r\n");
       Uart_SendByte(MODEM_NAK);
    }
    void Ack()
    {
       printf("ACK\r\n");
       Uart_SendByte(MODEM_ACK);
    }
    
    void Uart_SendByte(unsigned char x)
    {
      Serial1.write(x); //uart1 tx
    }
    unsigned char Uart_ReadByte()
    {
      return Serial1.read();   //uart1 rx
    }
    void Uart_RxEmpty()
    {
      Serial1.flush();
    }
    unsigned char Uart_Available()
    {
      return Serial1.available();
    }

    int Uart_RecvByteTimeout(unsigned char *ch);
    int modem_init(modem_struct &mblock); 
    int modem_recvdata(modem_struct &mblock); 
    //int crc_16(unsigned char *buf, int len); 
    void modem_cancle(void);
};

#endif 
