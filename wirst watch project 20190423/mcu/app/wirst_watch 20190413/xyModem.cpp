#include <arduino.h>
#include "xyModem.h"   

   
unsigned int buf_filelen(unsigned char *ptr);   

#define UART_RX_TIME_OUT     3000  //Time out after 3 Second
modem_struct  xymodem;


//return 0: success,  else timeout
int xyModemClass::Uart_RecvByteTimeout(unsigned char *ch)
{
    int time = UART_RX_TIME_OUT;

    while(--time)
    {
      if(Uart_Available() > 0)
      {
        *ch = Uart_ReadByte();
        return 0;
      }
      
      delay(1); //1ms
    }

    return 1;
}
  
int xyModemClass::modem_init(modem_struct &mblock)   
{   
    int stat;   
    int max_tries = MODEM_MAX_RETRIES;   
    int crc_tries =MODEM_CRC_RETRIES;   
    unsigned char *bufptr = mblock.buf;   
    unsigned char *namptr = mblock.filename;   

    mblock.nxt_num = 1;   //packet number start at 1
    mblock.modemtype = 1; //1:Select Xmodem   2: Ymodem
    mblock.rec_err = 0;   
    mblock.crc_mode = 0;  //Xmodem:128 byte muset be 0, 1K STX xmodem must be 1

    Uart_RxEmpty();
	
    while (max_tries-- > 0)   
    {      
        stat = modem_recvdata(mblock);   
        if (0 == stat)
        {   
            #if 0
            //file name   
            while (*bufptr != '\0')   
            {   
                *namptr++ = *bufptr++;   
            }   
            *namptr = '\0';   
            bufptr++;   
            while (*bufptr == ' ')   
            {   
                bufptr++;   
            }   
            //file length   
            mblock->filelen = buf_filelen(bufptr);   
            //other data;   
            //Uart_SendByte(MODEM_ACK); 
            #endif  
            return  0;   
        }   
        else if (2 == stat)
        {   
            return 2;   
        }   
        else if (-3 == stat)   
        {   
            if (mblock.cur_num == 1)   
            {   
                mblock.modemtype = 1;   
                mblock.nxt_num = 2;   
                return 1;   
            }   
        }   
        else
        {   
            if (crc_tries-- <= 0)   
            {   
                crc_tries = MODEM_CRC_RETRIES;   
                mblock.crc_mode = (mblock.crc_mode+1) & 1;   
            }   
        }   
    }   
    return -1;   
}   

/* 
int modem_recvdata
//return : 
         0:success
         -1:receive time out
         -2:packet error
         -3:packet number error
          1:message end
          2:cancel send
 */
int xyModemClass::modem_recvdata(modem_struct &mblock)   
{   
    int stat, hd_found=0, i;   
    int can_counter=0, eot_counter=0;   
    unsigned char *in_ptr = mblock.buf;   
    int cksum;   
    unsigned char ch, blk, cblk, crch, crcl;   

    //Uart_RxEmpty(); 
    while (!hd_found)
    {   
        stat = Uart_RecvByteTimeout(&ch);   
        if (stat == 0)   
        {   
            switch (ch)   
            {   
                case MODEM_SOH :   
                    hd_found = 1;   
                    mblock.len = 128;  
			      //printf("found head SOH 128 byte\r\n");
                    break;   
                case MODEM_STX :   
                    hd_found = 1;   
                    mblock.len = 1024;  
			      //printf("found head STX 1024 byte\r\n");
                    break;   
                case MODEM_CAN :   
                    if ((++can_counter) >= MODEM_CAN_COUNT)   
                    { 
                        return 2;   
                    }   
                    break;   
                case MODEM_EOT :   
			       printf("found head EOT  end recieve\r\n");
                    if ((++eot_counter) >= MODEM_EOT_COUNT)   
                    {   
                        Uart_SendByte(MODEM_ACK);   
                        if (mblock.modemtype == 2)
                        {   
                            Uart_SendByte(MODEM_C);
                            Uart_SendByte(MODEM_ACK);   
                            Uart_SendByte(MODEM_C);   
                            modem_cancle();        
                        }
                        return 1;   
                    }   
                    break;   
                default:   
                    break;   
            }   
        }   
        else   
        {   
            return -1;   
        }   
    }

    stat = Uart_RecvByteTimeout(&blk); 
    if (stat != 0)   
    {   
        return -1;   
    }   
   
    stat = Uart_RecvByteTimeout(&cblk);
    if (stat != 0)   
    {   
       return -1;   
    }
    
    for (i=0; i < mblock.len ; i++)   
    {   
        stat = Uart_RecvByteTimeout(in_ptr++);   
        if (stat != 0)   
        {   
            return -1;   
        }   
    }   

    stat = Uart_RecvByteTimeout(&crch);         //CRC   
    if (stat != 0)   
    {   
        return -1;   
    }   
       
    if (mblock.crc_mode)   
    {   
        stat = Uart_RecvByteTimeout(&crcl);                
        if (stat != 0)   
        { 
            printf("receive crc err\r\n");
            return -1;   
        }   
    }
	
    ch = blk^cblk;
    if(ch != 0xff)
    {   
        printf("pkt num err = %02x,~pk=%02x\r\n",blk,cblk);
        return (-2);   
    }   

    if (mblock.crc_mode)   
    {   
        in_ptr = mblock.buf;   
        cksum = 0;   
           
        for (stat=mblock.len ; stat>0; stat--)   
        {   
            cksum = cksum^(int)(*in_ptr++) << 8;   
            for (i=8; i!=0; i--)   
            {   
                if (cksum & 0x8000)   
                    cksum = cksum << 1 ^ 0x1021;   
                else   
                    cksum = cksum << 1;   
            }   
        }   
        cksum &= 0xffff;   
           
        if (cksum != (crch<<8 | crcl))   
        {   
            mblock.rec_err = 1;  
            printf("check crc err\r\n");
            return (-2);   
        }   
    }   
    else   
    {   
        cksum = 0;
        for (i=0; i<mblock.len; i++)
        {   
            cksum += mblock.buf[i];   
        }   
        if ((cksum&0xff)!=crch)   
        {   
            mblock.rec_err = 1;  
            printf("checksum err\r\n");
	          printf("cksum calcu=%02x, cksum Recv=%02x\r\n", (cksum&0xff),crch);
            return (-2);
        }   
    }   

    mblock.cur_num = blk;
    //printf("cur=%d,nxt=%d\r\n",mblock.cur_num,mblock.nxt_num);
    if (blk != mblock.nxt_num)
    {    
        printf("packet num err cur=%d,nxt=%d\r\n",mblock.cur_num,mblock.nxt_num);
        return (-3);   
    }   
    if(mblock.nxt_num == 255)
    {
      mblock.nxt_num = 0;
    }
    else
    {
      mblock.nxt_num++; 		
    }
    mblock.rec_err = 0;
    //printf("mblock->len:%d\r\n",mblock->len);
    return 0;   
}   
   
unsigned int xyModemClass::buf_filelen(unsigned char *ptr)   
{   
    int datatype=10, result=0;   
   
    if (ptr[0]=='0' && (ptr[1]=='x' && ptr[1]=='X'))   
    {   
        datatype = 16;   
        ptr += 2;   
    }   
   
    for ( ; *ptr!='\0'; ptr++)   
    {   
        if (*ptr>= '0' && *ptr<='9')   
        {   
            result =result*datatype+*ptr-'0';   
        }   
        else   
        {   
            if (datatype == 10)   
            {   
                return result;   
            }   
            else   
            {   
                if (*ptr>='A' && *ptr<='F')   
                {   
                    result = result*16 + *ptr-55;             //55 = 'A'-10   
                }   
                else if (*ptr>='a' && *ptr<='f')   
                {   
                    result = result*16 + *ptr-87;             //87 = 'a'-10   
                }   
                else   
                {   
                    return result;   
                }   
            }   
        }   
    }   
    return result;   
}   
   
   
void xyModemClass::modem_cancle(void)   
{   
    Uart_SendByte(0x18);   
    Uart_SendByte(0x18);   
    Uart_SendByte(0x18);   
    Uart_SendByte(0x18);   
    Uart_SendByte(0x18);   
}  
