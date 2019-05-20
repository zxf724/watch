#include <arduino.h>
#include "Vs10xx.h"
#include "Vs1053.h"
#include "Config.h"

//int VolmeCount = 50;
extern void SPI_WriteByte(unsigned char data);
extern unsigned char SPI_ReadByte();
extern void SPI_WriteBytes(unsigned char data,unsigned char len);
extern void FlashPageRead(u32 address, char *buf, unsigned int len);
extern void SPI_LowSpeed();
extern void SPI_HighSpeed();

void testPlayMP3File();

void SPIPutChar(unsigned char data)
{
  SPI_WriteByte(data);
}

unsigned char SPI_RecByte()
{ 
  unsigned char ReadData;
  ReadData = SPI_ReadByte();
  return ReadData;
}

void InitVS10xx()
{
  printf("InitVS10xx\r\n");
}

void Mp3WriteRegister(unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte)
{
  SPI_LowSpeed();
  MP3DeselectData();
  MP3SelectControl();
  SPIPutChar(VS_WRITE_COMMAND);
  SPIPutChar(addressbyte);
  SPIPutChar(highbyte);
  SPIPutChar(lowbyte);
  MP3DeselectControl();
  SPI_HighSpeed();
}

unsigned int Mp3ReadRegister(unsigned char addressbyte)
{
  unsigned int result = 0;
  unsigned char Read;

  SPI_LowSpeed();
  MP3DeselectData();
  MP3SelectControl();
  SPIPutChar(VS_READ_COMMAND);
  SPIPutChar(addressbyte);
  Read = SPI_RecByte();
  result = Read << 8;
  Read = SPI_RecByte();
  result |= Read;
  MP3DeselectControl();
  SPI_HighSpeed();
  
  return result;
}

void Mp3SetVolme(u_int8 leftchannel, u_int8 rightchannel)
{
  printf("Mp3SetVolme begin\r\n");
  Mp3WriteRegister(SCI_VOL, leftchannel, rightchannel);
  printf("Mp3SetVolme Left=%d,Right=%d\r\n",leftchannel,rightchannel);
}

void Mp3SoftReset()
{
  printf("Mp3SoftReset start \r\n");
  while(MP3_DREQ() == LOW);
  Mp3WriteRegister(SCI_MODE, 0x08, 0x04); //soft reset
  delay(2);
  while(MP3_DREQ() == LOW); //wait reset end
  Mp3WriteRegister(SCI_CLOCKF,0X98,0X00); //clk x 3
  delay(20);
  Mp3WriteRegister(SCI_AUDATA, 0XBB,0X81); //48K
  Mp3WriteRegister(SCI_BASS,0X00,0X55); //BASS
  Mp3SetVolme(0,0);
  printf("Mp3SoftReset end \r\n");
}

void Mp3Reset()
{
  printf("Mp3 HW Reset start\r\n");
  MP3PutInReset();
  delay(20);
  MP3DeselectData();
  MP3DeselectControl();
  MP3ReleaseFromReset();
  while(MP3_DREQ() == LOW);
  printf("Mp3 HW Reset end\r\n");
  delay(2);
  Mp3SetVolme(50,50);
  Mp3SoftReset();
}

bool CheckVS10XX_DRQ()
{
  return ((bool)MP3_DREQ());
}

void VsSineTest()
{
  printf("Mp3 VsSineTest start\r\n");
  MP3PutInReset();
  delay(200);
  SPIPutChar(0xff);
  MP3DeselectControl();
  MP3DeselectData();
  MP3ReleaseFromReset();
  delay(200);
  Mp3SetVolme(32,32);

  Mp3WriteRegister(SCI_MODE,0x08,0x20); //test mode
  while(MP3_DREQ() == LOW);

  MP3SelectData();
  SPI_LowSpeed();
  SPIPutChar(0x53);
  SPIPutChar(0xef);
  SPIPutChar(0x6e);
  SPIPutChar(0x24); //freq
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  delay(500);
  MP3DeselectData();

  MP3SelectData();
  SPIPutChar(0x45);
  SPIPutChar(0x78);
  SPIPutChar(0x69);
  SPIPutChar(0x74);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  SPIPutChar(0x00);
  delay(500);
  MP3DeselectData();
  printf("Mp3 VsSineTest end\r\n");
}

void VS10xx_WriteCMD(unsigned char addr,unsigned int data)
{
  Mp3WriteRegister(addr,data>>8,data);
}

unsigned int VS10xx_ReadCMD(unsigned char addr)
{
  return(Mp3ReadRegister(addr));
}

void VS10xx_WriteDAT(unsigned char data)
{
  MP3SelectData();
  SPIPutChar(data);
  MP3DeselectData();
  MP3DeselectControl();
}

void VS10xx_SetVirtualSurroundOn()
{
  unsigned char repeat;
  unsigned int Mode;

  repeat = 0;

  while(1)
  {
    Mode = VS10xx_ReadCMD(0x00);
    if(Mode & 0x0001)
    {
      break;
    }
    else
    {
      Mode |= 0x0001;
      VS10xx_WriteCMD(0,Mode);
    }

    repeat++;

    if(repeat >10) 
      break;
  }
}

void VS10xx_SetVirtualSurroundOff()
{
  unsigned char repeat;
  unsigned int Mode;

  repeat = 0;

  while(1)
  {
    Mode = VS10xx_ReadCMD(0x00);
    if(Mode & 0x0001)
    {
      break;
    }
    else
    {
      Mode |= 0x0001;
      VS10xx_WriteCMD(0,Mode);
    }

    repeat++;

    if(repeat >10) 
      break;
  }
}

void VS10xx_SetBassEnhance(unsigned char value,  unsigned char FreqID)
{
  unsigned char repeat;
  unsigned int WriteValue;
  unsigned int ReadValue;

  repeat = 0;
  WriteValue = VS10xx_ReadCMD(0x02);

  WriteValue &= 0Xff00;
  WriteValue |= value << 4;
  WriteValue &= (FreqID & 0x0f);

  while(1)
  {
    VS10xx_WriteCMD(2,WriteValue);
    ReadValue = VS10xx_ReadCMD(0x02);

    if(ReadValue == WriteValue)
    {
       break;
    }
    
    repeat++;
    if(repeat > 10) 
      break;

  }
}

void TestVS10xx()
{
  Mp3Reset();
  VsSineTest();
  Mp3SoftReset();
  testPlayMP3File();
}

unsigned char VS10xx_Init()
{
  unsigned char retry;

  printf("VS10xx_Init\r\n");
  InitVS10xx();
  Mp3Reset();
  
  retry = 0;
  while(VS10xx_ReadCMD(0x00) != 0x0800)
  {
    VS10xx_WriteCMD(0x00, 0x0800);
    if(retry++ > 10) break;
  }

  retry = 0;
  while(VS10xx_ReadCMD(0x03) != 0x9800)
  {
    VS10xx_WriteCMD(0x03, 0x9800);
    if(retry++ > 10) break;
  }

  retry = 0;
  while(VS10xx_ReadCMD(0x0b) != 50)
  {
    VS10xx_WriteCMD(0x0b, 50);
    if(retry++ > 10) break;
  }

  if(retry > 10) return 1;
  return 0;
}

void VS10xx_ClearRam()
{
  unsigned char i,j;

  printf("VS10xx_ClearRam start\r\n");
  for(i = 0; i<64; i++)
  {
    if(CheckVS10XX_DRQ())
    {
      MP3SelectData();
      for(j = 0; j < 32; j++)
      {
        VS10xx_WriteDAT(0x00);
      }
      MP3DeselectData();
    }
  }

  printf("VS10xx_ClearRam end\r\n");
}

void testPlayMP3File()
{
  unsigned char i,j;
  unsigned char buf[256];
  unsigned long data_pos = 0; 
  unsigned long FileSize = 149567; //149567; //music file size; //360960
  printf("Play MP3 start\r\n");
  FlashPageRead(MUSIC_FILE_FlASH_ADDRESS + data_pos, buf, 16);

  SPI_HighSpeed();
  while(data_pos < (MUSIC_FILE_FlASH_ADDRESS + FileSize))
  {
      MP3DeselectData();
      MP3DeselectControl();
      FlashPageRead(MUSIC_FILE_FlASH_ADDRESS + data_pos, buf, 256);
      data_pos = data_pos + 256;
      
      for(i = 0; i < 8; i++) // (256/32)=8
      {
         while(CheckVS10XX_DRQ()==LOW);
         MP3SelectData();
         for(j = 0; j<32; j++)
         {
           SPIPutChar(buf[i*32+j]);
         }
         MP3DeselectData();
      }
  }
  VS10xx_ClearRam();

  printf("Play MP3 end\r\n");
}
