/*********************************************************
  wirst watch
  Atmega2560 3.3V 8MHz
  
*********************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Vs1053.c"
#include "lis3dh_driver.c"
#include "xyModem.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Config.h"
#include "lis3dh_driver.h"
#include "Low_Filter.h"
#include "ITG3200.h"
#include "HMC5883.h"
#include "AT8307.c"

// constants won't change. They're used here to set pin numbers:
const int Key_center = 29; //PA7
const int Key_left = 28;  //PA6
const int Key_right = 27; //PA5
const int SQW = 70;       //PD4
const int Vibrate = 71;   //PD5
const int ledPin =  13;    //LED pin
const int Power_Hold = 26; //PA4 Power Hold Ctrl 
const int AMPOFF = 12; //PB6 amplifier on/off and AB/D swtich
const int CS_SST = 10; //PB4
const int GPS_PWR = 11; //PB5
const int Heart_Out = 30; //PC7
const int MCU_I2C_SDA = 20;// PD 1 ** 20 ** I2C_SDA  
const int MCU_I2C_SCL = 21;// PD 0 ** 21 ** I2C_SCL  

#define SENSOR_DATA_SIZE  20  //sensor data
  
#define GPS_POWER_ON()  {digitalWrite(GPS_PWR, LOW);}
#define GPS_POWER_OFF()  {digitalWrite(GPS_PWR, HIGH);}

#define LED_ON {digitalWrite(ledPin, HIGH);}
#define LED_OFF {digitalWrite(ledPin, LOW);}

#define FlashSelectControl()    { digitalWrite(CS_SST, LOW); }
#define FlashDeselectControl()  { digitalWrite(CS_SST, HIGH); }

volatile u8 State_Receive, State_Num=IDLE_STATE;  // State Machine
u8 Send_To_PC[SENSOR_DATA_SIZE], receive_tmp[SENSOR_DATA_SIZE];
u8 *Data_Ptr;         
s16 Nine_Axis_Data[MAX_DIMENTION], Filtered_Data[MAX_DIMENTION], Int_Data_Set[MAX_DIMENTION][B_LENGTH];
Configuration_type Config;

Error_type HMC5883_Init(Configuration_type config);
Error_type ITG3200_Init(Configuration_type config);
Error_type LIS3DH_Init(Configuration_type config);
Bool Write_Configuration(Configuration_type config);
Error_type Check_Configuration(Configuration_type config);

void WriteI2C(u8 data, u8 RegAddress, u8 DeviceAddress)
{
  //printf("WriteI2C() Begin\r\n");
  //printf("Wire.beginTransmission(DeviceAddress=0x%02X)\r\n", DeviceAddress);
  Wire.beginTransmission(DeviceAddress >> 1); //DeviceAddress 7bit
  
  //printf("Wire.write(RegAddress)=0x%02X\r\n", RegAddress);
  Wire.write(RegAddress);

  //printf("Wire.write(data)=0x%02X\r\n", data);
  Wire.write(data);
  
  //printf("Wire.endTransmission()\r\n");
  Wire.endTransmission();

  //printf("WriteI2C() End\r\n");
}

u8 ReadI2C(u8 RegAddress, u8 DeviceAddress)
{
  u8 data;
  
  //printf("ReadI2C() Begin\r\n");
  //printf("DeviceAddress=0x%02X\r\n",DeviceAddress);
  Wire.beginTransmission(DeviceAddress >> 1);  //DeviceAddress 7bit
  
  //printf("Wire.write(RegAddress)=0x%02X\r\n",RegAddress);
  Wire.write(RegAddress);
  
  //printf(" Wire.endTransmission\r\n");
  Wire.endTransmission();

  //printf("Wire.requestFrom(DeviceAddress,1)\r\n"); //DeviceAddress 7bit
  Wire.requestFrom(DeviceAddress >> 1,1);
  if(Wire.available() >= 1)
  {
    data = Wire.read();    
  }

  //printf("ReadI2C() End\r\n");
  
  return data;  
}

void SPI_LowSpeed()
{
  SPI.setClockDivider(SPI_CLOCK_DIV16); //SPI CLK speed 8/16 = 500Khz
}

void SPI_HighSpeed()
{
  SPI.setClockDivider(SPI_CLOCK_DIV2); //SPI CLK speed 8/2 = 4Mhz
}

void SPI_WriteBytes(unsigned char data,unsigned char len)
{
  SPI.transfer(data,len);
}

void SPI_WriteByte(unsigned char data)
{
  SPI.transfer(data);
}

unsigned char SPI_ReadByte()
{
  unsigned char data;
  data = SPI.transfer(0xff);
  return (data);
}

void PowerOn()
{
  int sec = 0;
  unsigned char ledblink = 0;

  digitalWrite(Power_Hold, LOW); //power off
  do
  {
    if (digitalRead(Key_center) == LOW)  
    {
       sec++;
       ledblink = ~ledblink;
       digitalWrite(ledPin, ledblink);
    }
    else
    {
      sec = 0;
    }
    delay(100);
   }while(sec < 30);  //wait 3 seconds

   digitalWrite(ledPin, HIGH);
   digitalWrite(Power_Hold, HIGH); //power on
   while(digitalRead(Key_center) == LOW); //not press
   delay(1000);
}

void PowerOff()
{
  static int sec = 0;
  char ledblink = 0;
  
  do
  {
    if (digitalRead(Key_center) == LOW)  
    {
       if(sec < 30)
       {
         sec++;
         ledblink = ~ledblink;
         digitalWrite(ledPin, ledblink);
       }
    }
    else
    {
      sec = 0;
      digitalWrite(ledPin, HIGH); //power off fail,stay led on  
      return;
    }
    delay(100);
   }while(sec < 30);  //wait 3 seconds
   
   digitalWrite(ledPin, LOW);
   delay(2000); 
   digitalWrite(Power_Hold, LOW); //power off
   delay(200); 
}

void Aplifier_AB_Mode()
{
  digitalWrite(AMPOFF, LOW);
  delayMicroseconds(5);
  digitalWrite(AMPOFF, HIGH);
}

void Aplifier_D_Mode()
{
  digitalWrite(AMPOFF, LOW);
  delayMicroseconds(5);
  digitalWrite(AMPOFF, HIGH);
  delayMicroseconds(5);
  digitalWrite(AMPOFF, LOW);
  delayMicroseconds(5);
  digitalWrite(AMPOFF, HIGH); 
}

void Aplifier_OFF_Mode()
{
  digitalWrite(AMPOFF, HIGH);
  delayMicroseconds(120);
  digitalWrite(AMPOFF, LOW);
  delayMicroseconds(120);
}

/***********************************FLASH*******************************/


int serial_putc(unsigned char c, struct __file*)
{
  Serial.write(c);  //uart0
  return c;  
}

void printf_begin()
{
  fdevopen(&serial_putc,0);  
}

void FlashWriteEnable()
{
  FlashSelectControl();
  SPI.transfer(0x06);
  FlashDeselectControl();
}

void FlashWriteDisable()
{
  FlashSelectControl();
  SPI.transfer(0x04);
  FlashDeselectControl();
}

unsigned char  FlashReadSR()
{
    unsigned char data = 0;
    FlashSelectControl();
    SPI.transfer(0x05);
    data = SPI.transfer(0xff);
    FlashDeselectControl();

    return data;
}

void FlashWaitBusy()
{
  while((FlashReadSR() & 0x01) == 0x01);
}

void FlashPageRead(u32 address, char *buf, unsigned int len)
{
  address &= 0x00ffffff; //24bit address
  FlashSelectControl();
  SPI.transfer(0x03);
  SPI.transfer(address >> 16);
  SPI.transfer(address >> 8);
  SPI.transfer(address & 0xff);
  for(int i = 0; i < len; i++)
  {
    *buf++ = SPI.transfer(0xff);
  }
  FlashDeselectControl();
}

void FlashPageWrite(u32 address, char *buf, unsigned int len)
{
  if(len > 256)  //max 256 bytes
    return;

  address &= 0x00ffffff; //24bit address
  FlashSelectControl();
  SPI.transfer(0x02);
  SPI.transfer((char)(address >> 16));
  SPI.transfer((char)(address >> 8));
  SPI.transfer((char)(address & 0xff));
  for(int i = 0; i < len; i++)
  {
    SPI.transfer(*buf++);
  }
  FlashDeselectControl();
  FlashWaitBusy();
}

void FlashReadID()
{
  FlashSelectControl();
  SPI.transfer(0xAB);//read device id cmd
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  byte id = SPI.transfer(0xFF);
  if(id == 0x16)
    printf("\r\nFlash 25Q64 Device ID:%02xh\r\n", id);
  else if(id == 0x17)
    printf("\r\nFlash 25Q128 Device ID:%02xh\r\n", id);  
  else
    printf("Flash Read ID Fail! ID:%02xh\r\n",id);
  FlashDeselectControl();
}

void W25QXX_Erase_Sector(u32 Dst_Addr)
{
  Dst_Addr*=4096;
  FlashWriteEnable();
  FlashWaitBusy();
  FlashSelectControl();
  SPI.transfer(0x20); //sector erase cmd
  SPI.transfer((u8)((Dst_Addr)>>16));
  SPI.transfer((u8)((Dst_Addr)>>8));
  SPI.transfer((u8)Dst_Addr & 0xff);
  FlashDeselectControl();
  FlashWaitBusy();
}

void Flash_Erase_Chip()
{
  printf("Flash Erase Chip\r\n");
  printf("Please wait...\r\n");
  FlashWriteEnable();
  FlashWaitBusy();
  FlashSelectControl();
  SPI.transfer(0xC7);
  FlashDeselectControl();
  FlashWaitBusy();
  FlashWriteDisable();
  printf("Flash Erase Chip OK\r\n");
}

void FlashTest()
{
  char buf[] = "0123456789ABCDEF";
  
  FlashReadID();
  printf("Flash Write: 0123456789ABCDEF\r\n");
  W25QXX_Erase_Sector(0);
  FlashWriteEnable();
  FlashPageWrite(0x0,buf,16);
  FlashWriteDisable();
  
  memset(buf, 0x0, 16);
  FlashPageRead(0, buf, 16);
  
  printf("Flash Read:");
  for(int i = 0; i < 16; i++)
    printf("%c", buf[i]);
  printf("\r\n");  
}

Error_type Check_Configuration(Configuration_type config)
{
  if(!(config.SAMPLE_RATE > 0))
    return SAMPLE_RATE_ERROR;
  if(!(config.ACC_RANGE > 0))
    return ACC_RANGE_ERROR;
  if(!(config.ACC_ODR > 0))
    return ACC_ODR_ERROR;
  if(!(config.GYR_RANGE > 0))
    return GYR_RANGE_ERROR;
  if(!(config.GYR_ODR > 0))
    return GYR_ODR_ERROR;
  if(!(config.MAG_RANGE > 0))
    return MAG_RANGE_ERROR;
  if(!(config.MAG_ODR > 0))
    return MAG_ODR_ERROR;
  return NO_ERROR;
}

Error_type HMC5883_Init(Configuration_type config)
{
  u8 mag_gain;

// Configuration Register A: Data outout rate and Measurement mode
  if(config.MAG_ODR != 75)
    return MAG_ODR_ERROR;   
  WriteI2C(Data_Out_Rate | MAG_Measurement, Config_Reg_A, MAGWriteAddress);

// Configuration Register B: Device gain
  switch(config.MAG_RANGE)
  {
    case 900:
      mag_gain = MAG_Gain_900;
      break;
    case 1200:
      mag_gain = MAG_Gain_1200;
      break;
    case 1900:
      mag_gain = MAG_Gain_1900;
      break;
    case 2500:
      mag_gain = MAG_Gain_2500;
      break;
    case 4000:
      mag_gain = MAG_Gain_4000;
      break;
    case 4600:
      mag_gain = MAG_Gain_4600;
      break;
    case 5500:
      mag_gain = MAG_Gain_5500;
      break;
    case 7900:
      mag_gain = MAG_Gain_7900;
      break;
    default: return MAG_RANGE_ERROR;
  }
  WriteI2C(mag_gain, Config_Reg_B, MAGWriteAddress);

// Configuration Mode Register
  WriteI2C(MAG_Mode_Single, Mode_Register, MAGWriteAddress);

  return NO_ERROR;
}

Error_type ITG3200_Init(Configuration_type config)
{
  s16 sample_rate_divider;

// Sample Rate: 10分频——1kHz/(Sample_Rate_Divider+1)
  sample_rate_divider = 1000 / config.GYR_ODR - 1;
  if((sample_rate_divider < 0) || (sample_rate_divider > 255))
    return GYR_ODR_ERROR;
  WriteI2C((u8)sample_rate_divider, SMPLRT_DIV, GYRWriteAddress);

  // Full Scale & Digital Low Pass Filter
  if(config.GYR_RANGE != 2000)
    return GYR_RANGE_ERROR;
  WriteI2C(FS_SEL | DLPF_CFG, DLPF_FS, GYRWriteAddress);

  return NO_ERROR;
}

Error_type LIS3DH_Init(Configuration_type config)
{   
  u8 response;

// Set ODR (turn ON device)
  switch(config.ACC_ODR)
  {
    case 50:
      response = SetODR(ODR_50Hz);
      break;
    case 100:
      response = SetODR(ODR_100Hz);
      break;
    case 200:
      response = SetODR(ODR_200Hz);
      break;
    case 400:
      response = SetODR(ODR_400Hz);
      break;
    default: return ACC_ODR_ERROR;
  }
  if(response == MEMS_ERROR) return CONFIG_ERROR_ACC;

// Set PowerMode 
  response = SetMode(NORMAL);
  if(response == MEMS_ERROR) return CONFIG_ERROR_ACC;

// Set Fullscale
  // FULLSCALE_2    1mg/digit
  // FULLSCALE_4    2mg/digit
  // FULLSCALE_8    4mg/digit
  // FULLSCALE_16   12mg/digit
  switch(config.ACC_RANGE)
  {
    case 2000:
      response = SetFullScale(FULLSCALE_2);
      break;
    case 4000:
      response = SetFullScale(FULLSCALE_4);
      break;
    case 8000:
      response = SetFullScale(FULLSCALE_8);
      break;
    case 16000:
      response = SetFullScale(FULLSCALE_16);
      break;
    default: return ACC_RANGE_ERROR;
  }
  if(response == MEMS_ERROR) return CONFIG_ERROR_ACC;

// Using the block data update (BDU) feature
  response = SetBDU(MEMS_ENABLE);
  if(response == MEMS_ERROR) return CONFIG_ERROR_ACC;

// Big-little endian selection
  response = SetBLE(BLE_LSB);
  if(response == MEMS_ERROR) return CONFIG_ERROR_ACC;

// Set axis Enable
  response = SetAxis(X_ENABLE | Y_ENABLE | Z_ENABLE);
  if(response == MEMS_ERROR) return CONFIG_ERROR_ACC;

  return NO_ERROR;
}

void Print_Error(Error_type err)
{
  if(err != NO_ERROR)
    while(1)
    {
      LED_ON;
      switch(err)
      {
        case CONFIG_ERROR_CHK:
          printf("CONFIG_ERROR_CHK!");
          break;
        case CONFIG_ERROR_TIM:
          printf("CONFIG_ERROR_TIM!");
          break;
        case CONFIG_ERROR_ACC:
          printf("CONFIG_ERROR_ACC!");
          break;
        case CONFIG_ERROR_GYR:
          printf("CONFIG_ERROR_GYR!");
          break;
        case CONFIG_ERROR_MAG:
          printf("CONFIG_ERROR_MAG!");
          break;
        case FLASH_ERROR_READ:
          printf("Flash read error! Please reconfig with config.txt.");
          break;
        case FLASH_ERROR_WRITE:
          printf("Flash write error! Please reconfig with config.txt.");
          break;
        case SAMPLE_RATE_ERROR:
          printf("SAMPLE_RATE_ERROR!");
          break;
        case ACC_RANGE_ERROR:
          printf("ACC_RANGE_ERROR!");
          break;
        case ACC_ODR_ERROR:
          printf("ACC_ODR_ERROR!");
          break;        
        case GYR_RANGE_ERROR:
          printf("GYR_RANGE_ERROR!");
          break;
        case GYR_ODR_ERROR:
          printf("GYR_ODR_ERROR!");
          break;        
        case MAG_RANGE_ERROR:
          printf("MAG_RANGE_ERROR!");
          break;
        case MAG_ODR_ERROR:
          printf("MAG_ODR_ERROR!");
          break;
        default: printf("Unknown error!!!");
      }
      printf("\r\n");
      if((err >= SAMPLE_RATE_ERROR) && (err <= MAG_ODR_ERROR))
        printf("Configuration is invalid! Please enter 'C' to reconfig with config.txt.\r\n");
      delay(50);
      LED_OFF;
      delay(950);
    }
}

void Collect_Data(s16* data)
{
  //Acc data 6 Bytes
  data[0] = ReadI2C(ACC_YOUT_H, ACCWriteAddress) << 8;
  data[0]|= ReadI2C(ACC_YOUT_L, ACCWriteAddress); //X and Y axes are exchanged
  data[0] >>= 4;//The last four digits of the sensor are always 0. What you need to do here is right alignment, so that a bit change represents 4mg.
  data[1] = ReadI2C(ACC_XOUT_H, ACCWriteAddress) << 8;
  data[1]|= ReadI2C(ACC_XOUT_L, ACCWriteAddress); // X and Y axes are exchanged
  data[1] = -data[1];
  data[1] >>= 4;//The last four digits of the sensor are always 0. What you need to do here is right alignment, so that a bit change represents 4mg.
  data[2] = ReadI2C(ACC_ZOUT_H, ACCWriteAddress) << 8;
  data[2]|= ReadI2C(ACC_ZOUT_L, ACCWriteAddress); // Z axes no change
  data[2] >>= 4;//The last four digits of the sensor are always 0. What you need to do here is right alignment, so that a bit change represents 4mg.
  data[2] = -data[2];

  //Gyro data 6 Bytes
  data[3] = ReadI2C(GYRO_XOUT_H, GYRWriteAddress) << 8;
  data[3]|= ReadI2C(GYRO_XOUT_L, GYRWriteAddress);
  data[4] = ReadI2C(GYRO_YOUT_H, GYRWriteAddress) << 8;
  data[4]|= ReadI2C(GYRO_YOUT_L, GYRWriteAddress);
  data[5] = ReadI2C(GYRO_ZOUT_H, GYRWriteAddress) << 8;
  data[5]|= ReadI2C(GYRO_ZOUT_L, GYRWriteAddress);
    
  //Mag data 6 Bytes
  data[6] = ReadI2C(MAG_XOUT_H, MAGWriteAddress) << 8;
  data[6]|= ReadI2C(MAG_XOUT_L, MAGWriteAddress);
  data[7] = ReadI2C(MAG_YOUT_H, MAGWriteAddress) << 8;
  data[7]|= ReadI2C(MAG_YOUT_L, MAGWriteAddress);
  data[8] = ReadI2C(MAG_ZOUT_H, MAGWriteAddress) << 8;
  data[8]|= ReadI2C(MAG_ZOUT_L, MAGWriteAddress);

  // mag start next sample
  WriteI2C(MAG_Mode_Single, Mode_Register, MAGWriteAddress);
}

u8 Cal_Checksum(u8 *buf, u16 size)
{
  u16 cksum = 0;
  for(; size > 0; size--)
    cksum += *buf++;
  cksum = (cksum>>8) + (cksum&0xFF);
  cksum += (cksum>>8);
  return ~cksum;
}

void Preparation(u8 node_address, u8* ptr, u8* data)
{
  u8 n;

  for(n = 2; n < 20; n += 2, ptr += 2)
  {
    data[n] = *(ptr+1);
    data[n+1] = *ptr;
  }
  data[0] = node_address;
  data[1] = 0;
  data[1] = Cal_Checksum(data, SENSOR_DATA_SIZE); //calcu chksum，packet size is 20bytes
}

void Send_To_BLE(u8 * data, u16 len)
{
  for(int i = 0; i < len; i++)
    printf("0x%04X ",data[i]);  
}

 void IMU_Test(void)
{
#if ENABLE_AVR_FILTER || ENABLE_FIR_FILTER
 u8 i;
#endif
  
  LED_ON;
  ///////////////////////////////////////////////////////////////
  // 采集9轴数据，共18字节
  Collect_Data(Nine_Axis_Data);

#if ENABLE_FABRICATE
  Data_Fabricate(Nine_Axis_Data);
#endif
  
  Data_Ptr = (u8*)Nine_Axis_Data; 
  Preparation(NODE_ADDRESS, Data_Ptr, Send_To_PC);                  
  //send To BLE
  printf("\r\n\r\n\r\n\r\nIMU data:");
  Send_To_BLE(Send_To_PC, SENSOR_DATA_SIZE);
  
// Digital Filter
  #if ENABLE_AVR_FILTER || ENABLE_FIR_FILTER
    if(TIM2_First_INT)
    {
      TIM2_First_INT--;
      for(i = 0; i < B_LENGTH; i++)
        Filter_Initial(Int_Data_Set, Nine_Axis_Data, MAX_DIMENTION, i);
    }
  #endif

  #if ENABLE_AVR_FILTER
    ///////////////////////////////////////////////////////////////
    // Average Filter
    Average_Filtering(Int_Data_Set, Filtered_Data, MAX_DIMENTION);
    ///////////////////////////////////////////////////////////////
    Data_Ptr = (u8*)Filtered_Data;  
    Preparation(NODE_ADDRESS+1, Data_Ptr, Send_To_PC);                  
    Transmition(Send_To_PC);
  #endif

  #if ENABLE_FIR_FILTER
    ///////////////////////////////////////////////////////////////
    // FIR Filter
    FIR_Filtering(Int_Data_Set, Nine_Axis_Data, Filtered_Data, MAX_DIMENTION);
    ///////////////////////////////////////////////////////////////
    Data_Ptr = (u8*)Filtered_Data;  
    Preparation(NODE_ADDRESS+2, Data_Ptr, Send_To_PC);                  
    Transmition(Send_To_PC);
  #endif
// Digital Filter.
  LED_OFF;
}

void IMU_Init()
{
  #if ENABLE_CONFIG_PARA
  Config.SAMPLE_RATE = SAMPLE_RATE_DEFAULT;
  Config.ACC_RANGE = ACC_RANGE_DEFAULT;
  Config.ACC_ODR = ACC_ODR_DEFAULT;
  Config.GYR_RANGE = GYR_RANGE_DEFAULT;
  Config.GYR_ODR = GYR_ODR_DEFAULT;
  Config.MAG_RANGE = MAG_RANGE_DEFAULT;
  Config.MAG_ODR = MAG_ODR_DEFAULT;
  #endif
    
  printf("\r\n Start Test IMU...\r\n");
  Print_Error(Check_Configuration(Config));
  printf("LIS3DH Init\r\n");
  Print_Error(LIS3DH_Init(Config)); 
  
  printf("ITG3200 Init\r\n");
  Print_Error(ITG3200_Init(Config)); 
  
  printf("HMC5883 Init\r\n");     
  Print_Error(HMC5883_Init(Config));        
}

void DownloadMusicFileToSpiFlash()
{
  unsigned char connectTry,receiveTry;
  unsigned long packetCount = 0,address;
  u8 rd[129];
  u32 i;
  xyModemClass xyModem;
    
  MP3DeselectData();
  MP3DeselectControl();
  FlashDeselectControl();
  SPI_HighSpeed();
  Flash_Erase_Chip();
  printf("Download Music File To Spi Flash\r\n");
  connectTry= 20;
  xyModem.modem.ok=0;
  while(--connectTry)
  {
    xyModem.Nak();
    packetCount = 0;
    address = MUSIC_FILE_FlASH_ADDRESS;
    int res = xyModem.modem_init(xyModem.modem);
    if((res == 0) && (xyModem.modem.len == 128))
    {
      //save to spi flash
      address += (packetCount * xyModem.modem.len);
      packetCount++;
      
      FlashWriteEnable();
      FlashPageWrite(address, xyModem.modem.buf, xyModem.modem.len);
      FlashWriteDisable();
       printf("xyModem init OK\r\n");
       //printf("address=%08X \r\n",address);
       //printf("len=%d \r\n",xyModem.modem.len);
       for(i = 0; i < xyModem.modem.len; i++)
       {
         printf("%02X \r\n", xyModem.modem.buf[i]); delay(2);
       }
       printf("\r\n");
          
       FlashPageRead(address, rd, 128);
       if(memcmp(rd, xyModem.modem.buf,128) != 0)
       {
           printf("Write Flash Error\r\n"); return;
       }
       xyModem.Ack();
       xyModem.modem.ok=1;
       receiveTry=20;
       while(receiveTry)
       {
         res = xyModem.modem_recvdata(xyModem.modem);
         if((res == 0) && (xyModem.modem.len == 128))
         {
            //save to spi flash
            address += (packetCount * xyModem.modem.len);
            packetCount++;
            FlashWriteEnable();
            FlashPageWrite(address, xyModem.modem.buf, xyModem.modem.len);
            FlashWriteDisable();
            //printf("address=%08X \r\n",address);
            //printf("len=%d \r\n",xyModem.modem.len);
            for(i = 0; i < xyModem.modem.len; i++)
            {
              printf("%02X \r\n", xyModem.modem.buf[i]);
            }
            printf("\r\n");delay(2);
       
            FlashPageRead(address, rd, 128);
            if(memcmp(rd, xyModem.modem.buf,128) != 0)
            {
              printf("Write Flash Error\r\n"); return;
            }
            xyModem.Ack();
         }
         else if(res == 1)
         {
            printf("Receive file success\r\n");
            printf("Total Paket:%d\r\n",packetCount);
            printf("File 128 Byte Head:\r\n");
            FlashPageRead(MUSIC_FILE_FlASH_ADDRESS, xyModem.modem.buf, 256);
            for(unsigned int i = 0; i < 256; i++)
            {
              printf("%02X", xyModem.modem.buf[i]); 
            }
            printf("\r\n");delay(2);
            SPI_LowSpeed(); 
            
            return;
         }
         else
         {
            receiveTry--;
            printf("receive error=%d\r\n",res);
            printf("try=%d\r\n",receiveTry); 
         }
       }
    }
    else
    {
      xyModem.Nak();
      delay(1000);
      xyModem.modem.ok=0;
      printf("xyModem init fail:%d\r\n",res);   
      printf("xyModem retry %d\r\n",connectTry); 
    }
  }
  SPI_LowSpeed(); 
}

void Gps_Test()
{
  unsigned int ms = 5000;
  while(ms--)
  {
    delay(1);
    if(Serial2.available())
    {
      Serial.write(Serial2.read()); //gps data
    }
  }
}

void Codec_Test()
{
  Aplifier_D_Mode();
  if(VS10xx_Init() == 0)
  {
    TestVS10xx();
  }
  Aplifier_OFF_Mode(); 
}

void Motor_Test()
{
  for(byte i = 0; i < 5; i++)
  {
    digitalWrite(Vibrate, HIGH);
    delay(500);
    digitalWrite(Vibrate, LOW);
    delay(500); 
  }
}

void Heart_Test()
{
  unsigned int ms = 5000;
  while(ms--)
  {
    delay(1);
   if (digitalRead(Heart_Out) == HIGH)
   {
     printf("Heart Sensor High\r\n");
   }
  }
}

void Test_Menu()
{
  if(Serial.available())
  {
    switch(Serial.read())
    {
      case '1':
        FlashTest();
        break;
      case '2':
        Gps_Test();
        break;  
      case '3':
        IMU_Test();
        break;   
      case '4':
        Codec_Test();
        break;
      case '5':
        RTC_ReadDateTime();      
        break;
      case '6':
        Motor_Test();
        break;  
      case '7':
        Heart_Test();
        break;
      case '8':
        DownloadMusicFileToSpiFlash();
        break;          
      case '?':
      case 'h':
      case 'H':
         printf("\r\n1. Flash Test\r\n");
         printf("2. GPS Test\r\n");
         printf("3. IMU Test\r\n");
         printf("4. Codec Test\r\n");
         printf("5. RTC Test\r\n");
         printf("6. Motor Test\r\n");
         printf("7. Heart Test\r\n");
         printf("8. Download Music\r\n");
         printf("H. Help\r\n");
         break;
       default:
         printf("error cmd\r\n");
         break; 
      }
      Serial.flush();
  }
}

void IO_Init()
{
  pinMode(ledPin, OUTPUT);
  pinMode(Vibrate, OUTPUT);
  pinMode(Power_Hold, OUTPUT);
  pinMode(AMPOFF, OUTPUT);
  pinMode(CS_SST, OUTPUT);
  pinMode(GPS_PWR, OUTPUT);
  GPS_POWER_OFF();
  pinMode(Heart_Out, INPUT_PULLUP);
  pinMode(Key_center, INPUT_PULLUP);
  pinMode(Key_left, INPUT_PULLUP);
  pinMode(Key_right, INPUT_PULLUP);
  
  pinMode(Codec_XDREQ, INPUT_PULLUP);
  pinMode(Codec_XDCD, OUTPUT);
  pinMode(Codec_XRST, OUTPUT);
  pinMode(Codec_XCS, OUTPUT);
  pinMode(VS1053_XCS, OUTPUT);
  MP3DeselectData();
  MP3DeselectControl();
  FlashDeselectControl(); 
}

void KeyScan()
{
   if (digitalRead(Key_center) == LOW)
   {
     printf("Center Key Press\r\n"); delay(200);
   }
   else if (digitalRead(Key_left) == LOW)
   {
      printf("Left Key Press\r\n"); delay(200);
   }
   else if (digitalRead(Key_right) == LOW)
   {
     printf("Righ Key Press\r\n"); delay(200);
   }  
}

void ble_firmware_decrypt()
{
  //debug uart --> ble uart
  if(Serial1.available())
  {
    Serial.write(Serial1.read());
    Serial.println();
  } 
  delay(5);
   
  //ble uart --> debug uart
  if(Serial.available())
  {
    Serial1.write(Serial.read());
    Serial1.println();
  }
}

void setup() { 
  IO_Init();
  /**********************************/
  //usb upgrage display
  int tick = 500;
  for(int x = 0; x < 20; x++)
  {
    LED_ON;
    delay(tick);
    LED_OFF;
    delay(tick);
  }
  /********************************/
  PowerOn();  
  Serial.begin(38400);  //uart0 ble 
  Serial1.begin(38400); //uart1 xymodem and debug
  Serial2.begin(9600);  //uart2 gps
  printf_begin();
 
  Aplifier_D_Mode(); // Turn on amplifier chip:
  GPS_POWER_ON();
  Wire.begin();  //I2C begin
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16); //SPI CLK speed 8/16 = 500Khz
  SPI.setDataMode(SPI_MODE0);  //can try MODE2  CPOL = HIGH  CPHA=2Edge
  SPI.setBitOrder(MSBFIRST);
  IMU_Init();
  RTC_SetDateTime();
}

void loop() 
{
   KeyScan();  
   Test_Menu();
   PowerOff();
   delay(2);
}
