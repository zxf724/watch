#include "Arduino.h"
#include "config.h"

#define AT8307_DEVICE_ADDRESS 0xD0

#define RTC_SECOND 0x00
#define RTC_MINUTE 0x01
#define RTC_HOUR   0x02
#define RTC_WEEK   0x03
#define RTC_DAY    0x04
#define RTC_MONTH  0x05
#define RTC_YEAR   0x06
#define RTC_CTRL   0x07

typedef struct
{
  u8 second;
  u8 minute;
  u8 hour;
  u8 week;
  u8 day;
  u8 month;
  u8 year;
  
}Rtc_DateTime;


Rtc_DateTime RTC;


void RTC_SetDateTime()
{
  //2018-11-04 12:01:05  sunday
  printf("RTC set: 2018-11-04 12:01:05\r\n");
  
  RTC.week= 0x01;
  
  RTC.second = 0x05;
  RTC.minute= 0x01;
  RTC.hour= 0x12;

  RTC.day = 0x04;
  RTC.month= 0x11;
  RTC.year = 0x18;

  WriteI2C(RTC.second,RTC_SECOND, AT8307_DEVICE_ADDRESS);
  WriteI2C(RTC.minute,RTC_MINUTE, AT8307_DEVICE_ADDRESS);
  WriteI2C(RTC.hour,RTC_HOUR, AT8307_DEVICE_ADDRESS);
  WriteI2C(RTC.day,RTC_DAY, AT8307_DEVICE_ADDRESS);
  WriteI2C(RTC.month,RTC_MONTH, AT8307_DEVICE_ADDRESS);
  WriteI2C(RTC.year,RTC_YEAR, AT8307_DEVICE_ADDRESS);
  WriteI2C(RTC.week,RTC_WEEK, AT8307_DEVICE_ADDRESS);
}

void RTC_ReadDateTime()
{
  RTC.second = ReadI2C(RTC_SECOND, AT8307_DEVICE_ADDRESS);
  RTC.minute = ReadI2C(RTC_MINUTE, AT8307_DEVICE_ADDRESS);
  RTC.hour = ReadI2C(RTC_HOUR, AT8307_DEVICE_ADDRESS);
  RTC.day = ReadI2C(RTC_DAY, AT8307_DEVICE_ADDRESS);
  RTC.month = ReadI2C(RTC_MONTH, AT8307_DEVICE_ADDRESS);
  RTC.year = ReadI2C(RTC_YEAR, AT8307_DEVICE_ADDRESS);
  RTC.week = ReadI2C(RTC_WEEK, AT8307_DEVICE_ADDRESS);

  u8 i,buf[30];

  for(i = 0; i < 30; i++)
    buf[i] = 0x00;

  i = 0;
  printf("20");
  buf[i++] = ((RTC.year >> 4) & 0x0f) + '0';
  buf[i++] = (RTC.year & 0x0f) + '0';
  buf[i++] = '-';
  buf[i++] = ((RTC.month >> 4) & 0x0f) + '0';
  buf[i++] = (RTC.month & 0x0f) + '0';
  buf[i++] = '-';
  buf[i++] = ((RTC.day >> 4) & 0x0f) + '0';
  buf[i++] = (RTC.day & 0x0f) + '0';
  buf[i++] = ' ';
  buf[i++] = ((RTC.hour >> 4) & 0x0f) + '0';
  buf[i++] = (RTC.hour & 0x0f) + '0';
  buf[i++] = ':';
  buf[i++] = ((RTC.minute >> 4) & 0x0f) + '0';
  buf[i++] = (RTC.minute & 0x0f) + '0';
  buf[i++] = ':';
  buf[i++] = ((RTC.second >> 4) & 0x0f) + '0';
  buf[i++] = (RTC.second & 0x0f) + '0';
  printf("%s", buf);
  printf("  ");
  switch(RTC.week)
  {
    case 1:
      printf("Sun");
      break;
    case 2:
      printf("Mon");
      break;
    case 3:
      printf("Tue");
      break;  
    case 4:
      printf("Wed");
      break;  
    case 5:
      printf("Thu");
      break;  
    case 6:
      printf("Fri");
      break;  
    case 7:
      printf("Sat");
      break;  
  }

  printf("\r\n");
}



