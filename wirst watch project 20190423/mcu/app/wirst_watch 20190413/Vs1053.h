#ifndef __VS1053B__H__
#define __VS1053B__H__

#include <Arduino.h> 

extern const int ledPin;

#define MUSIC_FILE_FlASH_ADDRESS  (4096u)

//VS1053B codec IO
const int Codec_MISO = 50; //PB3
const int Codec_MOSI = 51; //PB2
const int Codec_SCK  = 52; //PB1
const int Codec_XCS  = 53; //PB0
const int Codec_XDCD = 24; //PA2 XDCS
const int Codec_XDREQ = 23; //PA1
const int Codec_XRST = 22; //PA0
const int VS1053_XCS = 54; // PF 0 ** 54 ** A0

#define VS_WRITE_COMMAND 0x02
#define VS_READ_COMMAND  0x03


#define MP3PutInReset()       { digitalWrite(Codec_XRST, LOW); }
#define MP3ReleaseFromReset() { digitalWrite(Codec_XRST, HIGH); }

#define MP3SelectControl()    { digitalWrite(Codec_XCS, LOW); }
#define MP3DeselectControl()  { digitalWrite(Codec_XCS, HIGH); }
//#define MP3SelectControl()    { digitalWrite(VS1053_XCS, LOW); } //ADC0
//#define MP3DeselectControl()  { digitalWrite(VS1053_XCS, HIGH); }

#define MP3SelectData()       { digitalWrite(Codec_XDCD, LOW); }
#define MP3DeselectData()     { digitalWrite(Codec_XDCD, HIGH); }
#define MP3ClkLow()           {digitalWrite(Codec_SCK, LOW); }
#define MP3ClkHigh()          { digitalWrite(Codec_SCK, HIGH); }
#define MP3SiLow()            { digitalWrite(Codec_MOSI, LOW); }
#define MP3SiHigh()           { digitalWrite(Codec_MOSI, HIGH); }
#define MP3So()               (digitalRead(Codec_MISO))
#define MP3_DREQ()            (digitalRead(Codec_XDREQ))


extern unsigned char VS10xx_Init();
extern void TestVS10xx();


#endif


