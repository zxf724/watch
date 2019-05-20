/*===================================================*\
|                                                     |
|  Motion Capture System on STM32F101R8               |
|                                                     |
|  Copyright (c) 2012 - Yongpeng Xu                   |
|      xuyp188@gmail.com                              |
|      xuyp188@zju.edu.cn                             |
|                                                     |
|  Created:                                           |
|         2012/03/01                                  |
|  Modified:                                          |
|         2012/03/01                                  |
|                                                     |
|  Copyright 2012 Yongpeng Xu, All Rights Reserved.   |
|   For educational purposes only.                    |
|   Please do not republish in electronic or          |
|   print form without permission                     |
|                                                     |
|  Email me for the docs and new versions             |
|                                                     |
\*===================================================*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ITG3200_H
#define __ITG3200_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
// GYR ITG3200 传感器宏定义
#define GYRWriteAddress		0xD2	// 在I2C总线中的地址
#define GYRReadAddress		0xD3
// GYR Register Map
#define	ITG3200_WHO_AM_I	0x00
#define	SMPLRT_DIV			0x15	// Determines the sample rate of the ITG-3200 gyros
	#define Sample_Rate_Divider	9	// 10分频——1kHz/(9+1)=100Hz
#define	DLPF_FS				0x16	// Configures several parameters related to the sensor acquisition
	#define FS_SEL			3<<3		// 只能设置成3			
	#define DLPF_CFG		0x02		// Low Pass Filter Bandwidth = 98Hz, Internal Sample Rate = 1kHz
#define	INT_CFG				0x17
#define	INT_STATUS			0x1A
#define	TEMP_OUT_H			0x1B
#define	TEMP_OUT_L			0x1C
#define	GYRO_XOUT_H			0x1D
#define	GYRO_XOUT_L			0x1E
#define	GYRO_YOUT_H			0x1F
#define	GYRO_YOUT_L			0x20
#define	GYRO_ZOUT_H			0x21
#define	GYRO_ZOUT_L			0x22
#define WR_MEG				0x3E

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif
/***********************************文件结束***********************************/
