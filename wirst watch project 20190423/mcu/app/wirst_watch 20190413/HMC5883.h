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
#ifndef __HMC5883_H
#define __HMC5883_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
// MAG HMC5883 传感器宏定义
#define MAG_SCALE_OFFSET	14		// Scale的范围是+/-2
#define MAG_BIAS_OFFSET		6		// Bias 的范围是+/-10mGauss，对应的bit改变为+/-169
#define MAGWriteAddress		0x3C	// 在I2C总线中的地址
#define MAGReadAddress		0x3D
// MAG Register Map
#define Config_Reg_A		0		// Setting the data output rate and measurement configuration
	#define Data_Out_Rate	6<<2		// 75Hz
	#define MAG_Measurement	0			// Normal measurement configuration
#define Config_Reg_B		1		// Setting the device gain
	#define MAG_Gain_900	0<<5		// 0.9Ga——1024 counts/Gauss
	#define MAG_Gain_1200	1<<5		// 1.2Ga——1024 counts/Gauss
	#define MAG_Gain_1900	2<<5		// 1.9Ga——1024 counts/Gauss
	#define MAG_Gain_2500	3<<5		// 2.5Ga——1024 counts/Gauss
	#define MAG_Gain_4000	4<<5		// 4.0Ga——1024 counts/Gauss
	#define MAG_Gain_4600	5<<5		// 4.6Ga——1024 counts/Gauss
	#define MAG_Gain_5500	6<<5		// 5.5Ga——1024 counts/Gauss
	#define MAG_Gain_7900	7<<5		// 7.9Ga——1024 counts/Gauss
#define Mode_Register		2		// Select the operating mode of the device
	#define MAG_Mode_Contin	0			// Continuous-Conversion Mode
	#define MAG_Mode_Single	1			// Single-Conversion Mode
	#define MAG_Mode_Idle	2			// Idle Mode Mode
#define MAG_XOUT_H			3
#define MAG_XOUT_L			4
#define MAG_ZOUT_H			5
#define MAG_ZOUT_L			6
#define MAG_YOUT_H			7
#define MAG_YOUT_L			8
#define Status_Register		9
#define ID_Reg_A			10
#define ID_Reg_B			11
#define ID_Reg_C			12

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif
/***********************************文件结束***********************************/
