/*===================================================*\
|                                                     |
|  Current Meter System on STM32F101C8                |
|                                                     |
|  Copyright (c) 2012 - Yongpeng Xu                   |
|      xuyp188@gmail.com                              |
|      xuyp188@zju.edu.cn                             |
|                                                     |
|  Created:                                           |
|         2011/12/22                                  |
|  Modified:                                          |
|         2012/03/26                                  |
|                                                     |
|  Copyright 2012 Yongpeng Xu, All Rights Reserved.   |
|   For educational purposes only.                    |
|   Please do not republish in electronic or          |
|   print form without permission                     |
|                                                     |
|  Email me for the docs and new versions             |
|                                                     |
\*===================================================*/
/*===================================================*\
 * Discrete-Time FIR Filter (real)					 *
 * -------------------------------					 *
 * Filter Structure  : Direct-Form FIR				 *
 * Filter Length     : 21							 *
 * Stable            : Yes							 *
 * Linear Phase      : Yes (Type 1)					 *
 *													 *
 * Implementation Cost								 *
 * Number of Multipliers : 21						 *
 * Number of Adders      : 20						 *
 * Number of States      : 20						 *
 * MultPerInputSample    : 21						 *
 * AddPerInputSample     : 20						 *
\*===================================================*/
/* 防止重定义-----------------------------------------------------------------*/
#ifndef __LOW_FILTER_H
#define __LOW_FILTER_H

/* 包含的头文件---------------------------------------------------------------*/
#include "Config.h"


/* 类型定义 ------------------------------------------------------------------*/
/* 常量定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define	B_LENGTH			16
#define	MAX_DIMENTION		9

/* 函数定义------------------------------------------------------------------ */
void Filter_Initial(s16 data_set[][B_LENGTH], s16 *original_data, u8 dimention, u8 order);
void FIR_Filtering(s16 data_set[][B_LENGTH], s16 *original_data, s16 *filtered_data, u8 dimention);
void Average_Filtering(s16 data_set[][B_LENGTH], s16 *filtered_data, u8 dimention);

#endif /* __LOW_FILTER_H */

/************************************文件结束*********************************/
