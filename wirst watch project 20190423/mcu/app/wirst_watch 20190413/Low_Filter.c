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
|         2012/04/25                                  |
|                                                     |
|  Copyright 2012 Yongpeng Xu, All Rights Reserved.   |
|   For educational purposes only.                    |
|   Please do not republish in electronic or          |
|   print form without permission                     |
|                                                     |
|  Email me for the docs and new versions             |
|                                                     |
\*===================================================*/
/* 包含头文件 *****************************************************************/
#include "Low_Filter.h"

const float Breadth[B_LENGTH] = {
	0.0101,   -0.0004,   -0.0269,   -0.0446,   -0.0153,    0.0774,    0.2021,    0.2919,
	0.2919,    0.2021,    0.0774,   -0.0153,   -0.0446,   -0.0269,   -0.0004,    0.0101 
};

/*********************N阶FIR低通滤波**************************/
void Filter_Initial(s16 data_set[][B_LENGTH], s16 *original_data, u8 dimention, u8 order)
{
	u8 i;	

	for(i = 0; i < dimention; i++)
		data_set[i][order] = original_data[i];
}

void FIR_Filtering(s16 data_set[][B_LENGTH], s16* original_data, s16 *filtered_data, u8 dimention)
{
	u8 i, j;
	float float_buf[MAX_DIMENTION];
	
// FIR Filter
	for(i = 0; i < dimention; i++)
	{
		float_buf[i] = 0;
		data_set[i][B_LENGTH-1] = original_data[i];

		// 卷积滤波
		for(j = 0; j < B_LENGTH; j++) 
			float_buf[i] += Breadth[j] * (float)data_set[i][B_LENGTH-1 - j];

		// 数据集前移1位
		for(j = 0; j < B_LENGTH-1; j++)
			data_set[i][j] = data_set[i][j+1];

		// 输出滤波结果
		filtered_data[i] = float_buf[i];
	}
}

void Average_Filtering(s16 data_set[][B_LENGTH], s16 *filtered_data, u8 dimention)
{
	u8 i, j;
	s32 int_buf[MAX_DIMENTION];

// Average Filter
	for(i = 0; i < dimention; i++)
	{
		int_buf[i] = 0;

		// 平均值滤波
		for(j = 0; j < B_LENGTH; j++) 
			int_buf[i] += data_set[i][j];

		// 数据集前移1位
		for(j = 0; j < B_LENGTH-1; j++)
			data_set[i][j] = data_set[i][j+1];

		// 输出滤波结果
		filtered_data[i] = int_buf[i] >> 4;
	}
}

/************************************文件结束*********************************/
