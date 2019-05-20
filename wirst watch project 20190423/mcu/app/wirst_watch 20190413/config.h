/*===================================================*\
|                                                     |
|  Motion Capture System on STM32F101R8               |
|                                                     |
|  Copyright (c) 2012 - Yongpeng Xu                   |
|      xuyp188@gmail.com                              |
|      xuyp188@zju.edu.cn                             |
|                                                     |
|  Created:                                           |
|         2012/02/01                                  |
|  Modified:                                          |
|         2012/12/06                                  |
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
#ifndef __CONFIG_H
#define __CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#ifdef __cplusplus
  #define     __I     volatile                /*!< defines 'read only' permissions      */
#else
  #define     __I     volatile const          /*!< defines 'read only' permissions      */
#endif
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t U16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/* Exported types ------------------------------------------------------------*/
typedef enum {FALSE = 0, TRUE = !FALSE} Bool;
typedef enum {IDLE_STATE=0, RUNNING_STATE=1, PAUSE_STATE=2, CHANGE_CONFIG_STATE=3} StateType;
typedef struct _Configuration_type
{
	s16 SAMPLE_RATE;
	s16 ACC_RANGE;
	s16 ACC_ODR;
	s16 GYR_RANGE;
	s16 GYR_ODR;
	s16 MAG_RANGE;
	s16 MAG_ODR;
} Configuration_type;

typedef enum
{
	NO_ERROR			= 	0,
	CONFIG_ERROR_CHK	=	1,
	CONFIG_ERROR_TIM	=	2,
	CONFIG_ERROR_ACC	=	3,
	CONFIG_ERROR_GYR	=	4,
	CONFIG_ERROR_MAG	=	5,
	FLASH_ERROR_READ	=	6,
	FLASH_ERROR_WRITE	=	7,
	SAMPLE_RATE_ERROR	=	10,
	ACC_RANGE_ERROR		=	11,
	ACC_ODR_ERROR		=	12,
	GYR_RANGE_ERROR		=	13,
	GYR_ODR_ERROR		=	14,
	MAG_RANGE_ERROR		=	15,
	MAG_ODR_ERROR		=	16,
} Error_type;

/* Exported constants --------------------------------------------------------*/
#define RCC_PLLMul_Config	RCC_PLLMul_9	// 用HSI所能达到的最大倍频基数是4MHz，乘以此倍频数就是MCU主时钟
#define SENSOR_DATA_SIZE	20	// 每次采集所发送的数据包大小
#define FLASH_SIZE			64	// 根据MCU的Flash容量正确设置
#define NODE_ADDRESS		'A'	// 节点序号固定为'A'
#define ENABLE_FABRICATE	0	// 伪造数据，用于调试
#define ENABLE_AVR_FILTER	0	// 使能AVR滤波器
#define ENABLE_FIR_FILTER	0	// 使能FIR滤波器
#define ENABLE_CALIBRATION	0	// 使能校准，需精密仪器进行实际参数测量，店主可以有偿提供这个服务，详情请咨询店主 
#define ENABLE_FLASH_WRITE	0	// 写入校准参数时使用一次，其后关闭
#define ENABLE_FLASH_READ	0	// 与ENABLE_CALIBRATION配合使用，获取Flash中的校准参数
#define ENABLE_WIRE_UART	1	// 使能有线串口
#define ENABLE_BT_UART		1	// 使能蓝牙串口
#define ENABLE_UART_INT		1	// Use UART interrupt to receive characters
#define ENABLE_CONFIG_BT	0	// 配置蓝牙模块
#define ENABLE_CONFIG_PARA	1	// 配置默认参数
#define FLASH_ADDR			0x0800FC00	// Page 63
#define CONFIG_MAX_LENGTH	100
#define CONFIG_ITERMS		7
#define SAMPLE_RATE_DEFAULT	50
#define ACC_RANGE_DEFAULT	8000
#define ACC_ODR_DEFAULT		50
#define GYR_RANGE_DEFAULT	2000
#define GYR_ODR_DEFAULT		100
#define MAG_RANGE_DEFAULT	1200
#define MAG_ODR_DEFAULT		75

// PORT A
#define UART2_TXD			GPIO_Pin_2
#define UART2_RXD			GPIO_Pin_3
#define UART1_TXD			GPIO_Pin_9
#define UART1_RXD			GPIO_Pin_10

// PORT B
#define SCL					GPIO_Pin_10
#define SDA					GPIO_Pin_11
#define LED1				GPIO_Pin_15

// Peripherals Map
#define USART_Wire			USART1
#define USART_BT			USART2
#define USART_Wire_IRQn		USART1_IRQn
#define USART_BT_IRQn		USART2_IRQn

// DMA Channels
#define USART1_DMA_CHANNEL_TX	DMA1_Channel4
#define USART2_DMA_CHANNEL_TX	DMA1_Channel7

/* Exported macro ------------------------------------------------------------*/
// Basic IO
#define LED1_ON				GPIO_SetBits(GPIOB, LED1)
#define LED1_OFF			GPIO_ResetBits(GPIOB, LED1)

// SYSTICK
#define SYSTICK_BASE_MS		(RCC_ClockFreq.HCLK_Frequency / 1000)
#define SYSTICK_BASE_US		(RCC_ClockFreq.HCLK_Frequency / 1000 / 1000)
#define SYSTICK_CMP_MS		(1<<24) / SYSTICK_BASE_MS
#define SYSTICK_CMP_US		(1<<24) / SYSTICK_BASE_US

/* Exported functions --------------------------------------------------------*/
void SysTick_Delay_us(u32 nTime);
void SysTick_Delay_ms(u32 nTime);
void Convert_Configuration(u8 str[][CONFIG_MAX_LENGTH], Configuration_type* Config);
Error_type Write_Configuration_Procedure(Configuration_type config);
void Get_Configuration(Configuration_type* config);
void Collect_Data(s16* data);
u8 Cal_Checksum(uc8 *buf, U16 size); // 用来生成校验和
void Preparation(u8 node_address, u8* ptr, u8* data);
void Transmition(u8* data);
void WIRE_USART1_DMA(u8* send_to_pc, U16 length);
void BT_USART2_DMA(u8* send_to_pc, U16 length);
void Print_Error(Error_type err);
void My_SystemReset(void);

#endif
/***********************************文件结束***********************************/
