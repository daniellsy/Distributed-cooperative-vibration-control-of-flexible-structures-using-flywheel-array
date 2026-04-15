#ifndef USART_H
#define USART_H

#include "stm32f10x.h"
#include <stdio.h>
#include "copewit.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// 定义 USART2 数据缓冲区大小
#define USART2_BUFFER_SIZE 2048
#define USART2_TX_DMA_CHANNEL DMA1_Channel7
#define USART2_RX_DMA_CHANNEL DMA1_Channel6

// USART3 DMA通道定义（保持原有）
#define USART3_TX_DMA_CHANNEL DMA1_Channel2
#define USART3_RX_DMA_CHANNEL DMA1_Channel3

// 外部变量声明
extern uint8_t usart2_buffer[USART2_BUFFER_SIZE];  // USART2 数据缓冲区
extern volatile uint32_t buffer_head;              // 缓冲区头部指针
extern volatile uint32_t buffer_tail;              // 缓冲区尾部指针

// 函数声明
void My_USART1_Init(void);        // 初始化 USART1
void USART2_Config(void);         // 配置 USART2
void NVIC_Configuration(void);    // 配置 NVIC 中断优先级
void USART2_Init(uint32_t baudrate);
void USART3_Init(uint32_t baudrate);

// 控制状态枚举
typedef enum {
    WIT_start = 1,
		CTRL_start =2,
		CTRL_co_run = 3,
		CTRL_de_run = 4,
    CTRL_stop = 5,
} CtrlState;


#pragma pack(push, 1)//不用编译器的自动填充，让sizeof(struct)以对上包大小的参数
typedef struct {
    uint8_t id;
    float speed;
    float gyroz;
} Data;

#pragma pack(pop)

extern volatile CtrlState g_ctrlState;

#define BUFFER_SIZE 40 // 缓冲区大小
typedef struct {
    uint8_t data[BUFFER_SIZE];
    volatile uint16_t index; // 当前数据长度
} RxBuffer;

#pragma pack(push, 1)  // 强制1字节对齐[1](@ref)
typedef struct {
    uint16_t head;      // 帧头 0xAAFF
    uint8_t id;
		uint8_t state;
    float speed;
    float gyroz;
	  float angleyaw;
    uint8_t checksum;   // 校验和（ID+speed+gyroz字节和）
    uint16_t tail;      // 帧尾 0x55FF
} DataPacket;
#pragma pack(pop)       // 恢复默认对齐


extern DataPacket txPacket,rxPacket2,rxPacket3;
extern RxBuffer rx2Buffer,rx3Buffer;

void USART2_DMA_Init(void);
void USART3_DMA_Init(void);
uint8_t calculate_checksum(DataPacket* pkt);
uint8_t USART2_DMA_CheckComplete(void);
uint8_t USART3_DMA_CheckComplete(void);
void USART2_DMA_Send_Config(DataPacket* pData);
void USART3_DMA_Send_Config(DataPacket* pData);
int ParseDataPacket(DataPacket *pkt, RxBuffer *buffer ) ;

#endif // USART_H
