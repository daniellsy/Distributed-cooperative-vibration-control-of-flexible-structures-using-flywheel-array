#include "USART.h"


uint8_t usart2_buffer[USART2_BUFFER_SIZE];//23启用DMA传输，中断源用TC，1启用普通串口通信，中断源用RXNE。
volatile uint32_t buffer_head = 0;
volatile uint32_t buffer_tail = 0;
static volatile char s_cCmd = 0xff;
volatile CtrlState g_ctrlState = CTRL_stop; // 初始值

DataPacket txPacket,rxPacket2,rxPacket3;
RxBuffer rx2Buffer = {0};
RxBuffer rx3Buffer = {0};

// 对USART1进行初始化 PB6 - Tx PB7 - Rx 115200, 8, 1, None, 双向
//

void CopeCmdData(unsigned char ucData) {
	  static unsigned char s_ucCmdBuffer[3]; // 存储连续3字节指令，用static防止出栈销毁，是uint8_t
    static uint8_t s_ucIndex = 0;//同上

    // 存入缓冲区
    s_ucCmdBuffer[s_ucIndex] = ucData;
    s_ucIndex = (s_ucIndex + 1) % 3;//产生 0 1 2循环调用第三次才会进入if

    // 检测指令（每3字节触发一次检查）
    if (s_ucIndex == 0) {
        // 判断指令类型
        if (memcmp(s_ucCmdBuffer, "111", 3) == 0) {//memcmp作用比较a，b前3个字节
            g_ctrlState = WIT_start;
            printf("WIT_start\n");
        } 
        else if (memcmp(s_ucCmdBuffer, "222", 3) == 0) {
            g_ctrlState = CTRL_start;
            printf("CTRL_start\n");
        }
        else if (memcmp(s_ucCmdBuffer, "333", 3) == 0) {
            g_ctrlState = CTRL_co_run;
            printf("CTRL_run\n");
        }
				else if (memcmp(s_ucCmdBuffer, "444", 3) == 0) {
						g_ctrlState = CTRL_de_run;
						printf("CTRL_run\n");
        }
        else if (memcmp(s_ucCmdBuffer, "555", 3) == 0) {
            g_ctrlState = CTRL_stop;
            printf("CTRL_stop\n");
        }
        
        // 检测完指令后清零缓冲区
        memset(s_ucCmdBuffer, 0, sizeof(s_ucCmdBuffer));//memset(ptr, value, n)：从指针 ptr 开始的连续 n 个字节都填成 value（按字节填充）。给数组名自动衰变为指针
    }
}
void My_USART1_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);//开外设时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//选引脚PA9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设定翻转速度等级
	GPIO_Init(GPIOA, &GPIO_InitStructure); //传指针写入  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//参考上面
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	USART_InitStructure.USART_BaudRate = 115200;//波特率115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止
	USART_InitStructure.USART_Parity = USART_Parity_No ;//无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 无RTS/CTS，硬件流控，引入额外两个引脚发送两个流控信号
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;// 收发都开
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE); // 关闭“发送缓冲空”中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// 打开“接收非空”中断，第一个选择哪个串口，第二个选择中断源，第三个选择是启用还是关闭
	USART_ClearFlag(USART1,USART_FLAG_TC);// 清“发送完成”标志
	USART_Cmd(USART1, ENABLE);	// 使能串口外设
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//选择配置USART1中断，motor中已经配置中断分组为2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART2_Init(uint32_t baudrate) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 2. GPIO配置（PA2-TX, PA3-RX）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;  // TX
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;  // RX
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 串口参数配置
    USART_InitStruct.USART_BaudRate = baudrate;//函数传参
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;//同上
    USART_InitStruct.USART_StopBits = USART_StopBits_1;//一位停止
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    // 4. 中断配置
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //配置中断源为RXNE
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 5. 启动串口
    USART_Cmd(USART2, ENABLE);
}

// USART3初始化（PB10-TX, PB11-RX）
void USART3_Init(uint32_t baudrate) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // 2. GPIO配置（PB10-TX, PB11-RX）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;  // TX
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;  // RX
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. 串口参数配置
    USART_InitStruct.USART_BaudRate = baudrate;//同上
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStruct);

    // 4. 中断配置
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//配置中断源为RXNE
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 5. 启动串口
    USART_Cmd(USART3, ENABLE);
}




void CopeCmdData(unsigned char ucData);//让编译器在看到中断里调用 CopeCmdData(ucTemp); 时，先知道函数的签名。
void USART1_IRQHandler(void)//USART1串口中断程序
{
	unsigned char ucTemp;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//RXNE表示（Receive Data Register Not Empty）接收非空，为1表示有数据
	{
		ucTemp = USART_ReceiveData(USART1);
		CopeCmdData(ucTemp);//将数据送入读取指令的函数进行翻译
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	USART_ClearITPendingBit(USART2,USART_IT_ORE);// 这句很可疑：在 USART1 的中断里清 USART2 的 ORE 溢出标志
}

// USART2中断处理
void USART2_IRQHandler(void) {
   
}

// USART3中断处理
void USART3_IRQHandler(void) {
   
}
uint8_t calculate_checksum(DataPacket* pkt) {
    uint8_t sum = 0;//// 校验累加器，8位，溢出自动取模
    uint8_t* data = (uint8_t*)&(pkt->id); //(uint8_t*)表示强转为指向8位无符号数的指针从id开始计算
    
    // 校验范围：id(1) + speed(4) + gyroz(4) = 9字节
    for(int i=0; i<10; i++) {
        sum += data[i];
    }
    return sum;
}
//uint8_t calculate_checksum( DataPacket* pkt) {
//    uint8_t sum = 0;
//    // 从 id 开始，到 gyroz 结束（含 state）
//    const uint8_t* p = (const uint8_t*)&(pkt->id);
//    for (int i = 0; i < sizeof(pkt->id) + sizeof(pkt->state) +
//                         sizeof(pkt->speed) + sizeof(pkt->gyroz); ++i) {
//        sum += p[i];
//    }
//    return sum;
//}

// 在main()初始化时调用一次
void USART2_DMA_Init(void) {//开启DMA模式，DMA控制器直接从内存搬运数据写入DR，而不是通过CPU写入DR
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitTypeDef DMA_txInitStruct;
    // 发送通道配置（DMA1 Channel2）
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//开时钟才能配置
    DMA_DeInit(USART2_TX_DMA_CHANNEL);//将TX通道恢复到默认配置
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;//写入外设地址，(uint32_t)括号表示强转类型，意思是取到USART2串口的DR寄存器地址，将其强转为uint32_t类型，方便DMA控制器写入DR
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&txPacket;//同上
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;// 方向：内存 -> 外设
    DMA_InitStruct.DMA_BufferSize = sizeof(DataPacket);// 一次要搬多少字节，这就是为什么要用#pragma pack(push, 1)
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// DR 地址固定，不自增
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;// 内存地址每搬1字节+1
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// 以“字节”访问 DR，Byte 宽度保证仅写 DR 的低 8 位，适配 8N1。
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 以“字节”访问内存
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;// 普通模式：搬完停
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;// 优先级：高
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;// 禁用“内存到内存”模式
    DMA_Init(USART2_TX_DMA_CHANNEL, &DMA_InitStruct);//写入配置

    // 接收通道配置（DMA1 Channel3）
		DMA_DeInit(USART2_RX_DMA_CHANNEL);
    DMA_txInitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;//接受源
    DMA_txInitStruct.DMA_MemoryBaseAddr = (uint32_t)rx2Buffer.data;//data是结构体里面的数组名，自动衰变为了指针指向数组首个元素
    DMA_txInitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;    // 外设到内存
    DMA_txInitStruct.DMA_BufferSize = BUFFER_SIZE;       // 缓冲区大小这里设置的是 CNDTR，即DMA传输的字节数
    DMA_txInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//同发送
    DMA_txInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;//同发送
    DMA_txInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// 以字节取 DR
    DMA_txInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;// 以字节写内存
    DMA_txInitStruct.DMA_Mode = DMA_Mode_Normal;         // 必须为普通模式
    DMA_txInitStruct.DMA_Priority = DMA_Priority_High;
    DMA_txInitStruct.DMA_M2M = DMA_M2M_Disable;//禁用内存到内存
    DMA_Init(USART2_RX_DMA_CHANNEL, &DMA_txInitStruct);
		// 使能DMA接收完成中断
		DMA_ITConfig(USART2_RX_DMA_CHANNEL, DMA_IT_TC, ENABLE);//开TC中断
		 
    USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE); // 同时使能收发DMA
}

// 在main()初始化时调用一次
void USART3_DMA_Init(void) {
    DMA_InitTypeDef DMA_InitStruct;
    DMA_InitTypeDef DMA_txInitStruct;
    // 发送通道配置（DMA1 Channel2）
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(USART3_TX_DMA_CHANNEL);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&txPacket;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize = sizeof(DataPacket);
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(USART3_TX_DMA_CHANNEL, &DMA_InitStruct);

    // 接收通道配置（DMA1 Channel3）
		DMA_DeInit(USART3_RX_DMA_CHANNEL);
    DMA_txInitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_txInitStruct.DMA_MemoryBaseAddr = (uint32_t)rx3Buffer.data;
    DMA_txInitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;    // 外设到内存
    DMA_txInitStruct.DMA_BufferSize = BUFFER_SIZE;       // 缓冲区大小
    DMA_txInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_txInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_txInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_txInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_txInitStruct.DMA_Mode = DMA_Mode_Normal;         // 必须为普通模式
    DMA_txInitStruct.DMA_Priority = DMA_Priority_High;
    DMA_txInitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(USART3_RX_DMA_CHANNEL, &DMA_txInitStruct);
		// 使能DMA接收完成中断
		DMA_ITConfig(USART3_RX_DMA_CHANNEL, DMA_IT_TC, ENABLE);
		 
    USART_DMACmd(USART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE); // 同时使能收发DMA
}

uint8_t USART2_DMA_CheckComplete(void) {//检查是否完成由内存到DR
    if(DMA_GetFlagStatus(DMA1_FLAG_TC7) != RESET) {//TC为1满足if
        DMA_ClearFlag(DMA1_FLAG_TC7);//清0 TC
        return 1;
    }
    return 0;
}
uint8_t USART3_DMA_CheckComplete(void) {
    if(DMA_GetFlagStatus(DMA1_FLAG_TC2) != RESET) {
        DMA_ClearFlag(DMA1_FLAG_TC2);
        return 1;
    }
    return 0;
}


static void print_rx_buffer(const RxBuffer *b)//打印调试函数
{
    if (!b || !b->index) return;

    //printf("[RX_RAW] len=%u\r\n", b->index);
    for (uint16_t i = 0; i < b->index; i++) {
        printf("%02X ", b->data[i]);
    }
    printf("\r\n");
}


enum { PACKET_LEN = 19 };  // ?新长度
//int ParseDataPacket(DataPacket *pkt, RxBuffer *buffer ) {
//		//print_rx_buffer(buffer);   // <-- 加这里
//    // 需要至少2字节才能搜索帧头
//    if (buffer->index < 2) return -1;
//    // 1. 优先搜索帧头 FF AA
//    for (int i = 0; i <= buffer->index - 2; i++) {
//        if (buffer->data[i] == 0xFF && buffer->data[i+1] == 0xAA) {
//            // 2. 检查后续是否足够PACKET_LEN字节
//            if (i + PACKET_LEN > buffer->index) {
//                return -1; // 数据不完整
//            }
//            // 3. 提取完整数据包
//            memcpy(pkt, &buffer->data[i], PACKET_LEN);
//            // 4. 验证校验和与帧尾
//            if (pkt->checksum == calculate_checksum(pkt) && 
//                pkt->tail == 0x55FF) {
//                // 5. 移除已处理数据（临界区保护）
//                __disable_irq();
//                uint16_t remaining = buffer->index - (i + PACKET_LEN);
//                memmove(buffer->data, &buffer->data[i + PACKET_LEN], remaining);
//                buffer->index = remaining;
//                __enable_irq();
//                return 0; // 成功解析
//            } else {
//                // 校验失败，跳过错误的帧头继续搜索
//                i++; // 避免重复检查当前AA
//            }
//        }
//    }
//    // 6. 缓冲区满时清空（防止死锁）
//    if (buffer->index >= BUFFER_SIZE) {
//        __disable_irq();
//        buffer->index = 0;
//        __enable_irq();
//    }
//    return -1;
//}
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    while (len--) crc = crc8_table[crc ^ *data++];//先取到当前指针所指的值，取到后该值与crc 做异或，同时把指针加一指向下一个
    return crc;
}
/* 校验范围：id + state + speed + gyroz = 1+1+4+4 = 10 字节 */
uint8_t calculate_crc(const DataPacket *p)
{
    return crc8((uint8_t*)&p->id, 10);//从数据包里的ID开始到gyroz，参与校验
}


//int ParseDataPacket(DataPacket *pkt, RxBuffer *buffer ) {
//    // 需要至少2字节才能搜索帧头
//		//print_rx_buffer(buffer); 
//    if (buffer->index < 2) return -1;
//    // 1. 优先搜索帧头 FF AA
//    for (int i = 0; i <= buffer->index - PACKET_LEN; i++) {
//        if (buffer->data[i] == 0xFF && buffer->data[i+1] == 0xAA) {
//            // 2. 检查后续是否足够PACKET_LEN字节
//            if (i + PACKET_LEN > buffer->index) {
//                return -1; // 数据不完整
//            }
//						//print_rx_buffer(buffer); 
//            // 3. 提取完整数据包
//            memcpy(pkt, &buffer->data[i], PACKET_LEN);
//						printf("FF AA found at i=%d, tail=%04X, checksum=%02X, calc=%02X\n",
//           i, pkt->tail, pkt->checksum, calculate_crc(pkt));
//						uint8_t* p = (uint8_t*)pkt;
//						printf("Raw pkt (%d bytes): ", PACKET_LEN);
//						for (int j = 0; j < PACKET_LEN; j++) {
//								printf("%02X ", p[j]);
//						}
//            // 4. 验证校验和与帧尾
//            if (pkt->checksum == calculate_crc(pkt) && 
//                pkt->tail == 0x55FF) {
//                // 5. 移除已处理数据（临界区保护）
//									//printf("111");
//                __disable_irq();
//                uint16_t remaining = buffer->index - (i + PACKET_LEN);
//                memmove(buffer->data, &buffer->data[i + PACKET_LEN], remaining);
//                buffer->index = remaining;
//                __enable_irq();
//                return 0; // 成功解析
//            } else {
//                // 校验失败，跳过错误的帧头继续搜索
//                i++; // 避免重复检查当前AA
//            }
//        }
//    }
//    // 6. 缓冲区满时清空（防止死锁）
//    if (buffer->index >= BUFFER_SIZE) {
//        __disable_irq();
//        buffer->index = 0;
//        __enable_irq();
//    }
//    return -1;
//}

int ParseDataPacket(DataPacket *pkt, RxBuffer *buffer) {//在缓冲区找一帧数据并校验
    if (buffer->index < PACKET_LEN) return -1;//至少要有一个完整包

    // 最多检查到缓冲区的一半
    int maxCheck = buffer->index -PACKET_LEN;
		//print_rx_buffer(buffer); 
    for (int i = 0; i <= maxCheck; i++) {
        if (buffer->data[i] == 0xFF && buffer->data[i+1] == 0xAA) {//找帧头
            // 检查是否有足够的数据
            if (i + PACKET_LEN > buffer->index) {//找到判断剩下的是不是一个完整的数据
                return -1; // 数据不完整
            }

            // 拷贝一个完整包
            memcpy(pkt, &buffer->data[i], PACKET_LEN);//第一个是拷贝后数据存的地址，第二个是源数据地址
//						uint8_t* p = (uint8_t*)pkt;
//						printf("Raw pkt (%d bytes): ", PACKET_LEN);
//						for (int j = 0; j < PACKET_LEN; j++) {
//								printf("%02X ", p[j]);
//						}
            // 校验尾部和 CRC
            if (pkt->tail == 0x55FF &&
                pkt->checksum == calculate_crc(pkt)) {

                // 移除已处理数据，把帧尾后面的数搬到前面进行下一轮处理
                __disable_irq();//全局关中断
                uint16_t remaining = buffer->index - (i + PACKET_LEN);//帧尾后面还剩多少未处理有效字节
                memmove(buffer->data, &buffer->data[i + PACKET_LEN], remaining);//把数组里 从 i+PACKET_LEN 开始的这 remaining 个字节，整体搬到数组起始 data[0]。
                buffer->index = remaining;//更新有效长度
                __enable_irq();

                return 0; // 成功解析
            } else {
                // 找到帧头但校验失败 → 本次中断终止
                return -1;
            }
        }
    }

    // 没找到帧头 → 本次中断终止
    return -1;
}

void USART2_DMA_Send_Config(DataPacket* pData) {
		pData->head = 0xAAFF;       // 帧头
    pData->tail = 0x55FF;       // 帧尾
		pData->checksum = calculate_crc(pData); // <-- 换成 CRC8
    //pData->checksum = calculate_checksum(pData); // 计算校验和
		 // 打印数据包内容（调试用）
//    uint8_t *bytes = (uint8_t*)pData;
//    printf("Sending: ");
//    for (int i = 0; i < sizeof(DataPacket); i++) {
//        printf("%02X ", bytes[i]);
//    }
//    printf("\r\n");
    // 确保DMA传输完整数据包
    DMA_Cmd(USART2_TX_DMA_CHANNEL, DISABLE);  // 先关闭DMA
    DMA_SetCurrDataCounter(USART2_TX_DMA_CHANNEL, sizeof(DataPacket)); // 重置计数器
    DMA_Cmd(USART2_TX_DMA_CHANNEL, ENABLE);  // 重新使能DMA
    DMA_ClearFlag(DMA1_FLAG_TC7);    // 清除传输完成标志
}


void USART3_DMA_Send_Config(DataPacket* pData) {
		pData->head = 0xAAFF;       // 帧头
    pData->tail = 0x55FF;       // 帧尾
    pData->checksum = calculate_crc(pData); // <-- 换成 CRC8
//		 // 打印数据包内容（调试用）
//    uint8_t *bytes = (uint8_t*)pData;
//    printf("Sending: ");
//    for (int i = 0; i < sizeof(DataPacket); i++) {
//        printf("%02X ", bytes[i]);
//    }
//    printf("\r\n");
    // 确保DMA传输完整数据包
    DMA_Cmd(USART3_TX_DMA_CHANNEL, DISABLE);  // 先关闭DMA
    DMA_SetCurrDataCounter(USART3_TX_DMA_CHANNEL, sizeof(DataPacket)); // 重置计数器
    DMA_Cmd(USART3_TX_DMA_CHANNEL, ENABLE);  // 重新使能DMA
    DMA_ClearFlag(DMA1_FLAG_TC2);    // 清除传输完成标志
}


///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)//通过USART1发数据
{
		/* 发送一个字节数据到串口DEBUG_USART */
		USART_SendData(USART1, (uint8_t) ch);//USART_SendData(...) 实际就是把数据写进 USART1->DR（发送数据寄存器）。强转为 uint8_t：只发低8位
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//轮询等待 TXE=1（Transmit data register empty）：表示发送数据寄存器已空，上一字节已被移入移位寄存器，允许你写下一个字节。		
	
		return (ch);//返回刚发的字符，符合 fputc 约定。
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)//读USART1数据
{
		//当DR中有数据时RXNE为1
	  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}



void NVIC_Configuration(void)
{
	NVIC_InitTypeDef    NVIC_InitStructure; 					   
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;             
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	               
	NVIC_Init(&NVIC_InitStructure);			
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel3_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
    
  DMA_ITConfig(USART3_RX_DMA_CHANNEL, DMA_IT_TC, ENABLE); // 使能传输完成中断
	
	NVIC_InitTypeDef NVIC_InitStruct2;
	NVIC_InitStruct2.NVIC_IRQChannel = DMA1_Channel6_IRQn;
  NVIC_InitStruct2.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct2.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStruct2.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct2);
}

// DMA1通道3中断处理函数
void DMA1_Channel6_IRQHandler(void) {
    if(DMA_GetITStatus(DMA1_IT_TC6) != RESET) {
        // 获取实际传输的数据量
        uint16_t remaining = DMA_GetCurrDataCounter(USART2_RX_DMA_CHANNEL);
        rx2Buffer.index = BUFFER_SIZE - remaining;  // 计算实际接收的字节数
        DMA_ClearITPendingBit(DMA1_IT_TC6);//清除本次的 传输完成中断挂起标志，避免重复进中断。
        // 重新配置DMA接收
        DMA_Cmd(USART2_RX_DMA_CHANNEL, DISABLE); // 先关通道
        DMA_SetCurrDataCounter(USART2_RX_DMA_CHANNEL, BUFFER_SIZE);// NDTR 重新装入 BUFFER_SIZE
        DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);// 再开通道
    }
}
// DMA1通道3中断处理函数
void DMA1_Channel3_IRQHandler(void) {
    if(DMA_GetITStatus(DMA1_IT_TC3) != RESET) {
        // 获取实际传输的数据量
        uint16_t remaining = DMA_GetCurrDataCounter(USART3_RX_DMA_CHANNEL);
        rx3Buffer.index = BUFFER_SIZE - remaining;  // 计算实际接收的字节数
        DMA_ClearITPendingBit(DMA1_IT_TC3);
        // 重新配置DMA接收
        DMA_Cmd(USART3_RX_DMA_CHANNEL, DISABLE);
        DMA_SetCurrDataCounter(USART3_RX_DMA_CHANNEL, BUFFER_SIZE);
        DMA_Cmd(USART3_RX_DMA_CHANNEL, ENABLE);
    }
}
