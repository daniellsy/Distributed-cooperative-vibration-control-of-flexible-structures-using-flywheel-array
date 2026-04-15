#include "motor.h"

float Motor_Speed =0;
static int16_t last_encoder = 0;
static uint32_t last_time1 = 0;
#define ENCODER_PPR 20 // 编码器每转脉冲数（根据实际修改）
static uint16_t last_counter = 0; // 上一次的计数器值
static int32_t total_pulses = 0;  // 累积的脉冲总数（考虑溢出）
uint16_t arr = 5;

void Rotor_PWM_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 99;  // ARR
    TIM_TimeBaseInitStruct.TIM_Prescaler = 71;  // PSC
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
    
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 500;  
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

void Motor_and_button_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;//button
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;//button
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;//button
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
}

uint8_t Button_Pressed(void)
{
    static uint8_t button_state = 0;  
    uint8_t current_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1); 

    if (current_state == 0 && button_state == 0) 
    {
        Delay(10);
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0) 
        {
            button_state = 1; 
            return 1;         
        }
    }
    else if (current_state == 1 && button_state == 1) 
    {
        Delay(10); 
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1) 
        {
            button_state = 0; 
        }
    }
		return 0;
     
}

void TIM3_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 配置TIM3的CH1和CH2这两个通道对应端口为PA6和PA7
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // PA6 和 PA7
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 输入浮空
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置定时器基本参数
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStruct.TIM_Period = arr; // 自动重装载值（ARR）
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0; // 不分频
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    // 配置通道1和通道2为输入捕获
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿触发
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直接输入
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 不分频
    TIM_ICInitStruct.TIM_ICFilter = 0xF; // 输入滤波
    TIM_ICInit(TIM3, &TIM_ICInitStruct);

    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStruct);

    // 配置定时器为编码器模式
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//确定所有中断的分组方法

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
		NVIC_Init(&NVIC_InitStructure);
	

    // 启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

int32_t Encoder_Get(void)
{
    uint16_t current_counter = TIM_GetCounter(TIM3); // 获取当前计数器值
    int16_t delta_pulses;

    // 计算脉冲差（处理计数器溢出）
    if (current_counter >= last_counter) {
        delta_pulses = current_counter - last_counter;
    } else {
        delta_pulses = (0xFFFF - last_counter) + current_counter + 1;
    }

    // 更新累积脉冲总数
    total_pulses += delta_pulses;
    last_counter = current_counter;

    return total_pulses;
}

void TIM2_Interval_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}

// TIM2 每 10 ms 读一次 TIM3->CNT
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}


void TIM4_TimeBase_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF; // 自动重装载值（16位最大值）
    TIM_TimeBaseInitStruct.TIM_Prescaler = 720 - 1; // 72MHz /72 = 1MHz → 1us/计数
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
    
    TIM_Cmd(TIM4, ENABLE);
}

volatile int32_t last_pulses = 0; // 上一次的脉冲计数

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) // 判断是否是更新中断
    {
        int32_t current_pulses = TIM_GetCounter(TIM3); // 获取TIM3的脉冲计数
        uint32_t current_time = TIM_GetCounter(TIM4); // 获取TIM4的时间戳

        // 计算时间差（处理计数器溢出）
        uint32_t delta_time;
        if (current_time >= last_time1) {
            delta_time = current_time - last_time1;
        } else {
            delta_time = (0xFFFF - last_time1) + current_time + 1;  // 处理溢出
        }

        // 保存当前值为下一次计算
        

        // 计算转速（MT法）
        if (delta_time > 0)
        {
            float time_sec = delta_time * 1e-5f; // 时间差转换为秒
            //printf("%f %d %d %d %d\n", time_sec,current_time,last_time1, delta_time,current_pulses);
            // 假设每次中断时，编码器的脉冲数为 5
						float f_arr = (float)arr + 1.0;
            Motor_Speed = (f_arr / time_sec) / ENCODER_PPR; // 转速计算
        }
        else
        {
            Motor_Speed = 0;
        }
				if (Motor_Speed>60){
				Motor_Speed=10;
				}
				last_time1 = current_time;
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志
    }
}


