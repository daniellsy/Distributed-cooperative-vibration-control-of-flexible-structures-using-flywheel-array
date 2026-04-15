#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f10x.h"
#include "delay.h"

// 函数声明
void Rotor_PWM_Init(void);                // 初始化电机PWM
void Motor_and_button_Init(void);         // 初始化电机和按钮
void TIM3_Encoder_Init(void);             // 初始化编码器定时器
void TIM2_Interval_Init(void);            // 初始化定时器中断
int32_t Encoder_Get(void);                // 获取编码器值
uint8_t Button_Pressed(void);             // 检测按钮是否按下
void TIM4_TimeBase_Init(void);

// 外部变量声明
extern float Motor_Speed;                 // 电机转速（定义在其他源文件中）
#endif // MOTOR_H
