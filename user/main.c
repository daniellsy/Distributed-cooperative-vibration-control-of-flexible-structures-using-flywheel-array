#include "stm32f10x.h"
#include "delay.h"
#include <stdio.h>
#include "string.h"
#include "USART.h"
#include "motor.h"
#include "I2C.h"

/* 为三组数据各定义一个滤波器实例 */
static Bw2Filter gyroZ_filter;
static Bw2Filter speed1_filter;
static Bw2Filter speed2_filter;
static Bw2Filter motor_filter;

int main(void)
{
	//Delay_Init();
	Rotor_PWM_Init();
	Motor_and_button_Init();
	TIM4_TimeBase_Init();
	TIM3_Encoder_Init();
	TIM2_Interval_Init();
	SysTick_Init();
  My_USART1_Init();
	

	NVIC_Configuration();
  GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
	Witconfig();
	delay_ms(500);
	SensorData sensorData = {0};
	txPacket.id = 0x12;
	uint8_t UPSTREAM_ID = txPacket.id-1;
	USART2_Init(115200);
	USART3_Init(115200);
	USART2_DMA_Init();
	USART3_DMA_Init();
	DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);  // 启动DMA接收
  DMA_Cmd(USART3_RX_DMA_CHANNEL, ENABLE);  // 启动DMA接收
	float lowfre = 5.0;
	float highfre = 200.0;
	float motor_speed = 0.0;
	float speed222=0;
	float angle222=0;
	/* 初始化一次（采样率 100 Hz，截止 5 Hz） */
	Bw2Filter_Init(&gyroZ_filter,   lowfre, highfre);
	Bw2Filter_Init(&speed1_filter,  lowfre, highfre);
	Bw2Filter_Init(&speed2_filter,  lowfre, highfre);
	Bw2Filter_Init(&motor_filter,  100, 1000);
		while (1){
			 // 更新数据包
			if (ParseDataPacket(&rxPacket2,&rx2Buffer) == 0){//解包函数，成功返回0，第1个为解包后存放数据的地址
					//sensorData.gyroz1=rxPacket2.gyroz;
					sensorData.gyroz1=Bw2Filter_Update(&speed1_filter, rxPacket2.gyroz);
					if (rxPacket2.id == UPSTREAM_ID)
					{
							g_ctrlState = rxPacket2.state;   // 与上级保持一致
					}
			
			}
			if (ParseDataPacket(&rxPacket3,&rx3Buffer) == 0){
					//sensorData.gyroz2=rxPacket3.gyroz;
					sensorData.gyroz2=Bw2Filter_Update(&speed2_filter, rxPacket3.gyroz);
				  speed222=rxPacket3.speed;
				  angle222=rxPacket3.angleyaw;
					if (rxPacket3.id == UPSTREAM_ID)
					{
							g_ctrlState = rxPacket3.state;   // 与上级保持一致
					}

			}
			
			if (g_ctrlState == WIT_start) {
            // 执行控制逻辑（例如读取传感器并驱动电机）
						//GPIO_SetBits(GPIOA, GPIO_Pin_11);				//失能刹车
						Sensor_ReadData(&sensorData,0);						//传感器读取
							
						txPacket.gyroz = sensorData.fgyroz ;				//DMA更新
						txPacket.speed = Motor_Speed ;
						txPacket.state = g_ctrlState ;
						USART3_DMA_Send_Config(&txPacket); 					//DMA
						USART2_DMA_Send_Config(&txPacket); 					//DMA

						printf("ID=0x%02X, v1=%.2f, vZ1=%.2f,s=%d,ID2=0x%02X, v2=%.2f, vZ2=%.2f,s2=%d, ID3=0x%02X, v3=%.2f, vZ3=%.2f s3=%d,\n", 
						txPacket.id, txPacket.speed, txPacket.gyroz,txPacket.state,
						rxPacket2.id, rxPacket2.speed, sensorData.gyroz2,rxPacket2.state,
						rxPacket3.id, rxPacket3.speed, sensorData.gyroz2-txPacket.gyroz/2,rxPacket3.state);
							//(sensorData.gyroz2-txPacket.gyroz/2)
						//ControlMotorSpeed(sensorData.fgyroz, 0); 
				 } else if(g_ctrlState == CTRL_start){
									GPIO_SetBits(GPIOA, GPIO_Pin_11);
									Sensor_ReadData(&sensorData,0);
										
									txPacket.gyroz = sensorData.fgyroz ;	
									txPacket.speed = Motor_Speed ;
									txPacket.state = g_ctrlState ;
					        txPacket.angleyaw = sensorData.angle;
									USART2_DMA_Send_Config(&txPacket); 					//DMA
									USART3_DMA_Send_Config(&txPacket); 
									SetMotorSpeedRPS(sensorData.fgyroz, motor_speed,sensorData.gyroz1, sensorData.gyroz2,0, txPacket.id,sensorData.angle,speed222,angle222); 
//									printf("Target = %.2f Actual = %.2f gyroZ= %.2f target1= %.2f del1= %.2f,del2= %.2f\n", 10.0, Motor_Speed,
//																			sensorData.gyroz1,sensorData.gyroz2,txPacket.gyroz,10.0);
									
        } else if(g_ctrlState == CTRL_co_run){
									GPIO_SetBits(GPIOA, GPIO_Pin_11);
									Sensor_ReadData(&sensorData,0);
										
									txPacket.gyroz = sensorData.fgyroz ;	
									txPacket.speed = Motor_Speed ;
									txPacket.state = g_ctrlState ;
					        txPacket.angleyaw = sensorData.angle;
									USART2_DMA_Send_Config(&txPacket); 					//DMA
									USART3_DMA_Send_Config(&txPacket); 
									SetMotorSpeedRPS(sensorData.fgyroz, motor_speed,sensorData.gyroz1, sensorData.gyroz2,1, txPacket.id,sensorData.angle,speed222,angle222); 

				} else if(g_ctrlState == CTRL_de_run){
									GPIO_SetBits(GPIOA, GPIO_Pin_11);
									Sensor_ReadData(&sensorData,0);
										
									txPacket.gyroz = sensorData.fgyroz ;	
									txPacket.speed = Motor_Speed ;
									txPacket.state = g_ctrlState ;
					        txPacket.angleyaw = sensorData.angle;
									USART2_DMA_Send_Config(&txPacket); 					//DMA
									USART3_DMA_Send_Config(&txPacket); 
									SetMotorSpeedRPS(sensorData.fgyroz, motor_speed,sensorData.gyroz1, sensorData.gyroz2,2, txPacket.id,sensorData.angle,speed222,angle222); 

        }else if(g_ctrlState == CTRL_stop){
								 GPIO_ResetBits(GPIOA, GPIO_Pin_11);
								
								 txPacket.gyroz = sensorData.fgyroz ;	
								 txPacket.speed = Motor_Speed ;
								 txPacket.state = g_ctrlState ;
					       txPacket.angleyaw = sensorData.angle;
								 USART2_DMA_Send_Config(&txPacket); 					//DMA
								 USART3_DMA_Send_Config(&txPacket); 
								 delay_ms(10);
								//delay_ms(10);
				}

}
}


