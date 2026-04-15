#include "delay.h"

__IO uint64_t ulTicks;


//
// @简介：初始化延迟函数
// @返回值：无
//
void Delay_Init(void)
{
	RCC_ClocksTypeDef clockinfo = {0};
	uint32_t tmp;
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE; // 禁止SYSTICK

	ulTicks = 0;

	RCC_GetClocksFreq(&clockinfo);

	SysTick->CTRL |= SysTick_CTRL_TICKINT; // 开启中断

	// 设置中断优先级为0
	SCB->SHP[7] = 0;

	// 设置自动重装值以保证1ms的时钟
	tmp =  clockinfo.HCLK_Frequency / 1000;
	if(tmp > 0x00ffffff)
	{
		tmp = tmp / 8;
		SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE; 
	}
	else
	{
		SysTick->CTRL |= SysTick_CTRL_CLKSOURCE; 
	}
	SysTick->LOAD = tmp - 1;

	SysTick->CTRL |= SysTick_CTRL_ENABLE; 
}


void Delay(u32 count)
{
	unsigned int uiCnt = count*8;
	while (uiCnt --);
}

// @简介：获取当前时间，以毫秒（千分之一秒）为单位
// @参数：无
// @返回值：当前时间，单位为毫秒（千分之一秒）
uint64_t GetTick(void)
{
	return ulTicks;
}
static uint16_t  fac_us=0;
static uint16_t fac_ms=0;

static volatile uint32_t sv_uiDelay = 0;
void SysTick_Init(void)
{	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us=SystemCoreClock/8000000;
	fac_ms=(uint16_t)fac_us*1000;
}

void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;
	SysTick->VAL =0x00;
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL =0X00;	    
} 
