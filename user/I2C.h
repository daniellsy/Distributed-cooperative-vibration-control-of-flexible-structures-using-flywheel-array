#ifndef __IOI2C_H
#define __IOI2C_H
#include "stm32f10x.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "reg.h"
#include <math.h>
#include "motor.h"

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))//Cortex-M3 的一个“把寄存器中的一个 bit 映射成一块可单独读写的“别名地址””的机制
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))//宏定义把一个地址强转为指针并解引用
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))//结合前两个定义一个宏 

// 修改为PB6和PB7的地址
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C ORD寄存器可以驱动输出上拉还是下拉，也可以读但是读的是上次写入的锁存值，不一定是真实电平
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  //0x40010C08 读实际电平

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)//通过宏得到一个指针并解引用 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)   

// 修改为PB6和PB7的引脚定义
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x80000000;}//配置CRL寄存器其管PB0~PB7
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x30000000;}//GPIOB 被定义为指向 GPIO 外设寄存器块的指针（GPIO_TypeDef *），其基地址是 GPIOB_BASE（0x40010C00）。
	 
#define IIC_SCL    PBout(6) // SCL (PB6)参考上面
#define IIC_SDA    PBout(7) // SDA (PB7)
#define READ_SDA   PBin(7)  // 输入模式读取SDA (PB7)

// 函数声明保持不变
void IIC_Init(void);                			 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(uint8_t txd);			
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);//依旧函数声明，功能如名字所示
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);


#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN    2
#define WIT_PROTOCOL_I2C    3
#define REGSIZE 0x90

#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80

// 传感器数据结构体
typedef struct {
    float acc[3];    // 加速度 (m/s2)
    float gyro[3];   // 陀螺仪 (deg/s)
    float angle;  // 欧拉角 (度)
    int16_t mag[3];  // 磁力计原始数据
		float gyroz;  // 欧拉角 (度)
		float fgyroz;
		float gyroz1;
		float gyroz2;
} SensorData;

typedef struct {
    float x1, x2; // 输入缓存
    float y1, y2; // 输出缓存
} ButterworthFilterState;


typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);//函数指针类型，之后可以通过WitI2cWrite 函数名这种形式调用
typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
typedef void (*DelaymsCb)(uint16_t ucMs);


int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);//函数声明
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);
int32_t WitRegisterCallBack(RegUpdateCb update_func);
int32_t WitDelayMsRegister(DelaymsCb delayms_func);
void Sensor_ReadData(SensorData *pData,int show);
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
void AutoScanSensor(void);
void Delayms(uint16_t ucMs);
void Witconfig(void);
void IIC_Recover(void);
//滤波器
void InitializeFilters();
void InitializeButterworthFilter(ButterworthFilterState* state);
float ButterworthFilter(float input, ButterworthFilterState* state);
//电机
void ControlMotorSpeed(float gyroZ,int control_state);
void ControlMotor(float gyroZ);

extern int16_t sReg[REGSIZE];//16位数组一共REGSIZE个
extern  char s_cDataUpdate;  // 引用主文件中的全局变量

/* --------------  2阶巴特沃斯结构体  -------------- */
typedef struct {
    float x1, x2;        // 输入延迟线
    float y1, y2;        // 输出延迟线
    /* 下面三个系数在初始化时一次性算好，滤波时直接乘 */
    float b0, b1, b2;
    float a1, a2;        // 注意符号：滤波公式里 a1/a2 已经带负号
} Bw2Filter;

/* --------------  用户接口  -------------- */
void  Bw2Filter_Init(Bw2Filter *f, float cutOff_Hz, float sample_Hz);
float Bw2Filter_Update(Bw2Filter *f, float in);
void SetMotorSpeedRPS(float gyroZ,float motor_speed,float gyroZ1,float gyroZ2, uint8_t mode, uint8_t dev_id,float angle11,float speed22,float angle22);
#endif

