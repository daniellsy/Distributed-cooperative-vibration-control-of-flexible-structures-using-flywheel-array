#include "I2C.h"
#include "delay.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"



#define REGSIZE 0x90
//static SerialWrite p_WitSerialWriteFunc = NULL;
static WitI2cWrite p_WitI2cWriteFunc = NULL;//参考头文件里面定义函数指针类型
static WitI2cRead p_WitI2cReadFunc = NULL;
//static CanWrite p_WitCanWriteFunc = NULL;
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
static DelaymsCb p_WitDelaymsFunc = NULL;



static uint8_t s_ucAddr = 0xff;
static uint8_t s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];//I2C读取数据缓冲区
static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0, s_uiReadRegIndex = 0;
int16_t sReg[REGSIZE];
char s_cDataUpdate;//记录sReg内数据情况

int state_gyroz = 0 ;
int state_angle = 0 ;
const float epsilon = 0.001f;    // 浮点数比较误差阈值

static float b0 = 0.0200833655642112f;
static float b1 = 0.0401667311284225f;
static float b2 = 0.0200833655642112f;
static float a1 = -1.56101807580072f;  // 注意这里是负数
static float a2 = 0.641351538057563f;
ButterworthFilterState gyroFilterState;
ButterworthFilterState pwmFilterState;

#define K 4.0 // 假设的比例系数，需要根据您的系统进行调整
#define KI 0.11 // 假设的比例系数，需要根据您的系统进行调整
#define DELTA_T 0.01 // 采样时间间隔，10ms
float motorSpeed = 20.0; // 全局变量，用于存储电机当前速度
float tarmotorSpeed = 10.0; // 全局变量，用于存储电机当前速度
int isIncreasing = 1; // 使用 1 表示 true（递增），0 表示 false（递减）
int control =0;
uint16_t tarpwmValue=0;


float Kp = 4.0;  // 比例系数
float Ki = 40.0;  // 积分系数
float Kd = 0.02; 
float error = 0.0;       // 当前误差
float prev_error = 0.0;  // 上一次误差
float integral2 = 0.0;    // 积分项
float integral = 0.0;    // 积分项
float derivative = 0.0;  // 微分项
float pwmValue = 20.0;    // PWM 值
float filtered_pwmValue = 0.0; // 滤波后的PWM值
float prev_pwmValue = 0.0; // 上一次滤波后的PWM值
float prev_Motor_Speed = 0.0;
float alpha = 0.3;       // 滤波系数，可以根据需要调整

#define FuncW 0x06
#define FuncR 0x03
void IIC_Init(void)
{			
	IIC_Recover(); // 先释放总线
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SDA_OUT();     
	IIC_SDA=1;	  	  
	IIC_SCL=1;
}

void IIC_Recover(void) {
    SDA_OUT();
    IIC_SCL = 1;//搭配头文件里面的使用，解引用赋值给
    for (int i = 0; i < 9; i++) {//重复拍9次每次拉高SDA相当于NACK确保卡住的帧走完，并生成NACK
        IIC_SCL = 1;
        Delay(5);
        IIC_SDA = 1;
        Delay(5);
        IIC_SCL = 0;
        Delay(5);
    }
    IIC_Stop();
}

void IIC_Start(void)//生成START信号在SCL高的时候拉低SDA
{
	SDA_OUT();    
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	
	Delay(2);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	
	Delay(2);
	IIC_SCL=0;
}
	  
void IIC_Stop(void)//生成STOP信号在SDA高的时候拉高SDA
{
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	
		Delay(2);
	IIC_SCL=1; 
	IIC_SDA=1;
	
		Delay(2);							   	
}

u8 IIC_Wait_Ack(void)//函数功能为等待ACK
{
	u8 ucErrTime=0; 
	SDA_IN();     
	IIC_SDA=1;
		Delay(2);	  
	while(READ_SDA)//参考头文件定义，读取输入端SDA，看看是否拉低，拉低退出循环
	{
		ucErrTime++;//累加错误计数
		if(ucErrTime>50)//超过50认为无应答直接暂停
		{
			IIC_Stop();
			return 1;
		}
		Delay(2);//延时
	}  
	IIC_SCL=1;
	Delay(2); 
	IIC_SCL=0;
	return 0;  
} 

void IIC_Ack(void)//生成ACK信号
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
		Delay(2);
	IIC_SCL=1;
		Delay(2);
	IIC_SCL=0;
}
	    
void IIC_NAck(void)//生成NACK信号
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	
		Delay(2);
	IIC_SCL=1;
		Delay(2);
	IIC_SCL=0;
}					 				     
		  
void IIC_Send_Byte(u8 txd)//写数据参数是待写入数据
{                        
    u8 t; 
		SDA_OUT(); 	    
    IIC_SCL=0;
    for(t=0;t<8;t++)
    {              
			IIC_SDA=(txd&0x80)>>7;//按位发送
        txd<<=1; 	  
			
		Delay(2);   
		IIC_SCL=1;
		Delay(2);
		IIC_SCL=0;	
		Delay(2);
    }	 
} 	 
  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        
		Delay(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++; //按位读取1就给最低位赋值1，然后位移  
		
		Delay(2); 
    }					 
	  if (ack)//ack为1，发送ACK
        IIC_Ack();//继续读 
    else
        IIC_NAck();//读到此为止
    return receive;
}

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)//调用上面函数组合完成一次传输包括发送设备地址寄存器地址，存放数据缓冲区，读取长度
{//第一个参数为地址，第二个为读取从机寄存器地址，第三个为缓冲区指针，第四个为读取字节数
    uint32_t count = 0;

    IIC_Start();
    IIC_Send_Byte(dev);	
    if(IIC_Wait_Ack() == 1)return 0;//为1表示无应答直接退出
    IIC_Send_Byte(reg);
    if(IIC_Wait_Ack() == 1)return 0;
    IIC_Start();
    IIC_Send_Byte(dev+1); //地址最低位表示读或者写，加一表示切换到读取模式
    if(IIC_Wait_Ack() == 1)return 0;

    for(count=0; count<length; count++)
    {
        if(count!=length-1)data[count]=IIC_Read_Byte(1);
        else  data[count]=IIC_Read_Byte(0);	 
    }
    IIC_Stop();
    return 1;
}


int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length)
{
    uint32_t count = 0;
    IIC_Start();
    IIC_Send_Byte(dev);	   
    if(IIC_Wait_Ack() == 1)return 0;
    IIC_Send_Byte(reg);   
    if(IIC_Wait_Ack() == 1)return 0;
    for(count=0; count<length; count++)
    {
        IIC_Send_Byte(data[count]);
        if(IIC_Wait_Ack() == 1)return 0;
    }
    IIC_Stop();

    return 1; 
}
void Witconfig(void)//写注册表，相当于将WIT顶层与底层函数绑定，用哪些函数
{
	IIC_Init();
	WitInit(WIT_PROTOCOL_I2C, 0x50);
	WitI2cFuncRegister(IICwriteBytes, IICreadBytes);//绑定底层实现方式
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(Delayms);
	printf("Welcome to use!\r\n");
	AutoScanSensor();
	InitializeFilters();
}

int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;//检查协议是否合法取值为0到3分别代表不同协议
    s_uiProtoclo = uiProtocol;
    s_ucAddr = ucAddr;//传7位地址，实际用会左移一位加上R/W判断位，从机地址
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}

int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func)//传递指向该函数的指针
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}

int32_t WitRegisterCallBack(RegUpdateCb update_func)//为顶层注册所用底层函数
{
    if(!update_func)return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}

int32_t WitDelayMsRegister(DelaymsCb delayms_func)//为顶层注册所用底层函数
{
    if(!delayms_func)return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;//检查是否越界
    switch(s_uiProtoclo)
    {
        case WIT_PROTOCOL_I2C://看看是否定义为I2C协议
            if(p_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;//一个寄存器16位，把寄存器个数转换成字节个数即乘2
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(p_WitI2cReadFunc(s_ucAddr << 1, uiReg, s_ucWitDataBuff, usTemp) == 1)//7位地址左移留一个R/W位表示读还是写，读取函数会自动给该位改成读
            {//在 C 里，数组名在表达式中会“衰变”为指向首元素的指针第三个参数传数组名没问题，第一个地址第二个寄存器地址第三个指针第四个字节数
                if(p_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    sReg[i+uiReg] = ((uint16_t)s_ucWitDataBuff[(i<<1)+1] << 8) | s_ucWitDataBuff[i<<1];//最内部左移乘2，转成16位然后左移八位，表示高位数字与低位相或即可
                }
                p_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
			
            break;
		default: 
            return WIT_HAL_INVAL;
    }
    s_uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {//读取后调用，更新标记，告诉主循环什么数据得到更新了
    for (int i = 0; i < uiRegNum; i++) {
        switch (uiReg) {
            // 陀螺仪 GX/GY/GZ
//            case GX: case GY:
						case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
						case Yaw:
			          s_cDataUpdate |= ANGLE_UPDATE;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}
static void AutoScanSensor(void)
{
	int i, iRetry;
	for (i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(GZ, 1);
			delay_ms(5);
			if (s_cDataUpdate != 0)
			{
				printf("find %02X addr sensor\r\n", i);
				return;
			}
			iRetry--;
		} while (iRetry);
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

static void Delayms(uint16_t ucMs)
{
	delay_ms(ucMs);
}

// 修改：仅读取陀螺仪 Z 轴寄存器（假设 GZ 的寄存器地址为 0x05）
void Sensor_ReadData(SensorData *pData,int show) {
    WitReadReg(GZ, 1); // 只读 GZ 寄存器
	  WitReadReg(Yaw,1);
    if (s_cDataUpdate & GYRO_UPDATE) {
				
			if(state_gyroz != sReg[GZ]){
			state_gyroz = sReg[GZ];
			pData->gyroz = state_gyroz / 32768.0f * 2000.0f;
			pData->fgyroz = ButterworthFilter(pData->gyroz, &gyroFilterState);//送入滤波器后再写入到结构体中的fgyroz
			if(show){			
			printf("gyroz=%.3f\r\n", pData->gyroz);
			printf("filtered gyro Z=%.3f\r\n", pData->fgyroz);}
			}
        s_cDataUpdate &= ~GYRO_UPDATE;//清标志，表示这次已经处理完了
    }
		if (s_cDataUpdate & ANGLE_UPDATE) {
				
			if(state_angle != sReg[Yaw]){
			state_angle = sReg[Yaw];
			pData->angle = state_angle / 32768.0f * 180.0f;//送入滤波器后再写入到结构体中的fgyroz
			if(show){			
			printf("gyroz=%.3f\r\n", pData->gyroz);
			printf("filtered gyro Z=%.3f\r\n", pData->fgyroz);}
			}
        s_cDataUpdate &= ~ANGLE_UPDATE;//清标志，表示这次已经处理完了
    }
}

//void printacc(void) {
//    float fAcc[3], fGyro[3], fAngle[3];
//    int i;
//		if (s_bDataUpdated[GX] || s_bDataUpdated[GY] || s_bDataUpdated[GZ]) {
//        for (i = 0; i < 3; i++) {
//            fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
//        }

//        // 使用巴特沃斯滤波器处理陀螺仪 Z 轴数据
//        float filteredGyroZ = ButterworthFilter(fGyro[2], &gyroFilterState);

//        // 打印原始和滤波后的数据
//        printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
//        printf("filtered gyro Z:%.3f\r\n", filteredGyroZ);
//        // 使用滤波后的数据控制电机
//        ControlMotorSpeed(filteredGyroZ,0);
//				//ControlMotor(filteredGyroZ);
//        // 清除标志位
//        s_bDataUpdated[GX] = 0;
//        s_bDataUpdated[GY] = 0;
//        s_bDataUpdated[GZ] = 0;
//    }
//}

// 初始化滤波器状态
void InitializeFilters() {
    InitializeButterworthFilter(&gyroFilterState);//清零，传结构体地址
    InitializeButterworthFilter(&pwmFilterState);
}

// 初始化滤波器状态
void InitializeButterworthFilter(ButterworthFilterState* state) {
    state->x1 = state->x2 = state->y1 = state->y2 = 0.0f;
}

float ButterworthFilter(float input, ButterworthFilterState* state) {//固定系数的巴特沃斯滤波
    float x0 = input;
    float y0 = b0 * x0 + b1 * state->x1 + b2 * state->x2 - a1 * state->y1 - a2 * state->y2;

    // 更新缓存
    state->x2 = state->x1;
    state->x1 = x0;
    state->y2 = state->y1;
    state->y1 = y0;

    return y0;
}

void Bw2Filter_Init(Bw2Filter *f, float cutOff_Hz, float sample_Hz)//参数一结构体指针
{
    float  fs = sample_Hz;
    float  fc = cutOff_Hz;
    float  omega = 2.0f * 3.1415926f * fc / fs;
    float  sinO = sinf(omega);
    float  cosO = cosf(omega);
    float  alpha = sinO * sqrtf(2.0f);          // 2阶巴特沃斯 Q=1/sqrt(2)

    /* 计算双线性变换后的系数 */
    float  b0 = alpha;
    float  b1 = 0.0f;
    float  b2 = -alpha;
    float  a0 = 1.0f + alpha;
    float  a1 = -2.0f * cosO;
    float  a2 = 1.0f - alpha;

    /* 归一化，使 a0=1 */
    f->b0 = b0 / a0;
    f->b1 = b1 / a0;
    f->b2 = b2 / a0;
    f->a1 = a1 / a0;   // 注意：公式里已经带负号
    f->a2 = a2 / a0;

    /* 延迟线清零 */
    f->x1 = f->x2 = 0.0f;
    f->y1 = f->y2 = 0.0f;
}

/* 单步滤波：每来一个新数据调用一次 */
float Bw2Filter_Update(Bw2Filter *f, float in)
{
    float out = f->b0 * in + f->b1 * f->x1 + f->b2 * f->x2
                - f->a1 * f->y1 - f->a2 * f->y2;

    /* 更新延迟线 */
    f->x2 = f->x1;  f->x1 = in;
    f->y2 = f->y1;  f->y1 = out;

    return out;
}

void ControlMotorSpeed(float gyroZ,int control_state) {
		float fspeed=0;
    fspeed = alpha * Motor_Speed + (1 - alpha) * prev_Motor_Speed;
    prev_Motor_Speed = fspeed;// 根据陀螺仪 Z 轴数据调整目标转速
	
    float omega = KI * gyroZ; // 计算角速度对应的转速变化量
    tarmotorSpeed += omega * DELTA_T;// 计算误差
	
    error = tarmotorSpeed - fspeed;// 计算积分项
    integral += error * DELTA_T;// PID 控制器输出
    pwmValue = Kp * error + Ki * integral ;
    if (pwmValue > 20) {
        pwmValue = 20; // 如果超过 PWM 最大值，则设置为最大值
    } else if (pwmValue < 0) {
        pwmValue = 0; // 如果低于 PWM 最小值，则设置为最小值
    }
    prev_error = error;
		if(control_state==1){
			tarpwmValue = (uint16_t)pwmValue;
		}
		else 
		{
			tarpwmValue = 40;
		}
    TIM_SetCompare1(TIM1, tarpwmValue);
   
    // 打印当前电机速度和目标转速
    printf("Target = %.2f\r\n", tarmotorSpeed);
    printf("Actual = %.2f\r\n", Motor_Speed);
    printf("PWM = %d\r\n", tarpwmValue);
	}

void ControlMotor(float gyroZ) {
    // 根据陀螺仪 Z 轴数据调整电机速度
    float omega = K * gyroZ ; // 计算角速度对应的转速变化量
			
    // 更新电机速度
    motorSpeed += omega * DELTA_T;

    if (motorSpeed > 100) {
        pwmValue = 100; // 如果超过 PWM 最大值，则设置为最大值
    } else if (motorSpeed < 0) {
        pwmValue = 0; // 如果低于 PWM 最小值，则设置为最小值
    } else {
        pwmValue = (uint16_t)motorSpeed; // 否则，使用计算得到的电机速度
    }
    // 打印当前电机速度和目标转速
    printf("Target = %.2f\r\n", motorSpeed);
    printf("Actual = %.2f\r\n", Motor_Speed);
    printf("PWM = %f\r\n", pwmValue);
}

/*  新增：纯转速闭环 PI 调节
    target_rpm : 期望转速（单位与 Motor_Speed 保持一致）
    control_state : 1-闭环运行  0-直接给固定占空比 40
*/
void ControlMotorSpeed_PI(float target_speed,float motor_speed, int control_state)
{
    /* 1. 一阶低通滤波，抑制速度采样噪声 */
//    float fspeed = alpha * Motor_Speed + (1.0f - alpha) * prev_Motor_Speed;
//    prev_Motor_Speed = fspeed;

    /* 2. 计算误差（PI 的 P 项） */
    float error = target_speed - motor_speed;//fspeed

    /* 3. 积分分离：只在误差不大时积分，防止饱和 */
//    if (fabsf(error) < 20.0f)          /* 200 只是示例，按实际调 */
    integral2 += error * DELTA_T;
	
		/* 4. 微分项（新增）*/
    float derivative = (error - prev_error) / DELTA_T;
    prev_error = error;          /* 为下一次做准备 */
	
    /* 4. PI 输出 */
    float pwmValue = Kp * error + Ki * integral2 + Kd * derivative;

    /* 5. 输出限幅 */
    if (pwmValue > 80.0f) {
        pwmValue = 80.0f;
    } else if (pwmValue < 0.0f) {
        pwmValue = 0.0f;
    }

    /* 6. 根据 control_state 决定最终占空比 */
    uint16_t tarpwmValue;
    if (control_state == 1)
        tarpwmValue = (uint16_t)pwmValue;
    else
        tarpwmValue = 40;

    TIM_SetCompare1(TIM1, tarpwmValue);

    /* 7. 打印调试信息 */
    //printf("Target = %.2f Actual = %.2f PWM= %d \n", target_speed, Motor_Speed,  tarpwmValue);
}

//void SetMotorSpeedRPS(float gyroZ,float motor_speed,float gyroZ1,float gyroZ2, uint8_t mode)
//{
//    /* 1. 静态变量：记录当前正在执行的“斜坡目标” */
//    static float ramp_target = 30.0f;          // 单位 rps
//    static uint16_t weight = 0;            // ms 时间戳
//		
//		static float c123 = 0.06;  //0.1太大了

//		if(mode==1){
//			weight = 1;
//		}
//		else 
//		{
//			weight = 0;
//		}
//		float del1 = gyroZ1-gyroZ/2;
//		float del2 = gyroZ2-gyroZ/2;
//		float tempz;
//		if (gyroZ<0.2&&gyroZ>-0.2) tempz=0; else tempz = gyroZ;
//		if (del1<0.2&&del1>-0.2) del1=0;
//		if (del2<0.2&&del2>-0.2) del2=0;
//		
//		ramp_target = ramp_target+ weight*gyroZ*0.01 - c123*weight*del1 +  c123*weight*del2;//
//		//ramp_target = ramp_target+ weight*gyroZ*0.04 -c123*weight*del1- c123*weight*del2;
//		float ramp_target1 =ramp_target+ gyroZ*0.02- c123*del2;
//    /* 4. 速度丢给 PI 调节器 */
//		    /* 3. 最终限幅（电机机械允许最大 45 rps） */
//    if(ramp_target > 45.0f) ramp_target = 45.0f;
//    if(ramp_target < 0.0f)  ramp_target = 0.0f;
//    ControlMotorSpeed_PI(ramp_target,Motor_Speed, 1);   // 1 = 闭环状态
//		printf("Target = %.2f Actual = %.2f gyroZ= %.2f target1= %.2f del1= %.2f,del2= %.2f\n", ramp_target, Motor_Speed,gyroZ,gyroZ-del2,gyroZ2,-del2);
//}

typedef struct {
    uint8_t id;
    float   k_del1;   /* del1 的系数 */
    float   k_del2;   /* del2 的系数 */
} DiffGain_t;

//static const DiffGain_t DIFF_TAB[] = {
//    {0x11,  0.0f,  -0.16f},
//    {0x12, -0.05f,  -0.05f},
//    {0x13, -0.17f,  -0.17f},
//    {0x14,  -1.2f,  0.0f},
//};//定义一个结构体数组,[] 让编译器按初始化项个数自动推断长度
static const DiffGain_t DIFF_TAB[] = {
    {0x11,  0.0f,  -0.16f},
    {0x12, -0.2f,  -0.2f},
    {0x13, -0.17f,  -0.17f},
    {0x14,  -1.2f,  0.0f},
};//定义一个结构体数组,[] 让编译器按初始化项个数自动推断长度 for acutator 2 failure
#define DIFF_TAB_SZ  (sizeof(DIFF_TAB)/sizeof(DIFF_TAB[0]))

/* 查表辅助函数 ------------------------------------------------------*/
static void get_diff_gain(uint8_t dev_id, float *k1, float *k2)
{
    for (uint8_t i = 0; i < DIFF_TAB_SZ; i++) {
        if (DIFF_TAB[i].id == dev_id) {
            *k1 = DIFF_TAB[i].k_del1;
            *k2 = DIFF_TAB[i].k_del2;
            return;
        }
    }
    /* 默认系数：原 mode-1 逻辑 */
    *k1 = 0.0f; *k2 = 0.0f;
}

void SetMotorSpeedRPS(float gyroZ, float motor_speed,
                      float gyroZ1, float gyroZ2, uint8_t mode, uint8_t dev_id,float angle11,float speed22,float angle22)
{
    /* 静态变量 ----------------------------------------------------------*/
    static float ramp_target = 30.0f;      // 当前斜坡目标  [rps]
//    static float c123      = 0.008f;     // 差速权重 for actuator 134
    static float c123      = -0.08f;       // 差速权重 for acutator2
    /* 1. 根据模式决定权重 -----------------------------------------------*/
		uint16_t weight = (mode == 1 || mode == 2) ? 1 : 0;

    /* 2. 输入死区 -------------------------------------------------------*/
    float tempz = (gyroZ > -2.0f && gyroZ < 2.0f) ? 0.0f : gyroZ;
    float del1  = gyroZ1 - gyroZ * 1.0f;
    float del2  = gyroZ2 - gyroZ * 1.0f;
    if (del1 > -0.1f && del1 < 0.1f) del1 = 0.0f;
    if (del2 > -0.1f && del2 < 0.1f) del2 = 0.0f;

    /* 3. 斜坡目标更新 ---------------------------------------------------*/
    ramp_target += weight * tempz * 0.02f;          // 始终积分
    if (mode == 1) {
        float k1, k2;
        get_diff_gain(dev_id, &k1, &k2);
        ramp_target += c123 * weight * (k1 * del1 + k2 * del2);
    }
    /* mode == 2 时，差速项完全跳过 */

    /* 4. 限幅 ----------------------------------------------------------*/
    if (ramp_target > 45.0f) ramp_target = 45.0f;
    if (ramp_target <  0.0f) ramp_target =  0.0f;

    /* 5. 交给 PI -------------------------------------------------------*/
    ControlMotorSpeed_PI(ramp_target, Motor_Speed, 1);

    /* 6. 调试输出 -------------------------------------------------------*/
    printf("Target = %.2f  Actual = %.2f  gyroZ = %.2f  target1=%.2f  del1 = %.2f  del2 = %.2f   \r\n",Motor_Speed, gyroZ, angle11, speed22, gyroZ2, angle22);
}


