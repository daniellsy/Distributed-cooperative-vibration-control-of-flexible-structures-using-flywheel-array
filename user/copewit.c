//#include "copewit.h"
//#include "motor.h"
//static uint8_t s_ucWitDataBuff[256];
//static uint32_t s_uiWitDataCnt = 0, s_uiReadRegIndex = 0;
//int16_t sReg[REGSIZE];

//// 定义滤波器的参数
//#define FILTER_ORDER 2
//#define SAMPLE_RATE 100.0f // 采样频率
//#define CUTOFF_FREQ 5.0f   // 截止频率
//#define M_PI 3.1415926

//#define K 40.0 // 假设的比例系数，需要根据您的系统进行调整
//#define KI 1.1 // 假设的比例系数，需要根据您的系统进行调整
//#define DELTA_T 0.01 // 采样时间间隔，10ms
//float motorSpeed = 400.0; // 全局变量，用于存储电机当前速度
//float tarmotorSpeed = 25.0; // 全局变量，用于存储电机当前速度
//int isIncreasing = 1; // 使用 1 表示 true（递增），0 表示 false（递减）
//int control =0;
//uint16_t tarpwmValue=0;
//// 定义 PID 控制器参数
//float Kp = 300.0;  // 比例系数
//float Ki = 40;  // 积分系数
//float Kd = 0.05;  // 微分增益
//float error = 0.0;       // 当前误差
//float prev_error = 0.0;  // 上一次误差
//float integral = 0.0;    // 积分项
//float derivative = 0.0;  // 微分项
//float pwmValue = 300.0;    // PWM 值
//float filtered_pwmValue = 0.0; // 滤波后的PWM值
//float prev_pwmValue = 0.0; // 上一次滤波后的PWM值
//float prev_Motor_Speed = 0.0;
//float alpha = 0.3;       // 滤波系数，可以根据需要调整

//// 定义滤波器参数
//float beta = 0.5; // 滤波系数，可以根据需要调整
//float filteredGyroZ = 0.0; // 滤波后的陀螺仪Z轴数据

//// 巴特沃斯滤波器的系数
//static float b0 = 0.0200833655642112f;
//static float b1 = 0.0401667311284225f;
//static float b2 = 0.0200833655642112f;
//static float a1 = -1.56101807580072f;  // 注意这里是负数
//static float a2 = 0.641351538057563f;


//ButterworthFilterState gyroFilterState;
//ButterworthFilterState pwmFilterState;


/////重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//		/* 发送一个字节数据到串口DEBUG_USART */
//		USART_SendData(USART1, (uint8_t) ch);
//		
//		/* 等待发送完毕 */
//		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
//	
//		return (ch);
//}

/////重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
//int fgetc(FILE *f)
//{
//		/* 等待串口输入数据 */
//		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(USART1);
//}

//// 数据更新状态标记
//static uint8_t s_bDataUpdated[REGSIZE] = {0}; // 初始化为0，表示未更新

//static uint8_t __CaliSum(uint8_t *data, uint32_t len)
//{
//    uint32_t i;
//    uint8_t ucCheck = 0;
//    for(i=0; i<len; i++) ucCheck += *(data + i);
//    return ucCheck;
//}

//static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
//{
//    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
//    uint16_t *p_usReg1Val = p_data;
//    uint16_t *p_usReg2Val = p_data + 3;

//    uiReg1Len = 4; // 默认长度
//    switch(ucIndex)
//    {
//        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1; break;
//        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break;
//        case WIT_TIME:  uiReg1 = YYMM; 	break;
//        case WIT_GYRO:  uiReg1 = GX;     break;
//        case WIT_MAGNETIC: uiReg1 = HX; uiReg1Len = 3;   break;
//        case WIT_DPORT: uiReg1 = D0Status;   break;
//        case WIT_PRESS: uiReg1 = PressureL;  break;
//        case WIT_GPS:   uiReg1 = LonL;   break;
//        case WIT_VELOCITY: uiReg1 = GPSHeight;   break;
//        case WIT_QUATER:    uiReg1 = q0;   break;
//        case WIT_GSA:   uiReg1 = SVNUM;   break;
//        case WIT_REGVALUE:  uiReg1 = s_uiReadRegIndex;   break;
//		default:
//			return ;
//    }

//    // 处理寄存器数据
//    if (uiReg1Len)
//    {
//        memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len << 1); // 存储寄存器1的数据
//        for (uint32_t i = 0; i < uiReg1Len; i++) {
//            s_bDataUpdated[uiReg1 + i] = 1; // 标记寄存器1的数据为已更新
//        }
//    }
//    if (uiReg2Len)
//    {
//        memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len << 1); // 存储寄存器2的数据
//        for (uint32_t i = 0; i < uiReg2Len; i++) {
//            s_bDataUpdated[uiReg2 + i] = 1; // 标记寄存器2的数据为已更新
//        }
//    }
//}

//void ProcessWitProtocolNormal(uint8_t data)
//{
//    // 将接收到的数据存储到缓冲区
//    s_ucWitDataBuff[s_uiWitDataCnt++] = data;

//    // 处理普通协议 (WIT_PROTOCOL_NORMAL)
//    if (s_ucWitDataBuff[0] != 0x55)
//    {
//        // 如果起始字节不匹配，丢弃当前数据并重新同步
//        s_uiWitDataCnt--;
//        if (s_uiWitDataCnt > 0) memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
//        return;
//    }

//    if (s_uiWitDataCnt >= 11)
//    {
//        // 校验和计算
//        uint8_t ucSum = __CaliSum(s_ucWitDataBuff, 10);
//        if (ucSum != s_ucWitDataBuff[10])
//        {
//            // 校验失败，丢弃当前数据并重新同步
//            s_uiWitDataCnt--;
//            if (s_uiWitDataCnt > 0) memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
//            return;
//        }

//        // 解析数据
//        uint16_t usData[4];
//        usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | s_ucWitDataBuff[2];
//        usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | s_ucWitDataBuff[4];
//        usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | s_ucWitDataBuff[6];
//        usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | s_ucWitDataBuff[8];

//        // 调用数据处理函数
//        CopeWitData(s_ucWitDataBuff[1], usData, 4);

//        // 清空缓冲区
//        s_uiWitDataCnt = 0;
//    }
//}





//// 初始化滤波器状态
//void InitializeFilters() {
//    InitializeButterworthFilter(&gyroFilterState);
//    InitializeButterworthFilter(&pwmFilterState);
//}

//// 初始化滤波器状态
//void InitializeButterworthFilter(ButterworthFilterState* state) {
//    state->x1 = state->x2 = state->y1 = state->y2 = 0.0f;
//}

//float ButterworthFilter(float input, ButterworthFilterState* state) {
//    float x0 = input;
//    float y0 = b0 * x0 + b1 * state->x1 + b2 * state->x2 - a1 * state->y1 - a2 * state->y2;

//    // 更新缓存
//    state->x2 = state->x1;
//    state->x1 = x0;
//    state->y2 = state->y1;
//    state->y1 = y0;

//    return y0;
//}

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

//void ProcessSensorDataAndControlMotor(int control_state) {
//    float fGyro[3];
//    int i;

//    if (s_bDataUpdated[GX] || s_bDataUpdated[GY] || s_bDataUpdated[GZ]) {
//        for (i = 0; i < 3; i++) {
//            fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
//        }

//        // 使用巴特沃斯滤波器处理陀螺仪 Z 轴数据
//        float filteredGyroZ = ButterworthFilter(fGyro[2], &gyroFilterState);

//        // 打印原始和滤波后的数据
//        printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
//        printf("filtered gyro Z:%.3f\r\n", filteredGyroZ);
//        // 使用滤波后的数据控制电机
//        ControlMotorSpeed(filteredGyroZ,control_state);
//				//ControlMotor(filteredGyroZ);
//        // 清除标志位
//        s_bDataUpdated[GX] = 0;
//        s_bDataUpdated[GY] = 0;
//        s_bDataUpdated[GZ] = 0;
//    }
//}


//void ControlMotor(float gyroZ) {
//    // 根据陀螺仪 Z 轴数据调整电机速度
//    float omega = K * gyroZ; // 计算角速度对应的转速变化量
//			
//    // 更新电机速度
//    motorSpeed += omega * DELTA_T;

//    if (motorSpeed > 1000) {
//        pwmValue = 1000; // 如果超过 PWM 最大值，则设置为最大值
//    } else if (motorSpeed < 0) {
//        pwmValue = 0; // 如果低于 PWM 最小值，则设置为最小值
//    } else {
//        pwmValue = (uint16_t)motorSpeed; // 否则，使用计算得到的电机速度
//    }


////			if (isIncreasing) {
////					// 如果当前处于递增状态
////					if (pwmValue >= 400) {
////							// 达到或超过 600，切换到递减模式
////							pwmValue = 400; // 限制最大值
////							isIncreasing = 0; // 切换到递减模式
////					} else {
////							pwmValue += 10; // 继续递增
////					}
////			} else {
////					// 如果当前处于递减状态
////					if (pwmValue <= 100) {
////							// 降到或低于 200，切换到递增模式
////							pwmValue = 100; // 限制最小值
////							isIncreasing = 1; // 切换到递增模式
////					} else {
////							pwmValue -= 10; // 继续递减
////					}
////			}
//    // 设置 PWM 占空比
//    TIM_SetCompare1(TIM1, pwmValue);
//		
//    printf("Target: %.2f\r\n", tarmotorSpeed);
//    printf("Actual: %.2f\r\n", Motor_Speed);
//    printf("PWM:%f\r\n", pwmValue);
//}


//void ControlMotorSpeed(float gyroZ,int control_state) {
//		float fspeed=0;
//    fspeed = alpha * Motor_Speed + (1 - alpha) * prev_Motor_Speed;
//    prev_Motor_Speed = fspeed;// 根据陀螺仪 Z 轴数据调整目标转速
//	
//    float omega = KI * gyroZ; // 计算角速度对应的转速变化量
//    tarmotorSpeed += omega * DELTA_T;// 计算误差
//	
//    error = tarmotorSpeed - fspeed;// 计算积分项
//    integral += error * DELTA_T;// PID 控制器输出
//    pwmValue = Kp * error + Ki * integral ;
//    if (pwmValue > 1000) {
//        pwmValue = 1000; // 如果超过 PWM 最大值，则设置为最大值
//    } else if (pwmValue < 0) {
//        pwmValue = 0; // 如果低于 PWM 最小值，则设置为最小值
//    }
//    prev_error = error;
//		if(control_state==1){
//			tarpwmValue = (uint16_t)pwmValue;
//		}
//		else 
//		{
//			tarpwmValue = 400;
//		}
//    TIM_SetCompare1(TIM1, tarpwmValue);
//   
//    // 打印当前电机速度和目标转速
//    printf("Target: %.2f\r\n", tarmotorSpeed);
//    printf("Actual: %.2f\r\n", Motor_Speed);
//    printf("PWM:%d\r\n", tarpwmValue);
//		
//}




