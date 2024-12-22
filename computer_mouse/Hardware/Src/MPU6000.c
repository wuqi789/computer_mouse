#include "MPU6000.h"
#include <math.h>
#include "main.h"
#include "spi.h"
//#include "arm_math.h" 
#include "hardware.h" 

void MPUCS()    //CS信号拉低
{
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_RESET);
		DelayUs(20);
}

void MPUnCS() //CS信号拉高
{
		DelayUs(20);
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
}

uint8_t MPUReadReg(uint8_t RegAddr)
{
    uint8_t RegData,Reg;
    Reg = 0x80 | RegAddr;
    MPUCS();
    HAL_SPI_Transmit(&hspi1,&Reg,1,100);
    HAL_SPI_Receive(&hspi1,&RegData,1,100);
    MPUnCS();
    return RegData;
}

void MPUWriteReg(uint8_t RegAddr,uint8_t RegData)
{
    MPUCS();
    HAL_SPI_Transmit(&hspi1,&RegAddr,1,100);
    HAL_SPI_Transmit(&hspi1,&RegData,1,100);
    MPUnCS();
}

void MPUReadBuffer(uint8_t RegAddr,uint8_t len,uint8_t *RegData)
{
    uint8_t Reg;
    Reg = 0x80 | RegAddr;
    MPUCS();
    HAL_SPI_Transmit(&hspi1,&Reg,1,100);
    HAL_SPI_Receive(&hspi1,RegData,len,100);
    MPUnCS();
}
uint8_t ucMPUTBuf[32],ucMPURBuf[32];
uint8_t MPUReadBufferDMA(uint8_t RegAddr,uint8_t len)
{
		if(len>31)
			return 2;
//		memset(ucMPUTBuf,0xFF,32);
//		memset(ucMPURBuf,0x00,32);
    ucMPUTBuf[0] = 0x80 | RegAddr;
    MPUCS();
    if(HAL_OK != HAL_SPI_TransmitReceive_DMA(&hspi1,ucMPUTBuf,ucMPURBuf,len+1))
		{
			MPUnCS();
			HAL_SPI_DMAStop(&hspi1);
			return 1;
		}
		return 0;
}

void MPUWriteBuffer(uint8_t RegAddr,uint8_t len,uint8_t *RegData)
{
    MPUCS();
		
    HAL_SPI_Transmit(&hspi1,&RegAddr,1,100);
    HAL_SPI_Transmit(&hspi1,RegData,len,100);
    
		MPUnCS();
}

int16_t MPURead(uint8_t RegAddr)
{
    uint8_t RegData[2],Reg;
    Reg = 0x80 | RegAddr;
    MPUCS();
    HAL_SPI_Transmit(&hspi1,&Reg,1,100);
    HAL_SPI_Receive(&hspi1,RegData,2,100);
    MPUnCS();
    return (RegData[0]<<8)|RegData[1];
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1
 		失败为0
*******************************************************************************/
uint8_t MPUWriteBits(uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{
    uint8_t b;
    b = MPUReadReg(reg);
    uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
    data <<= (8 - length);
    data >>= (7 - bitStart);
    b &= mask;
    b |= data;
		DelayUs(200);
    MPUWriteReg(reg, b);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1
 		失败为0
*******************************************************************************/
uint8_t MPUWriteBit(uint8_t reg, uint8_t bitNum, uint8_t data) {
    __IO uint8_t b;
    b = MPUReadReg(reg);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
		DelayUs(200);
    MPUWriteReg(reg, b);
    return 1;
}
typedef struct
{
	Axis3f     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;
#define M_PI_F (float)3.14159265
BiasObj	gyroBiasRunning;
/*传感器偏置初始化*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}
/**
 * 设置二阶低通滤波截至频率
 */
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
	float fr = sample_freq/cutoff_freq;
	float ohm = tanf(M_PI_F/fr);
	float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;
	lpfData->b0 = ohm*ohm/c;
	lpfData->b1 = 2.0f*lpfData->b0;
	lpfData->b2 = lpfData->b0;
	lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
	lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	lpfData->delay_element_1 = 0.0f;
	lpfData->delay_element_2 = 0.0f;
}
/**
 * 二阶低通滤波
 */
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
	if (lpfData == NULL || cutoff_freq <= 0.0f) 
	{
		return;
	}

	lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
int16_t z;
bool bFirstBoot = true;
void MPUInit(void)
{	
    while(MPUReadReg(MPUREG_WHOAMI) != 0x68)
        HAL_Delay(10);
		MPUWriteBit(MPUREG_USER_CTRL, MPU6000_USERCTRL_I2C_IF_DIS_BIT, 1);
		HAL_Delay(10);
		uint8_t i;
		sensorsBiasObjInit(&gyroBiasRunning);
		
		MPUWriteBit(MPUREG_PWR_MGMT_1, MPUREG_PWR1_SLEEP_BIT, 0);
		HAL_Delay(100);		
		
		HAL_Delay(10);
		MPUWriteBit(MPUREG_PWR_MGMT_1, MPUREG_PWR1_DEVICE_RESET_BIT, 1);;	// 复位MPU6500
		HAL_Delay(250);	// 延时等待寄存器复位
		
		while(MPUReadReg(MPUREG_WHOAMI) != 0x68)
				HAL_Delay(10);
		
		MPUWriteBit(MPUREG_PWR_MGMT_1, MPUREG_PWR1_SLEEP_BIT, 0);
		HAL_Delay(100);	
		
		MPUWriteBits(MPUREG_PWR_MGMT_1, MPUREG_PWR1_CLKSEL_BIT,
				MPUREG_PWR1_CLKSEL_LENGTH, MPUREG_CLOCK_PLL_XGYRO);	// 设置X轴陀螺作为时钟	
		HAL_Delay(100);		// 延时等待时钟稳定	
		MPUWriteBit(MPUREG_PWR_MGMT_1, MPUREG_PWR1_TEMP_DIS_BIT, 0);	// 使能温度传感器	
		HAL_Delay(10);
//		MPUWriteReg(MPUREG_INT_ENABLE,0);		// 关闭中断	
	//	mpu6500SetI2CBypassEnabled(true);	// 旁路模式，磁力计和气压连接到主IIC	
		
		MPUWriteBits(MPUREG_GYRO_CONFIG, MPUREG_GCONFIG_FS_SEL_BIT,
				MPUREG_GCONFIG_FS_SEL_LENGTH, SENSORS_GYRO_FS_CFG);// 设置陀螺量程	
		HAL_Delay(10);
		MPUWriteBits(MPUREG_ACCEL_CONFIG, MPUREG_ACONFIG_AFS_SEL_BIT,
				MPUREG_ACONFIG_AFS_SEL_LENGTH,SENSORS_ACCEL_FS_CFG);// 设置加速计量程			
		HAL_Delay(10);
		MPUWriteBits(MPUREG_ACCEL_CONFIG_2, MPUREG_ACONFIG2_DLPF_BIT,
				MPUREG_ACONFIG2_DLPF_LENGTH, MPU6000_ACCEL_DLPF_BW_41);// 设置加速计数字低通滤波
		HAL_Delay(10);
		MPUWriteReg(MPUREG_SMPLRT_DIV, 3);// 设置采样速率: 1000 / (1 + 3) = 250Hz
		HAL_Delay(10);
		MPUWriteBits(MPUREG_CONFIG, MPUREG_CFG_DLPF_CFG_BIT,
				MPUREG_CFG_DLPF_CFG_LENGTH, MPU6000_DLPF_BW_98);// 设置陀螺数字低通滤波
		HAL_Delay(10);
//		MPUWriteBit(MPUREG_USER_CTRL, MPU6000_USERCTRL_I2C_IF_DIS_BIT, 1);		
		
		for (i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
		{
			lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
			lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
		}
		
//		while(1){
//			z = MPURead(MPUREG_GYRO_ZOUT_H);
//			if(fabs(z)>50)
//				HAL_Delay(10);
//		}
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_INT_LEVEL_BIT, 0); 		// 中断高电平有效
		HAL_Delay(10);
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_INT_OPEN_BIT, 0); 			// 推挽输出
		HAL_Delay(10);
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_LATCH_INT_EN_BIT, 0); 	// 中断锁存模式(0=50us-pulse, 1=latch-until-int-cleared)
		HAL_Delay(10);
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_INT_RD_CLEAR_BIT, 1); 	// 中断清除模式(0=status-read-only, 1=any-register-read)
		HAL_Delay(10);
		MPUWriteBit(MPUREG_INT_ENABLE, MPU6000_INTERRUPT_DATA_RDY_BIT, 1);
		HAL_Delay(10);
		z = MPUReadReg(MPUREG_INT_ENABLE);
//		z = MPURead(MPUREG_GYRO_ZOUT_H);
//		z = MPURead(MPUREG_GYRO_ZOUT_H);
//		z = MPURead(MPUREG_GYRO_ZOUT_H);
//		z = MPURead(MPUREG_GYRO_ZOUT_H);
//		z = MPURead(MPUREG_GYRO_ZOUT_H);
//		z = MPURead(MPUREG_GYRO_ZOUT_H);
//		HAL_Delay(200);
//		MPUReadBufferDMA(MPUREG_ACCEL_XOUT_H,14);
		__HAL_GPIO_EXTI_CLEAR_IT(MPU_INT_Pin);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}



static Axis3f  gyroBias;
Axis3f  gyroData;
static Axis3f  accData;
attitude_t attitudeData;
static bool gyroBiasFound = false;


static float accScaleSum = 0;
static float accScale = 1;

/**
 * 根据样本计算重力加速度缩放因子
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}
	return accBiasFound;
}
/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}
/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	uint32_t i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}

/**
 * 计算陀螺方差
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

float lpf2pApply(lpf2pData* lpfData, float sample)
{
	float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
	if (!isfinite(delay_element_0)) 
	{
		// don't allow bad values to propigate via the filter
		delay_element_0 = sample;
	}

	float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

	lpfData->delay_element_2 = lpfData->delay_element_1;
	lpfData->delay_element_1 = delay_element_0;
	return output;
}

/*二阶低通滤波*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (uint8_t i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
void processAccGyro(const uint8_t *buffer)
{
	/*注意传感器读取方向(旋转270°x和y交换)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*计算accScale*/
	}
	
	gyroData.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	gyroData.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	gyroData.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf(gyroLpf, &gyroData);	

	accData.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;	/*单位 g(9.8m/s^2)*/
	accData.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	accData.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;

	applyAxis3fLpf(accLpf, &accData);
}
float invSqrt(float x)	/*快速开平方求倒*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
static float q0 = 1.0f;	/*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
float Kp = 0.4f;		/*比例增益*/
float Ki = 0.001f;		/*积分增益*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/
static float rMat[3][3];/*旋转矩阵*/
static float maxError = 0.f;		/*最大误差*/
bool isGravityCalibrated = false;	/*是否校校准完成*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*静态加速度*/
#define ACCZ_SAMPLE		350
static void calBaseAcc(float* acc)	/*计算静态加速度*/
{
	static uint16_t cnt = 0;
	static float accZMin = 1.5;
	static float accZMax = 0.5;
	static float sumAcc[3] = {0.f};
	uint8_t i;
	for(i=0; i<3; i++)
		sumAcc[i] += acc[i];
		
	if(acc[2] < accZMin)	accZMin = acc[2];
	if(acc[2] > accZMax)	accZMax = acc[2];
	
	if(++cnt >= ACCZ_SAMPLE) /*缓冲区满*/
	{
		cnt = 0;
		maxError = accZMax - accZMin;
		accZMin = 1.5;
		accZMax = 0.5;
		
		if(maxError < 0.015f)
		{
			for(i=0; i<3; i++)
				baseAcc[i] = sumAcc[i] / ACCZ_SAMPLE;
			
			isGravityCalibrated = true;
			
//			ledseqRun(SYS_LED, seq_calibrated);	/*校准通过指示灯*/
		}
		
		for(i=0; i<3; i++)		
			sumAcc[i] = 0.f;		
	}	
}

/*计算旋转矩阵*/
void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuUpdate(float dt)	/*数据融合 互补滤波*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	Axis3f tempacc = accData;
	
	gyroData.x = gyroData.x * DEG2RAD;	/* 度转弧度 */
	gyroData.y = gyroData.y * DEG2RAD;
	gyroData.z = gyroData.z * DEG2RAD;

	/* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
	if((accData.x != 0.0f) || (accData.y != 0.0f) || (accData.z != 0.0f))
	{
		/*单位化加速计测量值*/
		normalise = invSqrt(accData.x * accData.x + accData.y * accData.y + accData.z * accData.z);
		accData.x *= normalise;
		accData.y *= normalise;
		accData.z *= normalise;

		/*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
		ex = (accData.y * rMat[2][2] - accData.z * rMat[2][1]);
		ey = (accData.z * rMat[2][0] - accData.x * rMat[2][2]);
		ez = (accData.x * rMat[2][1] - accData.y * rMat[2][0]);
		
		/*误差累计，与积分常数相乘*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
		gyroData.x += Kp * ex + exInt;
		gyroData.y += Kp * ey + eyInt;
		gyroData.z += Kp * ez + ezInt;
	}
	/* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyroData.x - q2Last * gyroData.y - q3Last * gyroData.z) * halfT;
	q1 += ( q0Last * gyroData.x + q2Last * gyroData.z - q3Last * gyroData.y) * halfT;
	q2 += ( q0Last * gyroData.y - q1Last * gyroData.z + q3Last * gyroData.x) * halfT;
	q3 += ( q0Last * gyroData.z + q1Last * gyroData.y - q2Last * gyroData.x) * halfT;
	
	/*单位化四元数*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*计算旋转矩阵*/
	
	/*计算roll pitch yaw 欧拉角*/
//	attitudeData.pitch = -asinf(rMat[2][0]) * RAD2DEG; 
//	attitudeData.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	attitudeData.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	
	if (!isGravityCalibrated)	/*未校准*/
	{		
//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
		calBaseAcc(accBuf);		/*计算静态加速度*/				
	}
}
#include "mouse.h"
__IO bool gbMpuUpdated = false;
void MPUUpdate(void)
{
		HAL_SPI_DMAStop(&hspi1);
		MPUnCS();
//		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
		processAccGyro(ucMPURBuf+1);
		imuUpdate(ATTITUDE_ESTIMAT_DT);
		gbMpuUpdated = true;
//		mouseTurnAngle();
//		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
}

//------------------End of File----------------------------
