#include "MPU6000.h"
#include <math.h>
#include "main.h"
#include "spi.h"
//#include "arm_math.h" 
#include "hardware.h" 

void MPUCS()    //CS�ź�����
{
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_RESET);
		DelayUs(20);
}

void MPUnCS() //CS�ź�����
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1
 		ʧ��Ϊ0
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1
 		ʧ��Ϊ0
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
/*������ƫ�ó�ʼ��*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}
/**
 * ���ö��׵�ͨ�˲�����Ƶ��
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
 * ���׵�ͨ�˲�
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
		MPUWriteBit(MPUREG_PWR_MGMT_1, MPUREG_PWR1_DEVICE_RESET_BIT, 1);;	// ��λMPU6500
		HAL_Delay(250);	// ��ʱ�ȴ��Ĵ�����λ
		
		while(MPUReadReg(MPUREG_WHOAMI) != 0x68)
				HAL_Delay(10);
		
		MPUWriteBit(MPUREG_PWR_MGMT_1, MPUREG_PWR1_SLEEP_BIT, 0);
		HAL_Delay(100);	
		
		MPUWriteBits(MPUREG_PWR_MGMT_1, MPUREG_PWR1_CLKSEL_BIT,
				MPUREG_PWR1_CLKSEL_LENGTH, MPUREG_CLOCK_PLL_XGYRO);	// ����X��������Ϊʱ��	
		HAL_Delay(100);		// ��ʱ�ȴ�ʱ���ȶ�	
		MPUWriteBit(MPUREG_PWR_MGMT_1, MPUREG_PWR1_TEMP_DIS_BIT, 0);	// ʹ���¶ȴ�����	
		HAL_Delay(10);
//		MPUWriteReg(MPUREG_INT_ENABLE,0);		// �ر��ж�	
	//	mpu6500SetI2CBypassEnabled(true);	// ��·ģʽ�������ƺ���ѹ���ӵ���IIC	
		
		MPUWriteBits(MPUREG_GYRO_CONFIG, MPUREG_GCONFIG_FS_SEL_BIT,
				MPUREG_GCONFIG_FS_SEL_LENGTH, SENSORS_GYRO_FS_CFG);// ������������	
		HAL_Delay(10);
		MPUWriteBits(MPUREG_ACCEL_CONFIG, MPUREG_ACONFIG_AFS_SEL_BIT,
				MPUREG_ACONFIG_AFS_SEL_LENGTH,SENSORS_ACCEL_FS_CFG);// ���ü��ټ�����			
		HAL_Delay(10);
		MPUWriteBits(MPUREG_ACCEL_CONFIG_2, MPUREG_ACONFIG2_DLPF_BIT,
				MPUREG_ACONFIG2_DLPF_LENGTH, MPU6000_ACCEL_DLPF_BW_41);// ���ü��ټ����ֵ�ͨ�˲�
		HAL_Delay(10);
		MPUWriteReg(MPUREG_SMPLRT_DIV, 3);// ���ò�������: 1000 / (1 + 3) = 250Hz
		HAL_Delay(10);
		MPUWriteBits(MPUREG_CONFIG, MPUREG_CFG_DLPF_CFG_BIT,
				MPUREG_CFG_DLPF_CFG_LENGTH, MPU6000_DLPF_BW_98);// �����������ֵ�ͨ�˲�
		HAL_Delay(10);
//		MPUWriteBit(MPUREG_USER_CTRL, MPU6000_USERCTRL_I2C_IF_DIS_BIT, 1);		
		
		for (i = 0; i < 3; i++)// ��ʼ�����ټƺ����ݶ��׵�ͨ�˲�
		{
			lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
			lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
		}
		
//		while(1){
//			z = MPURead(MPUREG_GYRO_ZOUT_H);
//			if(fabs(z)>50)
//				HAL_Delay(10);
//		}
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_INT_LEVEL_BIT, 0); 		// �жϸߵ�ƽ��Ч
		HAL_Delay(10);
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_INT_OPEN_BIT, 0); 			// �������
		HAL_Delay(10);
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_LATCH_INT_EN_BIT, 0); 	// �ж�����ģʽ(0=50us-pulse, 1=latch-until-int-cleared)
		HAL_Delay(10);
		MPUWriteBit(MPUREG_INT_PIN_CFG, MPU6000_INTCFG_INT_RD_CLEAR_BIT, 1); 	// �ж����ģʽ(0=status-read-only, 1=any-register-read)
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
 * �������������������ٶ���������
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
 * �����������ѭ�������������һ����ֵ�������������滻�ɵĵ�ֵ
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
/*���㷽���ƽ��ֵ*/
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
/*����������ƫ��ֵ*/
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
 * �������ݷ���
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

/*���׵�ͨ�˲�*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (uint8_t i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
void processAccGyro(const uint8_t *buffer)
{
	/*ע�⴫������ȡ����(��ת270��x��y����)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*����accScale*/
	}
	
	gyroData.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*��λ ��/s */
	gyroData.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	gyroData.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf(gyroLpf, &gyroData);	

	accData.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;	/*��λ g(9.8m/s^2)*/
	accData.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;	/*�������ٶ���������accScale ������������ó�*/
	accData.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;

	applyAxis3fLpf(accLpf, &accData);
}
float invSqrt(float x)	/*���ٿ�ƽ����*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
static float q0 = 1.0f;	/*��Ԫ��*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
float Kp = 0.4f;		/*��������*/
float Ki = 0.001f;		/*��������*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*��������ۼ�*/
static float rMat[3][3];/*��ת����*/
static float maxError = 0.f;		/*������*/
bool isGravityCalibrated = false;	/*�Ƿ�УУ׼���*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*��̬���ٶ�*/
#define ACCZ_SAMPLE		350
static void calBaseAcc(float* acc)	/*���㾲̬���ٶ�*/
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
	
	if(++cnt >= ACCZ_SAMPLE) /*��������*/
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
			
//			ledseqRun(SYS_LED, seq_calibrated);	/*У׼ͨ��ָʾ��*/
		}
		
		for(i=0; i<3; i++)		
			sumAcc[i] = 0.f;		
	}	
}

/*������ת����*/
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

void imuUpdate(float dt)	/*�����ں� �����˲�*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	Axis3f tempacc = accData;
	
	gyroData.x = gyroData.x * DEG2RAD;	/* ��ת���� */
	gyroData.y = gyroData.y * DEG2RAD;
	gyroData.z = gyroData.z * DEG2RAD;

	/* ���ٶȼ������Чʱ,���ü��ٶȼƲ���������*/
	if((accData.x != 0.0f) || (accData.y != 0.0f) || (accData.z != 0.0f))
	{
		/*��λ�����ټƲ���ֵ*/
		normalise = invSqrt(accData.x * accData.x + accData.y * accData.y + accData.z * accData.z);
		accData.x *= normalise;
		accData.y *= normalise;
		accData.z *= normalise;

		/*���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���*/
		ex = (accData.y * rMat[2][2] - accData.z * rMat[2][1]);
		ey = (accData.z * rMat[2][0] - accData.x * rMat[2][2]);
		ez = (accData.x * rMat[2][1] - accData.y * rMat[2][0]);
		
		/*����ۼƣ�����ֳ������*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*�ò���������PI����������ƫ�����������ݶ����е�ƫ����*/
		gyroData.x += Kp * ex + exInt;
		gyroData.y += Kp * ey + eyInt;
		gyroData.z += Kp * ez + ezInt;
	}
	/* һ�׽����㷨����Ԫ���˶�ѧ���̵���ɢ����ʽ�ͻ��� */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyroData.x - q2Last * gyroData.y - q3Last * gyroData.z) * halfT;
	q1 += ( q0Last * gyroData.x + q2Last * gyroData.z - q3Last * gyroData.y) * halfT;
	q2 += ( q0Last * gyroData.y - q1Last * gyroData.z + q3Last * gyroData.x) * halfT;
	q3 += ( q0Last * gyroData.z + q1Last * gyroData.y - q2Last * gyroData.x) * halfT;
	
	/*��λ����Ԫ��*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*������ת����*/
	
	/*����roll pitch yaw ŷ����*/
//	attitudeData.pitch = -asinf(rMat[2][0]) * RAD2DEG; 
//	attitudeData.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	attitudeData.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	
	if (!isGravityCalibrated)	/*δУ׼*/
	{		
//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
		calBaseAcc(accBuf);		/*���㾲̬���ٶ�*/				
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
