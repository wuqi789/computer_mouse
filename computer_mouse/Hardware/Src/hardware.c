#include "main.h"
#include "spi.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "math.h"
#include "motor.h"
#include "modbus.h"
#include "MPU6000.h"
#include "hardware.h"
#include "Micromouse.h"

uint16_t gusLedFlash = 0x1F;

struct GPIO_PIN StartKey= {
    START_GPIO_Port,START_Pin		//启动按键
};

struct GPIO_PIN RunLed= {
    LED_GPIO_Port,LED_Pin		//运行指示灯
};

struct GPIO_PIN IREnable[4]= {
    {IR_Tx_FL_GPIO_Port,IR_Tx_FL_Pin},		//红外前左
		{IR_Tx_SR_GPIO_Port,IR_Tx_SR_Pin},		//红外侧右
    {IR_Tx_SL_GPIO_Port,IR_Tx_SL_Pin},		//红外侧左
		{IR_Tx_FR_GPIO_Port,IR_Tx_FR_Pin},		//红外前右
};

__IO uint16_t usAdBuffer[6];
uint16_t usSunlight[4];				//环境光红外光
__IO int16_t gsIrLevel[4];		//检测反射红外光
__IO uint16_t gusDistance[4] = {0};                //  记录传感器状态
__IO bool gbFrontIRTrigger = false;		//前红外启动触发

void DelayUs (uint32_t uiD)
{
    for (; uiD; uiD--);
}

uint8_t startCheck (void)
{
    if (HAL_GPIO_ReadPin(StartKey.port, StartKey.pin) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        while(HAL_GPIO_ReadPin(StartKey.port, StartKey.pin) == GPIO_PIN_RESET);
        return(TRUE);
    } else {
        return(FALSE);
    }
}

/*********************************************************************************************************
** Function name:       irChnEnable
** Descriptions:        红外通道使能。
** input parameters:    cNumber，通道号
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
inline void irChnEnable(int8_t  cNumber)
{
    HAL_GPIO_WritePin(IREnable[cNumber].port,IREnable[cNumber].pin,GPIO_PIN_SET);
}
/*********************************************************************************************************
** Function name:       irChnDisable
** Descriptions:        红外通道禁止。
** input parameters:    cNumber，通道号
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
inline void irChnDisable(int8_t  cNumber)
{
    HAL_GPIO_WritePin(IREnable[cNumber].port,IREnable[cNumber].pin,GPIO_PIN_RESET);
}
/*********************************************************************************************************
** Function name:       irCheck
** Descriptions:        红外采样处理。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
#define IRDELAY 100
//	{1230,210,135},编号：N101
//	{1850,825,325},
//	{1490,625,270},
//	{1210,220,145}
uint16_t IR_TABLE[4][16]={
//	{1690,250,151},//最近（调头位置）、一格、最远（侧传感器一格），红外前左
//	{2143,855,315},//最近（贴墙）、中心、最远（另一侧贴墙），红外侧右
//	{2305,855,286},//最近（贴墙）、中心、最远（另一侧贴墙），红外侧左
//	{1140,315,216}//最近（调头位置）、一格、最远（侧传感器一格），红外前右
	{250,70,55},
	{870,520,230},
	{1130,480,230},
	{350,100,75}
};
const uint16_t IR_DIST[4][3]={
	{20,180,230},//最近（调头位置）、一格、最远（侧传感器一格）
	{0,60,110},//最近（贴墙）、中心、最远（另一侧贴墙）
	{0,60,110},//最近（贴墙）、中心、最远（另一侧贴墙）
	{20,180,230}//最近（调头位置）、一格、最远（侧传感器一格）
};
#define FILTER_COUNT 5
//uint8_t IrScale[4] = {3,2,2,3};
uint16_t getIrDistance(uint8_t ind,uint16_t ir)
{
		static uint16_t circleBuf[4][FILTER_COUNT]={0};
		uint16_t processBuf[FILTER_COUNT]={0};
		uint32_t sum = 0;
		int16_t dist = 0;
		uint8_t i,j;
		for(i=0;i<3;i++)
		{
				if(IR_TABLE[ind][i] <= ir)
						break;
		}
		if(i<=1)
		{
			dist = IR_DIST[ind][1] - (ir - IR_TABLE[ind][1])*(IR_DIST[ind][1]-IR_DIST[ind][0])/(IR_TABLE[ind][0]-IR_TABLE[ind][1]);
		}
		else
		{
			dist = IR_DIST[ind][2] -	 (ir - IR_TABLE[ind][2])*(IR_DIST[ind][2]-IR_DIST[ind][1])/(IR_TABLE[ind][1]-IR_TABLE[ind][2]);
		}
		if(dist < 0)
			dist = 0;
		for(i=0;i<(FILTER_COUNT-1);i++)
		{
			processBuf[i] =  circleBuf[ind][i+1];
			circleBuf[ind][i] = processBuf[i];
		}
		processBuf[FILTER_COUNT-1] = dist;
		circleBuf[ind][FILTER_COUNT-1] = dist;
		
		//冒泡排序
		for (j = 0; j < FILTER_COUNT - 1; j++) 
		{
			for (i = 0; i < FILTER_COUNT - 1 - j; i++)
			{
				if (processBuf[i] > processBuf[i + 1])
				{
					uint16_t temp = processBuf[i];
					processBuf[i] = processBuf[i + 1];
					processBuf[i + 1] = temp;
				}
			}
		}
		sum = 0;
		for(i=1;i<FILTER_COUNT-1;i++)
			sum += processBuf[i];
		
		return (sum/(FILTER_COUNT-2));//*IrScale[ind];
}

void irCheck (void)
{
    static uint8_t ucIndex = 0;
    uint8_t ucNext = 0;
		uint8_t ucState = (ucIndex>>1);
		if((ucIndex &0x01) == 0)
		{
			usSunlight[ucState]=usAdBuffer[ucState];
			irChnEnable (ucState);
		}
		else
		{		
			if(usAdBuffer[ucState] >= usSunlight[ucState])
				gsIrLevel[ucState] = usAdBuffer[ucState]-usSunlight[ucState];
			gusDistance[ucState] = getIrDistance(ucState,gsIrLevel[ucState]);
			ucNext = (ucState+1)&0x03;
			irChnDisable(ucNext);
		}
		if(gusDistance[FRONTRIGHT] < WALL_FRONT_FAR_DIST)
				gbFrontIRTrigger = true;
		else
				gbFrontIRTrigger = false;
    ucIndex = (ucIndex + 1) & 0x07;
}
__IO bool bMpuWorked = false;
HAL_StatusTypeDef res;
extern int16_t z;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM9)
    {
				static uint8_t ucIndex = 0;
				static uint16_t usTimeOut = 0;
        irCheck();
				if(ucIndex == 2)
				{
						
				}
				else if(ucIndex == 1)
				{
//						HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
				}
				else if(ucIndex == 3)
				{

						if(bMpuWorked == true)
						{
							mouseControl();
						}						
				}
				ucIndex = (ucIndex + 1) & 0x07;
				usTimeOut++;
				if(usTimeOut > 400)
				{
					usTimeOut = 0;
					modbusProcess();
				}
    }
}

/*********************************************************************************************************
** Function name:       mouseInit
** Descriptions:        电脑鼠初始化。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseMoveDist(uint16_t usCM);
void mouseStop(void);
void mouseInit(void)
{
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)usAdBuffer,ADC_BUF_CHN);
    
    HAL_Delay(10);
		FRAMInit();

    gmMouse.mMotors[RIGHTMOTOR].cDir = MOTORSTOP;
    gmMouse.mMotors[LEFTMOTOR].cDir = MOTORSTOP;
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

//		HAL_Delay(5000);
		MPUInit();
		PIDInit();
		EncInit();
	
		while(bMpuWorked == false);
		
		gusLedFlash = 0x1F;
		modbusInit();

		HAL_TIM_Base_Start_IT(&htim9);

//		gmMouse.pPids[SPEEDPID].sRef = 150;
//		gmMouse.pPids[ROTATEPID].sRef = 0;
//		gmMouse.msMouseState = MOUSERUNTEST;
//		HAL_Delay(2000);
//		while(1)modbusProcess();
//		
//		gmMouse.pPids[SPEEDPID].sRef = 0;
//		gmMouse.pPids[ROTATEPID].sRef = 0;
//		gmMouse.msMouseState = MOUSESTOP;
		
//		gmMouse.msMouseState = MOUSETEST;
//		gmMouse.mMotors[RIGHTMOTOR].cDir = MOTORFORWARD;
//		gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = 1000;
//		gmMouse.mMotors[LEFTMOTOR].cDir = MOTORFORWARD;
//		gmMouse.mMotors[LEFTMOTOR].sOutSpeed = 1000;
//		rightMotorCtrl();
//		leftMotorCtrl();


//		HAL_Delay(2000);
//		gmMouse.pPids[SPEEDPID].sRef = 100;
//		gmMouse.msMouseState = MOUSERUN;
//		
//		mouseMoveDist(18*8);
//		mouseStop();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1)
    {
        MPUUpdate();
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		static uint16_t ucMpuCount = 0;
		if(GPIO_Pin == MPU_INT_Pin)
		{
			if(bMpuWorked == false)
			{
				if(ucMpuCount>10)
				{
					bMpuWorked = true;
				}
			}
			else
			{
				MPUReadBufferDMA(MPUREG_ACCEL_XOUT_H,14);
				ucMpuCount &= gusLedFlash;
				if((ucMpuCount)==1){
					if(usAdBuffer[4] > 0xC00)
						HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
					else
						HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
				}
			}
			ucMpuCount++;
			
			
		}
}
