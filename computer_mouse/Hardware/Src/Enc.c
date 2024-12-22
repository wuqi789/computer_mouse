#include "main.h"
#include "tim.h"
#include "enc.h"
#include "motor.h"
#include "math.h"
#include "hardware.h"

void EncInit(void)
{
		__HAL_TIM_SET_COUNTER(&htim2, 32768);
		__HAL_TIM_SET_COUNTER(&htim3, 32768);
		HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
}
__IO uint16_t usCurFeedBack,usLastFeedBack;
__IO int16_t usTmpFeedBack;
uint16_t usFeedBackArr[1024]={0};
uint16_t usFeedBackInx = 0;
uint16_t usFeedBackAvg = 0;
uint16_t usFeedBackMax = 0;
uint16_t usFeedBackMin = 0x4000;
uint32_t usFeedBackSum = 0;
//void EncUpdate(void)
//{
//		static uint16_t Dir_L;
//    static uint16_t Dir_R;
//		uint16_t usTmpFeedBack;

//    gmMouse.mMotors[LEFTMOTOR].usEncoder = __HAL_TIM_GET_COUNTER(&htim2);
//    gmMouse.mMotors[RIGHTMOTOR].usEncoder = __HAL_TIM_GET_COUNTER(&htim3);

//    Dir_L=TIM2->CR1;
//    Dir_L=(Dir_L&0x0010)>>4;
//	
//		Dir_R=TIM3->CR1;
//    Dir_R=(Dir_R&0x0010)>>4;

//    if(Dir_L==1)//向下计数  向后退
//    {
//					usTmpFeedBack = 32768 - gmMouse.mMotors[LEFTMOTOR].usEncoder;
//					__HAL_TIM_SET_COUNTER(&htim2, 32768);
//					gmMouse.mMotors[LEFTMOTOR].uiPulseCtr += usTmpFeedBack;
//					gmMouse.mMotors[LEFTMOTOR].sFeedBack= -1*usTmpFeedBack;
//    }
//    else
//    {
//					usTmpFeedBack = gmMouse.mMotors[LEFTMOTOR].usEncoder - 32768;
//					__HAL_TIM_SET_COUNTER(&htim2, 32768);
//					gmMouse.mMotors[LEFTMOTOR].uiPulseCtr += usTmpFeedBack;
//					gmMouse.mMotors[LEFTMOTOR].sFeedBack= usTmpFeedBack;
//    }
//    if(Dir_R==1)//向下计数  前进
//    {
//					usTmpFeedBack = 32768 - gmMouse.mMotors[RIGHTMOTOR].usEncoder;
//					__HAL_TIM_SET_COUNTER(&htim3, 32768);
//					gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr += usTmpFeedBack;
//					gmMouse.mMotors[RIGHTMOTOR].sFeedBack = usTmpFeedBack;
//    }
//    else
//    {
//					usTmpFeedBack = gmMouse.mMotors[RIGHTMOTOR].usEncoder - 32768;
//					__HAL_TIM_SET_COUNTER(&htim3, 32768);
//					gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr += usTmpFeedBack;
//					gmMouse.mMotors[RIGHTMOTOR].sFeedBack = -1*usTmpFeedBack;
//    }
//    gmMouse.pPids[SPEEDPID].sFeedBack = (gmMouse.mMotors[RIGHTMOTOR].sFeedBack + gmMouse.mMotors[LEFTMOTOR].sFeedBack)/2 ;
//    gmMouse.pPids[ROTATEPID].sFeedBack = (gmMouse.mMotors[RIGHTMOTOR].sFeedBack - gmMouse.mMotors[LEFTMOTOR].sFeedBack)/2 ;
//}

void EncUpdate(void)
{
		static uint16_t Dir_L;
    static uint16_t Dir_R;
		uint16_t usLeftFeedBack,usRightFeedBack,usTmpFeedBack;

    usLeftFeedBack = __HAL_TIM_GET_COUNTER(&htim2);
    usRightFeedBack = __HAL_TIM_GET_COUNTER(&htim3);

    Dir_L=TIM2->CR1;
    Dir_L=(Dir_L&0x0010)>>4;
	
		Dir_R=TIM3->CR1;
    Dir_R=(Dir_R&0x0010)>>4;

    if(Dir_L==1)//向下计数  向后退
    {
					usTmpFeedBack = gmMouse.mMotors[LEFTMOTOR].usEncoder - usLeftFeedBack;
					if(usTmpFeedBack > 50000)
						usTmpFeedBack = 65535 - usTmpFeedBack;
					gmMouse.mMotors[LEFTMOTOR].uiPulseCtr += usTmpFeedBack;
					gmMouse.mMotors[LEFTMOTOR].sFeedBack= -1*usTmpFeedBack;
    }
    else
    {
					usTmpFeedBack = usLeftFeedBack - gmMouse.mMotors[LEFTMOTOR].usEncoder;
					if(usTmpFeedBack > 50000)
						usTmpFeedBack = 65535 - usTmpFeedBack;
					gmMouse.mMotors[LEFTMOTOR].uiPulseCtr += usTmpFeedBack;
					gmMouse.mMotors[LEFTMOTOR].sFeedBack= usTmpFeedBack;
    }
    if(Dir_R==1)//向下计数  前进
    {
					usTmpFeedBack = gmMouse.mMotors[RIGHTMOTOR].usEncoder - usRightFeedBack;
					if(usTmpFeedBack > 50000)
						usTmpFeedBack = 65535 - usTmpFeedBack;
					gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr += usTmpFeedBack;
					gmMouse.mMotors[RIGHTMOTOR].sFeedBack = usTmpFeedBack;
    }
    else
    {
					usTmpFeedBack = usRightFeedBack - gmMouse.mMotors[RIGHTMOTOR].usEncoder;
					if(usTmpFeedBack > 50000)
						usTmpFeedBack = 65535 - usTmpFeedBack;
					gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr += usTmpFeedBack;
					gmMouse.mMotors[RIGHTMOTOR].sFeedBack = -1*usTmpFeedBack;
    }
		
		gmMouse.mMotors[LEFTMOTOR].usEncoder = usLeftFeedBack;
    gmMouse.mMotors[RIGHTMOTOR].usEncoder = usRightFeedBack;
    gmMouse.pPids[SPEEDPID].sFeedBack = (gmMouse.mMotors[RIGHTMOTOR].sFeedBack + gmMouse.mMotors[LEFTMOTOR].sFeedBack)/2 ;
    gmMouse.pPids[ROTATEPID].sFeedBack = (gmMouse.mMotors[RIGHTMOTOR].sFeedBack - gmMouse.mMotors[LEFTMOTOR].sFeedBack)/2 ;
}
