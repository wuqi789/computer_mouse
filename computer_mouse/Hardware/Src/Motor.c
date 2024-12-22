#include "main.h"
#include "tim.h"
#include "motor.h"
#include "mouse.h"
#include "hardware.h"

MouseData gmMouse;
__IO int16_t   gsTunningStraight                       = 0;                  // 左轮校正减少的速度值

/*********************************************************************************************************
** Function name:       rightMotorCtrl
** Descriptions:        右直流电机驱动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void rightMotorCtrl(void)
{
    switch (gmMouse.mMotors[RIGHTMOTOR].cDir)
    {
    case MOTORBACK:
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,gmMouse.mMotors[RIGHTMOTOR].sOutSpeed);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
        break;

    case MOTORFORWARD:
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,gmMouse.mMotors[RIGHTMOTOR].sOutSpeed);

        break;

    case MOTORSTOP:
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
        break;

    default:
        break;
    }
}

/*********************************************************************************************************
** Function name:       leftMotorCtrl
** Descriptions:        左直流电机驱动
** input parameters:    gmMotors[LEFTMOTOR].cDir :电机运行方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void leftMotorCtrl(void)
{
    switch (gmMouse.mMotors[LEFTMOTOR].cDir)
    {
    case MOTORBACK:
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,gmMouse.mMotors[LEFTMOTOR].sOutSpeed);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
        break;

    case MOTORFORWARD:
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,gmMouse.mMotors[LEFTMOTOR].sOutSpeed);
        break;

    case MOTORSTOP:
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
        break;

    default:
        break;
    }
}

/*********************************************************************************************************
** Function name:       PIDInit
** Descriptions:        PID初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void ClearPID(uint8_t idx)
{
    gmMouse.pPids[idx].sRef = 0 ;        //速度设定值
    gmMouse.pPids[idx].sFeedBack = 0 ;
    gmMouse.pPids[idx].sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack
    gmMouse.pPids[idx].sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror;
    gmMouse.pPids[idx].fOutput = 0 ;      //电机控制输出值
    gmMouse.pPids[idx].fSum = 0 ;
}

void ClearPIDAll(void)
{
    uint8_t i;
    for(i=0; i<4; i++)
    {
        ClearPID(i);
    }
}

void PIDInit(void)
{
    ClearPIDAll();
    FRAMReadBuffer(PID_SAVE_OFFSET,(uint8_t *)(&gmMouse.pPids[SPEEDPID].fKp),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+4,(uint8_t *)(&gmMouse.pPids[SPEEDPID].fKi),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+8,(uint8_t *)(&gmMouse.pPids[SPEEDPID].fKd),4);

    FRAMReadBuffer(PID_SAVE_OFFSET+12,(uint8_t *)(&gmMouse.pPids[ROTATEPID].fKp),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+16,(uint8_t *)(&gmMouse.pPids[ROTATEPID].fKi),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+20,(uint8_t *)(&gmMouse.pPids[ROTATEPID].fKd),4);

    FRAMReadBuffer(PID_SAVE_OFFSET+24,(uint8_t *)(&gmMouse.pPids[STRAIGHTPID].fKp),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+28,(uint8_t *)(&gmMouse.pPids[STRAIGHTPID].fKi),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+32,(uint8_t *)(&gmMouse.pPids[STRAIGHTPID].fKd),4);

    FRAMReadBuffer(PID_SAVE_OFFSET+36,(uint8_t *)(&gmMouse.pPids[ANGLEPID].fKp),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+40,(uint8_t *)(&gmMouse.pPids[ANGLEPID].fKi),4);
    FRAMReadBuffer(PID_SAVE_OFFSET+44,(uint8_t *)(&gmMouse.pPids[ANGLEPID].fKd),4);

    FRAMReadBuffer(PID_SAVE_OFFSET+48,(uint8_t *)(&guscMAXSPEED),2);
    FRAMReadBuffer(PID_SAVE_OFFSET+50,(uint8_t *)(&guscSEARCHSPEED),2);
    FRAMReadBuffer(PID_SAVE_OFFSET+52,(uint8_t *)(&guscMINSPEED),2);

    FRAMReadBuffer(IR_SAVE_OFFSET,(uint8_t *)(IR_TABLE),128);

    guscSEARCHSPEED = 145;
    giCurMaxSpeed = guscSEARCHSPEED;

    gmMouse.pPids[SPEEDPID].fKp = 10.0;
    gmMouse.pPids[SPEEDPID].fKi = 1.5;
    gmMouse.pPids[SPEEDPID].fKd = 0.00;

    gmMouse.pPids[ROTATEPID].fKp = 15.0;
    gmMouse.pPids[ROTATEPID].fKi = 2.0;
    gmMouse.pPids[ROTATEPID].fKd = 0.00;

    gmMouse.pPids[STRAIGHTPID].fKp = 0.50;
    gmMouse.pPids[STRAIGHTPID].fKi = 0.00;
    gmMouse.pPids[STRAIGHTPID].fKd = 0.20;

    gmMouse.pPids[ANGLEPID].fKp = 1.25;
    gmMouse.pPids[ANGLEPID].fKi = 0.00000;
    gmMouse.pPids[ANGLEPID].fKd = 0.7500;
}

/*********************************************************************************************************
** Function name:       PIDCompute
** Descriptions:        PID控制计算
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/

void PIDIncCompute(PIDData * pid, int16_t sTunning)
{
    static uint8_t   K_I=1;
    float  error,d_error,dd_error;
    error = pid->sRef + sTunning - pid->sFeedBack; // 偏差计算
    d_error = error - pid->sPreError;
    dd_error = d_error - pid->sPreDerror;
    if(error> Deadband)
        error -= Deadband;
    else if(error < -Deadband)
        error += Deadband;
    else
        error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
        K_I=0;
    else
        K_I=1;

    pid->sPreError = error; //存储当前偏差
    pid->sPreDerror = d_error;
    pid->fOutput += (pid->fKp * d_error + K_I*pid->fKi * error  + pid->fKd*dd_error);

    if(pid->fOutput > 5000)
        pid->fOutput = 5000;
    if(pid->fOutput < -5000)
        pid->fOutput = -5000;
}

void PIDAbsCompute(PIDData * pid, int16_t maxOut)
{
    float  error;
    error = pid->sRef - pid->sFeedBack; // 偏差计算

    pid->fSum += error;
    pid->fOutput = (pid->fKp * error + pid->fKi * pid->fSum  + pid->fKd*(error - pid->sPreError));

    pid->sPreError = error; //存储当前偏差到上一次误差

    if(pid->fOutput > maxOut)
    {
        pid->fOutput = maxOut;
    }
    if(pid->fOutput < -maxOut)
    {
        pid->fOutput = -maxOut;
    }
}

/*********************************************************************************************************
** Function name:      PIDCtrl
** Descriptions:        PID控制，通过脉冲数控制电机
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PIDCtrl(void)
{
    PIDIncCompute(&(gmMouse.pPids[SPEEDPID]),0);
    PIDIncCompute(&(gmMouse.pPids[ROTATEPID]),gsTunningStraight);
    gmMouse.mMotors[LEFTMOTOR].sOutSpeed = (gmMouse.pPids[SPEEDPID].fOutput - gmMouse.pPids[ROTATEPID].fOutput);
    if(gmMouse.mMotors[LEFTMOTOR].sOutSpeed>=0) {
        gmMouse.mMotors[LEFTMOTOR].cDir=MOTORFORWARD;
        if( gmMouse.mMotors[LEFTMOTOR].sOutSpeed >= U_MAX )   //速度PID，防止调节最高溢出
            gmMouse.mMotors[LEFTMOTOR].sOutSpeed = U_MAX;
        if( gmMouse.mMotors[LEFTMOTOR].sOutSpeed <= U_MIN ) //速度PID，防止调节最低溢出
            gmMouse.mMotors[LEFTMOTOR].sOutSpeed = U_MIN;
    }
    else
    {
        gmMouse.mMotors[LEFTMOTOR].cDir=MOTORBACK;
        gmMouse.mMotors[LEFTMOTOR].sOutSpeed *=-1;
        if( gmMouse.mMotors[LEFTMOTOR].sOutSpeed >= U_MAX )   //速度PID，防止调节最高溢出
            gmMouse.mMotors[LEFTMOTOR].sOutSpeed = U_MAX;
        if( gmMouse.mMotors[LEFTMOTOR].sOutSpeed <= U_MIN ) //速度PID，防止调节最低溢出
            gmMouse.mMotors[LEFTMOTOR].sOutSpeed = U_MIN;
    }
    gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = (gmMouse.pPids[SPEEDPID].fOutput + gmMouse.pPids[ROTATEPID].fOutput);
    if(gmMouse.mMotors[RIGHTMOTOR].sOutSpeed>=0) {
        gmMouse.mMotors[RIGHTMOTOR].cDir=MOTORFORWARD;
        if( gmMouse.mMotors[RIGHTMOTOR].sOutSpeed >= U_MAX )   //速度PID，防止调节最高溢出
            gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = U_MAX;
        if( gmMouse.mMotors[RIGHTMOTOR].sOutSpeed <= U_MIN ) //速度PID，防止调节最低溢出
            gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = U_MIN;
    }
    else
    {
        gmMouse.mMotors[RIGHTMOTOR].cDir=MOTORBACK;
        gmMouse.mMotors[RIGHTMOTOR].sOutSpeed *=-1;
        if( gmMouse.mMotors[RIGHTMOTOR].sOutSpeed >= U_MAX )   //速度PID，防止调节最高溢出
            gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = U_MAX;
        if( gmMouse.mMotors[RIGHTMOTOR].sOutSpeed <= U_MIN ) //速度PID，防止调节最低溢出
            gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = U_MIN;
    }

    leftMotorCtrl();
    rightMotorCtrl();

    //gmMouse.pPids[SPEEDPID].sFeedBack = (gmMouse.mMotors[RIGHTMOTOR].sFeedBack + gmMouse.mMotors[LEFTMOTOR].sFeedBack)/2 ;
    //gmMouse.pPids[ROTATEPID].sFeedBack = (gmMouse.mMotors[RIGHTMOTOR].sFeedBack - gmMouse.mMotors[LEFTMOTOR].sFeedBack)/2 ;
}

/*********************************************************************************************************
** Function name:       speedUp
** Descriptions:        电脑鼠加速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void speedUp (void)
{
    uint16_t Speed;
    Speed=gmMouse.pPids[SPEEDPID].sFeedBack;
    if(gmMouse.pPids[SPEEDPID].sRef<giCurMaxSpeed) {
        if(Speed >=gmMouse.pPids[SPEEDPID].sRef)
        {
            gmMouse.pPids[SPEEDPID].sRef=gmMouse.pPids[SPEEDPID].sRef+9;
        }
    }
    else
    {
        if(Speed >=gmMouse.pPids[SPEEDPID].sRef)
        {
            gmMouse.pPids[SPEEDPID].sRef=gmMouse.pPids[SPEEDPID].sRef-12.5;
        }
    }
}
extern uint16_t gusLeftDist;
extern uint16_t gusRightDist;

void StraightPidCtrl(void)
{
    int16_t err = 0;
//    static uint8_t ucLeftCount = 0, ucRightCount = 0;
//    if(gusDistance[FRONTLEFT] > WALL_FRONT_FAR_DIST
//            && gusDistance[FRONTRIGHT] >WALL_FRONT_FAR_DIST)
    {
        /*if((gusDistance[SIDELEFT] + gusDistance[SIDERIGHT]) < 160)
        {
            ucLeftCount = 0;
            ucRightCount = 0;
            err = gusDistance[SIDELEFT] - gusDistance[SIDERIGHT];
        }
        else */if((gusDistance[SIDERIGHT]<(WALL_FARDIST)) && (gusDistance[SIDERIGHT] < gusDistance[SIDELEFT]))//右侧有墙
        {
//			ucLeftCount = 0;
//			ucRightCount++;
//			if(ucRightCount > 3)
            err = (gusRightDist - gusDistance[SIDERIGHT])*2;//正值偏右
        }
        else if((gusDistance[SIDELEFT]<WALL_FARDIST) && (gusDistance[SIDERIGHT] > gusDistance[SIDELEFT]))//左侧有墙
        {
//			ucRightCount = 0;
//			ucLeftCount++;
//			if(ucLeftCount > 3)
            err = (gusDistance[SIDELEFT] - gusLeftDist)*2;//正值偏右
        }
        else
        {
//            ucLeftCount = 0;
//            ucRightCount = 0;
            err = 0;
        }
				
        if(err > 100)
            err = 100;
        if(err < -100)
            err = -100;
				
        gmMouse.pPids[STRAIGHTPID].sFeedBack = -err;
        PIDAbsCompute(&gmMouse.pPids[STRAIGHTPID],500);
        if(err == 0)
            gsTunningStraight = 0;
        else
            gsTunningStraight = gmMouse.pPids[STRAIGHTPID].fOutput;
    }
//    else
//    {
//        gsTunningStraight = 0;
//    }


}

void mouseControl(void)
{
    static uint8_t ucSpeedUpCount=0;
    EncUpdate();
    switch(gmMouse.msMouseState)
    {
    case MOUSERUN:
        if (gucMouseState == MOUSE_GOAHEAD)                                 /*  根据传感器状态微调电机位置  */
        {
            
            if(gucSpeedCtrl==SPEEDUP)
            {
                ucSpeedUpCount=(ucSpeedUpCount+1)%0x0F;//20
                if(ucSpeedUpCount==1)
                {
                    speedUp();
                }
								if(ucSpeedUpCount==1 || ucSpeedUpCount==5 || ucSpeedUpCount==9 || ucSpeedUpCount==13)
                {
                    StraightPidCtrl();
                }
            }
        }
        else {
            gsTunningStraight = 0;
        }
        PIDCtrl();
        break;
    case MOUSETEST:

        break;
    case MOUSERUNTEST:
        PIDCtrl();
        break;
    case MOUSESTOP:
//        gmMouse.mMotors[LEFTMOTOR].sOutSpeed = 0;
////        gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = 0;
////        gmMouse.mMotors[LEFTMOTOR].uiPulseRef = 0;
//        gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = 0;
////        gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = 0;
////        gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = 0;
//        gmMouse.pPids[0].sRef = 0;
//        gmMouse.pPids[1].sRef = 0;
//        gmMouse.pPids[2].sRef = 0;
//        gmMouse.pPids[3].sRef = 0;
//        gmMouse.pPids[0].fOutput = 0;
//        gmMouse.pPids[1].fOutput = 0;
//        gmMouse.pPids[2].fOutput = 0;
//        gmMouse.pPids[3].fOutput = 0;
//        leftMotorCtrl();
//        rightMotorCtrl();
        break;
    }
}
