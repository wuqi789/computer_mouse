#ifndef __MOTOR_H__
#define __MOTOR_H__

#define	LEFTMOTOR 0
#define RIGHTMOTOR 1

//#define	LEFTPID 0
//#define RIGHTPID 1
#define	SPEEDPID 0
#define ROTATEPID 1
#define STRAIGHTPID 2
#define ANGLEPID 3
/*********************************************************************************************************
  常量宏定义--电机状态
*********************************************************************************************************/
//
/*********************************************************************************************************
  常量宏定义--电机运行方向
*********************************************************************************************************/
#define MOTORFORWARD      0                                           /*  电机前进                    */
#define MOTORBACK       1                                           /*  电机后退                    */
#define MOTORSTOP     2                                           /*  电机反向制动                */

/*********************************************************************************************************
  常量宏定义--PID
*********************************************************************************************************/
#define __KP 0.5 //80  //比例 30
#define __KI 0.01    //积分 0.01
#define __KD 0           //微分

#define U_MAX 5000       //返回的最大值,是pwm的周期值 
#define U_MIN 1000 
#define error_IMAX 500     //积分限幅 
#define Deadband 5   //速度PID，设置死区范围

#define WALL_FRONT_FAR_DIST 195 //前方远距离障碍
#define WALL_FARDIST 120
#define WALL_FARMIDDIST 80
#define WALL_MIDDIST 20
#define WALL_NEARDIST 30

struct pidData       //定义数法核心数据 
{
    __IO float sRef;
    __IO float sFeedBack;
    __IO float sPreError;  //速度PID，前一次，速度误差,vi_Ref - vi_FeedBack 
    __IO float sPreDerror; //速度PID，前一次，速度误差之差，d_error-PreDerror; 
  
    __IO float fKp;      //速度PID，Ka = Kp 
    __IO float fKi;      //速度PID，Kb = Kp * ( T / Ti ) 
    __IO float fKd;      //速度PID，
    
		__IO float fSum;
	
    __IO float fOutput;    //电机控制输出值      
};
typedef struct pidData PIDData;

struct motorData {
    __IO int8_t    cDir;                                                       /*  电机运行方向                */
//    int8_t    cRealDir;                                                      /*  电机运行方向                */
    __IO uint32_t  uiPulseRef;                                                 /*  电机需要运行的脉冲          */
    __IO uint32_t  uiPulseCtr;                                                 /*  电机已运行的脉冲            */
    __IO int16_t   sOutSpeed;                                                  /*  当前占空比                    */
		__IO uint16_t usEncoder; //编码器
		__IO int16_t sFeedBack;//速度PID，速度反馈值
};

typedef struct motorData MotorData;

typedef enum 
{
	MOUSESTOP = 0,
	MOUSERUN,
//	MOUSETUNNING,
	MOUSETEST,
	MOUSERUNTEST
} MouseState;

struct mouseData{
	__IO MouseState msMouseState; 
	MotorData mMotors[2];	//两个电机
	PIDData pPids[4];			//速度环、角速度环、位置环、角度环
};

typedef struct mouseData MouseData;

extern MouseData gmMouse;

void rightMotorCtrl(void);
void leftMotorCtrl(void);
void PIDCtrl(void);
void PIDInit(void);
void ClearPID(uint8_t idx);
void ClearPIDAll(void);
void mouseControl(void);
void PIDAbsCompute(PIDData * pid, int16_t maxOut);
#endif
