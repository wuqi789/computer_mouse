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
  �����궨��--���״̬
*********************************************************************************************************/
//
/*********************************************************************************************************
  �����궨��--������з���
*********************************************************************************************************/
#define MOTORFORWARD      0                                           /*  ���ǰ��                    */
#define MOTORBACK       1                                           /*  �������                    */
#define MOTORSTOP     2                                           /*  ��������ƶ�                */

/*********************************************************************************************************
  �����궨��--PID
*********************************************************************************************************/
#define __KP 0.5 //80  //���� 30
#define __KI 0.01    //���� 0.01
#define __KD 0           //΢��

#define U_MAX 5000       //���ص����ֵ,��pwm������ֵ 
#define U_MIN 1000 
#define error_IMAX 500     //�����޷� 
#define Deadband 5   //�ٶ�PID������������Χ

#define WALL_FRONT_FAR_DIST 195 //ǰ��Զ�����ϰ�
#define WALL_FARDIST 120
#define WALL_FARMIDDIST 80
#define WALL_MIDDIST 20
#define WALL_NEARDIST 30

struct pidData       //���������������� 
{
    __IO float sRef;
    __IO float sFeedBack;
    __IO float sPreError;  //�ٶ�PID��ǰһ�Σ��ٶ����,vi_Ref - vi_FeedBack 
    __IO float sPreDerror; //�ٶ�PID��ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
  
    __IO float fKp;      //�ٶ�PID��Ka = Kp 
    __IO float fKi;      //�ٶ�PID��Kb = Kp * ( T / Ti ) 
    __IO float fKd;      //�ٶ�PID��
    
		__IO float fSum;
	
    __IO float fOutput;    //����������ֵ      
};
typedef struct pidData PIDData;

struct motorData {
    __IO int8_t    cDir;                                                       /*  ������з���                */
//    int8_t    cRealDir;                                                      /*  ������з���                */
    __IO uint32_t  uiPulseRef;                                                 /*  �����Ҫ���е�����          */
    __IO uint32_t  uiPulseCtr;                                                 /*  ��������е�����            */
    __IO int16_t   sOutSpeed;                                                  /*  ��ǰռ�ձ�                    */
		__IO uint16_t usEncoder; //������
		__IO int16_t sFeedBack;//�ٶ�PID���ٶȷ���ֵ
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
	MotorData mMotors[2];	//�������
	PIDData pPids[4];			//�ٶȻ������ٶȻ���λ�û����ǶȻ�
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
