#include "Mouse.h"
#include "hardware.h"
#include "maze.h"
#include "motor.h"
#include "math.h"
#include "modbus.h"
#include "mouse.h"
#define guscSEARCHSPEED1 150
/*********************************************************************************************************
  ����ȫ�ֱ���
*********************************************************************************************************/
MazeCoor          	gmcCurrentCoor                  = {0,0};              /*  ���������ǰλ������      */
uint8_t             gucMouseDir                     = MAP_UP;                 /*  ���������ǰ����          */
uint16_t             gucMapWay[MAZETYPE*2][MAZETYPE*2] = {0};                /*  gucMapWay[x][y]           */
uint8_t             gucMapVisited[MAZETYPE*2][MAZETYPE*2] = {0};               /*  �����㷨         */
//uint8_t             gucMapWall[MAZETYPE][MAZETYPE]= {0x0f};
                                                   
static int16_t   gsTunningPusle = 0;
bool gbFrontBlocked = false;

uint8_t    gucPreDirect                      = 0;
uint8_t    gucNextDirect                        = 0;

uint16_t		 guscMAXSPEED = 1000;					/*  �����������ʱ������ٶ�  ÿ��������������  */
uint16_t		 guscSEARCHSPEED = 300;				/*  �����������ʱ�������ٶ�  ÿ��������������  */
uint16_t		 guscMINSPEED = 100;					/*  �����������ʱ����С�ٶ�  ÿ��������������  */
int16_t		 guscROTATESPEED = 108;//120

__IO int32_t    giCurMaxSpeed = 0;        /*  ���浱ǰ�������е�����ٶ�      */
uint8_t 	gucSpeedCtrl = SPEEDUP;
uint8_t		gucMouseState = MOUSE_STOP;

extern __IO int16_t   gsTunningStraight;

void DebugWait(uint8_t code);

/*********************************************************************************************************
** Function name:       mouseRunBLocks
** Descriptions:        ������ǰ��ָ��������
** input parameters:    cNBlock������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mouseRunBLocks1(int8_t  cNBlock)
{
    int8_t cNBlock0 = cNBlock;
    int8_t cTmpBlock = 0;
    bool bLeftCheck = false, bRightCheck = false, bCoorUpdate = true;
    //    if(gucMouseTask == SPURT)
    //			DebugWait(cNBlock);
    if (gmMouse.msMouseState != MOUSESTOP) {
        bCoorUpdate = false;
    }
    if (cNBlock <= 3)
    {
        bLeftCheck = true;
        bRightCheck = true;
//                    if(gucPreDirect == gucNextDirect)
//                    {
//                        gsTunningPusle = PULSE_PER_CM*0;
//                    }
//                    else
//                    {
//                        gsTunningPusle = PULSE_PER_CM*3;//�����䣿
//                                    
//                    }
        gsTunningPusle = PULSE_PER_CM * 0.0;
        guscSEARCHSPEED = 250;
        giCurMaxSpeed = guscSEARCHSPEED;
    }
    else {
        gsTunningPusle = 0;
        guscSEARCHSPEED = 350;
        giCurMaxSpeed = guscSEARCHSPEED;
    }


    if (isGoal() == TRUE) {
        bLeftCheck = false;
        bRightCheck = false;
        gsTunningPusle = 0;
    }

    if ((gucMouseState == MOUSE_TURNRIGHT) || (gucMouseState == MOUSE_TURNLEFT))
    {
        //        gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM*13;
        //        gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM*13;
        if (cNBlock == 1)
        {
            gsTunningPusle = PULSE_PER_CM * 2.5;//������//4.5
            gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM * 17.5;
            gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM * 17.5;
        }
        else
        {
            gsTunningPusle = PULSE_PER_CM * 1.5;//
            gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM * 9;
            gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM * 9;
        }
    }

    cTmpBlock = cNBlock;
    gucMouseState = MOUSE_GOAHEAD;
    giCurMaxSpeed = guscSEARCHSPEED;
    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseRef + cNBlock * ONEBLOCK;
    gmMouse.mMotors[LEFTMOTOR].uiPulseRef = gmMouse.mMotors[LEFTMOTOR].uiPulseRef + cNBlock * ONEBLOCK;
    gmMouse.msMouseState = MOUSERUN;
    gucSpeedCtrl = SPEEDUP;
    while (gmMouse.msMouseState != MOUSESTOP)
    {
        if (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr >= ONEBLOCK)
        {   /*  �ж��Ƿ�����һ��            */
            gmMouse.mMotors[LEFTMOTOR].uiPulseRef -= ONEBLOCK;
            gmMouse.mMotors[LEFTMOTOR].uiPulseCtr -= ONEBLOCK;
            if (bCoorUpdate == true) {
                cNBlock--; //�Ѿ��߹�һ���ˣ�ʣ�µĸ���

                if(cNBlock0 == 4)
                {
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 300;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 5)
                {
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 300;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;
                        goto End;
                    }
                } 
                else if(cNBlock0 == 6)//��������и��������else if
                {
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                } 
                else if(cNBlock0 == 7)//��������и��������else if
                {
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 8)//��������и��������else if
                {
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 400;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 9)//��������и��������else if
                {
                    if (cNBlock == 8)
                    {
                        guscSEARCHSPEED = 400;
                    }
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 10)//��������и��������else if
                {
                    if (cNBlock == 9)
                    {
                        guscSEARCHSPEED = 400;
                    }
                    if (cNBlock == 8)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 300;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 11)//��������и��������else if
                {
                    if (cNBlock == 10)
                    {
                        guscSEARCHSPEED = 400;
                    }                    
                    if (cNBlock == 9)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 8)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 12)//��������и��������else if
                {
                    if (cNBlock == 11)
                    {
                        guscSEARCHSPEED = 400;
                    }                    
                    if (cNBlock == 10)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 9)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 8)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 13)//��������и��������else if
                {
                    if (cNBlock == 12)
                    {
                        guscSEARCHSPEED = 400;
                    }                    
                    if (cNBlock == 11)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 10)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 9)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 8)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 14)//��������и��������else if
                {
                    if (cNBlock == 13)
                    {
                        guscSEARCHSPEED = 400;
                    }                     
                    if (cNBlock == 12)
                    {
                        guscSEARCHSPEED = 350;
                    }                    
                    if (cNBlock == 11)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 10)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 9)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 8)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }
                else if(cNBlock0 == 15)//��������и��������else if
                {
                    if (cNBlock == 14)
                    {
                        guscSEARCHSPEED = 400;
                    }                     
                    if (cNBlock == 13)
                    {
                        guscSEARCHSPEED = 350;
                    }                     
                    if (cNBlock == 12)
                    {
                        guscSEARCHSPEED = 350;
                    }                    
                    if (cNBlock == 11)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 10)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 9)
                    {
                        guscSEARCHSPEED = 350;
                    }
                    if (cNBlock == 8)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 7)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 6)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 5)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 4)
                    {
                        guscSEARCHSPEED = 250;
                    } 
                    /*����������ʼ�ٶ���350*/                    
                    if (cNBlock == 3)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 2)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 1)
                    {
                        guscSEARCHSPEED = 250;
                    }
                    if (cNBlock == 0)
                    {
                        guscSEARCHSPEED = guscSEARCHSPEED1;//��������ɻ󣿣��������Ӱ���̻��յ����һ�ε��ٶ� guscROTATESPEED�����ٶ�
                        goto End;
                    }
                }                
                else//(cNBlock0 <= 3)
                {
                    if (cNBlock == 0)
                    {
                        //guscSEARCHSPEED = 250;
                        goto End;
                    }
                }

                if (cNBlock < cTmpBlock - 1)//������һ��ʱ��
                    gucSpeedCtrl = SPEEDUP;
            }
            else {
                bCoorUpdate = true;
            }

        }

        if (gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            gmMouse.mMotors[RIGHTMOTOR].uiPulseRef -= ONEBLOCK;
            gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr -= ONEBLOCK;
        }
    Check:
        if (cNBlock < 2) {
            //						if(gucPreDirect == gucNextDirect)
            //						{
            //								gsTunningPusle = PULSE_PER_CM*3;
            //						}
            //						else
            //						{
            //								gsTunningPusle = PULSE_PER_CM*3;//�����䣿
            //								
            //						}
            if (gmMouse.pPids[SPEEDPID].sFeedBack > guscSEARCHSPEED) {
                gmMouse.pPids[SPEEDPID].sRef = guscSEARCHSPEED;
            }
            if (bLeftCheck == true)
            {   /*  �Ƿ����������            */
                if (gusDistance[SIDELEFT] > WALL_FARDIST)             /*  �����֧·����������        */
                {
                    gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;//ת���ٶ�
                    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM * 9 - gsTunningPusle;    //3094(89mm)
                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_PER_CM * 9 - gsTunningPusle;
                    //									 gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr  + PULSE_PER_CM*9 - gsTunningPusle;    //3094(89mm)
                    //                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef  = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr   + PULSE_PER_CM*9 - gsTunningPusle;
                    while (gusDistance[SIDELEFT] > (WALL_FARDIST - 10))
                    {
                        if ((gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_HALF_CM) > gmMouse.mMotors[LEFTMOTOR].uiPulseRef)
                        {
                            //														DebugWait();
                            goto End;
                        }
                    }
                    goto End;
                }
            }
            else {                                                    /*  �����ǽʱ��ʼ���������  */
                if (gusDistance[SIDELEFT] < WALL_FARDIST) {
                    bLeftCheck = true;
                }
            }
            if (bRightCheck == true)
            {   /*  �Ƿ��������ұ�            */
                if (gusDistance[SIDERIGHT] > WALL_FARDIST)               /*  �ұ���֧·����������        */
                {
                    gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
                    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM * 9 - gsTunningPusle;
                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_PER_CM * 9 - gsTunningPusle;
                    //									                    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef  = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM*9- gsTunningPusle;
                    //                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef   = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr  + PULSE_PER_CM*9- gsTunningPusle;
                    while (gusDistance[SIDERIGHT] > (WALL_FARDIST - 10))
                    {

                        if ((gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_HALF_CM) > gmMouse.mMotors[LEFTMOTOR].uiPulseRef)
                        {
                            goto End;
                        }
                    }
                    goto End;
                }
            }
            else {
                if (gusDistance[SIDERIGHT] < WALL_FARDIST) {                   /*  �ұ���ǽʱ��ʼ�������ұ�  */
                    bRightCheck = true;
                }
            }
        }
    }
    /*
     *  �趨���������õ������ߵ�֧·������λ��
     */
End:;//gucSpeedCtrl= 3;
    gucPreDirect = gucNextDirect;
    //gmMouse.pPids[SPEEDPID].sRef=23;
//		DebugWait();

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mouseRunBLocks (int8_t  cNBlock)                                    
{
		int8_t cTmpBlock = 0;
		bool bLeftCheck = false, bRightCheck = false, bCoorUpdate = true;
//    if(gucMouseTask == SPURT)
//			DebugWait(cNBlock);
    if (gmMouse.msMouseState != MOUSESTOP) {
        bCoorUpdate = false;
    }
    if(cNBlock==1)
    {
        bLeftCheck = true;
        bRightCheck = true;
//                    if(gucPreDirect == gucNextDirect)
//                    {
//                        gsTunningPusle = PULSE_PER_CM*0;
//                    }
//                    else
//                    {
//                        gsTunningPusle = PULSE_PER_CM*3;//�����䣿
//                                    
//                    }
				gsTunningPusle = PULSE_PER_CM*0.0;
        giCurMaxSpeed = guscSEARCHSPEED;
    }
    else {
        gsTunningPusle = 0;
				giCurMaxSpeed = guscSEARCHSPEED;
    }
    
		
    if(isGoal() == TRUE){
        bLeftCheck = false;
        bRightCheck = false;
        gsTunningPusle = 0;
    }
		
    if((gucMouseState==MOUSE_TURNRIGHT)||(gucMouseState==MOUSE_TURNLEFT))
    {
//        gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM*13;
//        gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM*13;
				if(cNBlock == 1)
				{
						gsTunningPusle = PULSE_PER_CM*2.5;//������ 4.5
						gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM*17.5;
						gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM*17.5;
        }
        else
        {
            gsTunningPusle = PULSE_PER_CM*1.5;//
						gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM*9;
						gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM*9;
        }
    }

    cTmpBlock=cNBlock;
    gucMouseState   = MOUSE_GOAHEAD;
		giCurMaxSpeed      =   guscSEARCHSPEED;
    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseRef + cNBlock * ONEBLOCK ;
    gmMouse.mMotors[LEFTMOTOR].uiPulseRef  = gmMouse.mMotors[LEFTMOTOR].uiPulseRef  + cNBlock * ONEBLOCK ;
    gmMouse.msMouseState = MOUSERUN;
		gucSpeedCtrl=SPEEDUP;
    while (gmMouse.msMouseState != MOUSESTOP)
    {
        if (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr >= ONEBLOCK)
        {   /*  �ж��Ƿ�����һ��            */
            gmMouse.mMotors[LEFTMOTOR].uiPulseRef -= ONEBLOCK;
            gmMouse.mMotors[LEFTMOTOR].uiPulseCtr -= ONEBLOCK;
            if (bCoorUpdate == true) {
                cNBlock--;
                if(cNBlock==0)
								{
										goto End;
								}
                
                if(cNBlock<cTmpBlock-1)//������һ��ʱ��
                    gucSpeedCtrl=SPEEDUP;
            } else {
                bCoorUpdate = true;
            }

        }

        if (gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            gmMouse.mMotors[RIGHTMOTOR].uiPulseRef -= ONEBLOCK;
            gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr -= ONEBLOCK;
        }
				Check:				
        if (cNBlock < 2) {
//						if(gucPreDirect == gucNextDirect)
//						{
//								gsTunningPusle = PULSE_PER_CM*3;
//						}
//						else
//						{
//								gsTunningPusle = PULSE_PER_CM*3;//�����䣿
//								
//						}
            if(gmMouse.pPids[SPEEDPID].sFeedBack>guscSEARCHSPEED) {
                gmMouse.pPids[SPEEDPID].sRef=guscSEARCHSPEED;
            }
            if (bLeftCheck == true)
            {   /*  �Ƿ����������            */
                if (gusDistance[SIDELEFT] > WALL_FARDIST)             /*  �����֧·����������        */
                {
										gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
                    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr  + PULSE_PER_CM*9 - gsTunningPusle;    //3094(89mm)
                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef  = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr   + PULSE_PER_CM*9 - gsTunningPusle;
//									 gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr  + PULSE_PER_CM*9 - gsTunningPusle;    //3094(89mm)
//                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef  = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr   + PULSE_PER_CM*9 - gsTunningPusle;
                    while (gusDistance[SIDELEFT] > (WALL_FARDIST-10))
                    {
                        if ((gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_HALF_CM) > gmMouse.mMotors[LEFTMOTOR].uiPulseRef)
                        {
//														DebugWait();
                            goto End;
                        }
                    }
										goto End;
                }
            } else {                                                    /*  �����ǽʱ��ʼ���������  */
                if (gusDistance[SIDELEFT] < WALL_FARDIST) {
                    bLeftCheck = true;
                }
            }
            if (bRightCheck == true)
            {   /*  �Ƿ��������ұ�            */
                if (gusDistance[SIDERIGHT] > WALL_FARDIST)               /*  �ұ���֧·����������        */
                {
										gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
                    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef  = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM*9- gsTunningPusle;
                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef   = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr  + PULSE_PER_CM*9- gsTunningPusle;
//									                    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef  = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM*9- gsTunningPusle;
//                    gmMouse.mMotors[LEFTMOTOR].uiPulseRef   = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr  + PULSE_PER_CM*9- gsTunningPusle;
                    while (gusDistance[SIDERIGHT] > (WALL_FARDIST-10))
                    {

                        if ((gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_HALF_CM) > gmMouse.mMotors[LEFTMOTOR].uiPulseRef)
                        {
                            goto End;
                        }
                    }
										goto End;
                }
            } else {
                if (gusDistance[SIDERIGHT] < WALL_FARDIST) {                   /*  �ұ���ǽʱ��ʼ�������ұ�  */
                    bRightCheck = true;
                }
            }
        }
    }
    /*
     *  �趨���������õ������ߵ�֧·������λ��
     */
End:     ;//gucSpeedCtrl= 3;
		gucPreDirect = gucNextDirect;
    //gmMouse.pPids[SPEEDPID].sRef=23;
//		DebugWait();
		
}

/*********************************************************************************************************
** Function name:       mouseStop
** Descriptions:        ������ֹͣ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseStop(void)
{
		ClearPIDAll();
    gmMouse.msMouseState = MOUSESTOP;
//    gmMouse.pPids[SPEEDPID].sRef = 0;
//		gmMouse.pPids[ROTATEPID].sRef = 0;
    gmMouse.mMotors[RIGHTMOTOR].sOutSpeed = 0;
    rightMotorCtrl();
    gmMouse.mMotors[LEFTMOTOR].sOutSpeed = 0;
    leftMotorCtrl();
    gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = 0;
    gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = 0;
		gmMouse.mMotors[RIGHTMOTOR].cDir = MOTORSTOP;
    gmMouse.mMotors[LEFTMOTOR].cDir = MOTORSTOP;
    gucSpeedCtrl=SPEEDHOLD;
	
}

float absf(float v)
{
		if(v>0.0f)return v;
		return (-1.0f*v);
}


float CalcAngleErr(float fOld, float fTarget)
{
		float err = 0.0f;
		
		if(fTarget > 0.0f)
		{
//			if(attitudeData.yaw > fOld)
			{
				err = attitudeData.yaw - fOld - fTarget;
				if(err > 180.0f)
					err -= 360.0f;
				else if(err < -180.0f)
					err += 360.0f;
			}
//			else
			{
//				err = attitudeData.yaw + 360.0f - fOld - fTarget;
			}
		}
		else
		{
//			if(attitudeData.yaw < fOld)
			{
				err = attitudeData.yaw - fOld - fTarget;
				if(err > 180.0f)
					err -= 360.0f;
				else if(err < -180.0f)
					err += 360.0f;
			}
//			else
			{
				//err = attitudeData.yaw - 360.0f - fOld - fTarget;
			}
		}
		return err;
}

float AnglePidCtrl(float fOld, float fTarget)
{
		float err = 0.0f;
		err = CalcAngleErr(fOld,fTarget);
		//err = (attitudeData.yaw - fTarget);
		
		gmMouse.pPids[ANGLEPID].sFeedBack = err;
		PIDAbsCompute(&gmMouse.pPids[ANGLEPID],500);
		
		gmMouse.pPids[ROTATEPID].sRef = gmMouse.pPids[ANGLEPID].fOutput;

		return absf(err);
}

#define TurnTimeout 500
/****************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        ��ת
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��         
*********************************************************************************************************/
void mouseTurnright(void)
{
		uint16_t mpuCount = 0;
		gucMouseState   = MOUSE_TURNRIGHT;
    gsTunningStraight = 0;
    gmMouse.msMouseState=MOUSERUN;        //�üӵ�����״̬
    
    gucMouseDir     = (gucMouseDir + 1) % 4;                            /*  ������                    */
		ClearPID(ANGLEPID);
		ClearPID(ROTATEPID);

    gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
 
		__IO float fOldAngle = attitudeData.yaw;

    while(gucMouseState == MOUSE_TURNRIGHT)
    {
				if(gbMpuUpdated == true)
				{
					gbMpuUpdated = false;
					if(AnglePidCtrl(fOldAngle,-90.0f) < 5.0f)
						break;
					if(mpuCount++ > TurnTimeout)
					{
						DebugWait(0xff);//��ʱ
						break;
					}
				}
    }
    ClearPID(ROTATEPID);
//    gucSpeedCtrl=SPEEDHOLD;
		
    gucMouseState   = MOUSE_TURNRIGHT;
    gucSpeedCtrl=SPEEDUP;
		gmMouse.msMouseState = MOUSESTOP;
		
//		DebugWait(11);
}
/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        ��ת
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnleft(void)
{
		uint16_t mpuCount = 0;
    
		gucMouseState   = MOUSE_TURNLEFT;
		gsTunningStraight = 0;
    gmMouse.msMouseState = MOUSERUN;
	
    gucMouseDir     = (gucMouseDir + 3) % 4;                            /*  ������                    */
    ClearPID(ANGLEPID);
		ClearPID(ROTATEPID);

    gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;

		__IO float fOldAngle = attitudeData.yaw;

    while(gucMouseState == MOUSE_TURNLEFT)
    {
				if(gbMpuUpdated == true)
				{
					gbMpuUpdated = false;
					if(AnglePidCtrl(fOldAngle,90.0f) < 5.0f)
						break;
					if(mpuCount++ > TurnTimeout)
					{
						DebugWait(0xff);//��ʱ
						break;
					}
				}
    }
    ClearPID(ROTATEPID);
//    gucSpeedCtrl=SPEEDHOLD;

    gucMouseState   = MOUSE_TURNLEFT;
    gucSpeedCtrl=SPEEDUP;
		gmMouse.msMouseState = MOUSESTOP;
}


/*********************************************************************************************************
** Function name:       mouseMoveDist
** Descriptions:        ������ǰ��ָ������
** input parameters:    usCM��ָ�����룬��λ����
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseMoveDist(uint16_t usCM)
{
		MouseState oldState = gmMouse.msMouseState;
		gmMouse.msMouseState = MOUSERUN;
    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + usCM*PULSE_PER_CM;
    //gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr=0;
    gmMouse.mMotors[LEFTMOTOR].uiPulseRef = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + usCM*PULSE_PER_CM;
    //gmMouse.mMotors[LEFTMOTOR].uiPulseCtr=0;
    while ((gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr+PULSE_HALF_CM ) <= gmMouse.mMotors[RIGHTMOTOR].uiPulseRef);
    while ((gmMouse.mMotors[LEFTMOTOR].uiPulseCtr+PULSE_HALF_CM ) <= gmMouse.mMotors[LEFTMOTOR].uiPulseRef);
		gmMouse.msMouseState = oldState;
}

/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        ����ǰ�����࣬��ת180��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/

void mouseTurnback(void)
{
		uint16_t mpuCount = 0;
		if(gbFrontBlocked == true)
		{
			gbFrontBlocked = false;
			gucSpeedCtrl=SPEEDHOLD;
			gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
			while((gusDistance[FRONTRIGHT] > 50)||(gusDistance[FRONTLEFT] > 50))
				HAL_Delay(1);
//			mouseMoveDist(3);
		}
		
    mouseStop();

    gucMouseState   = MOUSE_TURNBACK;
		gsTunningStraight = 0;
    gmMouse.msMouseState = MOUSERUN;
    gucMouseDir = (gucMouseDir + 2) % 4;
    
		ClearPID(ANGLEPID);
		ClearPID(ROTATEPID);
		
    gmMouse.pPids[SPEEDPID].sRef = 0;
		
		__IO float fOldAngle = attitudeData.yaw;

    while(gucMouseState == MOUSE_TURNBACK)
    {
				if(gbMpuUpdated == true)
				{
					gbMpuUpdated = false;
					if(AnglePidCtrl(fOldAngle,-180.0f) < 5.0f)
						break;
					if(mpuCount++ > (TurnTimeout*4))
					{
						DebugWait(0xff);//��ʱ
						break;
					}
				}
    }
    ClearPID(ROTATEPID);
		
//    gucMouseState   = MOUSE_GOBACK ;
//    gmMouse.pPids[SPEEDPID].sRef=-50;
//    mouseMoveDist(3);
    ClearPID(ANGLEPID);
//		ClearPID(ROTATEPID);
//		HAL_Delay(50);
//		gucMouseState   = MOUSE_GOAHEAD ;
//		gucSpeedCtrl=SPEEDUP;
		gmMouse.msMouseState = MOUSESTOP;
    gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = 5*PULSE_PER_CM;
    gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = 5*PULSE_PER_CM;

}




/*********************************************************************************************************
** Function name:       mouseTurnbackNo
** Descriptions:        ����ǰ�����࣬��ת180��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/

void mouseTurnbackNo(void)
{
    uint16_t mpuCount = 0;
//    if(gbFrontBlocked == true)
//		{
//			gbFrontBlocked = false;
//			gmMouse.pPids[SPEEDPID].sRef = 50;
//			gucSpeedCtrl=SPEEDHOLD;
//			while((gusDistance[FRONTRIGHT] > 60)||(gusDistance[FRONTLEFT] > 60))
//				HAL_Delay(10);
//			mouseMoveDist(3);
//		}
		
    mouseStop();

    gucMouseState   = MOUSE_TURNBACK;
		gsTunningStraight = 0;
    gmMouse.msMouseState = MOUSERUN;
    gucMouseDir = (gucMouseDir + 2) % 4;
    
		ClearPID(ANGLEPID);
		ClearPID(ROTATEPID);
		
    gmMouse.pPids[SPEEDPID].sRef = 0;
		
		__IO float fOldAngle = attitudeData.yaw;

    while(gucMouseState == MOUSE_TURNBACK)
    {
				if(gbMpuUpdated == true)
				{
					gbMpuUpdated = false;
					if(AnglePidCtrl(fOldAngle,-180.0f) < 5.0f)
						break;
					if(mpuCount++ > (TurnTimeout*4))
					{
						DebugWait(0xff);//��ʱ
						break;
					}
				}
    }
    ClearPID(ROTATEPID);
		
//    gucMouseState   = MOUSE_GOBACK ;
//    gmMouse.pPids[SPEEDPID].sRef=-30;
//    mouseMoveDist(6);
//    ClearPID(ANGLEPID);
//		ClearPID(ROTATEPID);
		HAL_Delay(50);
		gucMouseState   = MOUSE_GOAHEAD ;
		gucSpeedCtrl=SPEEDUP;
		gmMouse.msMouseState = MOUSESTOP;
//		gmMouse.pPids[SPEEDPID].sRef=60;//?
    gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = 0*PULSE_PER_CM;
    gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = 0*PULSE_PER_CM;
}

/*********************************************************************************************************
** Function name:       searchCrossWay
** Descriptions:        ǰ��ֱ�����ֲ�·
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
uint8_t ucRightCount = 0;
uint8_t searchCrossWay(void)                  //��������ת��
{
    bool bLeftCheck = false, bRightCheck = false, bCoorUpdate = true;
    
    if (gmMouse.msMouseState != MOUSESTOP)
    {
        bCoorUpdate = false;
    }
		giCurMaxSpeed      =   guscSEARCHSPEED;
    if((gucMouseState==MOUSE_TURNRIGHT)||(gucMouseState==MOUSE_TURNLEFT))//��һ��Ϊת��
    {
				if(((gusDistance[FRONTRIGHT] < WALL_FRONT_FAR_DIST)&&(gusDistance[FRONTLEFT] < WALL_FRONT_FAR_DIST))
									&&(gusDistance[SIDELEFT] < WALL_FARDIST)
									&&(gusDistance[SIDERIGHT] < WALL_FARDIST))//��Ҫ��ͷ
				{
						gmMouse.mMotors[LEFTMOTOR].uiPulseCtr =PULSE_PER_CM*17.5;//15.5
						gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr =PULSE_PER_CM*17;
						bCoorUpdate = true;
						giCurMaxSpeed      =   guscROTATESPEED;
//						DebugWait(99);
				}
				else
				{
						gmMouse.mMotors[LEFTMOTOR].uiPulseCtr =PULSE_PER_CM*17.5;//15.5
						gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr =PULSE_PER_CM*17;//�����һ����ת�䣬����һ���ľ��룬������һ��������µ�λ��
				}
        bLeftCheck = true;
        bRightCheck = true;
				
        if(((gusDistance[FRONTRIGHT] < WALL_FRONT_FAR_DIST + 25)||(gusDistance[FRONTLEFT] < WALL_FRONT_FAR_DIST + 25))
					||(gusDistance[SIDELEFT] > WALL_FARDIST)
					||(gusDistance[SIDERIGHT] > WALL_FARDIST)) {
						
            if((gusDistance[FRONTRIGHT] < WALL_FRONT_FAR_DIST + 25)||(gusDistance[FRONTLEFT] < WALL_FRONT_FAR_DIST + 25))
						{
//								DebugWait(11);
                gsTunningPusle = PULSE_PER_CM*3.5;		//ǰ��Զ�������ϰ�����Ҫ����ת�䣬���ٲ���ֵ		
								gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
//								gucMouseState   = MOUSE_GO;
//								bCoorUpdate = true;
						}
            else
						{
//								DebugWait(22);
								gsTunningPusle = PULSE_PER_CM*3.0;		//ǰ�����ϰ�����Ҫ����ת�䣬���ٲ���ֵ
								gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
//								gucMouseState   = MOUSE_GO;
						}
        }
        else {
						//DebugWait(00);
            gsTunningPusle = PULSE_PER_CM*0;
						gucMouseState   = MOUSE_GOAHEAD;
        }
    }
    else {
        gsTunningPusle =0;
				gucMouseState   = MOUSE_GOAHEAD;
    }
    
    giCurMaxSpeed      =   guscSEARCHSPEED;
    gmMouse.mMotors[RIGHTMOTOR].uiPulseRef =   MAZETYPE * ONEBLOCK;
    gmMouse.mMotors[LEFTMOTOR].uiPulseRef  =   MAZETYPE * ONEBLOCK;
    gmMouse.msMouseState = MOUSERUN;
    gucSpeedCtrl=SPEEDUP;
    while (gmMouse.msMouseState != MOUSESTOP)
    {
        if (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr >= ONEBLOCK)
        {   /*  �ж��Ƿ�����һ��*/
            gmMouse.mMotors[LEFTMOTOR].uiPulseRef -= ONEBLOCK;
            gmMouse.mMotors[LEFTMOTOR].uiPulseCtr -= ONEBLOCK;
            if (bCoorUpdate == true)
            {
								if(((gusDistance[FRONTRIGHT] < WALL_FRONT_FAR_DIST)&&(gusDistance[FRONTLEFT] < WALL_FRONT_FAR_DIST))
									&&(gusDistance[SIDELEFT] < WALL_FARDIST)
									&&(gusDistance[SIDERIGHT] < WALL_FARDIST))//��Ҫ��ͷ
								{
//										DebugWait(0x11);
										gbFrontBlocked = true;
										mouseCoorUpdate(); 		//��ͷ��ʱ�����
										return 0;
								}
								mouseCoorUpdate();		//  ��������������                    
            }
            else
            {
                bCoorUpdate = true;
            }
        }
        if (gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            gmMouse.mMotors[RIGHTMOTOR].uiPulseRef -= ONEBLOCK;
            gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr -= ONEBLOCK;
        }
				


        if (bLeftCheck == true) {                                                       /*  �Ƿ����������            */
            if  (gusDistance[SIDELEFT] > WALL_FARDIST)
            {   /*  �����֧·����������        */
//								ucRightCount++;
//								if(ucRightCount >= 2)
//									DebugWait(11);
								gucMouseState = MOUSE_GO;gsTunningStraight = 0;
//								gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
//                gmMouse.mMotors[RIGHTMOTOR].uiPulseRef =  gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM*8 - gsTunningPusle; 
//                gmMouse.mMotors[LEFTMOTOR].uiPulseRef  =  gmMouse.mMotors[LEFTMOTOR].uiPulseCtr  + PULSE_PER_CM*8 - gsTunningPusle;
							                gmMouse.mMotors[RIGHTMOTOR].uiPulseRef =  gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM*9 - gsTunningPusle; 
                gmMouse.mMotors[LEFTMOTOR].uiPulseRef  =  gmMouse.mMotors[LEFTMOTOR].uiPulseCtr  + PULSE_PER_CM*9 - gsTunningPusle;

								while(gusDistance[SIDELEFT] > 100)
                {
                    if ((gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_HALF_CM) > gmMouse.mMotors[LEFTMOTOR].uiPulseRef)
                    {
												break;
                    }
                }
								if (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr >= ONEBLOCK*0.75)
									mouseCoorUpdate(); //������·�ڸ�������
								return 0;
            }
        } else {                                                        /*  �����ǽʱ��ʼ���������  */
            if (gusDistance[SIDELEFT] < WALL_FARDIST) {
                bLeftCheck = true;
            }
        }
        if (bRightCheck == true) {                                                       /*  �Ƿ��������ұ�            */
            if  (gusDistance[SIDERIGHT] > WALL_FARDIST)
							{              /*  �ұ���֧·����������        */
								gucMouseState = MOUSE_GO;gsTunningStraight = 0;
//								gmMouse.pPids[SPEEDPID].sRef = guscROTATESPEED;
//                gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM*8 - gsTunningPusle; 
//                gmMouse.mMotors[LEFTMOTOR].uiPulseRef  = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr  + PULSE_PER_CM*8 - gsTunningPusle;
								                gmMouse.mMotors[RIGHTMOTOR].uiPulseRef = gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr + PULSE_PER_CM*9 - gsTunningPusle; 
                gmMouse.mMotors[LEFTMOTOR].uiPulseRef  = gmMouse.mMotors[LEFTMOTOR].uiPulseCtr  + PULSE_PER_CM*9 - gsTunningPusle;

								while(gusDistance[SIDERIGHT] > WALL_FARDIST){
                    if ((gmMouse.mMotors[LEFTMOTOR].uiPulseCtr + PULSE_HALF_CM) > gmMouse.mMotors[LEFTMOTOR].uiPulseRef)
                    {
												break;
                    }
                }
								if (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr >= ONEBLOCK*0.75)
									mouseCoorUpdate(); //������·�ڸ�������
								
								return 0;
            }
        } else {
            if (gusDistance[SIDERIGHT] < WALL_FARDIST)
            {
                bRightCheck = true;
            }
        }
    }
		return 0;
}


/*********************************************************************************************************
** Function name:       mouseCoorUpdate
** Descriptions:        ���ݵ�ǰ�����������ֵ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseCoorUpdate (void)
{
    switch (gucMouseDir) {

    case MAP_UP:
			if(gmcCurrentCoor.cY < 15)
        gmcCurrentCoor.cY++;
        break;

    case MAP_RIGHT:
			if(gmcCurrentCoor.cX < 15)
        gmcCurrentCoor.cX++;
        break;

    case MAP_DOWN:
			if(gmcCurrentCoor.cY > 0)
        gmcCurrentCoor.cY--;
        break;

    case MAP_LEFT:
			if(gmcCurrentCoor.cX > 0)
        gmcCurrentCoor.cX--;
        break;

    default:
        break;
    }
		
		gucMapVisited[gmcCurrentCoor.cX][gmcCurrentCoor.cY] = 0xff;
    wallCheck();
//		static int i = 0;
//		if(/*gmcCurrentCoor.cX <= 12 && */gmcCurrentCoor.cX == 2 /*&& gmcCurrentCoor.cY == 4*/)
//					DebugWait(66);
}

/*********************************************************************************************************
** Function name:       wallCheck
** Descriptions:        ���ݴ�����������ж��Ƿ����ǽ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      �ޡ�
*********************************************************************************************************/
void wallCheck (void)
{
    uint8_t ucMap = 0;
    gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] |= MOUSEWAY_B;
		
    if (gusDistance[SIDELEFT] < WALL_FARDIST)//����е���
    {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] &= ~MOUSEWAY_L;          //��Է���������ǽ����ȴת���ɾ�������
    }
    else
    {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] |=  MOUSEWAY_L;         //��Է���������ǽ����ȴת���ɾ�������
    }

    if ((gusDistance[FRONTLEFT] < WALL_FRONT_FAR_DIST)
			&&(gusDistance[FRONTRIGHT] < WALL_FRONT_FAR_DIST))//ǰ�����ϰ�
    {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] &= ~MOUSEWAY_F;
    }
    else
    {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] |=  MOUSEWAY_F;
    }
		
    if (gusDistance[SIDERIGHT] < WALL_FARDIST)//�ұ��е���
    {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] &= ~MOUSEWAY_R;
    } else {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] |=  MOUSEWAY_R;
    }
		ucMap = gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY];
//    gucMapWall[next.cX][next.cY]=ucMap;
		
//    if(gmcCurrentCoor.cY<(MAZETYPE-1))
//    {
//        gucMapWall[next.cX][next.cY+1] &= ~(((~ucMap)&0x01)*4);   /*����������Χ����ǽ�����ϸ���  ע����ˮ��*/
//    }
//    if(gmcCurrentCoor.cX<(MAZETYPE-1))
//    {
//        gucMapWall[next.cX+1][next.cY]&= ~(((~ucMap)&0x02)*4);
//    }
//    if(gmcCurrentCoor.cY>0)
//    {
//        gucMapWall[next.cX][next.cY-1]&= ~(((~ucMap)&0x04)/4);
//    }
//    if(gmcCurrentCoor.cX>0)
//    {
//        gucMapWall[next.cX-1][next.cY]&= ~(((~ucMap)&0x08)/4);
//    }

    if(gmcCurrentCoor.cY<(MAZETYPE-1))
    {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY+1] |=  ((ucMap&MAP_UP_BIT)*4);   /*����������Χ����ǽ�����ϸ���  ע���ڳ�ʼΪ��ǽʱ����*/
    }
    if(gmcCurrentCoor.cX<(MAZETYPE-1))
    {
        gucMapWay[gmcCurrentCoor.cX+1][gmcCurrentCoor.cY]|=  ((ucMap&MAP_RIGHT_BIT)*4);
    }
    if(gmcCurrentCoor.cY>0)
    {
        gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY-1]|=  ((ucMap&MAP_DOWN_BIT)/4);
    }
    if(gmcCurrentCoor.cX>0)
    {
        gucMapWay[gmcCurrentCoor.cX-1][gmcCurrentCoor.cY]|=  ((ucMap&MAP_LEFT_BIT)/4);
    }
}


extern uint16_t gucMapTmp[5];
void DebugWait(uint8_t code)
{
		
		gucMapTmp[0] = code;//gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY];
		gucMapTmp[1] = gucMapWay[0][3];
		gucMapTmp[2] = gucMapWay[0][4];
		gucMapTmp[3] = (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr>>8)&0xff;;
		gucMapTmp[4] = (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr)&0xff;;
//		if(gmcCurrentCoor.cY<(MAZETYPE-1))
//    {
//        gucMapTmp[1] = gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY];//gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY+1];
//    }
//    if(gmcCurrentCoor.cX<(MAZETYPE-1))
//    {
//        gucMapTmp[2] = gucMouseDir;//gucMapWay[gmcCurrentCoor.cX+1][gmcCurrentCoor.cY];
//    }
//    //if(gmcCurrentCoor.cY>0)
//    {
//        gucMapTmp[3] = (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr>>8)&0xff;//gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY-1];
//    }
//    //if(gmcCurrentCoor.cX>0)
//    {
//        gucMapTmp[4] = (gmMouse.mMotors[LEFTMOTOR].uiPulseCtr)&0xff;//gucMapWay[gmcCurrentCoor.cX-1][gmcCurrentCoor.cY];
//    }
		mouseStop();
		modbusInit();
		while(1)
		{
			
				gucMouseTask = SOFTBREAK;
				modbusProcess();
		}
}

void mouseReset(void)
{
		gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM*0;       //1182(34mm)
		gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM*0;
		gucMouseState=MOUSE_GOAHEAD;
		gmMouse.msMouseState = MOUSESTOP;
}

void mouseTurn(uint32_t rl)
{
		gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM*9;       //1182(34mm)
		gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM*9;
		gucMouseState=rl;
		gmMouse.msMouseState = MOUSESTOP;
}
void mouseSetPulse(uint16_t usCM)
{
		gmMouse.mMotors[LEFTMOTOR].uiPulseCtr = PULSE_PER_CM*usCM;       //1182(34mm)
		gmMouse.mMotors[RIGHTMOTOR].uiPulseCtr = PULSE_PER_CM*usCM;
}
//if(gmcCurrentCoor.cX == 3 && gmcCurrentCoor.cY == 5)
//									DebugWait();
