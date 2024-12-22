/**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse.Drive.h
** Last modified Date:  
** Last Version: 
** Description:         �ײ���������ͷ�ļ�
** 
**--------------------------------------------------------------------------------------------------------
** Created By: 
** Created date: 
** Version: 
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/


#ifndef __Mouse_Drive_h
#define __Mouse_Drive_h

/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "main.h"
#include "Micromouse.h"
#include "tim.h"
#include "usart.h"
#include "string.h"

/*********************************************************************************************************
  �����궨��--������״̬
*********************************************************************************************************/
#define MOUSE_STOP              0                                           /*  ������ֹͣ                  */
#define MOUSE_GOAHEAD           1                                           /*  ������ֱ��ǰ��              */
#define MOUSE_GO		            2                                           /*  ������ǰ��                  */
#define MOUSE_TURNLEFT          3                                           /*  ����������ת                */
#define MOUSE_TURNRIGHT         4                                           /*  ����������ת                */
#define MOUSE_TURNBACK          5                                           /*  ���������ת                */
#define MOUSE_TURNLEFTY         7                                           /*  ����������ת                */
#define MOUSE_TURNRIGHTY        8                                           /*  ����������ת                */
#define MOUSE_GOBACK            9
/*********************************************************************************************************
  �����궨��--����Ӽ��ٶ�
*********************************************************************************************************/
#define SPEEDUP         0                                           /*  �������                */
#define SPEEDDOWN       1                                           /*  �������                */
#define SPEEDHOLD       2                                           /*  �������                */

/*********************************************************************************************************
  �ṹ�嶨��
*********************************************************************************************************/

/*********************************************************************************************************
  �����궨��
*********************************************************************************************************/

uint8_t searchCrossWay(void);                                                  /*  ǰ��N��                     */
void mouseTurnleft(void);                                               /*  ����ת90��                  */
void mouseTurnright(void);                                              /*  ����ת90��                  */
void mouseTurnback(void);                                               /*  ���ת                      */
void mouseTurnbackNo(void);
void mouseMoveDist(uint16_t usCM);

void mouseCoorUpdate(void);

void wallCheck(void);                                          /*  ǽ�ڼ��                    */

void mouseRunBLocks(int8_t  cNBlock);                                       /*  ǰ��N��                     */
void mouseRunBLocks1(int8_t  cNBlock); 

void PIDInit(void) ;

extern uint8_t ucUartTxBuf[64];
extern uint8_t ucUartRxBuf[16];

void mouseInit(void);
extern uint8_t    gucMouseTask ;
extern __IO int32_t    giCurMaxSpeed;
extern uint8_t		gucMouseState;
extern uint8_t 	gucSpeedCtrl;
extern MazeCoor          	gmcCurrentCoor;
extern uint16_t		 guscMAXSPEED;					/*  �����������ʱ������ٶ�  ÿ��������������  */
extern uint16_t		 guscSEARCHSPEED;				/*  �����������ʱ�������ٶ�  ÿ��������������  */
extern uint16_t		 guscMINSPEED;						/*  �����������ʱ����С�ٶ�  ÿ��������������  */
extern bool gbFrontBlocked;

void DebugWait(uint8_t code);
void mouseTurnAngle(void);
void mouseSetPulse(uint16_t usCM);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
