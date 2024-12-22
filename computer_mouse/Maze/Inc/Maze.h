/**--------------File Info---------------------------------------------------------------------------------
** File Name:           Maze.h
** Last modified Date: 
** Last Version: 
** Description:         �����󶥲���Ƴ���ͷ�ļ�
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


#ifndef __Maze_h
#define __Maze_h


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Micromouse.h"
#include "Mouse.h"
#include "hardware.h"
/*********************************************************************************************************
  �����궨�� -- ��������������״̬
*********************************************************************************************************/
#define  WAIT           0                                               /*  �ȴ�״̬                    */
#define  START          1                                               /*  ����״̬                    */
#define  MAZESEARCH     2                                               /*  ��Ѱ״̬                    */
#define  SPURT          3                                               /*  ���״̬                    */
#define  RESTART        4                                               /*  ��������״̬                    */
#define  SPURTL         5                                               /*  ���״̬                    */
#define  RESTARTL       6                                               /*  ��������״̬                    */
#define  SOFTBREAK			7

/*********************************************************************************************************
  ������Ҫʹ�õ��ⲿ����
*********************************************************************************************************/

extern void  mouseTurnleft(void);                                       /*  ����ת90��                  */
extern void  mouseTurnright(void);                                      /*  ����ת90��                  */
extern void  mouseTurnback(void);                                       /*  ���ת                      */

extern void  mouseStop(void);
extern void  mouseReset(void);
extern void  mouseTurn(uint32_t rl);
void cornerMethodgo(void);
void centralMethodnew(void);
/*********************************************************************************************************
  ������Ҫʹ�õ��ⲿ����
*********************************************************************************************************/
extern MazeCoor gmcCurrentCoor;                                               /*  gmcCurrentCoor.x :�����������    */
                                                                        /*  gmcCurrentCoor.y :������������    */
                                                                        
extern uint8_t    gucMouseDir;                                            /*  �������ǰ������            */
extern uint16_t    gucMapWay[MAZETYPE*2][MAZETYPE*2];                        /*  gucMapWay[x][y]           */
                                                                         /*  x,������;y,������;          */
extern uint8_t    gucMapWall[MAZETYPE*2][MAZETYPE*2];  
extern uint8_t    gucMapVisited[MAZETYPE*2][MAZETYPE*2];  /*���ˮʱǽ������*/                                                       /*  bit3~bit0�ֱ������������   */

extern uint8_t    map;                                                                        /*  0:�÷�����·��1:�÷�����·  */
                 
extern uint8_t    gucNextDirect;
                
//������趨ֵ�����ĳ�������ʽ
extern int16_t gsIrDistance[4][3]; //����

static void  mapStepUpdate(int8_t  cX, int8_t  cY);
static void  mouseSpurt(void);
static void  gotoCoor(int8_t  cXdst, int8_t  cYdst);
static void  gotoCoor1(int8_t  cXdst, int8_t  cYdst);

static uint8_t getNextStep(uint8_t  ucDirTemp);
static void  rightMethod(void);
static void  leftMethod(void);
static void  frontRightMethod(void);
static void  frontLeftMethod(void);
static void  centralMethod(void);

uint8_t isGoal(void);
void mainTask (void);
void ProcessUart(void);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
