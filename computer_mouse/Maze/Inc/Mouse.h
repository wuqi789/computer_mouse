/**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse.Drive.h
** Last modified Date:  
** Last Version: 
** Description:         底层驱动程序头文件
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
  包含头文件
*********************************************************************************************************/
#include "main.h"
#include "Micromouse.h"
#include "tim.h"
#include "usart.h"
#include "string.h"

/*********************************************************************************************************
  常量宏定义--电脑鼠状态
*********************************************************************************************************/
#define MOUSE_STOP              0                                           /*  电脑鼠停止                  */
#define MOUSE_GOAHEAD           1                                           /*  电脑鼠直线前进              */
#define MOUSE_GO		            2                                           /*  电脑鼠前进                  */
#define MOUSE_TURNLEFT          3                                           /*  电脑鼠向左转                */
#define MOUSE_TURNRIGHT         4                                           /*  电脑鼠向右转                */
#define MOUSE_TURNBACK          5                                           /*  电脑鼠向后转                */
#define MOUSE_TURNLEFTY         7                                           /*  电脑鼠向左转                */
#define MOUSE_TURNRIGHTY        8                                           /*  电脑鼠向右转                */
#define MOUSE_GOBACK            9
/*********************************************************************************************************
  常量宏定义--电机加减速度
*********************************************************************************************************/
#define SPEEDUP         0                                           /*  电机加速                */
#define SPEEDDOWN       1                                           /*  电机减速                */
#define SPEEDHOLD       2                                           /*  电机保持                */

/*********************************************************************************************************
  结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
  常量宏定义
*********************************************************************************************************/

uint8_t searchCrossWay(void);                                                  /*  前进N格                     */
void mouseTurnleft(void);                                               /*  向左转90度                  */
void mouseTurnright(void);                                              /*  向右转90度                  */
void mouseTurnback(void);                                               /*  向后转                      */
void mouseTurnbackNo(void);
void mouseMoveDist(uint16_t usCM);

void mouseCoorUpdate(void);

void wallCheck(void);                                          /*  墙壁检测                    */

void mouseRunBLocks(int8_t  cNBlock);                                       /*  前进N格                     */
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
extern uint16_t		 guscMAXSPEED;					/*  电机减速运行时的最大速度  每控制周期脉冲数  */
extern uint16_t		 guscSEARCHSPEED;				/*  电机减速运行时的搜索速度  每控制周期脉冲数  */
extern uint16_t		 guscMINSPEED;						/*  电机减速运行时的最小速度  每控制周期脉冲数  */
extern bool gbFrontBlocked;

void DebugWait(uint8_t code);
void mouseTurnAngle(void);
void mouseSetPulse(uint16_t usCM);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
