/**--------------File Info---------------------------------------------------------------------------------
** File Name:           Maze.h
** Last modified Date: 
** Last Version: 
** Description:         电脑鼠顶层控制程序头文件
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
  包含头文件
*********************************************************************************************************/
#include "Micromouse.h"
#include "Mouse.h"
#include "hardware.h"
/*********************************************************************************************************
  常量宏定义 -- 定义电脑鼠的四种状态
*********************************************************************************************************/
#define  WAIT           0                                               /*  等待状态                    */
#define  START          1                                               /*  启动状态                    */
#define  MAZESEARCH     2                                               /*  搜寻状态                    */
#define  SPURT          3                                               /*  冲刺状态                    */
#define  RESTART        4                                               /*  重新启动状态                    */
#define  SPURTL         5                                               /*  冲刺状态                    */
#define  RESTARTL       6                                               /*  重新启动状态                    */
#define  SOFTBREAK			7

/*********************************************************************************************************
  申明需要使用的外部函数
*********************************************************************************************************/

extern void  mouseTurnleft(void);                                       /*  向左转90度                  */
extern void  mouseTurnright(void);                                      /*  向右转90度                  */
extern void  mouseTurnback(void);                                       /*  向后转                      */

extern void  mouseStop(void);
extern void  mouseReset(void);
extern void  mouseTurn(uint32_t rl);
void cornerMethodgo(void);
void centralMethodnew(void);
/*********************************************************************************************************
  申明需要使用的外部变量
*********************************************************************************************************/
extern MazeCoor gmcCurrentCoor;                                               /*  gmcCurrentCoor.x :电脑鼠横坐标    */
                                                                        /*  gmcCurrentCoor.y :电脑鼠纵坐标    */
                                                                        
extern uint8_t    gucMouseDir;                                            /*  电脑鼠的前进方向            */
extern uint16_t    gucMapWay[MAZETYPE*2][MAZETYPE*2];                        /*  gucMapWay[x][y]           */
                                                                         /*  x,横坐标;y,纵坐标;          */
extern uint8_t    gucMapWall[MAZETYPE*2][MAZETYPE*2];  
extern uint8_t    gucMapVisited[MAZETYPE*2][MAZETYPE*2];  /*存洪水时墙壁资料*/                                                       /*  bit3~bit0分别代表左下右上   */

extern uint8_t    map;                                                                        /*  0:该方向无路，1:该方向有路  */
                 
extern uint8_t    gucNextDirect;
                
//红外的设定值，最后改成数组形式
extern int16_t gsIrDistance[4][3]; //红外

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
