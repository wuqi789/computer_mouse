/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Maze.h"
#include "usart.h"
#include "string.h"
#include "modbus.h"

/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
static MazeCoor    gmcStart = {0,0};                /*  起点横坐标                  */

MazeCoor gucGoal[4]={
	{XDST0,YDST0},
	{XDST0,YDST1},
	{XDST1,YDST0},
	{XDST1,YDST1},
};
uint8_t    gucMouseTask                        = WAIT;             /*  状态机，初始状态为等待      */
static uint8_t    gucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */

/*********************************************************************************************************
** Function name:       mapStepUpdate
** Descriptions:        制作以目标点为起点的等高图
** input parameters:    cX:    目的地横坐标
**                      cY:    目的地纵坐标
** output parameters:   gucMapStep[][]:  各坐标上的等高值
** Returned value:      无
*********************************************************************************************************/
void mapStepUpdate (int8_t  cX, int8_t  cY)
{
    uint8_t n         = 0;                                                /*  GmcStack[]下标              */
    uint8_t ucStep    = 1;                                                /*  等高值                      */
    uint8_t ucStat    = 0;                                                /*  统计可前进的方向数          */
    uint8_t i,j;
		static MazeCoor mcStack[MAZETYPE * MAZETYPE]       = {0};              /*  在mapStepEdit()中作堆栈使用 */
		
    mcStack[n].cX  = cX;                                               /*  起点X值入栈                 */
    mcStack[n].cY  = cY;                                               /*  起点Y值入栈                 */
    n++;
    /*
     *  初始化各坐标等高值
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            gucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  制作等高图，直到堆栈中所有数据处理完毕
     */
    while (n) {
        gucMapStep[cX][cY] = ucStep++;                                  /*  填入等高值                  */

        /*
         *  对当前坐标格里可前进的方向统计
         */
        ucStat = 0;
        if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                             /*  前方有路                    */
                (gucMapStep[cX][cY + 1] > (ucStep))) {                      /*  前方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                             /*  右方有路                    */
                (gucMapStep[cX + 1][cY] > (ucStep))) {                      /*  右方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&
                (gucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&
                (gucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        /*
         *  没有可前进的方向，则跳转到最近保存的分支点
         *  否则任选一可前进方向前进
         */
        if (ucStat == 0) {
            n--;
            cX = mcStack[n].cX;
            cY = mcStack[n].cY;
            ucStep = gucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  有多个可前进方向，保存坐标  */
                mcStack[n].cX = cX;                                    /*  横坐标X值入栈               */
                mcStack[n].cY = cY;                                    /*  纵坐标Y值入栈               */
                n++;
            }
            /*
             *  任意选择一条可前进的方向前进
             */
            if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                         /*  上方有路                    */
                    (gucMapStep[cX][cY + 1] > (ucStep))) {                  /*  上方等高值大于计划设定值    */
                cY++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                         /*  右方有路                    */
                    (gucMapStep[cX + 1][cY] > (ucStep))) {                  /*  右方等高值大于计划设定值    */
                cX++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                         /*  下方有路                    */
                    (gucMapStep[cX][cY - 1] > (ucStep))) {                  /*  下方等高值大于计划设定值    */
                cY--;                                                   /*  修改坐标                    */
                continue;
            }
            if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                         /*  左方有路                    */
                    (gucMapStep[cX - 1][cY] > (ucStep))) {                  /*  左方等高值大于计划设定值    */
                cX--;                                                   /*  修改坐标                    */
                continue;
            }
        }
    }
}


/*********************************************************************************************************
** Function name:       mouseSpurt
** Descriptions:        电脑鼠冲刺到终点
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseSpurt (void)
{
	guscSEARCHSPEED = 225;
    uint8_t ucTemp = 0xff;
    int8_t cXdst = 0,cYdst = 0;
    /*
     *  对终点的四个坐标分别制作等高图
     *  取离起点最近的一个点作为目标点
     */
    if (gucMapWay[gucGoal[0].cX][gucGoal[0].cY] & (MAP_LEFT_BIT|MAP_DOWN_BIT)) {/*  判断该终点坐标是否有出口    */
        mapStepUpdate(gucGoal[0].cX,gucGoal[0].cY);                               /*  制作等高图                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  保存离起点最近的坐标        */
            cXdst  = gucGoal[0].cX;
            cYdst  = gucGoal[0].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
    if (gucMapWay[gucGoal[1].cX][gucGoal[1].cY] & (MAP_LEFT_BIT|MAP_UP_BIT)) {                     /*  判断该终点坐标是否有出口    */
        mapStepUpdate(gucGoal[1].cX,gucGoal[1].cY);                               /*  制作等高图                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  保存离起点最近的坐标        */
            cXdst  = gucGoal[1].cX;
            cYdst  = gucGoal[1].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
    if (gucMapWay[gucGoal[2].cX][gucGoal[2].cY] & (MAP_RIGHT_BIT|MAP_DOWN_BIT)) {                     /*  判断该终点坐标是否有出口    */
        mapStepUpdate(gucGoal[2].cX,gucGoal[2].cY);                               /*  制作等高图                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  保存离起点最近的坐标        */
            cXdst  = gucGoal[2].cX;
            cYdst  = gucGoal[2].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
    if (gucMapWay[gucGoal[3].cX][gucGoal[3].cY] & (MAP_RIGHT_BIT|MAP_UP_BIT)) {                     /*  判断该终点坐标是否有出口    */
        mapStepUpdate(gucGoal[3].cX,gucGoal[3].cY);                               /*  制作等高图                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  保存离起点最近的坐标        */
            cXdst  = gucGoal[3].cX;
            cYdst  = gucGoal[3].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
		
    gotoCoor(cXdst,cYdst);                                        /*  运行到指定目标点            */

}

/*********************************************************************************************************
** Function name:       gotoCoor
** Descriptions:        电脑鼠运行到指定坐标
** input parameters:    cX:    目的地横坐标
**                      cY:    目的地纵坐标
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////
void gotoCoor1 (int8_t  cXdst, int8_t  cYdst)
{
    uint8_t ucStep = 1;
    int8_t  cNBlock = 0, cDirTemp;
    int8_t cX,cY;
    cX = gmcCurrentCoor.cX;
    cY = gmcCurrentCoor.cY;
    mapStepUpdate(cXdst,cYdst);							/*  制作等高图                  */
    /*
     *  根据等高值向目标点运动，直到达到目的地
     */
    while ((cX != cXdst) || (cY != cYdst)) {

        ucStep = gucMapStep[cX][cY];
        /*
         *  任选一个等高值比当前自身等高值小的方向前进
         */
        if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                             /*  上方有路                    */
                (gucMapStep[cX][cY + 1] < ucStep)) {                        /*  上方等高值较小              */
            cDirTemp = MAP_UP;                                              /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                             /*  右方有路                    */
                (gucMapStep[cX + 1][cY] < ucStep)) {                        /*  右方等高值较小              */
            cDirTemp = MAP_RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                             /*  下方有路                    */
                (gucMapStep[cX][cY - 1] < ucStep)) {                        /*  下方等高值较小              */
            cDirTemp = MAP_DOWN;                                            /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                             /*  左方有路                    */
                (gucMapStep[cX - 1][cY] < ucStep)) {                        /*  左方等高值较小              */
            cDirTemp = MAP_LEFT;                                            /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        cDirTemp = (cDirTemp + 4 - gucMouseDir)%4;                      /*  计算方向偏移量              */
        gucNextDirect = cDirTemp;

        if (cNBlock > 0) {
            mouseRunBLocks1(cNBlock);                                    /*  前进cNBlock步               */
        }
				
        cNBlock = 0;                                                    /*  任务清零                    */

        /*
         *  控制电脑鼠转弯
         */
        switch (cDirTemp) {

        case MAP_RIGHT:
            mouseTurnright();
            break;

        case MAP_DOWN:
            mouseTurnback();
            break;

        case MAP_LEFT:
            mouseTurnleft();
            break;

        default:
						mouseReset();
            break;
        }
        gmcCurrentCoor.cX=cX;
        gmcCurrentCoor.cY=cY;
				
    }
    /*
     *  判断任务是否完成，否则继续前进
     */

    if (cNBlock > 0) {
        mouseRunBLocks1(cNBlock);
        gmcCurrentCoor.cX=cX;
        gmcCurrentCoor.cY=cY;
				mouseReset();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void gotoCoor (int8_t  cXdst, int8_t  cYdst)
{
    uint8_t ucStep = 1;
    int8_t  cNBlock = 0, cDirTemp;
    int8_t cX,cY;
    cX = gmcCurrentCoor.cX;
    cY = gmcCurrentCoor.cY;
    mapStepUpdate(cXdst,cYdst);							/*  制作等高图                  */
    /*
     *  根据等高值向目标点运动，直到达到目的地
     */
    while ((cX != cXdst) || (cY != cYdst)) {

        ucStep = gucMapStep[cX][cY];
        /*
         *  任选一个等高值比当前自身等高值小的方向前进
         */
        if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                             /*  上方有路                    */
                (gucMapStep[cX][cY + 1] < ucStep)) {                        /*  上方等高值较小              */
            cDirTemp = MAP_UP;                                              /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                             /*  右方有路                    */
                (gucMapStep[cX + 1][cY] < ucStep)) {                        /*  右方等高值较小              */
            cDirTemp = MAP_RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                             /*  下方有路                    */
                (gucMapStep[cX][cY - 1] < ucStep)) {                        /*  下方等高值较小              */
            cDirTemp = MAP_DOWN;                                            /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                             /*  左方有路                    */
                (gucMapStep[cX - 1][cY] < ucStep)) {                        /*  左方等高值较小              */
            cDirTemp = MAP_LEFT;                                            /*  记录方向                    */
            if (cDirTemp == gucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                continue;                                               /*  跳过本次循环                */
            }
        }
        cDirTemp = (cDirTemp + 4 - gucMouseDir)%4;                      /*  计算方向偏移量              */
        gucNextDirect = cDirTemp;

        if (cNBlock > 0) {
            mouseRunBLocks(cNBlock);                                    /*  前进cNBlock步               */
        }
				
        cNBlock = 0;                                                    /*  任务清零                    */

        /*
         *  控制电脑鼠转弯
         */
        switch (cDirTemp) {

        case MAP_RIGHT:
            mouseTurnright();
            break;

        case MAP_DOWN:
            mouseTurnback();
            break;

        case MAP_LEFT:
            mouseTurnleft();
            break;

        default:
						mouseReset();
            break;
        }
        gmcCurrentCoor.cX=cX;
        gmcCurrentCoor.cY=cY;
				
    }
    /*
     *  判断任务是否完成，否则继续前进
     */

    if (cNBlock > 0) {
        mouseRunBLocks(cNBlock);
        gmcCurrentCoor.cX=cX;
        gmcCurrentCoor.cY=cY;
				mouseReset();
    }
}

/*********************************************************************************************************
** Function name:       getNextStep
** Descriptions:        根据电脑鼠的相对方向，取出该方向上迷宫格的墙壁资料
** input parameters:    ucDirTemp: 电脑鼠的相对方向
** output parameters:   无
** Returned value:      gucMapWay[cX][cY] : 墙壁资料
*********************************************************************************************************/
uint8_t getNextStep (uint8_t  ucDirTemp)
{
    int8_t cX = 0,cY = 0;

    /*
     *  把电脑鼠的相对方向转换为绝对方向
     */
    switch (ucDirTemp) {

    case MOUSEFRONT:
        ucDirTemp = gucMouseDir;
        break;

    case MOUSELEFT:
        ucDirTemp = (gucMouseDir + 3) % 4;
        break;

    case MOUSERIGHT:
        ucDirTemp = (gucMouseDir + 1) % 4;
        break;

    default:
        break;
    }

    /*
     *  根据绝对方向计算该方向上相邻格的坐标
     */
    switch (ucDirTemp) {

    case MAP_UP:
        cX = gmcCurrentCoor.cX;
        cY = gmcCurrentCoor.cY + 1;
        break;

    case MAP_RIGHT:
        cX = gmcCurrentCoor.cX + 1;
        cY = gmcCurrentCoor.cY;
        break;

    case MAP_DOWN:
        cX = gmcCurrentCoor.cX;
        cY = gmcCurrentCoor.cY - 1;
        break;

    case MAP_LEFT:
        cX = gmcCurrentCoor.cX - 1;
        cY = gmcCurrentCoor.cY;
        break;

    default:
        break;
    }

    return(gucMapVisited[cX][cY]);                                        /*  返回迷宫格上的资料          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        右手法则，优先向右前进
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void rightMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
            (getNextStep(MOUSELEFT) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        左手法则，优先向左运动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void leftMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
            (getNextStep(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                 /*  电脑鼠左转                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        中右法则，优先向前运行，其次向右
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontRightMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */

        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
            (getNextStep(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        中左法则，优先向前运行，其次向左
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
            (getNextStep(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}

/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        中心法则，根据电脑鼠目前在迷宫中所处的位置觉定使用何种搜索法则
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void centralMethod (void)
{
    if (gmcCurrentCoor.cX & 0x08) {
        if (gmcCurrentCoor.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的右上角
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  当前电脑鼠向上              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case MAP_RIGHT:                                                 /*  当前电脑鼠向右              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case MAP_DOWN:                                                  /*  当前电脑鼠向下              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case MAP_LEFT:                                                  /*  当前电脑鼠向左              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的右下角
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  当前电脑鼠向上              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case MAP_RIGHT:                                                 /*  当前电脑鼠向右              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case MAP_DOWN:                                                  /*  当前电脑鼠向下              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case MAP_LEFT:                                                  /*  当前电脑鼠向左              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (gmcCurrentCoor.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的左上角
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  当前电脑鼠向上              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case MAP_RIGHT:                                                 /*  当前电脑鼠向右              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case MAP_DOWN:                                                  /*  当前电脑鼠向下              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case MAP_LEFT:                                                  /*  当前电脑鼠向左              */
                leftMethod();                                           /*  左手法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的左下角
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  当前电脑鼠向上              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case MAP_RIGHT:                                                 /*  当前电脑鼠向右              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case MAP_DOWN:                                                  /*  当前电脑鼠向下              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case MAP_LEFT:                                                  /*  当前电脑鼠向左              */
                rightMethod();                                          /*  右手法则                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        统计某坐标存在还未走过的支路数
** input parameters:    ucX，需要检测点的横坐标
**                      ucY，需要检测点的纵坐标
** output parameters:   无
** Returned value:      ucCt，未走过的支路数
*********************************************************************************************************/
uint8_t crosswayCheck (int8_t  cX, int8_t  cY)
{
    uint8_t ucCt = 0;
    if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                                 /*  绝对方向，迷宫上方有路      */
            (gucMapVisited[cX][cY + 1]) == 0x00) {                            /*  绝对方向，迷宫上方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                                 /*  绝对方向，迷宫右方有路      */
            (gucMapVisited[cX + 1][cY]) == 0x00) {                            /*  绝对方向，迷宫右方没有走过  */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                                 /*  绝对方向，迷宫下方有路      */
            (gucMapVisited[cX][cY - 1]) == 0x00) {                            /*  绝对方向，迷宫下方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                                 /*  绝对方向，迷宫左方有路      */
            (gucMapVisited[cX - 1][cY]) == 0x00) {                            /*  绝对方向，迷宫左方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        选择一条支路作为前进方向
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void crosswayChoice (void)
{
    switch (SEARCHMETHOD) {

    case RIGHTMETHOD:
        rightMethod();
        break;

    case LEFTMETHOD:
        leftMethod();
        break;

    case CENTRALMETHOD:
        centralMethod();
        break;

    case FRONTRIGHTMETHOD:
        frontRightMethod();
        break;

    case FRONTLEFTMETHOD:
        frontLeftMethod();
        break;


    default:
        break;
    }
}
uint16_t gusLeftDist = 60;
uint16_t gusRightDist = 60;
/*********************************************************************************************************
** Function name:       mainTask
** Descriptions:        主任务
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mainTask (void)
{
		static uint8_t res          = 0;
    static uint8_t n          = 0;                                               /*  mcCrossway[]下标           */
    static uint8_t ucRoadStat = 0;                                               /*  统计某一坐标可前进的支路数  */
    static uint8_t ucTemp     = 0;                                               /*  用于START状态中坐标转换 */
    static bool bStartClicked = false;
		static MazeCoor mcCrossway[MAZETYPE * MAZETYPE]    = {0};              /* 暂存未走过支路坐标  */
		
    switch (gucMouseTask) {                                         /*  状态机处理                  */
    case WAIT:
        HAL_Delay(100);
        if (startCheck() == TRUE)
        {
            bStartClicked = true;
						gusLedFlash = 0x3F;
        }
        if((bStartClicked == true)&&(gbFrontIRTrigger == true))
        {
						uint32_t sum = 0;
            bStartClicked = false;
            gucMouseTask = START;
            while(gbFrontIRTrigger == true);
            HAL_Delay(1000);
						
						for(n = 0; n< 32; n++)
						{
							sum += (gusDistance[SIDELEFT] + gusDistance[SIDERIGHT]);
						}
						sum = sum >> 6;
//						gusLeftDist = sum;
//						gusRightDist = sum;
        }
        break;
    case START:                                                     /*  判断电脑鼠起点的横坐标      */
        gusLedFlash = 0x7F;
				mouseStop();
				res = searchCrossWay();                                               /*  向前搜索                    */
        if (gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & 0x08) {         /*  判断电老鼠左边是否存在出口  */
            if (MAZETYPE == 16) {                                    /*  修改四分之一迷宫的终点坐标  */
//                GucXGoal0 = 8;
//                GucXGoal1 = 7;
            }
            gmcStart.cX   = MAZETYPE - 1;                             /*  修改电脑鼠起点的横坐标      */
            gmcCurrentCoor.cX = MAZETYPE - 1;                             /*  修改电脑鼠当前位置的横坐标  */
            /*
             *  由于默认的起点为(0,0)，现在需要把已记录的墙壁资料转换过来
             */
            ucTemp = gmcCurrentCoor.cY;
            do {
                gucMapWay[MAZETYPE - 1][ucTemp] = gucMapWay[0][ucTemp];
                gucMapWay[0][ucTemp] = 0;
            } while (ucTemp--);
            /*
             *  在OFFSHOOT[0]中保存起点坐标
             */
            mcCrossway[n].cX = MAZETYPE - 1;
            mcCrossway[n].cY = 0;
            n++;
						if(res == 255)
							gucMouseTask = MAZESEARCH;
						else
							gucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
        }
        if (gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & 0x02) {         /*  判断电老鼠右边是否存在出口  */
            /*
             *  在OFFSHOOT[0]中保存起点坐标
             */
            mcCrossway[n].cX = 0;
            mcCrossway[n].cY = 0;
            n++;
            gucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
        }
        break;

    case MAZESEARCH:
        if(isGoal() == TRUE)/*   判断是否到达终点   */
        {
            mouseMoveDist(6);
            mouseTurnbackNo(); 
            mouseMoveDist(3);
            gotoCoor(gmcStart.cX,gmcStart.cY);
						
            mouseMoveDist(4);
//						gbFrontBlocked = true;
            mouseTurnback();
						//mouseStop();
            gucMouseTask = SPURT;
            break;
        }
        else {
						wallCheck();
            ucRoadStat = crosswayCheck(gmcCurrentCoor.cX,gmcCurrentCoor.cY);        /*  统计可前进的支路数          */
            if (ucRoadStat > 0)
            {   /*  有可前进方向                */
                if (ucRoadStat > 1)
                {   /*  有多条可前进方向，保存坐标  */
                    mcCrossway[n].cX = gmcCurrentCoor.cX;
                    mcCrossway[n].cY = gmcCurrentCoor.cY;
                    n++;
                }
								
                crosswayChoice();                                       /*  用右手法则搜索选择前进方向  */
                searchCrossWay();                                           /*  前进一格                    */
            }
            else if(ucRoadStat==1)
            {
                crosswayChoice();                                       /*  用中心法则搜索选择前进方向  */
                searchCrossWay();
            }
            else
            {   /*  没有可前进方向，回到最近支路*/
                mouseTurnback();
                n=n-1;
                gotoCoor(mcCrossway[n].cX,mcCrossway[n].cY);

                ucRoadStat = crosswayCheck(gmcCurrentCoor.cX,gmcCurrentCoor.cY);
                if (ucRoadStat > 1) {
                    mcCrossway[n].cX = gmcCurrentCoor.cX;
                    mcCrossway[n].cY = gmcCurrentCoor.cY;
                    n++;
                }
                crosswayChoice();
								
                searchCrossWay();
            }
        }
        break;

    case SPURT:
				gusLedFlash = 0x0F;
        mouseStop();
				mouseSpurt();                                          /*  以最优路径冲向终点          */
        mouseMoveDist(6);
        mouseTurnbackNo();
				mouseMoveDist(4);
				mouseSetPulse(0);
        gotoCoor(gmcStart.cX,gmcStart.cY);                      /*  回起点          */
        mouseMoveDist(5);
        mouseTurnbackNo();                       /*  向后转，恢复出发姿势        */
				mouseStop();
				gusLedFlash = 0xFF;
        while (1)
        {
            if (startCheck() == TRUE)
            {
								gusLedFlash = 0x1F;
								gucMouseTask = WAIT;
                break;
            }
        }
        break;
		case SOFTBREAK:
				modbusProcess();
				break;
    default:
        break;
    }

}

/*********************************************************************************************************
** Function name:       isGoal
** Descriptions:        是否达到终点
** input parameters:    无
** output parameters:   无
** Returned value:      TRUE，到达终点；FALSE，未到达终点
*********************************************************************************************************/
uint8_t isGoal(void)
{
	if (((gmcCurrentCoor.cX==gucGoal[0].cX)&&(gmcCurrentCoor.cY==gucGoal[0].cY))
		||((gmcCurrentCoor.cX==gucGoal[1].cX)&&(gmcCurrentCoor.cY==gucGoal[1].cY))
    ||((gmcCurrentCoor.cX==gucGoal[2].cX)&&(gmcCurrentCoor.cY==gucGoal[2].cY))
		||((gmcCurrentCoor.cX==gucGoal[3].cX)&&(gmcCurrentCoor.cY==gucGoal[3].cY)))
            return TRUE;
	return FALSE;
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
