/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Maze.h"
#include "usart.h"
#include "string.h"
#include "modbus.h"

/*********************************************************************************************************
  ȫ�ֱ�������
*********************************************************************************************************/
static MazeCoor    gmcStart = {0,0};                /*  ��������                  */

MazeCoor gucGoal[4]={
	{XDST0,YDST0},
	{XDST0,YDST1},
	{XDST1,YDST0},
	{XDST1,YDST1},
};
uint8_t    gucMouseTask                        = WAIT;             /*  ״̬������ʼ״̬Ϊ�ȴ�      */
static uint8_t    gucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */

/*********************************************************************************************************
** Function name:       mapStepUpdate
** Descriptions:        ������Ŀ���Ϊ���ĵȸ�ͼ
** input parameters:    cX:    Ŀ�ĵغ�����
**                      cY:    Ŀ�ĵ�������
** output parameters:   gucMapStep[][]:  �������ϵĵȸ�ֵ
** Returned value:      ��
*********************************************************************************************************/
void mapStepUpdate (int8_t  cX, int8_t  cY)
{
    uint8_t n         = 0;                                                /*  GmcStack[]�±�              */
    uint8_t ucStep    = 1;                                                /*  �ȸ�ֵ                      */
    uint8_t ucStat    = 0;                                                /*  ͳ�ƿ�ǰ���ķ�����          */
    uint8_t i,j;
		static MazeCoor mcStack[MAZETYPE * MAZETYPE]       = {0};              /*  ��mapStepEdit()������ջʹ�� */
		
    mcStack[n].cX  = cX;                                               /*  ���Xֵ��ջ                 */
    mcStack[n].cY  = cY;                                               /*  ���Yֵ��ջ                 */
    n++;
    /*
     *  ��ʼ��������ȸ�ֵ
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            gucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  �����ȸ�ͼ��ֱ����ջ���������ݴ������
     */
    while (n) {
        gucMapStep[cX][cY] = ucStep++;                                  /*  ����ȸ�ֵ                  */

        /*
         *  �Ե�ǰ��������ǰ���ķ���ͳ��
         */
        ucStat = 0;
        if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                             /*  ǰ����·                    */
                (gucMapStep[cX][cY + 1] > (ucStep))) {                      /*  ǰ���ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                             /*  �ҷ���·                    */
                (gucMapStep[cX + 1][cY] > (ucStep))) {                      /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&
                (gucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&
                (gucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        /*
         *  û�п�ǰ���ķ�������ת���������ķ�֧��
         *  ������ѡһ��ǰ������ǰ��
         */
        if (ucStat == 0) {
            n--;
            cX = mcStack[n].cX;
            cY = mcStack[n].cY;
            ucStep = gucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  �ж����ǰ�����򣬱�������  */
                mcStack[n].cX = cX;                                    /*  ������Xֵ��ջ               */
                mcStack[n].cY = cY;                                    /*  ������Yֵ��ջ               */
                n++;
            }
            /*
             *  ����ѡ��һ����ǰ���ķ���ǰ��
             */
            if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                         /*  �Ϸ���·                    */
                    (gucMapStep[cX][cY + 1] > (ucStep))) {                  /*  �Ϸ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                         /*  �ҷ���·                    */
                    (gucMapStep[cX + 1][cY] > (ucStep))) {                  /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cX++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                         /*  �·���·                    */
                    (gucMapStep[cX][cY - 1] > (ucStep))) {                  /*  �·��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY--;                                                   /*  �޸�����                    */
                continue;
            }
            if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                         /*  ����·                    */
                    (gucMapStep[cX - 1][cY] > (ucStep))) {                  /*  �󷽵ȸ�ֵ���ڼƻ��趨ֵ    */
                cX--;                                                   /*  �޸�����                    */
                continue;
            }
        }
    }
}


/*********************************************************************************************************
** Function name:       mouseSpurt
** Descriptions:        �������̵��յ�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseSpurt (void)
{
	guscSEARCHSPEED = 225;
    uint8_t ucTemp = 0xff;
    int8_t cXdst = 0,cYdst = 0;
    /*
     *  ���յ���ĸ�����ֱ������ȸ�ͼ
     *  ȡ����������һ������ΪĿ���
     */
    if (gucMapWay[gucGoal[0].cX][gucGoal[0].cY] & (MAP_LEFT_BIT|MAP_DOWN_BIT)) {/*  �жϸ��յ������Ƿ��г���    */
        mapStepUpdate(gucGoal[0].cX,gucGoal[0].cY);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  ������������������        */
            cXdst  = gucGoal[0].cX;
            cYdst  = gucGoal[0].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
    if (gucMapWay[gucGoal[1].cX][gucGoal[1].cY] & (MAP_LEFT_BIT|MAP_UP_BIT)) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepUpdate(gucGoal[1].cX,gucGoal[1].cY);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  ������������������        */
            cXdst  = gucGoal[1].cX;
            cYdst  = gucGoal[1].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
    if (gucMapWay[gucGoal[2].cX][gucGoal[2].cY] & (MAP_RIGHT_BIT|MAP_DOWN_BIT)) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepUpdate(gucGoal[2].cX,gucGoal[2].cY);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  ������������������        */
            cXdst  = gucGoal[2].cX;
            cYdst  = gucGoal[2].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
    if (gucMapWay[gucGoal[3].cX][gucGoal[3].cY] & (MAP_RIGHT_BIT|MAP_UP_BIT)) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepUpdate(gucGoal[3].cX,gucGoal[3].cY);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > gucMapStep[gmcStart.cX][gmcStart.cY]) {                /*  ������������������        */
            cXdst  = gucGoal[3].cX;
            cYdst  = gucGoal[3].cY;
            ucTemp = gucMapStep[gmcStart.cX][gmcStart.cY];
        }
    }
		
    gotoCoor(cXdst,cYdst);                                        /*  ���е�ָ��Ŀ���            */

}

/*********************************************************************************************************
** Function name:       gotoCoor
** Descriptions:        ���������е�ָ������
** input parameters:    cX:    Ŀ�ĵغ�����
**                      cY:    Ŀ�ĵ�������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////
void gotoCoor1 (int8_t  cXdst, int8_t  cYdst)
{
    uint8_t ucStep = 1;
    int8_t  cNBlock = 0, cDirTemp;
    int8_t cX,cY;
    cX = gmcCurrentCoor.cX;
    cY = gmcCurrentCoor.cY;
    mapStepUpdate(cXdst,cYdst);							/*  �����ȸ�ͼ                  */
    /*
     *  ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
     */
    while ((cX != cXdst) || (cY != cYdst)) {

        ucStep = gucMapStep[cX][cY];
        /*
         *  ��ѡһ���ȸ�ֵ�ȵ�ǰ����ȸ�ֵС�ķ���ǰ��
         */
        if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                             /*  �Ϸ���·                    */
                (gucMapStep[cX][cY + 1] < ucStep)) {                        /*  �Ϸ��ȸ�ֵ��С              */
            cDirTemp = MAP_UP;                                              /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                             /*  �ҷ���·                    */
                (gucMapStep[cX + 1][cY] < ucStep)) {                        /*  �ҷ��ȸ�ֵ��С              */
            cDirTemp = MAP_RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                             /*  �·���·                    */
                (gucMapStep[cX][cY - 1] < ucStep)) {                        /*  �·��ȸ�ֵ��С              */
            cDirTemp = MAP_DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                             /*  ����·                    */
                (gucMapStep[cX - 1][cY] < ucStep)) {                        /*  �󷽵ȸ�ֵ��С              */
            cDirTemp = MAP_LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        cDirTemp = (cDirTemp + 4 - gucMouseDir)%4;                      /*  ���㷽��ƫ����              */
        gucNextDirect = cDirTemp;

        if (cNBlock > 0) {
            mouseRunBLocks1(cNBlock);                                    /*  ǰ��cNBlock��               */
        }
				
        cNBlock = 0;                                                    /*  ��������                    */

        /*
         *  ���Ƶ�����ת��
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
     *  �ж������Ƿ���ɣ��������ǰ��
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
    mapStepUpdate(cXdst,cYdst);							/*  �����ȸ�ͼ                  */
    /*
     *  ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
     */
    while ((cX != cXdst) || (cY != cYdst)) {

        ucStep = gucMapStep[cX][cY];
        /*
         *  ��ѡһ���ȸ�ֵ�ȵ�ǰ����ȸ�ֵС�ķ���ǰ��
         */
        if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                             /*  �Ϸ���·                    */
                (gucMapStep[cX][cY + 1] < ucStep)) {                        /*  �Ϸ��ȸ�ֵ��С              */
            cDirTemp = MAP_UP;                                              /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                             /*  �ҷ���·                    */
                (gucMapStep[cX + 1][cY] < ucStep)) {                        /*  �ҷ��ȸ�ֵ��С              */
            cDirTemp = MAP_RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                             /*  �·���·                    */
                (gucMapStep[cX][cY - 1] < ucStep)) {                        /*  �·��ȸ�ֵ��С              */
            cDirTemp = MAP_DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                             /*  ����·                    */
                (gucMapStep[cX - 1][cY] < ucStep)) {                        /*  �󷽵ȸ�ֵ��С              */
            cDirTemp = MAP_LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == gucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        cDirTemp = (cDirTemp + 4 - gucMouseDir)%4;                      /*  ���㷽��ƫ����              */
        gucNextDirect = cDirTemp;

        if (cNBlock > 0) {
            mouseRunBLocks(cNBlock);                                    /*  ǰ��cNBlock��               */
        }
				
        cNBlock = 0;                                                    /*  ��������                    */

        /*
         *  ���Ƶ�����ת��
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
     *  �ж������Ƿ���ɣ��������ǰ��
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
** Descriptions:        ���ݵ��������Է���ȡ���÷������Թ����ǽ������
** input parameters:    ucDirTemp: ���������Է���
** output parameters:   ��
** Returned value:      gucMapWay[cX][cY] : ǽ������
*********************************************************************************************************/
uint8_t getNextStep (uint8_t  ucDirTemp)
{
    int8_t cX = 0,cY = 0;

    /*
     *  �ѵ��������Է���ת��Ϊ���Է���
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
     *  ���ݾ��Է������÷��������ڸ������
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

    return(gucMapVisited[cX][cY]);                                        /*  �����Թ����ϵ�����          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        ���ַ�����������ǰ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void rightMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  ������������·            */
            (getNextStep(MOUSELEFT) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        ���ַ������������˶�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void leftMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  ������������·            */
            (getNextStep(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                 /*  ��������ת                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        ���ҷ���������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontRightMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */

        return;                                                         /*  ��������ת��              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  ������������·            */
            (getNextStep(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        ������������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
            (getNextStep(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_L) &&         /*  ������������·            */
            (getNextStep(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
            (getNextStep(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}

/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        ���ķ��򣬸��ݵ�����Ŀǰ���Թ���������λ�þ���ʹ�ú�����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void centralMethod (void)
{
    if (gmcCurrentCoor.cX & 0x08) {
        if (gmcCurrentCoor.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case MAP_RIGHT:                                                 /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case MAP_DOWN:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case MAP_LEFT:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case MAP_RIGHT:                                                 /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case MAP_DOWN:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case MAP_LEFT:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (gmcCurrentCoor.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case MAP_RIGHT:                                                 /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case MAP_DOWN:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case MAP_LEFT:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */
            switch (gucMouseDir) {

            case MAP_UP:                                                    /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case MAP_RIGHT:                                                 /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case MAP_DOWN:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case MAP_LEFT:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        ͳ��ĳ������ڻ�δ�߹���֧·��
** input parameters:    ucX����Ҫ����ĺ�����
**                      ucY����Ҫ�����������
** output parameters:   ��
** Returned value:      ucCt��δ�߹���֧·��
*********************************************************************************************************/
uint8_t crosswayCheck (int8_t  cX, int8_t  cY)
{
    uint8_t ucCt = 0;
    if ((gucMapWay[cX][cY] & MAP_UP_BIT) &&                                 /*  ���Է����Թ��Ϸ���·      */
            (gucMapVisited[cX][cY + 1]) == 0x00) {                            /*  ���Է����Թ��Ϸ�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((gucMapWay[cX][cY] & MAP_RIGHT_BIT) &&                                 /*  ���Է����Թ��ҷ���·      */
            (gucMapVisited[cX + 1][cY]) == 0x00) {                            /*  ���Է����Թ��ҷ�û���߹�  */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((gucMapWay[cX][cY] & MAP_DOWN_BIT) &&                                 /*  ���Է����Թ��·���·      */
            (gucMapVisited[cX][cY - 1]) == 0x00) {                            /*  ���Է����Թ��·�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((gucMapWay[cX][cY] & MAP_LEFT_BIT) &&                                 /*  ���Է����Թ�����·      */
            (gucMapVisited[cX - 1][cY]) == 0x00) {                            /*  ���Է����Թ���δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        ѡ��һ��֧·��Ϊǰ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mainTask (void)
{
		static uint8_t res          = 0;
    static uint8_t n          = 0;                                               /*  mcCrossway[]�±�           */
    static uint8_t ucRoadStat = 0;                                               /*  ͳ��ĳһ�����ǰ����֧·��  */
    static uint8_t ucTemp     = 0;                                               /*  ����START״̬������ת�� */
    static bool bStartClicked = false;
		static MazeCoor mcCrossway[MAZETYPE * MAZETYPE]    = {0};              /* �ݴ�δ�߹�֧·����  */
		
    switch (gucMouseTask) {                                         /*  ״̬������                  */
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
    case START:                                                     /*  �жϵ��������ĺ�����      */
        gusLedFlash = 0x7F;
				mouseStop();
				res = searchCrossWay();                                               /*  ��ǰ����                    */
        if (gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & 0x08) {         /*  �жϵ���������Ƿ���ڳ���  */
            if (MAZETYPE == 16) {                                    /*  �޸��ķ�֮һ�Թ����յ�����  */
//                GucXGoal0 = 8;
//                GucXGoal1 = 7;
            }
            gmcStart.cX   = MAZETYPE - 1;                             /*  �޸ĵ��������ĺ�����      */
            gmcCurrentCoor.cX = MAZETYPE - 1;                             /*  �޸ĵ�����ǰλ�õĺ�����  */
            /*
             *  ����Ĭ�ϵ����Ϊ(0,0)��������Ҫ���Ѽ�¼��ǽ������ת������
             */
            ucTemp = gmcCurrentCoor.cY;
            do {
                gucMapWay[MAZETYPE - 1][ucTemp] = gucMapWay[0][ucTemp];
                gucMapWay[0][ucTemp] = 0;
            } while (ucTemp--);
            /*
             *  ��OFFSHOOT[0]�б����������
             */
            mcCrossway[n].cX = MAZETYPE - 1;
            mcCrossway[n].cY = 0;
            n++;
						if(res == 255)
							gucMouseTask = MAZESEARCH;
						else
							gucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
        }
        if (gucMapWay[gmcCurrentCoor.cX][gmcCurrentCoor.cY] & 0x02) {         /*  �жϵ������ұ��Ƿ���ڳ���  */
            /*
             *  ��OFFSHOOT[0]�б����������
             */
            mcCrossway[n].cX = 0;
            mcCrossway[n].cY = 0;
            n++;
            gucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
        }
        break;

    case MAZESEARCH:
        if(isGoal() == TRUE)/*   �ж��Ƿ񵽴��յ�   */
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
            ucRoadStat = crosswayCheck(gmcCurrentCoor.cX,gmcCurrentCoor.cY);        /*  ͳ�ƿ�ǰ����֧·��          */
            if (ucRoadStat > 0)
            {   /*  �п�ǰ������                */
                if (ucRoadStat > 1)
                {   /*  �ж�����ǰ�����򣬱�������  */
                    mcCrossway[n].cX = gmcCurrentCoor.cX;
                    mcCrossway[n].cY = gmcCurrentCoor.cY;
                    n++;
                }
								
                crosswayChoice();                                       /*  �����ַ�������ѡ��ǰ������  */
                searchCrossWay();                                           /*  ǰ��һ��                    */
            }
            else if(ucRoadStat==1)
            {
                crosswayChoice();                                       /*  �����ķ�������ѡ��ǰ������  */
                searchCrossWay();
            }
            else
            {   /*  û�п�ǰ�����򣬻ص����֧·*/
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
				mouseSpurt();                                          /*  ������·�������յ�          */
        mouseMoveDist(6);
        mouseTurnbackNo();
				mouseMoveDist(4);
				mouseSetPulse(0);
        gotoCoor(gmcStart.cX,gmcStart.cY);                      /*  �����          */
        mouseMoveDist(5);
        mouseTurnbackNo();                       /*  ���ת���ָ���������        */
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
** Descriptions:        �Ƿ�ﵽ�յ�
** input parameters:    ��
** output parameters:   ��
** Returned value:      TRUE�������յ㣻FALSE��δ�����յ�
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
