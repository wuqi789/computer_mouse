#include "modbus.h"
#include "hardware.h"
#include "motor.h"
#include "mouse.h"
#define ModbusBufSize 2048
#define ModbusRegNum 1024
#define ModbusWriteOffset 512
///////////////////////////////////////////////////////////
//uint32_t RS485_Baudrate=9600;//ͨѶ������
//uint8_t RS485_Parity=0;//0��У�飻1��У�飻2żУ��
uint8_t ModbusSlaveAddr;//�ӻ���ַ
//uint16_t RS485_Frame_Distance=4;//����֡��С�����ms),������ʱ������Ϊ����һ֡

uint8_t gucModbusRxBuf[ModbusBufSize];//���ջ�����2048�ֽ�
uint16_t gusModbusRecvLen;//���ճ���
bool gbModbusRecved;//֡�������
uint8_t gucModbusTxBuf[ModbusBufSize];//���ͻ�����

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Modbus�Ĵ����͵�Ƭ���Ĵ�����ӳ���ϵ
__IO uint32_t *Modbus_InputIO[100];//���뿪�����Ĵ���ָ��(����ʹ�õ���λ������)
__IO uint32_t *Modbus_OutputIO[100];//����������Ĵ���ָ��(����ʹ�õ���λ������)
uint16_t *Modbus_HoldReg[ModbusRegNum];//���ּĴ���ָ��
uint16_t gusCammand;

static void Modbus_02_Solve(void);
static void Modbus_01_Solve(void);
static void Modbus_05_Solve(void);
static void Modbus_15_Solve(void);
static void Modbus_03_Solve(void);
static void Modbus_06_Solve(void);
static void Modbus_16_Solve(void);

extern uint16_t             gucMapWay[MAZETYPE][MAZETYPE];

uint16_t gucMapTmp[5];
void Modbus_RegMap(void)
{
    uint8_t i,j;

    //���ּĴ���ָ��ָ��
    Modbus_HoldReg[0]=(uint16_t*)&gsIrLevel[0];
    Modbus_HoldReg[1]=(uint16_t*)&gsIrLevel[1];
    Modbus_HoldReg[2]=(uint16_t*)&gsIrLevel[2];
    Modbus_HoldReg[3]=(uint16_t*)&gsIrLevel[3];

    Modbus_HoldReg[4]=(uint16_t*)&gusDistance[0];
    Modbus_HoldReg[5]=(uint16_t*)&gusDistance[1];
    Modbus_HoldReg[6]=(uint16_t*)&gusDistance[2];
    Modbus_HoldReg[7]=(uint16_t*)&gusDistance[3];

    Modbus_HoldReg[8]=(uint16_t*)&usAdBuffer[4];

    Modbus_HoldReg[9]=(uint16_t*)&gmcCurrentCoor;
//		for(i=0; i<16; i++)
//		{
//			for(j=0; j<16; j++)
//				Modbus_HoldReg[10+i*16+j]=(uint16_t*)&gucMapWay[i][j];
//		}

    for(j=0; j<5; j++)
        Modbus_HoldReg[10+j]=(uint16_t*)&gucMapTmp[j];
				
		Modbus_HoldReg[15]=(uint16_t*)&(attitudeData.yaw);
		Modbus_HoldReg[16]=((uint16_t*)&(attitudeData.yaw))+1;

    Modbus_HoldReg[ModbusWriteOffset]=(uint16_t*)&gusCammand;//д��ָ��

    for(i=0; i<4; i++)
    {
        Modbus_HoldReg[ModbusWriteOffset+i*6+1]=(uint16_t*)&gmMouse.pPids[i].fKp;
        Modbus_HoldReg[ModbusWriteOffset+i*6+2]=((uint16_t*)(&gmMouse.pPids[i].fKp))+1;
        Modbus_HoldReg[ModbusWriteOffset+i*6+3]=(uint16_t*)&gmMouse.pPids[i].fKi;
        Modbus_HoldReg[ModbusWriteOffset+i*6+4]=((uint16_t*)(&gmMouse.pPids[i].fKi))+1;
        Modbus_HoldReg[ModbusWriteOffset+i*6+5]=(uint16_t*)&gmMouse.pPids[i].fKd;
        Modbus_HoldReg[ModbusWriteOffset+i*6+6]=((uint16_t*)(&gmMouse.pPids[i].fKd))+1;
    }

    Modbus_HoldReg[ModbusWriteOffset+25]=((uint16_t*)(&guscMAXSPEED));
    Modbus_HoldReg[ModbusWriteOffset+26]=((uint16_t*)(&guscSEARCHSPEED));
    Modbus_HoldReg[ModbusWriteOffset+27]=((uint16_t*)(&guscMINSPEED));

    for(i=0; i<4; i++)
    {
        for(j=0; j<16; j++)
        {
            Modbus_HoldReg[ModbusWriteOffset+256+i*16+j] = ((uint16_t*)(&IR_TABLE[i][j]));
        }

    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CRCУ�� �Լ�������ӵ�

const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;


const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;


static uint16_t CrcCompute(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF ;
    uint8_t uchCRCLo = 0xFF ;
    uint32_t uIndex ;
    while (usDataLen--)
    {
        uIndex = uchCRCHi ^ *puchMsg++ ;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }
    return ((uchCRCHi<< 8)  | (uchCRCLo)) ;
}//uint16 crc16(uint8 *puchMsg, uint16 usDataLen)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//��ʼ��USART1
void modbusInit(void)
{
//    RS485_TX_EN=0;//Ĭ��Ϊ����ģʽ
    ModbusSlaveAddr = 1;
    gusModbusRecvLen = 0;
    gbModbusRecved = false;
    gusCammand = 0;
    HAL_UART_Receive_DMA(&huart1,gucModbusRxBuf,ModbusBufSize);
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    Modbus_RegMap();//Modbus�Ĵ���ӳ��
}

void Usart_Idle_Callback()
{
    uint32_t sr_flag;
    sr_flag = __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);
    if(sr_flag != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        sr_flag = huart1.Instance->SR;
        sr_flag = huart1.Instance->DR;
        HAL_UART_DMAStop(&huart1);
        gusModbusRecvLen = (ModbusBufSize - hdma_usart1_rx.Instance->NDTR);
        gbModbusRecved = true;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		uint32_t sr_flag;
		if(huart->Instance == USART1)
		{
				HAL_UART_DMAStop(&huart1);
				sr_flag = huart1.Instance->SR;
        sr_flag = huart1.Instance->DR;
        gusModbusRecvLen = 0;
        gbModbusRecved = false;
				HAL_UART_Receive_DMA(&huart1,gucModbusRxBuf,ModbusBufSize);
		}
		UNUSED(sr_flag);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
		uint32_t sr_flag;
		if(huart->Instance == USART1)
		{
				HAL_UART_DMAStop(&huart1);
				sr_flag = huart1.Instance->SR;
        sr_flag = huart1.Instance->DR;
        gusModbusRecvLen = 0;
        gbModbusRecved = false;
				HAL_UART_Receive_DMA(&huart1,gucModbusRxBuf,ModbusBufSize);
		}
		UNUSED(sr_flag);
}

//////////////////////////////////////////////////////////////////////////////
//����n���ֽ�����
//buff:�������׵�ַ
//len�����͵��ֽ���
void modbusSendBuf(uint8_t *buff,uint8_t len)
{
    HAL_UART_Transmit_DMA(&huart1,buff,len);
}


/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
//RS485����������ڴ�����յ�������(������������ѭ������)
uint16_t startRegAddr;
uint16_t RegNum;
uint16_t calCRC;

void modbusProcess(void)
{
    uint16_t recCRC;
    if(gbModbusRecved==1)
    {
        if(gucModbusRxBuf[0]==ModbusSlaveAddr)//��ַ��ȷ
        {
            if((gucModbusRxBuf[1]==01)||(gucModbusRxBuf[1]==02)||(gucModbusRxBuf[1]==03)||(gucModbusRxBuf[1]==05)||(gucModbusRxBuf[1]==06)||(gucModbusRxBuf[1]==15)||(gucModbusRxBuf[1]==16))//��������ȷ
            {
                startRegAddr=(((uint16_t)gucModbusRxBuf[2])<<8)|gucModbusRxBuf[3];//��ȡ�Ĵ�����ʼ��ַ
                if(startRegAddr<ModbusRegNum)//�Ĵ�����ַ�ڷ�Χ��
                {
                    calCRC=CrcCompute(gucModbusRxBuf,gusModbusRecvLen-2);//�������������ݵ�CRC
                    recCRC=gucModbusRxBuf[gusModbusRecvLen-1]|(((uint16_t)gucModbusRxBuf[gusModbusRecvLen-2])<<8);//���յ���CRC(���ֽ���ǰ�����ֽ��ں�)
                    if(calCRC==recCRC)//CRCУ����ȷ
                    {
                        //LED1=0;
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        switch(gucModbusRxBuf[1])//���ݲ�ͬ�Ĺ�������д���
                        {
                        case 2://�����뿪����
                        {
                            Modbus_02_Solve();
                            break;
                        }

                        case 1://�����������
                        {
                            Modbus_01_Solve();
                            break;
                        }

                        case 5://д�������������
                        {
                            Modbus_05_Solve();
                            break;
                        }

                        case 15://д������������
                        {
                            Modbus_15_Solve();
                            break;
                        }

                        case 03: //������Ĵ���
                        {
                            Modbus_03_Solve();
                            break;
                        }

                        case 06: //д�����Ĵ���
                        {
                            Modbus_06_Solve();
                            break;
                        }

                        case 16: //д����Ĵ���
                        {
                            Modbus_16_Solve();
                            break;
                        }

                        }
                        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    }
                    else//CRCУ�����
                    {

                        gucModbusTxBuf[0]=gucModbusRxBuf[0];
                        gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
                        gucModbusTxBuf[2]=0x04; //�쳣��
                        modbusSendBuf(gucModbusTxBuf,3);
                    }
                }
                else//�Ĵ�����ַ������Χ
                {
                    gucModbusTxBuf[0]=gucModbusRxBuf[0];
                    gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
                    gucModbusTxBuf[2]=0x02; //�쳣��
                    modbusSendBuf(gucModbusTxBuf,3);
                }
            }
            else//���������
            {
                gucModbusTxBuf[0]=gucModbusRxBuf[0];
                gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
                gucModbusTxBuf[2]=0x01; //�쳣��
                modbusSendBuf(gucModbusTxBuf,3);
            }
        }

        gbModbusRecved=false;//��λ֡������־
        gusModbusRecvLen=0;//���ռ���������
//        RS485_TX_EN=0;//��������ģʽ

        if((*Modbus_HoldReg[ModbusWriteOffset]) == 0x31)
        {
            FRAMWriteBuffer(PID_SAVE_OFFSET,(uint8_t *)(&gmMouse.pPids[SPEEDPID].fKp),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+4,(uint8_t *)(&gmMouse.pPids[SPEEDPID].fKi),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+8,(uint8_t *)(&gmMouse.pPids[SPEEDPID].fKd),4);

            FRAMWriteBuffer(PID_SAVE_OFFSET+12,(uint8_t *)(&gmMouse.pPids[ROTATEPID].fKp),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+16,(uint8_t *)(&gmMouse.pPids[ROTATEPID].fKi),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+20,(uint8_t *)(&gmMouse.pPids[ROTATEPID].fKd),4);

            FRAMWriteBuffer(PID_SAVE_OFFSET+24,(uint8_t *)(&gmMouse.pPids[STRAIGHTPID].fKp),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+28,(uint8_t *)(&gmMouse.pPids[STRAIGHTPID].fKi),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+32,(uint8_t *)(&gmMouse.pPids[STRAIGHTPID].fKd),4);

            FRAMWriteBuffer(PID_SAVE_OFFSET+36,(uint8_t *)(&gmMouse.pPids[ANGLEPID].fKp),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+40,(uint8_t *)(&gmMouse.pPids[ANGLEPID].fKi),4);
            FRAMWriteBuffer(PID_SAVE_OFFSET+44,(uint8_t *)(&gmMouse.pPids[ANGLEPID].fKd),4);

            FRAMWriteBuffer(PID_SAVE_OFFSET+48,(uint8_t *)(&guscMAXSPEED),2);
            FRAMWriteBuffer(PID_SAVE_OFFSET+50,(uint8_t *)(&guscSEARCHSPEED),2);
            FRAMWriteBuffer(PID_SAVE_OFFSET+52,(uint8_t *)(&guscMINSPEED),2);
            (*Modbus_HoldReg[ModbusWriteOffset]) = 0;
        }
        if((*Modbus_HoldReg[ModbusWriteOffset]) == 0x41)
        {
            FRAMWriteBuffer(IR_SAVE_OFFSET,(uint8_t *)(IR_TABLE),128);
            (*Modbus_HoldReg[ModbusWriteOffset]) = 0;
        }
    }
}

//Modbus������02�������/////////////////////////////////////////////////////��������֤OK -----����������PE4 PE3 PE2 PA0 ��ʼ�������ſ���OK    KEY_Init();
//�����뿪����
static void Modbus_02_Solve(void)
{
    uint16_t ByteNum;
    uint16_t i;
    RegNum= (((uint16_t)gucModbusRxBuf[4])<<8)|gucModbusRxBuf[5];//��ȡ�Ĵ�������
    if((startRegAddr+RegNum)<100)//�Ĵ�����ַ+�����ڷ�Χ��
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1];
        ByteNum=RegNum/8;//�ֽ���
        if(RegNum%8) ByteNum+=1;//���λ���������������ֽ���+1
        gucModbusTxBuf[2]=ByteNum;//����Ҫ��ȡ���ֽ���
        for(i=0; i<RegNum; i++)
        {
            if(i%8==0) gucModbusTxBuf[3+i/8]=0x00;
            gucModbusTxBuf[3+i/8]>>=1;//��λ�ȷ���
            gucModbusTxBuf[3+i/8]|=((*Modbus_InputIO[startRegAddr+i])<<7)&0x80;
            if(i==RegNum-1)//���͵����һ��λ��
            {
                if(RegNum%8) gucModbusTxBuf[3+i/8]>>=8-(RegNum%8);//������һ���ֽڻ�����������ʣ��MSB���0
            }
        }
        calCRC=CrcCompute(gucModbusTxBuf,ByteNum+3);
        gucModbusTxBuf[ByteNum+3]=(calCRC>>8)&0xFF;
        gucModbusTxBuf[ByteNum+4]=(calCRC)&0xFF;
        modbusSendBuf(gucModbusTxBuf,ByteNum+5);
    }
    else//�Ĵ�����ַ+����������Χ
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
        gucModbusTxBuf[2]=0x02; //�쳣��
        modbusSendBuf(gucModbusTxBuf,3);
    }
}

//Modbus������01������� ///////////////////////////////////////////////////////////��������֤OK
//�����������
static void Modbus_01_Solve(void)
{
    uint16_t ByteNum;
    uint16_t i;
    RegNum= (((uint16_t)gucModbusRxBuf[4])<<8)|gucModbusRxBuf[5];//��ȡ�Ĵ�������
    if((startRegAddr+RegNum)<100)//�Ĵ�����ַ+�����ڷ�Χ��
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1];
        ByteNum=RegNum/8;//�ֽ���
        if(RegNum%8) ByteNum+=1;//���λ���������������ֽ���+1
        gucModbusTxBuf[2]=ByteNum;//����Ҫ��ȡ���ֽ���
        for(i=0; i<RegNum; i++)
        {
            if(i%8==0) gucModbusTxBuf[3+i/8]=0x00;
            gucModbusTxBuf[3+i/8]>>=1;//��λ�ȷ���
            gucModbusTxBuf[3+i/8]|=((*Modbus_OutputIO[startRegAddr+i])<<7)&0x80;
            if(i==RegNum-1)//���͵����һ��λ��
            {
                if(RegNum%8) gucModbusTxBuf[3+i/8]>>=8-(RegNum%8);//������һ���ֽڻ�����������ʣ��MSB���0
            }
        }
        calCRC=CrcCompute(gucModbusTxBuf,ByteNum+3);
        gucModbusTxBuf[ByteNum+3]=(calCRC>>8)&0xFF;
        gucModbusTxBuf[ByteNum+4]=(calCRC)&0xFF;
        modbusSendBuf(gucModbusTxBuf,ByteNum+5);
    }
    else//�Ĵ�����ַ+����������Χ
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
        gucModbusTxBuf[2]=0x02; //�쳣��
        modbusSendBuf(gucModbusTxBuf,3);
    }
}

//Modbus������05�������   ///////////////////////////////////////////////////////��������֤OK
//д�������������
static void Modbus_05_Solve(void)
{
    if(startRegAddr<100)//�Ĵ�����ַ�ڷ�Χ��
    {
        if((gucModbusRxBuf[4]==0xFF)||(gucModbusRxBuf[5]==0xFF)) *Modbus_OutputIO[startRegAddr]=0x01;
        else *Modbus_OutputIO[startRegAddr]=0x00;

        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1];
        gucModbusTxBuf[2]=gucModbusRxBuf[2];
        gucModbusTxBuf[3]=gucModbusRxBuf[3];
        gucModbusTxBuf[4]=gucModbusRxBuf[4];
        gucModbusTxBuf[5]=gucModbusRxBuf[5];

        calCRC=CrcCompute(gucModbusTxBuf,6);
        gucModbusTxBuf[6]=(calCRC>>8)&0xFF;
        gucModbusTxBuf[7]=(calCRC)&0xFF;
        modbusSendBuf(gucModbusTxBuf,8);
    }
    else//�Ĵ�����ַ������Χ
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
        gucModbusTxBuf[2]=0x02; //�쳣��
        modbusSendBuf(gucModbusTxBuf,3);
    }
}

//Modbus������15�������   //////////////////////////////////////////////////////��������֤OK
//д������������
static void Modbus_15_Solve(void)
{
    uint16_t i;
    RegNum=(((uint16_t)gucModbusRxBuf[4])<<8)|gucModbusRxBuf[5];//��ȡ�Ĵ�������
    if((startRegAddr+RegNum)<100)//�Ĵ�����ַ+�����ڷ�Χ��
    {
        for(i=0; i<RegNum; i++)
        {
            if(gucModbusRxBuf[7+i/8]&0x01) *Modbus_OutputIO[startRegAddr+i]=0x01;
            else *Modbus_OutputIO[startRegAddr+i]=0x00;
            gucModbusRxBuf[7+i/8]>>=1;//�ӵ�λ��ʼ
        }

        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1];
        gucModbusTxBuf[2]=gucModbusRxBuf[2];
        gucModbusTxBuf[3]=gucModbusRxBuf[3];
        gucModbusTxBuf[4]=gucModbusRxBuf[4];
        gucModbusTxBuf[5]=gucModbusRxBuf[5];
        calCRC=CrcCompute(gucModbusTxBuf,6);
        gucModbusTxBuf[6]=(calCRC>>8)&0xFF;
        gucModbusTxBuf[7]=(calCRC)&0xFF;
        modbusSendBuf(gucModbusTxBuf,8);
    }
    else//�Ĵ�����ַ+����������Χ
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
        gucModbusTxBuf[2]=0x02; //�쳣��
        modbusSendBuf(gucModbusTxBuf,3);
    }
}

//Modbus������03�������///////////////////////////////////////////////////////////////////////////////////////����֤����OK
//�����ּĴ���
static void Modbus_03_Solve(void)
{
    uint8_t i;
    RegNum= (((uint16_t)gucModbusRxBuf[4])<<8)|gucModbusRxBuf[5];//��ȡ�Ĵ�������
    if((startRegAddr+RegNum)<ModbusRegNum)//�Ĵ�����ַ+�����ڷ�Χ��
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1];
        gucModbusTxBuf[2]=RegNum*2;
        for(i=0; i<RegNum; i++)
        {
            gucModbusTxBuf[4+i*2]=(*Modbus_HoldReg[startRegAddr+i]>>8)&0xFF;//           /////////�ȷ��͸��ֽ�--�ڷ��͵��ֽ�
            gucModbusTxBuf[3+i*2]=(*Modbus_HoldReg[startRegAddr+i])&0xFF; //
        }
        calCRC=CrcCompute(gucModbusTxBuf,RegNum*2+3);
        gucModbusTxBuf[RegNum*2+3]=(calCRC>>8)&0xFF;         //CRC�ߵ�λ������  // �ȸߺ��
        gucModbusTxBuf[RegNum*2+4]=(calCRC)&0xFF;
        modbusSendBuf(gucModbusTxBuf,RegNum*2+5);
    }
    else//�Ĵ�����ַ+����������Χ
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
        gucModbusTxBuf[2]=0x02; //�쳣��
        modbusSendBuf(gucModbusTxBuf,3);
    }
}


//Modbus������06�������   //////////////////////////////////////////////////////////////////////////////////����֤����OK
//д�������ּĴ���
static void Modbus_06_Solve(void)
{
    *Modbus_HoldReg[startRegAddr]=gucModbusRxBuf[4]<<8;//���ֽ���ǰ                    ////////�޸�Ϊ���ֽ���ǰ�����ֽ��ں�
    *Modbus_HoldReg[startRegAddr]|=((uint16_t)gucModbusRxBuf[5]);//���ֽ��ں�

    gucModbusTxBuf[0]=gucModbusRxBuf[0];
    gucModbusTxBuf[1]=gucModbusRxBuf[1];
    gucModbusTxBuf[2]=gucModbusRxBuf[2];
    gucModbusTxBuf[3]=gucModbusRxBuf[3];
    gucModbusTxBuf[4]=gucModbusRxBuf[4];
    gucModbusTxBuf[5]=gucModbusRxBuf[5];

    calCRC=CrcCompute(gucModbusTxBuf,6);
    gucModbusTxBuf[6]=(calCRC>>8)&0xFF;
    gucModbusTxBuf[7]=(calCRC)&0xFF;
    modbusSendBuf(gucModbusTxBuf,8);


}

//Modbus������16������� /////////////////////////////////////////////////////////////////////////////////////////////////����֤����OK
//д������ּĴ���
static void Modbus_16_Solve(void)
{
    uint8_t i;
    RegNum= (((uint16_t)gucModbusRxBuf[4])<<8)|((gucModbusRxBuf[5]));//��ȡ�Ĵ�������
    if((startRegAddr+RegNum)<ModbusRegNum)//�Ĵ�����ַ+�����ڷ�Χ��
    {
        for(i=0; i<RegNum; i++)
        {
            *Modbus_HoldReg[startRegAddr+i]=gucModbusRxBuf[7+i*2]; //���ֽ���ǰ                 /////// ���ֽ���ǰ�����ֽ��ں�����
            *Modbus_HoldReg[startRegAddr+i]|=((uint16_t)gucModbusRxBuf[8+i*2])<<8; //���ֽ��ں�
        }

        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1];
        gucModbusTxBuf[2]=gucModbusRxBuf[2];
        gucModbusTxBuf[3]=gucModbusRxBuf[3];
        gucModbusTxBuf[4]=gucModbusRxBuf[4];
        gucModbusTxBuf[5]=gucModbusRxBuf[5];

        calCRC=CrcCompute(gucModbusTxBuf,6);
        gucModbusTxBuf[6]=(calCRC>>8)&0xFF;
        gucModbusTxBuf[7]=(calCRC)&0xFF;
        modbusSendBuf(gucModbusTxBuf,8);
    }
    else//�Ĵ�����ַ+����������Χ
    {
        gucModbusTxBuf[0]=gucModbusRxBuf[0];
        gucModbusTxBuf[1]=gucModbusRxBuf[1]|0x80;
        gucModbusTxBuf[2]=0x02; //�쳣��
        modbusSendBuf(gucModbusTxBuf,3);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(&huart1,gucModbusRxBuf,ModbusBufSize);
}


