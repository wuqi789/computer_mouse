#include "main.h"
#include "spi.h"
#include "fmem.h"

#define FRAM_TIMEOUT   100                                      //SPI��ʱ���

static uint8_t WRITEFRAME[1024];     //��FRAMд���ݵ�����������
static uint8_t READFRAME[3];      //��FRAM�����ݵ�����������
static uint8_t WRITE_SR_FRAME[2];  //��FRAMд״̬�Ĵ���������������

inline static void FRAMCS()    //CS�ź�����
{
    HAL_GPIO_WritePin(FM_CS_GPIO_Port,FM_CS_Pin,GPIO_PIN_RESET);
}

inline static void FRAMnCS() //CS�ź�����
{
    HAL_GPIO_WritePin(FM_CS_GPIO_Port,FM_CS_Pin,GPIO_PIN_SET);
}

static void FRAMWriteEN(void)
{
    uint8_t op_code = FM25CL64_WREN;
    FRAMCS();                                                                                        //��CS�ź�����
    HAL_SPI_Transmit(&hspi2,&op_code,1,FRAM_TIMEOUT);            //  ͨ��SPI2,�����緢��ʹ��д�����Ĳ�����0000 0110b��
    FRAMnCS();                                                                                      //��CS�ź�����
}

static void FRAMWriteDI(void)
{
    uint8_t op_code = FM25CL64_WRDI;
    FRAMCS();
    HAL_SPI_Transmit(&hspi2,&op_code,1,FRAM_TIMEOUT);           // ԭ���ʹ��д����һ����ֻ�Ƿ��͵Ĳ������� ��ֹд�����룺0000 0100b
    FRAMnCS();
}

static uint8_t FRAMReadSR(void)
{
    uint8_t SRData;
    uint8_t op_code = FM25CL64_RDSR;

    FRAMCS();
    HAL_SPI_Transmit(&hspi2,&op_code,1,FRAM_TIMEOUT);          // ���Ͷ�״̬�Ĵ��������� 00000101b
    HAL_SPI_Receive(&hspi2,&SRData,1,FRAM_TIMEOUT) ;           //�����յ������ݷ���SRData
    FRAMnCS();                       // Disable FRAM CS

    return(SRData);
}

void FRAMWriteSR(uint8_t Data)
{
    FRAMWriteEN();                   //ʹ��д����  һ��Ҫ�У�����
    FRAMCS();                        // Enable FRAM CS
    WRITE_SR_FRAME[0] = FM25CL64_WRSR;      //д״̬�Ĵ��������������ȷ�д״̬�Ĵ����Ĳ����룬�����ŷ���Ҫд�����ݡ�
    WRITE_SR_FRAME[1] = Data;                             //���ԣ�����һ�����飬��Ų���������ݡ� ��SPI��������������������
    HAL_SPI_Transmit(&hspi2,&WRITE_SR_FRAME[0],2,FRAM_TIMEOUT);    // &WRITE_SR_FRAME[0]��ʾ��һ��������ʼ��ַ������ġ�2����ʾҪ��������8λ����
    FRAMnCS();                       // Disable FRAM CS
    FRAMWriteDI();                   // Disable Write operation   //д���������󣬼ǵõ��ý�ֹд����
}

void FRAMInit(void)
{
    FRAMWriteSR(0x80);               // ��ʵ������״̬�Ĵ���д0x80�����������Ϊʲô��������������ֲ��֪���ˡ�
}

uint8_t FRAMRead(uint16_t Addr)   //Addr��Ҫ��ȡ��������FRAM�д�ŵĵ�ַ�� FM25CL64�ĵ�ַ��0000��1FFF��
{
    uint8_t Data;                                   //����һ���޷���8λ����׼����Ŷ�ȡ��������
    uint8_t AddrH,AddrL;                      //��16λ�ĵ�ַ��Ϊ����8λ��
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMCS();                           // Enable FRAM CS
    READFRAME[0] = FM25CL64_READ;    //��FRAM���ݵ�������Ϊ����FRAM���Ͷ����ݲ����룬������Ϊ���ݵ�ַ��8λ��������Ϊ���ݵ�ַ��8λ��
    READFRAME[1] = AddrH;                        //����һ�������Ŷ����ݲ����룬��ַ�߰�λ����ַ�Ͱ�λ
    READFRAME[2] = AddrL;

    HAL_SPI_Transmit(&hspi2,&READFRAME[0],3,FRAM_TIMEOUT);    // ���÷��ͺ����������������������������

    HAL_SPI_Receive(&hspi2,&Data,1,FRAM_TIMEOUT);        //  �����յ������ݲ����󣬻᷵�ص�ַ��Ӧ�����ݡ���SPI�Ľ��պ����������ݣ������Data�

    FRAMnCS();                          // Disable FRAM CS

    return(Data);
}

void FRAMReadBuffer(uint16_t Addr,uint8_t *Data,uint8_t len)   //Addr��Ҫ��ȡ��������FRAM�д�ŵĵ�ַ�� FM25CL64�ĵ�ַ��0000��1FFF��
{
    uint8_t AddrH,AddrL;                      //��16λ�ĵ�ַ��Ϊ����8λ��
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMCS();                           // Enable FRAM CS
    READFRAME[0] = FM25CL64_READ;    //��FRAM���ݵ�������Ϊ����FRAM���Ͷ����ݲ����룬������Ϊ���ݵ�ַ��8λ��������Ϊ���ݵ�ַ��8λ��
    READFRAME[1] = AddrH;                        //����һ�������Ŷ����ݲ����룬��ַ�߰�λ����ַ�Ͱ�λ
    READFRAME[2] = AddrL;

    HAL_SPI_Transmit(&hspi2,&READFRAME[0],3,FRAM_TIMEOUT);    // ���÷��ͺ����������������������������

    HAL_SPI_Receive(&hspi2,Data,len,FRAM_TIMEOUT);        //  �����յ������ݲ����󣬻᷵�ص�ַ��Ӧ�����ݡ���SPI�Ľ��պ����������ݣ������Data�

    FRAMnCS();                          // Disable FRAM CS
}

void FRAMWrite(uint16_t Addr, uint8_t Data)    //��Dataд��FRAM�ڲ�Addr�����ַ
{
    uint8_t AddrH,AddrL;                                       //��16λ��ַ��Ϊ����8λ��
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMWriteEN();                   // һ��Ҫʹ��д����

    FRAMCS();                        // Enable FRAM CS
    WRITEFRAME[0] = FM25CL64_WRITE;            //FM25CL64��д�����������ǣ��ȷ������룬�ٷ���ַ�߰�λ���ٷ���ַ�Ͱ�λ���ٷ����ݡ�
    WRITEFRAME[1] = AddrH;                                 //���������δ�Ų����룬��ַ�߰�λ����ַ�Ͱ�λ������
    WRITEFRAME[2] = AddrL;
    WRITEFRAME[3] = Data;
    HAL_SPI_Transmit(&hspi2,&WRITEFRAME[0],4,FRAM_TIMEOUT);           // ����SPI���ͺ���������������ĸ�������������
    FRAMnCS();                       // Disable FRAM CS

    FRAMWriteDI();                   // ��ֹд����
}

void FRAMWriteBuffer(uint16_t Addr, uint8_t *Data,uint8_t len)    //��Dataд��FRAM�ڲ�Addr�����ַ
{
    uint8_t AddrH,AddrL,i;                                       //��16λ��ַ��Ϊ����8λ��
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMWriteEN();                   // һ��Ҫʹ��д����

    FRAMCS();                        // Enable FRAM CS
    WRITEFRAME[0] = FM25CL64_WRITE;            //FM25CL64��д�����������ǣ��ȷ������룬�ٷ���ַ�߰�λ���ٷ���ַ�Ͱ�λ���ٷ����ݡ�
    WRITEFRAME[1] = AddrH;                                 //���������δ�Ų����룬��ַ�߰�λ����ַ�Ͱ�λ������
    WRITEFRAME[2] = AddrL;
    for(i=0; i<len; i++)
    {
        WRITEFRAME[3+i] = Data[i];
    }
    HAL_SPI_Transmit(&hspi2,&WRITEFRAME[0],3+len,FRAM_TIMEOUT);           // ����SPI���ͺ���������������ĸ�������������
    FRAMnCS();                       // Disable FRAM CS

    FRAMWriteDI();                   // ��ֹд����
}
