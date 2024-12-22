#include "main.h"
#include "spi.h"
#include "fmem.h"

#define FRAM_TIMEOUT   100                                      //SPI超时检测

static uint8_t WRITEFRAME[1024];     //向FRAM写数据的数据流数组
static uint8_t READFRAME[3];      //从FRAM读数据的数据流数组
static uint8_t WRITE_SR_FRAME[2];  //向FRAM写状态寄存器的数据流数组

inline static void FRAMCS()    //CS信号拉低
{
    HAL_GPIO_WritePin(FM_CS_GPIO_Port,FM_CS_Pin,GPIO_PIN_RESET);
}

inline static void FRAMnCS() //CS信号拉高
{
    HAL_GPIO_WritePin(FM_CS_GPIO_Port,FM_CS_Pin,GPIO_PIN_SET);
}

static void FRAMWriteEN(void)
{
    uint8_t op_code = FM25CL64_WREN;
    FRAMCS();                                                                                        //将CS信号拉低
    HAL_SPI_Transmit(&hspi2,&op_code,1,FRAM_TIMEOUT);            //  通过SPI2,向铁电发送使能写操作的操作码0000 0110b。
    FRAMnCS();                                                                                      //将CS信号拉高
}

static void FRAMWriteDI(void)
{
    uint8_t op_code = FM25CL64_WRDI;
    FRAMCS();
    HAL_SPI_Transmit(&hspi2,&op_code,1,FRAM_TIMEOUT);           // 原理和使能写操作一样，只是发送的操作码是 禁止写操作码：0000 0100b
    FRAMnCS();
}

static uint8_t FRAMReadSR(void)
{
    uint8_t SRData;
    uint8_t op_code = FM25CL64_RDSR;

    FRAMCS();
    HAL_SPI_Transmit(&hspi2,&op_code,1,FRAM_TIMEOUT);          // 发送读状态寄存器操作码 00000101b
    HAL_SPI_Receive(&hspi2,&SRData,1,FRAM_TIMEOUT) ;           //将接收到的数据放在SRData
    FRAMnCS();                       // Disable FRAM CS

    return(SRData);
}

void FRAMWriteSR(uint8_t Data)
{
    FRAMWriteEN();                   //使能写操作  一定要有！！！
    FRAMCS();                        // Enable FRAM CS
    WRITE_SR_FRAME[0] = FM25CL64_WRSR;      //写状态寄存器的数据流是先发写状态寄存器的操作码，紧跟着发送要写的数据。
    WRITE_SR_FRAME[1] = Data;                             //所以，建立一个数组，存放操作码和数据。 用SPI将这两个数据连续发送
    HAL_SPI_Transmit(&hspi2,&WRITE_SR_FRAME[0],2,FRAM_TIMEOUT);    // &WRITE_SR_FRAME[0]表示第一个数的起始地址，这里的“2”表示要发送两个8位数。
    FRAMnCS();                       // Disable FRAM CS
    FRAMWriteDI();                   // Disable Write operation   //写操作结束后，记得调用禁止写操作
}

void FRAMInit(void)
{
    FRAMWriteSR(0x80);               // 其实就是向状态寄存器写0x80这个数。至于为什么是这个，看技术手册就知道了。
}

uint8_t FRAMRead(uint16_t Addr)   //Addr是要读取的数据在FRAM中存放的地址。 FM25CL64的地址是0000到1FFF。
{
    uint8_t Data;                                   //声明一个无符号8位数，准备存放读取到的数据
    uint8_t AddrH,AddrL;                      //将16位的地址分为两个8位数
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMCS();                           // Enable FRAM CS
    READFRAME[0] = FM25CL64_READ;    //读FRAM数据的数据流为，向FRAM发送读数据操作码，紧接着为数据地址高8位，紧接着为数据地址低8位。
    READFRAME[1] = AddrH;                        //声明一个数组存放读数据操作码，地址高八位，地址低八位
    READFRAME[2] = AddrL;

    HAL_SPI_Transmit(&hspi2,&READFRAME[0],3,FRAM_TIMEOUT);    // 调用发送函数，将数组里的三个数连续发送

    HAL_SPI_Receive(&hspi2,&Data,1,FRAM_TIMEOUT);        //  铁电收到读数据操作后，会返回地址对应的数据。用SPI的接收函数接收数据，存放在Data里。

    FRAMnCS();                          // Disable FRAM CS

    return(Data);
}

void FRAMReadBuffer(uint16_t Addr,uint8_t *Data,uint8_t len)   //Addr是要读取的数据在FRAM中存放的地址。 FM25CL64的地址是0000到1FFF。
{
    uint8_t AddrH,AddrL;                      //将16位的地址分为两个8位数
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMCS();                           // Enable FRAM CS
    READFRAME[0] = FM25CL64_READ;    //读FRAM数据的数据流为，向FRAM发送读数据操作码，紧接着为数据地址高8位，紧接着为数据地址低8位。
    READFRAME[1] = AddrH;                        //声明一个数组存放读数据操作码，地址高八位，地址低八位
    READFRAME[2] = AddrL;

    HAL_SPI_Transmit(&hspi2,&READFRAME[0],3,FRAM_TIMEOUT);    // 调用发送函数，将数组里的三个数连续发送

    HAL_SPI_Receive(&hspi2,Data,len,FRAM_TIMEOUT);        //  铁电收到读数据操作后，会返回地址对应的数据。用SPI的接收函数接收数据，存放在Data里。

    FRAMnCS();                          // Disable FRAM CS
}

void FRAMWrite(uint16_t Addr, uint8_t Data)    //将Data写在FRAM内部Addr这个地址
{
    uint8_t AddrH,AddrL;                                       //将16位地址分为两个8位数
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMWriteEN();                   // 一定要使能写操作

    FRAMCS();                        // Enable FRAM CS
    WRITEFRAME[0] = FM25CL64_WRITE;            //FM25CL64的写操作数据流是：先发操作码，再发地址高八位，再发地址低八位，再发数据。
    WRITEFRAME[1] = AddrH;                                 //用数组依次存放操作码，地址高八位，地址低八位，数据
    WRITEFRAME[2] = AddrL;
    WRITEFRAME[3] = Data;
    HAL_SPI_Transmit(&hspi2,&WRITEFRAME[0],4,FRAM_TIMEOUT);           // 调用SPI发送函数，将数组里的四个数据连续发送
    FRAMnCS();                       // Disable FRAM CS

    FRAMWriteDI();                   // 禁止写操作
}

void FRAMWriteBuffer(uint16_t Addr, uint8_t *Data,uint8_t len)    //将Data写在FRAM内部Addr这个地址
{
    uint8_t AddrH,AddrL,i;                                       //将16位地址分为两个8位数
    AddrH = (Addr>>8);
    AddrL = (uint8_t)Addr;
    FRAMWriteEN();                   // 一定要使能写操作

    FRAMCS();                        // Enable FRAM CS
    WRITEFRAME[0] = FM25CL64_WRITE;            //FM25CL64的写操作数据流是：先发操作码，再发地址高八位，再发地址低八位，再发数据。
    WRITEFRAME[1] = AddrH;                                 //用数组依次存放操作码，地址高八位，地址低八位，数据
    WRITEFRAME[2] = AddrL;
    for(i=0; i<len; i++)
    {
        WRITEFRAME[3+i] = Data[i];
    }
    HAL_SPI_Transmit(&hspi2,&WRITEFRAME[0],3+len,FRAM_TIMEOUT);           // 调用SPI发送函数，将数组里的四个数据连续发送
    FRAMnCS();                       // Disable FRAM CS

    FRAMWriteDI();                   // 禁止写操作
}
