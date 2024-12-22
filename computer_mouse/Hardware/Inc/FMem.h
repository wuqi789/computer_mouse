#ifndef __FMEM_H__
#define __FMEM_H__

#define        FM25CL64_WREN                0x06                //使能
#define        FM25CL64_WRDI                0x04                //失能
#define        FM25CL64_RDSR                0x05                //读状态
#define        FM25CL64_WRSR                0x01                //写状态
#define        FM25CL64_READ                0x03                //读数据
#define        FM25CL64_WRITE            		0x02                //写数据

#define				 PID_SAVE_OFFSET 100
#define				 IR_SAVE_OFFSET 300
void FRAMInit(void);                                                    //初始化FRAM状态寄存器函数声明
void FRAMWrite(uint16_t Addr, uint8_t Data);            //向FRAM写数据函数声明
uint8_t FRAMRead(uint16_t Addr);                             //从FRAM读数据函数声明
uint8_t MPUReadReg(uint8_t REG_Address);
void FRAMReadBuffer(uint16_t Addr,uint8_t *Data,uint8_t len);
void FRAMWriteBuffer(uint16_t Addr, uint8_t *Data,uint8_t len);

#endif
