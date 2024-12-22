#ifndef __FMEM_H__
#define __FMEM_H__

#define        FM25CL64_WREN                0x06                //ʹ��
#define        FM25CL64_WRDI                0x04                //ʧ��
#define        FM25CL64_RDSR                0x05                //��״̬
#define        FM25CL64_WRSR                0x01                //д״̬
#define        FM25CL64_READ                0x03                //������
#define        FM25CL64_WRITE            		0x02                //д����

#define				 PID_SAVE_OFFSET 100
#define				 IR_SAVE_OFFSET 300
void FRAMInit(void);                                                    //��ʼ��FRAM״̬�Ĵ�����������
void FRAMWrite(uint16_t Addr, uint8_t Data);            //��FRAMд���ݺ�������
uint8_t FRAMRead(uint16_t Addr);                             //��FRAM�����ݺ�������
uint8_t MPUReadReg(uint8_t REG_Address);
void FRAMReadBuffer(uint16_t Addr,uint8_t *Data,uint8_t len);
void FRAMWriteBuffer(uint16_t Addr, uint8_t *Data,uint8_t len);

#endif
