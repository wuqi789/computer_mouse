#ifndef _MODBUS_H
#define _MODBUS_H
#include "main.h"
#include "tim.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

#define RS485_TX_EN PDout(7) 

void modbusInit(void);
void modbusProcess(void);


#endif
