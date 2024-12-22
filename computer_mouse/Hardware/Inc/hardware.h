#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "fmem.h"
#include "MPU6000.h"
#include "enc.h"
#include "main.h"

#define TRUE 1
#define FALSE 0

#define FRONTLEFT 0
#define SIDERIGHT 1
#define FRONTRIGHT 3
#define SIDELEFT 2
#define BATTERY 4

#define NEAR 0
#define MID 1
#define FAR 2

struct GPIO_PIN
{
	GPIO_TypeDef * port;
	uint16_t pin;
};

extern struct GPIO_PIN StartKey;
extern struct GPIO_PIN RunLed;
extern struct GPIO_PIN IREnable[4];

#define ADC_BUF_CHN (5)
extern __IO uint16_t usAdBuffer[6];

uint8_t startCheck (void);
void mouseInit(void);
	



#define READ_STATUS					0x8001			//8000
#define READ_ANGLE_VALUE		0x8021			//8020
#define READ_SPEED_VALUE		0x8031

//void UpdateDmp(void);
void DelayUs (uint32_t uiD);
uint8_t startCheck (void);

extern __IO int16_t gsIrLevel[4];
extern __IO uint16_t gusDistance[4];
extern __IO bool gbFrontIRTrigger;
extern uint16_t gusLedFlash;
extern uint16_t IR_TABLE[4][16];
#endif
