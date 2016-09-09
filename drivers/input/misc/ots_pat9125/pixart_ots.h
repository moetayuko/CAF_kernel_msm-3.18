#ifndef _PIXART_OTS_H_
#define _PIXART_OTS_H_

#include  "pixart_platform.h"

/* export funtions */
void OTS_Reset_Variables(void);
uint8_t OTS_Detect_Rotation(int16_t dx16, int16_t dy16);
uint8_t OTS_Detect_Pressing(int16_t dx16, int16_t dy16);
bool OTS_Sensor_Init(void);
void OTS_Sensor_ReadMotion(int16_t *dx, int16_t *dy);

/* Global variables for Press Detection */
#define OTS_ROT_NO_CHANGE	0x00
#define OTS_ROT_UP		0x01
#define OTS_ROT_DOWN		0x02

#define OTS_BTN_NO_CHANGE	0x00
#define OTS_BTN_RELEASE		0x01
#define OTS_BTN_PRESS		0x02

#endif
