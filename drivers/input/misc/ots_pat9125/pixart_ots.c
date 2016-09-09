
#include "pixart_ots.h"

/* Global Variables */
static int32_t x_sum;
static int32_t pre_dsCountX;
static int32_t ds_x_sum;
/* Local function */
static void OTS_WriteRead(uint8_t address, uint8_t wdata);

void OTS_Reset_Variables(void)
{
	/* reset variables */
	x_sum = 0;
	pre_dsCountX = 0;
	ds_x_sum = 0;
}

uint8_t OTS_Detect_Pressing(int16_t dx16, int16_t dy16)
{
	#define PRESS			1
	#define RELEASE			0

	#define DX_ROTATE_TH	2
	#define DY_VALID_TH		1
	#define ACCY_PRESS_TH	5
	#define DY_RELEASE_TH	(-1)

	uint8_t OutBtnState = OTS_BTN_NO_CHANGE;
	static int32_t AccY;
	static uint8_t State = RELEASE; /* 0:release, 1:press */

	if ((dx16 >= DX_ROTATE_TH) || (dx16 <= (-DX_ROTATE_TH))) {
		AccY = 0;
	}	else {
		if (State == PRESS) {
			if (dy16 <= DY_RELEASE_TH) {
				State = RELEASE;
				OutBtnState = OTS_BTN_RELEASE;
			}
		}	else {
			if (dy16 < DY_VALID_TH)	{
				AccY = 0;
			}	else {
				AccY += dy16;
				if (AccY >= ACCY_PRESS_TH) {
					AccY = 0;
					State = PRESS;
					OutBtnState = OTS_BTN_PRESS;
				}
			}
		}
	}
	return OutBtnState;

}

static uint8_t Detect_Rotation(int32_t dsCountX)
{
#define EVENT_NUM_PER_ROUND	10
#define EVENT_COUNT_TH		(EXPECTED_COUNT_PER_ROUND / EVENT_NUM_PER_ROUND)

	int32_t diff_count = 0;
	uint8_t OutRotState = OTS_ROT_NO_CHANGE;

	diff_count = dsCountX - pre_dsCountX;
	if (diff_count >= EVENT_COUNT_TH)	{
		pre_dsCountX = dsCountX;
		OutRotState = OTS_ROT_UP;
	}	else if (diff_count <= (-EVENT_COUNT_TH))	{
		pre_dsCountX = dsCountX;
		OutRotState = OTS_ROT_DOWN;
	}

	return OutRotState;
}

static int32_t OTS_Resolution_Downscale(int16_t delta_count)
{
	int32_t ret = 0;
	x_sum += delta_count;
	ret = (x_sum * EXPECTED_COUNT_PER_ROUND / REAL_AVG_COUNT_PER_ROUND);
	return ret;
}

uint8_t OTS_Detect_Rotation(int16_t dx16, int16_t dy16)
{
	ds_x_sum = OTS_Resolution_Downscale(dx16);
	return Detect_Rotation(ds_x_sum);
}
bool OTS_Sensor_Init(void)
{
	unsigned char sensor_pid = 0, read_id_ok = 0;

	/* Read sensor_pid in address 0x00 to check if the
	 serial link is valid, read value should be 0x31. */
	sensor_pid = ReadData(0x00);

	if (sensor_pid == 0x31) {
		read_id_ok = 1;

		/* PAT9125 sensor recommended settings: */
		/* switch to bank0, not allowed to perform OTS_RegWriteRead */
		WriteData(0x7F, 0x00);
		/* software reset (i.e. set bit7 to 1).
		It will reset to 0 automatically */
		/* so perform OTS_RegWriteRead is not allowed. */
		WriteData(0x06, 0x97);

		delay_ms(1);				/* delay 1ms */

		/* disable write protect */
		OTS_WriteRead(0x09, 0x5A);
		/* set X-axis resolution (depends on application) */
		OTS_WriteRead(0x0D, 0x65);
		/* set Y-axis resolution (depends on application) */
		OTS_WriteRead(0x0E, 0xFF);
		/* set 12-bit X/Y data format (depends on application) */
		OTS_WriteRead(0x19, 0x04);
		/* ONLY for VDD=VDDA=1.7~1.9V: for power saving */
		OTS_WriteRead(0x4B, 0x04);

		if (ReadData(0x5E) == 0x04) {
			OTS_WriteRead(0x5E, 0x08);
			if (ReadData(0x5D) == 0x10)
				OTS_WriteRead(0x5D, 0x19);
		}
		OTS_WriteRead(0x09, 0x00);/* enable write protect */
	}
	return read_id_ok;
}


void OTS_Sensor_ReadMotion(int16_t *dx, int16_t *dy)
{
	int16_t deltaX_l = 0, deltaY_l = 0, deltaXY_h = 0;
	int16_t deltaX_h = 0, deltaY_h = 0;

	/* check motion bit in bit7 */
	if (ReadData(0x02) & 0x80) {
		deltaX_l = ReadData(0x03);
		deltaY_l = ReadData(0x04);
		deltaXY_h = ReadData(0x12);

		deltaX_h = (deltaXY_h << 4) & 0xF00;
		if (deltaX_h & 0x800)
			deltaX_h |= 0xf000;

		deltaY_h = (deltaXY_h << 8) & 0xF00;
		if (deltaY_h & 0x800)
			deltaY_h |= 0xf000;
	}
	/* inverse the data (depends on sensor's orientation and application) */
	*dx = -(deltaX_h | deltaX_l);
	/* inverse the data (depends on sensor's orientation and application) */
	*dy = -(deltaY_h | deltaY_l);
}

static void OTS_WriteRead(uint8_t address, uint8_t wdata)
{
	uint8_t read_value;
	do {
		/* Write data to specified address */
		WriteData(address, wdata);
		/* Read back previous written data */
		read_value = ReadData(address);
		/* Check if the data is correctly written */
	} while (read_value != wdata);
	return;
}

