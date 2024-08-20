// rcc.c

#include "rcc.h"

uint16_t ahb_prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};

uint8_t apb_prescaler[4] = {2, 4, 8, 16};

uint32_t rcc_get_apb1_freq()
{
	uint32_t sysclk = 0;
	uint16_t ahb = 0;
	uint8_t  apb = 0;
	uint8_t  tmp = 0;

	switch ((RCC->cfgr >> 2) & 0x3)
	{
		case 0:
			sysclk = 16000000;
		break;

		case 1:
			sysclk = 8000000;
		break;

		default:
		break;
	}

	tmp = (RCC->cfgr >> 4) & 0xF;

	if (tmp < 8)
	{
		ahb = 1;
	}
	else
	{
		ahb = ahb_prescaler[tmp - 8];
	}

	tmp = (RCC->cfgr >> 10) & 0x7;

	if (tmp < 4)
	{
		apb = 1;
	}
	else
	{
		apb = apb_prescaler[tmp - 4];
	}

	return ((sysclk / ahb) / apb);
}

uint32_t rcc_get_apb2_freq()
{
	uint32_t sysclk = 0;
	uint16_t ahb = 0;
	uint8_t  apb = 0;
	uint8_t  tmp = 0;

	switch ((RCC->cfgr >> 2) & 0x3)
	{
		case 0:
			sysclk = 16000000;
		break;

		case 1:
			sysclk = 8000000;
		break;

		default:
		break;
	}

	tmp = (RCC->cfgr >> 4) & 0xF;

	if (tmp < 8)
	{
		ahb = 1;
	}
	else
	{
		ahb = ahb_prescaler[tmp - 8];
	}

	tmp = (RCC->cfgr >> 13) & 0x7;

	if (tmp < 4)
	{
		apb = 1;
	}
	else
	{
		apb = apb_prescaler[tmp - 4];
	}

	return ((sysclk / ahb) / apb);
}
