/*
currectCrc: previous crc value, set 0 if it's first section
src: source stream data
lengthInBytes: length
*/
#include <crc.h>

void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
	uint32_t crc = *currectCrc;
	uint32_t j;
	for (j = 0; j < lengthInBytes; ++j)
	{
		uint32_t i;
		uint32_t byte = src[j];
		crc ^= byte << 8;
		for (i = 0; i < 8; ++i)
		{
			uint32_t temp = crc << 1;
			if (crc & 0x8000)
			{
				temp ^= 0x1021;
			}
			crc = temp;
		}
	}
	*currectCrc = crc;
}