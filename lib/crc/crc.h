
#ifndef CRC_H
#define CRC_H

#include <Arduino.h>

extern void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);

#endif