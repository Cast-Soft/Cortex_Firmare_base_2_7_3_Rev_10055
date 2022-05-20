#include "util.h"

#include "stm32f10x.h"

uint8_t crc8(const void *vptr, uint8_t len)
{
        const uint8_t *data = vptr;
        uint16_t crc = 0;
        uint8_t i, j;

        for (j = len; j; j--, data++) {
                crc ^= (*data << 8);
                for(i = 8; i; i--) {
                        if (crc & 0x8000)
                                crc ^= (0x1070 << 3);
                        crc <<= 1;
                }
        }

        return (uint8_t)(crc >> 8);
}
