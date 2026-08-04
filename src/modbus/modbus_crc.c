/**
 * @file modbus_crc.c
 * @brief CRC-16 Modbus RTU.
 */

#include "modbus_crc.h"

#include <stddef.h>

#define MODBUS_CRC_INITIAL_VALUE   ((uint16_t)0xFFFFU)
#define MODBUS_CRC_POLYNOMIAL      ((uint16_t)0xA001U)
#define MODBUS_CRC_BITS_PER_BYTE   ((uint8_t)8U)

uint16_t modbus_crc_calculate(const uint8_t *data, uint16_t length)
{
    uint16_t crc = MODBUS_CRC_INITIAL_VALUE;
    uint16_t index;

    if ((data == NULL) && (length > 0U))
    {
        return 0U;
    }

    for (index = 0U; index < length; index++)
    {
        uint8_t bit;

        crc ^= (uint16_t)data[index];
        for (bit = 0U; bit < MODBUS_CRC_BITS_PER_BYTE; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ MODBUS_CRC_POLYNOMIAL);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}
