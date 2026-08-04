/**
 * @file modbus_crc.h
 * @brief CRC-16 Modbus RTU.
 */

#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>

/** Calcula CRC-16 Modbus, inicial 0xFFFF e polinômio refletido 0xA001. */
uint16_t modbus_crc_calculate(const uint8_t *data, uint16_t length);

#endif /* MODBUS_CRC_H */
