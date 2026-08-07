/**
 * @file rs485_master.h
 * @brief Transporte físico RS485 sobre UART1 do ESP32-S3.
 */

#ifndef RS485_MASTER_H
#define RS485_MASTER_H

#include "esp_err.h"

#include <stdint.h>

typedef enum
{
    RS485_TRANSFER_OK = 0,
    RS485_TRANSFER_TIMEOUT,
    RS485_TRANSFER_UART_ERROR,
    RS485_TRANSFER_PARITY_ERROR,
    RS485_TRANSFER_FRAMING_ERROR,
    RS485_TRANSFER_OVERFLOW,
    RS485_TRANSFER_INVALID_ARGUMENT
} rs485_transfer_status_t;

/** Configura UART1 em 9600 8E1 e modo RS485 half-duplex automático. */
esp_err_t rs485_master_init(void);

/**
 * Envia uma solicitação e recebe um quadro delimitado por silêncio.
 * Esta função deve ser chamada exclusivamente pela tarefa Modbus.
 */
rs485_transfer_status_t rs485_master_transceive(
    const uint8_t *request,
    uint16_t request_length,
    uint8_t *response,
    uint16_t response_capacity,
    uint16_t *response_length,
    uint32_t timeout_ms);

/** Nome curto e estavel para diagnostico do transporte fisico. */
const char *rs485_transfer_status_to_string(rs485_transfer_status_t status);

#endif /* RS485_MASTER_H */
