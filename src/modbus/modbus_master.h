/**
 * @file modbus_master.h
 * @brief Construção e validação do subconjunto Modbus RTU mestre.
 */

#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "comm_config.h"

#include <stdbool.h>
#include <stdint.h>

#define MODBUS_READ_MAX_REGISTERS          ((uint16_t)125U)
#define MODBUS_WRITE_MAX_REGISTERS         ((uint16_t)123U)

typedef enum
{
    MODBUS_FUNCTION_READ_HOLDING = 0x03U,
    MODBUS_FUNCTION_WRITE_SINGLE = 0x06U,
    MODBUS_FUNCTION_WRITE_MULTIPLE = 0x10U
} modbus_function_t;

typedef enum
{
    MODBUS_PARSE_OK = 0,
    MODBUS_PARSE_INVALID_ARGUMENT,
    MODBUS_PARSE_BUFFER_TOO_SMALL,
    MODBUS_PARSE_INVALID_LENGTH,
    MODBUS_PARSE_CRC_ERROR,
    MODBUS_PARSE_WRONG_ADDRESS,
    MODBUS_PARSE_WRONG_FUNCTION,
    MODBUS_PARSE_INVALID_BYTE_COUNT,
    MODBUS_PARSE_INVALID_ECHO,
    MODBUS_PARSE_EXCEPTION
} modbus_parse_status_t;

typedef struct
{
    uint8_t slave_address;
    modbus_function_t function;
    uint16_t start_address;
    uint16_t quantity;
    uint16_t values[MODBUS_WRITE_MAX_REGISTERS];
} modbus_request_t;

typedef struct
{
    uint16_t values[MODBUS_READ_MAX_REGISTERS];
    uint16_t quantity;
    uint8_t exception_code;
} modbus_response_t;

/** Inicializa uma solicitação 0x03 validada. */
bool modbus_master_prepare_read(modbus_request_t *request,
                                uint16_t start_address,
                                uint16_t quantity);

/** Inicializa uma solicitação 0x06 validada. */
bool modbus_master_prepare_write_single(modbus_request_t *request,
                                        uint16_t address,
                                        uint16_t value);

/** Inicializa uma solicitação 0x10 validada. */
bool modbus_master_prepare_write_multiple(modbus_request_t *request,
                                          uint16_t start_address,
                                          const uint16_t *values,
                                          uint16_t quantity);

/** Monta o ADU RTU e acrescenta CRC em ordem baixa/alta. */
modbus_parse_status_t modbus_master_build_request(
    const modbus_request_t *request,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_length);

/** Valida integralmente a resposta e converte somente dados uint16_t. */
modbus_parse_status_t modbus_master_parse_response(
    const modbus_request_t *request,
    const uint8_t *frame,
    uint16_t frame_length,
    modbus_response_t *response);

#endif /* MODBUS_MASTER_H */
