/**
 * @file modbus_master.c
 * @brief Construção e validação do subconjunto Modbus RTU mestre.
 */

#include "modbus_master.h"

#include "modbus_crc.h"

#include <stddef.h>
#include <string.h>

#define MODBUS_FIXED_REQUEST_LENGTH       ((uint16_t)8U)
#define MODBUS_EXCEPTION_RESPONSE_LENGTH  ((uint16_t)5U)
#define MODBUS_WRITE_RESPONSE_LENGTH      ((uint16_t)8U)
#define MODBUS_CRC_LENGTH                 ((uint16_t)2U)
#define MODBUS_EXCEPTION_MASK             ((uint8_t)0x80U)

static void write_u16_be(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value >> 8U);
    destination[1] = (uint8_t)(value & 0x00FFU);
}

static uint16_t read_u16_be(const uint8_t *source)
{
    return (uint16_t)(((uint16_t)source[0] << 8U) | (uint16_t)source[1]);
}

static void append_crc(uint8_t *frame, uint16_t payload_length)
{
    const uint16_t crc = modbus_crc_calculate(frame, payload_length);

    frame[payload_length] = (uint8_t)(crc & 0x00FFU);
    frame[payload_length + 1U] = (uint8_t)(crc >> 8U);
}

static bool request_range_is_valid(uint16_t start_address, uint16_t quantity)
{
    return (quantity > 0U) &&
           (((uint32_t)start_address + (uint32_t)quantity - 1U) <= UINT16_MAX);
}

bool modbus_master_prepare_read(modbus_request_t *request,
                                uint16_t start_address,
                                uint16_t quantity)
{
    if ((request == NULL) ||
        (quantity > MODBUS_READ_MAX_REGISTERS) ||
        !request_range_is_valid(start_address, quantity))
    {
        return false;
    }

    (void)memset(request, 0, sizeof(*request));
    request->slave_address = COMM_MODBUS_SLAVE_ADDRESS;
    request->function = MODBUS_FUNCTION_READ_HOLDING;
    request->start_address = start_address;
    request->quantity = quantity;
    return true;
}

bool modbus_master_prepare_write_single(modbus_request_t *request,
                                        uint16_t address,
                                        uint16_t value)
{
    if (request == NULL)
    {
        return false;
    }

    (void)memset(request, 0, sizeof(*request));
    request->slave_address = COMM_MODBUS_SLAVE_ADDRESS;
    request->function = MODBUS_FUNCTION_WRITE_SINGLE;
    request->start_address = address;
    request->quantity = 1U;
    request->values[0] = value;
    return true;
}

bool modbus_master_prepare_write_multiple(modbus_request_t *request,
                                          uint16_t start_address,
                                          const uint16_t *values,
                                          uint16_t quantity)
{
    if ((request == NULL) ||
        (values == NULL) ||
        (quantity > MODBUS_WRITE_MAX_REGISTERS) ||
        !request_range_is_valid(start_address, quantity))
    {
        return false;
    }

    (void)memset(request, 0, sizeof(*request));
    request->slave_address = COMM_MODBUS_SLAVE_ADDRESS;
    request->function = MODBUS_FUNCTION_WRITE_MULTIPLE;
    request->start_address = start_address;
    request->quantity = quantity;
    (void)memcpy(request->values, values, (size_t)quantity * sizeof(uint16_t));
    return true;
}

modbus_parse_status_t modbus_master_build_request(
    const modbus_request_t *request,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_length)
{
    uint16_t required_length;
    uint16_t index;

    if (frame_length != NULL)
    {
        *frame_length = 0U;
    }

    if ((request == NULL) || (frame == NULL) || (frame_length == NULL))
    {
        return MODBUS_PARSE_INVALID_ARGUMENT;
    }

    if (!request_range_is_valid(request->start_address, request->quantity))
    {
        return MODBUS_PARSE_INVALID_ARGUMENT;
    }

    if ((request->function == MODBUS_FUNCTION_READ_HOLDING) ||
        (request->function == MODBUS_FUNCTION_WRITE_SINGLE))
    {
        required_length = MODBUS_FIXED_REQUEST_LENGTH;
    }
    else if (request->function == MODBUS_FUNCTION_WRITE_MULTIPLE)
    {
        if (request->quantity > MODBUS_WRITE_MAX_REGISTERS)
        {
            return MODBUS_PARSE_INVALID_ARGUMENT;
        }
        required_length = (uint16_t)(9U + (2U * request->quantity));
    }
    else
    {
        return MODBUS_PARSE_INVALID_ARGUMENT;
    }

    if (required_length > frame_capacity)
    {
        return MODBUS_PARSE_BUFFER_TOO_SMALL;
    }

    frame[0] = request->slave_address;
    frame[1] = (uint8_t)request->function;
    write_u16_be(&frame[2], request->start_address);

    if (request->function == MODBUS_FUNCTION_WRITE_SINGLE)
    {
        write_u16_be(&frame[4], request->values[0]);
        append_crc(frame, 6U);
    }
    else if (request->function == MODBUS_FUNCTION_READ_HOLDING)
    {
        if (request->quantity > MODBUS_READ_MAX_REGISTERS)
        {
            return MODBUS_PARSE_INVALID_ARGUMENT;
        }
        write_u16_be(&frame[4], request->quantity);
        append_crc(frame, 6U);
    }
    else
    {
        write_u16_be(&frame[4], request->quantity);
        frame[6] = (uint8_t)(2U * request->quantity);
        for (index = 0U; index < request->quantity; index++)
        {
            write_u16_be(&frame[7U + (2U * index)], request->values[index]);
        }
        append_crc(frame, (uint16_t)(required_length - MODBUS_CRC_LENGTH));
    }

    *frame_length = required_length;
    return MODBUS_PARSE_OK;
}

modbus_parse_status_t modbus_master_parse_response(
    const modbus_request_t *request,
    const uint8_t *frame,
    uint16_t frame_length,
    modbus_response_t *response)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint16_t index;

    if ((request == NULL) || (frame == NULL) || (response == NULL))
    {
        return MODBUS_PARSE_INVALID_ARGUMENT;
    }

    (void)memset(response, 0, sizeof(*response));

    if (frame_length < MODBUS_EXCEPTION_RESPONSE_LENGTH)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    if (frame[0] != request->slave_address)
    {
        return MODBUS_PARSE_WRONG_ADDRESS;
    }

    received_crc = (uint16_t)((uint16_t)frame[frame_length - 2U] |
                              ((uint16_t)frame[frame_length - 1U] << 8U));
    calculated_crc = modbus_crc_calculate(frame,
                                          (uint16_t)(frame_length -
                                                     MODBUS_CRC_LENGTH));
    if (received_crc != calculated_crc)
    {
        return MODBUS_PARSE_CRC_ERROR;
    }

    if (frame[1] == ((uint8_t)request->function | MODBUS_EXCEPTION_MASK))
    {
        if (frame_length != MODBUS_EXCEPTION_RESPONSE_LENGTH)
        {
            return MODBUS_PARSE_INVALID_LENGTH;
        }
        response->exception_code = frame[2];
        return MODBUS_PARSE_EXCEPTION;
    }

    if (frame[1] != (uint8_t)request->function)
    {
        return MODBUS_PARSE_WRONG_FUNCTION;
    }

    if (request->function == MODBUS_FUNCTION_READ_HOLDING)
    {
        const uint16_t expected_byte_count = (uint16_t)(2U * request->quantity);
        const uint16_t expected_length =
            (uint16_t)(3U + expected_byte_count + MODBUS_CRC_LENGTH);

        if (frame[2] != (uint8_t)expected_byte_count)
        {
            return MODBUS_PARSE_INVALID_BYTE_COUNT;
        }
        if (frame_length != expected_length)
        {
            return MODBUS_PARSE_INVALID_LENGTH;
        }

        response->quantity = request->quantity;
        for (index = 0U; index < request->quantity; index++)
        {
            response->values[index] = read_u16_be(&frame[3U + (2U * index)]);
        }
        return MODBUS_PARSE_OK;
    }

    if (frame_length != MODBUS_WRITE_RESPONSE_LENGTH)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    if ((read_u16_be(&frame[2]) != request->start_address) ||
        (read_u16_be(&frame[4]) !=
         ((request->function == MODBUS_FUNCTION_WRITE_SINGLE)
              ? request->values[0]
              : request->quantity)))
    {
        return MODBUS_PARSE_INVALID_ECHO;
    }

    response->quantity = request->quantity;
    return MODBUS_PARSE_OK;
}
