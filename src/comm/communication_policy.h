#ifndef COMMUNICATION_POLICY_H
#define COMMUNICATION_POLICY_H

#include "modbus_master.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    COMM_E08_CLEAR_COMPLETE = 0,
    COMM_E08_CLEAR_WAIT_HEARTBEATS,
    COMM_E08_CLEAR_RESYNCHRONIZE
} comm_e08_clear_action_t;

/* Uma excecao Modbus possui endereco, funcao e CRC validos: o enlace vive. */
bool communication_policy_parse_keeps_link_alive(
    modbus_parse_status_t parse_status);

/* Decide a recuperacao sem confundir E08 ausente com falha de handshake. */
comm_e08_clear_action_t communication_policy_e08_clear_action(
    bool clear_succeeded,
    bool illegal_value_exception,
    bool remote_status_available,
    uint16_t remote_status_word);

#endif /* COMMUNICATION_POLICY_H */
