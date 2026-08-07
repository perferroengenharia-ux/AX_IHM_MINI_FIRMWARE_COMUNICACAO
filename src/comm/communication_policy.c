#include "communication_policy.h"

#include "protocol/register_map.h"

bool communication_policy_parse_keeps_link_alive(
    modbus_parse_status_t parse_status)
{
    return (parse_status == MODBUS_PARSE_OK) ||
           (parse_status == MODBUS_PARSE_EXCEPTION);
}

comm_e08_clear_action_t communication_policy_e08_clear_action(
    bool clear_succeeded,
    bool illegal_value_exception,
    bool remote_status_available,
    uint16_t remote_status_word)
{
    if (clear_succeeded)
    {
        return COMM_E08_CLEAR_COMPLETE;
    }

    if (illegal_value_exception && remote_status_available)
    {
        if ((remote_status_word & REG_STATUS_E08_ACTIVE_MASK) == 0U)
        {
            return COMM_E08_CLEAR_COMPLETE;
        }
        if ((remote_status_word & REG_STATUS_PARAMETERS_SYNCED_MASK) == 0U)
        {
            return COMM_E08_CLEAR_RESYNCHRONIZE;
        }
    }

    return COMM_E08_CLEAR_WAIT_HEARTBEATS;
}
