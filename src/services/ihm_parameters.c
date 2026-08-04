/**
 * @file ihm_parameters.c
 * @brief Regras puras dos parametros persistentes da IHM.
 */

#include "ihm_parameters.h"

#include "modbus_crc.h"
#include "protocol/register_map.h"

#include <stddef.h>
#include <string.h>

static const uint16_t s_factory_defaults[IHM_PARAM_COUNT] =
{
    [IHM_PARAM_P10] = 10U,
    [IHM_PARAM_P11] = 5U,
    [IHM_PARAM_P12] = 1U,
    [IHM_PARAM_P20] = 500U,
    [IHM_PARAM_P21] = 6000U,
    [IHM_PARAM_P30] = 1U,
    [IHM_PARAM_P31] = 3U,
    [IHM_PARAM_P32] = 0U,
    [IHM_PARAM_P33] = 0U,
    [IHM_PARAM_P35] = 0U,
    [IHM_PARAM_P41] = 60U,
    [IHM_PARAM_P42] = 10U,
    [IHM_PARAM_P43] = 600U,
    [IHM_PARAM_P44] = 0U,
    [IHM_PARAM_P45] = 180U,
    [IHM_PARAM_P51] = 0U,
    [IHM_PARAM_P81] = 1U,
    [IHM_PARAM_P82] = 1U,
    [IHM_PARAM_P85] = 1U,
    [IHM_PARAM_P86] = 0U,
    [IHM_PARAM_P91] = 47U
};

static const char *const s_codes[IHM_PARAM_COUNT] =
{
    "P10", "P11", "P12", "P20", "P21", "P30", "P31",
    "P32", "P33", "P35", "P41", "P42", "P43", "P44",
    "P45", "P51", "P81", "P82", "P85", "P86", "P91"
};

static const uint16_t s_addresses[IHM_PARAM_COUNT] =
{
    REG_P10_ACCELERATION_RAMP,
    REG_P11_DECELERATION_RAMP,
    REG_P12_REMEMBER_FREQUENCY,
    REG_P20_MINIMUM_FREQUENCY,
    REG_P21_MAXIMUM_FREQUENCY,
    REG_P30_PANEL_WET_DELAY,
    REG_P31_PANEL_DRY_DELAY,
    REG_P32_DRYING_SPEED,
    REG_P33_REVERSE_DURING_DRYING,
    REG_P35_TORQUE_COMPENSATION,
    REG_P41_NOMINAL_FREQUENCY,
    REG_P42_SWITCHING_FREQUENCY,
    REG_P43_OVERLOAD_CURRENT,
    REG_P44_AUTO_RESET_MODE,
    REG_P45_MINIMUM_VOLTAGE_CONTROL,
    REG_P51_ROTATION_DIRECTION,
    REG_P81_SWING_CONFIGURATION,
    REG_P82_PUMP_CONFIGURATION,
    REG_P85_LEVEL_SENSOR_MODE,
    REG_P86_EXHAUST_TIME,
    UINT16_MAX
};

static uint16_t calculate_crc(const ihm_parameter_blob_t *parameters)
{
    return modbus_crc_calculate((const uint8_t *)parameters,
                                (uint16_t)offsetof(ihm_parameter_blob_t, crc));
}

static bool value_is_valid(ihm_parameter_id_t id, uint16_t value)
{
    switch (id)
    {
        case IHM_PARAM_P10:
        case IHM_PARAM_P11:
            return (value >= 5U) && (value <= 60U);
        case IHM_PARAM_P12:
        case IHM_PARAM_P33:
        case IHM_PARAM_P51:
        case IHM_PARAM_P81:
            return value <= 1U;
        case IHM_PARAM_P20:
            return (value >= 100U) && (value <= 2400U);
        case IHM_PARAM_P21:
            return (value >= 2300U) && (value <= 9000U);
        case IHM_PARAM_P30:
        case IHM_PARAM_P31:
        case IHM_PARAM_P86:
            return value <= 240U;
        case IHM_PARAM_P32:
            return (value == 0U) || ((value >= 100U) && (value <= 9000U));
        case IHM_PARAM_P35:
            return value <= 9U;
        case IHM_PARAM_P41:
            return (value == 50U) || (value == 60U);
        case IHM_PARAM_P42:
            return (value == 5U) || (value == 10U) || (value == 20U);
        case IHM_PARAM_P43:
            return value <= 900U;
        case IHM_PARAM_P44:
        case IHM_PARAM_P82:
        case IHM_PARAM_P85:
            return value <= 2U;
        case IHM_PARAM_P45:
            return (value >= 100U) && (value <= 200U);
        case IHM_PARAM_P91:
            return value <= 100U;
        default:
            return false;
    }
}

void ihm_parameters_load_defaults(ihm_parameter_blob_t *parameters)
{
    if (parameters == NULL)
    {
        return;
    }
    (void)memset(parameters, 0, sizeof(*parameters));
    parameters->schema_version = IHM_PARAMETER_SCHEMA_VERSION;
    (void)memcpy(parameters->values,
                 s_factory_defaults,
                 sizeof(s_factory_defaults));
    ihm_parameters_finalize(parameters);
}

void ihm_parameters_finalize(ihm_parameter_blob_t *parameters)
{
    if (parameters != NULL)
    {
        parameters->schema_version = IHM_PARAMETER_SCHEMA_VERSION;
        parameters->crc = calculate_crc(parameters);
    }
}

bool ihm_parameters_validate(const ihm_parameter_blob_t *parameters)
{
    uint16_t index;

    if (parameters == NULL)
    {
        return false;
    }
    for (index = 0U; index < (uint16_t)IHM_PARAM_COUNT; index++)
    {
        if (!value_is_valid((ihm_parameter_id_t)index,
                            parameters->values[index]))
        {
            return false;
        }
    }
    return (parameters->values[IHM_PARAM_P20] <=
            parameters->values[IHM_PARAM_P21]) &&
           ((parameters->values[IHM_PARAM_P32] == 0U) ||
            (parameters->values[IHM_PARAM_P32] <=
             parameters->values[IHM_PARAM_P21]));
}

bool ihm_parameters_blob_is_valid(const ihm_parameter_blob_t *parameters)
{
    return (parameters != NULL) &&
           (parameters->schema_version == IHM_PARAMETER_SCHEMA_VERSION) &&
           (parameters->crc == calculate_crc(parameters)) &&
           ihm_parameters_validate(parameters);
}

bool ihm_parameters_set(ihm_parameter_blob_t *parameters,
                        ihm_parameter_id_t id,
                        uint16_t value)
{
    ihm_parameter_blob_t candidate;

    if ((parameters == NULL) || (id >= IHM_PARAM_COUNT) ||
        !value_is_valid(id, value))
    {
        return false;
    }
    candidate = *parameters;
    candidate.values[id] = value;
    if (!ihm_parameters_validate(&candidate))
    {
        return false;
    }
    *parameters = candidate;
    ihm_parameters_finalize(parameters);
    return true;
}

bool ihm_parameters_get(const ihm_parameter_blob_t *parameters,
                        ihm_parameter_id_t id,
                        uint16_t *value)
{
    if ((parameters == NULL) || (value == NULL) || (id >= IHM_PARAM_COUNT))
    {
        return false;
    }
    *value = parameters->values[id];
    return true;
}

bool ihm_parameters_find(const char *code, ihm_parameter_id_t *id)
{
    uint16_t index;

    if ((code == NULL) || (id == NULL))
    {
        return false;
    }
    for (index = 0U; index < (uint16_t)IHM_PARAM_COUNT; index++)
    {
        if (strcmp(code, s_codes[index]) == 0)
        {
            *id = (ihm_parameter_id_t)index;
            return true;
        }
    }
    return false;
}

const char *ihm_parameters_code(ihm_parameter_id_t id)
{
    return (id < IHM_PARAM_COUNT) ? s_codes[id] : "P??";
}

uint16_t ihm_parameters_register_address(ihm_parameter_id_t id)
{
    return (id < IHM_PARAM_COUNT) ? s_addresses[id] : UINT16_MAX;
}
