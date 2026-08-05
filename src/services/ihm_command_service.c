#include "ihm_command_service.h"

#include "comm_diagnostics.h"
#include "parameter_storage.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <stddef.h>

static const char *TAG = "ihm_service";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static ihm_parameter_blob_t s_parameters;
static bool s_edit_unlocked;
static bool s_sync_pending;
static bool s_handshake_complete;
static bool s_e08_active;
static uint16_t s_remembered_motor_frequency;

static bool motor_frequency_is_valid(uint16_t centihz,
                                     const ihm_parameter_blob_t *parameters)
{
    return (centihz >= parameters->values[IHM_PARAM_P20]) &&
           (centihz <= parameters->values[IHM_PARAM_P21]) &&
           (centihz <= 6000U);
}

esp_err_t ihm_command_service_init(void)
{
    bool defaults_loaded;
    esp_err_t error = parameter_storage_init(&s_parameters, &defaults_loaded);

    if (error != ESP_OK)
    {
        return error;
    }
    s_edit_unlocked = false;
    s_sync_pending = true;
    s_handshake_complete = false;
    s_e08_active = false;
    s_remembered_motor_frequency = s_parameters.values[IHM_PARAM_P20];
    if ((s_parameters.values[IHM_PARAM_P12] != 0U) &&
        ((parameter_storage_load_motor_frequency(
              &s_remembered_motor_frequency) != ESP_OK) ||
         !motor_frequency_is_valid(s_remembered_motor_frequency,
                                   &s_parameters)))
    {
        s_remembered_motor_frequency = s_parameters.values[IHM_PARAM_P20];
        (void)parameter_storage_save_motor_frequency(
            s_remembered_motor_frequency);
    }
    (void)comm_diagnostics_set_p91(
        (uint8_t)s_parameters.values[IHM_PARAM_P91]);
    ESP_LOGI(TAG,
             "Parâmetros carregados da %s",
             defaults_loaded ? "configuração de fábrica" : "NVS");
    return ESP_OK;
}

ihm_command_status_t ihm_command_service_p00(uint16_t command)
{
    if (command == 7U)
    {
        portENTER_CRITICAL(&s_lock);
        s_edit_unlocked = !s_edit_unlocked;
        portEXIT_CRITICAL(&s_lock);
        return IHM_COMMAND_OK;
    }
    if (command == 101U)
    {
        ihm_parameter_blob_t defaults;

        ihm_parameters_load_defaults(&defaults);
        if (parameter_storage_save(&defaults) != ESP_OK)
        {
            return IHM_COMMAND_STORAGE_ERROR;
        }
        portENTER_CRITICAL(&s_lock);
        s_parameters = defaults;
        s_edit_unlocked = false;
        s_sync_pending = true;
        s_handshake_complete = false;
        s_e08_active = false;
        portEXIT_CRITICAL(&s_lock);
        (void)comm_diagnostics_set_p91(
            (uint8_t)defaults.values[IHM_PARAM_P91]);
        s_remembered_motor_frequency = defaults.values[IHM_PARAM_P20];
        (void)parameter_storage_save_motor_frequency(
            s_remembered_motor_frequency);
        return IHM_COMMAND_OK;
    }
    return IHM_COMMAND_INVALID_ARGUMENT;
}

ihm_command_status_t ihm_command_service_set_parameter(ihm_parameter_id_t id,
                                                       uint16_t value)
{
    ihm_parameter_blob_t candidate;

    portENTER_CRITICAL(&s_lock);
    if (!s_edit_unlocked)
    {
        portEXIT_CRITICAL(&s_lock);
        return IHM_COMMAND_PARAMETER_LOCKED;
    }
    candidate = s_parameters;
    portEXIT_CRITICAL(&s_lock);

    if (!ihm_parameters_set(&candidate, id, value))
    {
        return IHM_COMMAND_INVALID_ARGUMENT;
    }
    if (parameter_storage_save(&candidate) != ESP_OK)
    {
        return IHM_COMMAND_STORAGE_ERROR;
    }

    portENTER_CRITICAL(&s_lock);
    s_parameters = candidate;
    if (id != IHM_PARAM_P91)
    {
        s_sync_pending = true;
        s_handshake_complete = false;
    }
    portEXIT_CRITICAL(&s_lock);
    if (id == IHM_PARAM_P91)
    {
        (void)comm_diagnostics_set_p91((uint8_t)value);
    }
    else if ((id == IHM_PARAM_P12) || (id == IHM_PARAM_P20) ||
             (id == IHM_PARAM_P21))
    {
        portENTER_CRITICAL(&s_lock);
        if ((s_parameters.values[IHM_PARAM_P12] == 0U) ||
            !motor_frequency_is_valid(s_remembered_motor_frequency,
                                      &s_parameters))
        {
            s_remembered_motor_frequency =
                s_parameters.values[IHM_PARAM_P20];
        }
        value = s_remembered_motor_frequency;
        portEXIT_CRITICAL(&s_lock);
        (void)parameter_storage_save_motor_frequency(value);
    }
    return IHM_COMMAND_OK;
}

ihm_command_status_t ihm_command_service_save(void)
{
    ihm_parameter_blob_t copy;

    ihm_command_service_get_parameters(&copy);
    return (parameter_storage_save(&copy) == ESP_OK) ?
           IHM_COMMAND_OK : IHM_COMMAND_STORAGE_ERROR;
}

void ihm_command_service_get_parameters(ihm_parameter_blob_t *parameters)
{
    if (parameters == NULL)
    {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *parameters = s_parameters;
    portEXIT_CRITICAL(&s_lock);
}

bool ihm_command_service_is_edit_unlocked(void)
{
    bool value;
    portENTER_CRITICAL(&s_lock);
    value = s_edit_unlocked;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

bool ihm_command_service_is_sync_pending(void)
{
    bool value;
    portENTER_CRITICAL(&s_lock);
    value = s_sync_pending;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

bool ihm_command_service_is_handshake_complete(void)
{
    bool value;
    portENTER_CRITICAL(&s_lock);
    value = s_handshake_complete;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

void ihm_command_service_request_sync(void)
{
    portENTER_CRITICAL(&s_lock);
    s_sync_pending = true;
    s_handshake_complete = false;
    portEXIT_CRITICAL(&s_lock);
}

void ihm_command_service_set_sync_result(bool success)
{
    portENTER_CRITICAL(&s_lock);
    s_sync_pending = !success;
    s_handshake_complete = success;
    portEXIT_CRITICAL(&s_lock);
}

void ihm_command_service_set_e08_active(bool active)
{
    portENTER_CRITICAL(&s_lock);
    s_e08_active = active;
    portEXIT_CRITICAL(&s_lock);
}

bool ihm_command_service_is_e08_active(void)
{
    bool value;
    portENTER_CRITICAL(&s_lock);
    value = s_e08_active;
    portEXIT_CRITICAL(&s_lock);
    return value;
}

uint16_t ihm_command_service_get_motor_start_frequency(void)
{
    uint16_t value;

    portENTER_CRITICAL(&s_lock);
    value = (s_parameters.values[IHM_PARAM_P12] != 0U) ?
            s_remembered_motor_frequency :
            s_parameters.values[IHM_PARAM_P20];
    portEXIT_CRITICAL(&s_lock);
    return value;
}

ihm_command_status_t ihm_command_service_remember_motor_frequency(
    uint16_t centihz)
{
    bool remember;

    portENTER_CRITICAL(&s_lock);
    if (!motor_frequency_is_valid(centihz, &s_parameters))
    {
        portEXIT_CRITICAL(&s_lock);
        return IHM_COMMAND_INVALID_ARGUMENT;
    }
    remember = s_parameters.values[IHM_PARAM_P12] != 0U;
    if (remember)
    {
        s_remembered_motor_frequency = centihz;
    }
    portEXIT_CRITICAL(&s_lock);

    if (remember &&
        (parameter_storage_save_motor_frequency(centihz) != ESP_OK))
    {
        return IHM_COMMAND_STORAGE_ERROR;
    }
    return IHM_COMMAND_OK;
}

const char *ihm_command_status_to_string(ihm_command_status_t status)
{
    switch (status)
    {
        case IHM_COMMAND_OK:
            return "OK";
        case IHM_COMMAND_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";
        case IHM_COMMAND_PARAMETER_LOCKED:
            return "PARAM_LOCKED";
        case IHM_COMMAND_STORAGE_ERROR:
            return "STORAGE_ERROR";
        default:
            return "UNKNOWN";
    }
}
