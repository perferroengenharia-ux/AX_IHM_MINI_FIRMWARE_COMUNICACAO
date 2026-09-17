/**
 * @file remote_protocol.h
 * @brief Contrato JSON do aplicativo e ponte segura para os comandos validados.
 */

#ifndef REMOTE_PROTOCOL_H
#define REMOTE_PROTOCOL_H

#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <time.h>

#define REMOTE_FIRMWARE_VERSION "COMUNICACAO-SENSORES-MQTT-1.3.6"
#define REMOTE_PROTOCOL_SCHEMA  "axon.ihm.v1"

typedef enum
{
    REMOTE_JSON_STATUS = 0,
    REMOTE_JSON_STATE,
    REMOTE_JSON_CAPABILITIES,
    REMOTE_JSON_DIAGNOSTICS,
    REMOTE_JSON_EVENTS,
    REMOTE_JSON_ERRORS,
    REMOTE_JSON_SCHEDULES
} remote_json_kind_t;

esp_err_t remote_protocol_init(void);

void remote_protocol_set_network_state(bool station_connected,
                                       bool access_point_active,
                                       bool mqtt_connected);
void remote_protocol_set_device_id(const char *device_id);
void remote_protocol_get_device_id(char *buffer, size_t buffer_size);

/** O chamador libera *json com free(). */
esp_err_t remote_protocol_build_json(remote_json_kind_t kind, char **json);

/**
 * Executa um DeviceCommandMessage. Quando wrapped=true, aceita o corpo HTTP
 * {"command": DeviceCommandMessage}. O retorno JSON e sempre um ACK valido
 * para comandos sintaticamente validos, mesmo quando o STM32 os rejeita.
 */
esp_err_t remote_protocol_handle_command(const char *json,
                                         bool wrapped,
                                         char **ack_json,
                                         bool *applied);

/** Valida, persiste e devolve o snapshot canonico de agendamentos. */
esp_err_t remote_protocol_handle_schedules(const char *json,
                                           char **response_json,
                                           bool *changed);

/** Verdadeiro para um snapshot de agendamentos publicado pela propria IHM. */
bool remote_protocol_is_own_schedules_payload(const char *json);

/** Executa, no maximo uma vez por minuto, os horarios salvos na IHM. */
esp_err_t remote_protocol_process_schedules(time_t now);

#endif /* REMOTE_PROTOCOL_H */
