/**
 * @file remote_protocol.c
 * @brief Serializacao do protocolo axon.ihm.v1 e comandos para o STM32.
 *
 * Este modulo nao acessa UART nem altera a politica Modbus validada. Todos os
 * acessos passam pela fila publica de app.c, a mesma usada pelo terminal.
 */

#include "remote_protocol.h"

#include "app.h"
#include "comm_diagnostics.h"
#include "ihm_command_service.h"
#include "ihm_parameters.h"
#include "parameter_cache.h"
#include "protocol/register_map.h"

#include "cJSON.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define REMOTE_DEVICE_ID_MAX          48U
#define REMOTE_COMMAND_ID_MAX         64U
#define REMOTE_COMMAND_TYPE_MAX       32U
#define REMOTE_EVENT_TEXT_MAX         160U
#define REMOTE_SCHEDULES_MAX_JSON     6144U
#define REMOTE_COMMAND_WAIT_MS        7000U
#define REMOTE_SYNC_WAIT_MS           7000U
#define REMOTE_E08_WAIT_MS            12000U
#define REMOTE_NVS_NAMESPACE          "remote_access"
#define REMOTE_NVS_SCHEDULES_KEY      "schedules"

typedef enum
{
    REMOTE_COMMAND_IDLE = 0,
    REMOTE_COMMAND_SENDING,
    REMOTE_COMMAND_APPLIED,
    REMOTE_COMMAND_FAILED
} remote_command_status_t;

typedef struct
{
    bool station_connected;
    bool access_point_active;
    bool mqtt_connected;
    remote_command_status_t command_status;
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    char last_command_id[REMOTE_COMMAND_ID_MAX + 1U];
    char last_command_type[REMOTE_COMMAND_TYPE_MAX + 1U];
    char last_event_title[64];
    char last_event_message[REMOTE_EVENT_TEXT_MAX + 1U];
    char last_command_error[48];
    char last_command_error_message[REMOTE_EVENT_TEXT_MAX + 1U];
    uint32_t event_sequence;
} remote_runtime_t;

typedef struct
{
    char id[REMOTE_COMMAND_ID_MAX + 1U];
    char type[REMOTE_COMMAND_TYPE_MAX + 1U];
    const cJSON *payload;
    const char *behavior;
} parsed_command_t;

static const char *TAG = "remote_protocol";
static remote_runtime_t s_runtime;
static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_command_lock;
static portMUX_TYPE s_request_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_next_request_id;
static char *s_schedules_json;

static void copy_text(char *destination, size_t size, const char *source)
{
    if ((destination == NULL) || (size == 0U))
    {
        return;
    }
    if (source == NULL)
    {
        destination[0] = '\0';
        return;
    }
    (void)snprintf(destination, size, "%s", source);
}

static void format_timestamp(char *buffer, size_t size)
{
    time_t now = time(NULL);
    struct tm utc = {0};

    if ((buffer == NULL) || (size == 0U))
    {
        return;
    }
    (void)gmtime_r(&now, &utc);
    if (strftime(buffer, size, "%Y-%m-%dT%H:%M:%SZ", &utc) == 0U)
    {
        copy_text(buffer, size, "1970-01-01T00:00:00Z");
    }
}

static const char *error_name(uint16_t code)
{
    switch (code)
    {
        case REG_ERROR_NONE: return "NONE";
        case REG_ERROR_DC_BUS_OVERVOLTAGE: return "E02";
        case REG_ERROR_DC_BUS_UNDERVOLTAGE: return "E03";
        case REG_ERROR_IGBT_OVERTEMPERATURE: return "E04";
        case REG_ERROR_MOTOR_OVERLOAD: return "E05";
        case REG_ERROR_HARDWARE_OVERCURRENT: return "E06";
        case REG_ERROR_COMMUNICATION: return "E08";
        case REG_ERROR_MAINS_UNDERVOLTAGE: return "E09";
        default: return "UNKNOWN";
    }
}

static const char *error_message(uint16_t code)
{
    switch (code)
    {
        case REG_ERROR_DC_BUS_OVERVOLTAGE: return "Sobretensao no barramento CC";
        case REG_ERROR_DC_BUS_UNDERVOLTAGE: return "Subtensao no barramento CC";
        case REG_ERROR_IGBT_OVERTEMPERATURE: return "Sobretemperatura do IPM/IGBT";
        case REG_ERROR_MOTOR_OVERLOAD: return "Sobrecarga de corrente do motor";
        case REG_ERROR_HARDWARE_OVERCURRENT: return "Sobrecorrente detectada pelo hardware";
        case REG_ERROR_COMMUNICATION: return "Falha de comunicacao entre IHM e inversor";
        case REG_ERROR_MAINS_UNDERVOLTAGE: return "Subtensao da rede de alimentacao";
        default: return "Falha nao identificada";
    }
}

static const char *command_status_name(remote_command_status_t status)
{
    switch (status)
    {
        case REMOTE_COMMAND_SENDING: return "sending";
        case REMOTE_COMMAND_APPLIED: return "applied";
        case REMOTE_COMMAND_FAILED: return "failed";
        case REMOTE_COMMAND_IDLE:
        default: return "idle";
    }
}

static bool comm_is_available(comm_state_t state)
{
    return (state == COMM_STATE_ONLINE) || (state == COMM_STATE_DEGRADED);
}

static cJSON *create_metadata(void)
{
    char timestamp[32];
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    cJSON *root = cJSON_CreateObject();

    if (root == NULL)
    {
        return NULL;
    }
    format_timestamp(timestamp, sizeof(timestamp));
    remote_protocol_get_device_id(device_id, sizeof(device_id));
    cJSON_AddStringToObject(root, "schema", REMOTE_PROTOCOL_SCHEMA);
    cJSON_AddStringToObject(root, "deviceId", device_id);
    cJSON_AddStringToObject(root, "timestamp", timestamp);
    cJSON_AddStringToObject(root, "source", "ihm");
    return root;
}

static char *finish_json(cJSON *root)
{
    char *json;

    if (root == NULL)
    {
        return NULL;
    }
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

static void take_runtime_snapshot(remote_runtime_t *runtime)
{
    if ((runtime == NULL) || (s_lock == NULL))
    {
        return;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    *runtime = s_runtime;
    (void)xSemaphoreGive(s_lock);
}

static void get_snapshots(parameter_cache_snapshot_t *cache,
                          comm_diagnostics_snapshot_t *comm,
                          ihm_parameter_blob_t *parameters,
                          remote_runtime_t *runtime)
{
    parameter_cache_get_snapshot(cache);
    comm_diagnostics_get_snapshot(comm);
    ihm_command_service_get_parameters(parameters);
    take_runtime_snapshot(runtime);
}

static const char *connection_mode(const remote_runtime_t *runtime)
{
    return runtime->mqtt_connected ? "cloud" :
           (runtime->access_point_active ? "local-ap" : "local-lan");
}

static const char *ready_state(const parameter_cache_snapshot_t *cache,
                               const comm_diagnostics_snapshot_t *comm)
{
    if (!comm_is_available(comm->state) || !cache->runtime_valid)
    {
        return "offline";
    }
    if (((cache->status_word & REG_STATUS_MOTOR_FAULT_MASK) != 0U) ||
        (cache->current_error != REG_ERROR_NONE) ||
        (cache->active_fault_mask != 0U))
    {
        return "fault";
    }
    if ((cache->status_word & REG_STATUS_CYCLE_ACTIVE_MASK) != 0U)
    {
        return "draining";
    }
    if ((cache->status_word & REG_STATUS_MOTOR_RUNNING_MASK) != 0U)
    {
        return "running";
    }
    return "ready";
}

static const char *peripheral_state(uint16_t status,
                                    uint16_t active_mask,
                                    bool available)
{
    if (!available)
    {
        return "unavailable";
    }
    return ((status & active_mask) != 0U) ? "on" : "off";
}

static const char *water_state(uint16_t sensor_status, uint16_t mode)
{
    if (mode == 0U)
    {
        return "disabled";
    }
    if ((sensor_status & REG_SENSOR_LEVEL_VALID_MASK) == 0U)
    {
        return "unknown";
    }
    return ((sensor_status & REG_SENSOR_WATER_SHORTAGE_MASK) != 0U) ?
           "low" : "ok";
}

static void add_status_fields(cJSON *object,
                              const parameter_cache_snapshot_t *cache,
                              const comm_diagnostics_snapshot_t *comm,
                              const remote_runtime_t *runtime)
{
    char timestamp[32];
    const bool online = comm_is_available(comm->state) && cache->runtime_valid;

    cJSON_AddBoolToObject(object, "deviceOnline", online);
    cJSON_AddStringToObject(object, "connectionMode", connection_mode(runtime));
    if (online)
    {
        format_timestamp(timestamp, sizeof(timestamp));
        cJSON_AddStringToObject(object, "lastSeen", timestamp);
    }
    else
    {
        cJSON_AddNullToObject(object, "lastSeen");
    }
    cJSON_AddStringToObject(object, "readyState", ready_state(cache, comm));
}

static cJSON *create_state_object(const parameter_cache_snapshot_t *cache,
                                  const comm_diagnostics_snapshot_t *comm,
                                  const ihm_parameter_blob_t *parameters,
                                  const remote_runtime_t *runtime)
{
    const uint16_t peripheral = cache->runtime_valid ?
                                cache->peripheral_status : 0U;
    const bool pump_available = parameters->values[IHM_PARAM_P82] != 0U;
    const bool swing_available = parameters->values[IHM_PARAM_P81] != 0U;
    const uint16_t level_mode = parameters->values[IHM_PARAM_P85];
    const uint16_t current_error = cache->snapshot_valid ?
                                   cache->current_error : REG_ERROR_NONE;
    cJSON *object = cJSON_CreateObject();
    cJSON *sensors;
    cJSON *outputs;
    cJSON *communication;
    cJSON *parameter_values;
    uint16_t parameter_index;

    if (object == NULL)
    {
        return NULL;
    }
    add_status_fields(object, cache, comm, runtime);
    cJSON_AddBoolToObject(object, "inverterRunning",
                          (peripheral & REG_PERIPHERAL_MOTOR_ACTIVE_MASK) != 0U ||
                          (cache->status_word & REG_STATUS_MOTOR_RUNNING_MASK) != 0U);
    cJSON_AddNumberToObject(object, "freqCurrentHz",
                            cache->output_frequency.valid ?
                            cache->output_frequency.converted_value : 0.0);
    cJSON_AddNumberToObject(object, "freqTargetHz",
                            (double)ihm_command_service_get_motor_start_frequency() /
                            100.0);
    cJSON_AddStringToObject(object, "pumpState",
                            peripheral_state(peripheral,
                                             REG_PERIPHERAL_PUMP_ACTIVE_MASK,
                                             pump_available));
    cJSON_AddStringToObject(object, "swingState",
                            peripheral_state(peripheral,
                                             REG_PERIPHERAL_SWING_ACTIVE_MASK,
                                             swing_available));
    cJSON_AddStringToObject(object, "drainState",
                            (cache->status_word & REG_STATUS_CYCLE_ACTIVE_MASK) != 0U ?
                            "on" : "off");
    cJSON_AddStringToObject(object, "waterLevelState",
                            water_state(cache->sensor_status, level_mode));
    cJSON_AddStringToObject(object, "lastCommandStatus",
                            command_status_name(runtime->command_status));
    if (current_error != REG_ERROR_NONE)
    {
        cJSON_AddStringToObject(object, "lastErrorCode", error_name(current_error));
    }
    else if (runtime->last_command_error[0] != '\0')
    {
        cJSON_AddStringToObject(object, "lastErrorCode",
                                runtime->last_command_error);
    }
    else
    {
        cJSON_AddNullToObject(object, "lastErrorCode");
    }

    /* Extensoes opcionais: a versao anterior do app ignora campos desconhecidos. */
    sensors = cJSON_AddObjectToObject(object, "sensors");
    cJSON_AddBoolToObject(sensors, "snapshotValid", cache->snapshot_valid);
    cJSON_AddNumberToObject(sensors, "dcBusVoltageV",
                            cache->dc_bus_voltage.converted_value);
    cJSON_AddNumberToObject(sensors, "outputCurrentA",
                            cache->output_current.converted_value);
    cJSON_AddNumberToObject(sensors, "outputVoltageV",
                            cache->output_voltage.converted_value);
    cJSON_AddNumberToObject(sensors, "igbtTemperatureC",
                            cache->igbt_temperature.converted_value);
    cJSON_AddNumberToObject(sensors, "lastFaultCode",
                            cache->last_error.raw_value);
    cJSON_AddNumberToObject(sensors, "currentErrorCode", current_error);
    cJSON_AddNumberToObject(sensors, "sensorStatusRaw",
                            cache->sensor_status);
    cJSON_AddNumberToObject(sensors, "activeFaultMask",
                            cache->active_fault_mask);
    cJSON_AddBoolToObject(sensors, "adcReady",
                          (cache->sensor_status &
                           REG_SENSOR_ACQUISITION_READY_MASK) != 0U);
    cJSON_AddBoolToObject(sensors, "opampOffsetReady",
                          (cache->sensor_status &
                           REG_SENSOR_OFFSET_READY_MASK) != 0U);
    cJSON_AddBoolToObject(sensors, "temperatureValid",
                          (cache->sensor_status &
                           REG_SENSOR_TEMPERATURE_VALID_MASK) != 0U);
    cJSON_AddBoolToObject(sensors, "busVoltageValid",
                          (cache->sensor_status & REG_SENSOR_VBUS_VALID_MASK) != 0U);
    cJSON_AddBoolToObject(sensors, "currentValid",
                          (cache->sensor_status & REG_SENSOR_CURRENT_VALID_MASK) != 0U);

    outputs = cJSON_AddObjectToObject(object, "peripherals");
    cJSON_AddBoolToObject(outputs, "pumpRequested",
                          (peripheral & REG_PERIPHERAL_PUMP_REQUEST_MASK) != 0U);
    cJSON_AddBoolToObject(outputs, "pumpAllowed",
                          (peripheral & REG_PERIPHERAL_PUMP_ALLOWED_MASK) != 0U);
    cJSON_AddBoolToObject(outputs, "pumpActive",
                          (peripheral & REG_PERIPHERAL_PUMP_ACTIVE_MASK) != 0U);
    cJSON_AddBoolToObject(outputs, "swingRequested",
                          (peripheral & REG_PERIPHERAL_SWING_REQUEST_MASK) != 0U);
    cJSON_AddBoolToObject(outputs, "swingAllowed",
                          (peripheral & REG_PERIPHERAL_SWING_ALLOWED_MASK) != 0U);
    cJSON_AddBoolToObject(outputs, "swingActive",
                          (peripheral & REG_PERIPHERAL_SWING_ACTIVE_MASK) != 0U);
    cJSON_AddBoolToObject(outputs, "bypassActive",
                          (peripheral & REG_PERIPHERAL_BYPASS_ACTIVE_MASK) != 0U);
    cJSON_AddBoolToObject(outputs, "motorActive",
                          (peripheral & REG_PERIPHERAL_MOTOR_ACTIVE_MASK) != 0U);

    communication = cJSON_AddObjectToObject(object, "communication");
    cJSON_AddStringToObject(communication, "state",
                            comm_diagnostics_state_to_string(comm->state));
    cJSON_AddNumberToObject(communication, "lossPercent", comm->p90_loss_percent);
    cJSON_AddNumberToObject(communication, "latencyMs", comm->last_latency_ms);
    cJSON_AddNumberToObject(communication, "validResponses", comm->valid_responses);
    cJSON_AddNumberToObject(communication, "timeouts", comm->response_timeouts);
    cJSON_AddNumberToObject(communication, "crcErrors", comm->crc_errors);
    cJSON_AddNumberToObject(communication, "protocolErrors", comm->protocol_errors);

    /*
     * Snapshot somente-leitura dos parametros persistidos na IHM. Os valores
     * permanecem na unidade bruta do contrato Modbus (por exemplo, P20 em
     * centesimos de hertz). Aplicativos antigos ignoram este campo opcional.
     */
    parameter_values = cJSON_AddObjectToObject(object, "parameters");
    for (parameter_index = 0U;
         parameter_index < (uint16_t)IHM_PARAM_COUNT;
         parameter_index++)
    {
        cJSON_AddNumberToObject(
            parameter_values,
            ihm_parameters_code((ihm_parameter_id_t)parameter_index),
            parameters->values[parameter_index]);
    }
    return object;
}

static esp_err_t build_status(char **json)
{
    parameter_cache_snapshot_t cache;
    comm_diagnostics_snapshot_t comm;
    ihm_parameter_blob_t parameters;
    remote_runtime_t runtime;
    cJSON *root = create_metadata();
    cJSON *status;

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    get_snapshots(&cache, &comm, &parameters, &runtime);
    status = cJSON_AddObjectToObject(root, "status");
    add_status_fields(status, &cache, &comm, &runtime);
    *json = finish_json(root);
    return *json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t build_state(char **json)
{
    parameter_cache_snapshot_t cache;
    comm_diagnostics_snapshot_t comm;
    ihm_parameter_blob_t parameters;
    remote_runtime_t runtime;
    cJSON *root = create_metadata();
    cJSON *state;

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    get_snapshots(&cache, &comm, &parameters, &runtime);
    state = create_state_object(&cache, &comm, &parameters, &runtime);
    if (state == NULL)
    {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(root, "state", state);
    *json = finish_json(root);
    return *json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t build_capabilities(char **json)
{
    ihm_parameter_blob_t p;
    cJSON *root = create_metadata();
    cJSON *caps;
    uint16_t pump_mode;
    uint16_t sensor_mode;

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    ihm_command_service_get_parameters(&p);
    pump_mode = p.values[IHM_PARAM_P82];
    sensor_mode = p.values[IHM_PARAM_P85];
    caps = cJSON_AddObjectToObject(root, "capabilities");
    cJSON_AddNumberToObject(caps, "fMinHz", (double)p.values[IHM_PARAM_P20] / 100.0);
    cJSON_AddNumberToObject(caps, "fMaxHz", (double)p.values[IHM_PARAM_P21] / 100.0);
    cJSON_AddBoolToObject(caps, "pumpAvailable", pump_mode != 0U);
    cJSON_AddBoolToObject(caps, "swingAvailable", p.values[IHM_PARAM_P81] != 0U);
    cJSON_AddBoolToObject(caps, "drainAvailable",
                          (p.values[IHM_PARAM_P32] != 0U) ||
                          (p.values[IHM_PARAM_P86] != 0U));
    cJSON_AddBoolToObject(caps, "waterSensorEnabled", sensor_mode != 0U);
    cJSON_AddStringToObject(caps, "drainMode",
                            sensor_mode == 0U ? "timed" : "hybrid");
    cJSON_AddNumberToObject(caps, "drainTimeSec",
                            p.values[IHM_PARAM_P86] * 60U);
    cJSON_AddNumberToObject(caps, "drainReturnDelaySec", 0U);
    cJSON_AddStringToObject(caps, "pumpLogicMode",
                            pump_mode == 2U ? "independent" :
                            (pump_mode == 1U ? "linked" : "forced-off"));
    cJSON_AddStringToObject(caps, "waterSensorMode",
                            sensor_mode == 2U ? "inverted" :
                            (sensor_mode == 1U ? "normal" : "disabled"));
    cJSON_AddNumberToObject(caps, "preWetSec", p.values[IHM_PARAM_P30] * 60U);
    cJSON_AddNumberToObject(caps, "dryPanelSec", p.values[IHM_PARAM_P31] * 60U);
    cJSON_AddNumberToObject(caps, "dryPanelFreqHz",
                            (double)p.values[IHM_PARAM_P32] / 100.0);
    cJSON_AddStringToObject(caps, "resumeMode",
                            p.values[IHM_PARAM_P12] != 0U ?
                            "resume-last-state" : "always-off");
    cJSON_AddStringToObject(caps, "autoResetMode",
                            p.values[IHM_PARAM_P44] != 0U ? "enabled" : "disabled");
    *json = finish_json(root);
    return *json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t build_diagnostics(char **json)
{
    parameter_cache_snapshot_t cache;
    comm_diagnostics_snapshot_t comm;
    ihm_parameter_blob_t parameters;
    remote_runtime_t runtime;
    app_sync_snapshot_t sync;
    cJSON *root = create_metadata();
    cJSON *diag;
    char summary[REMOTE_EVENT_TEXT_MAX + 1U];
    char timestamp[32];

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    get_snapshots(&cache, &comm, &parameters, &runtime);
    app_get_sync_snapshot(&sync);
    diag = cJSON_AddObjectToObject(root, "diagnostics");
    cJSON_AddStringToObject(diag, "firmwareVersion", REMOTE_FIRMWARE_VERSION);
    (void)snprintf(summary, sizeof(summary),
                   "RS485 %s; sync %s; Wi-Fi %s; MQTT %s",
                   comm_diagnostics_state_to_string(comm.state),
                   app_sync_state_to_string(sync.state),
                   runtime.station_connected ? "conectado" : "desconectado",
                   runtime.mqtt_connected ? "conectado" : "desconectado");
    cJSON_AddStringToObject(diag, "connectionSummary", summary);
    cJSON_AddStringToObject(diag, "transportStatus",
                            comm.state == COMM_STATE_ONLINE ? "connected" :
                            (comm.state == COMM_STATE_DEGRADED ? "degraded" :
                             (comm.state == COMM_STATE_CONNECTING ? "connecting" : "error")));
    if (comm.last_success_timestamp_ms != 0U)
    {
        format_timestamp(timestamp, sizeof(timestamp));
        cJSON_AddStringToObject(diag, "lastSyncAt", timestamp);
    }
    else
    {
        cJSON_AddNullToObject(diag, "lastSyncAt");
    }
    if (runtime.last_command_error_message[0] != '\0')
    {
        cJSON_AddStringToObject(diag, "lastErrorMessage",
                                runtime.last_command_error_message);
    }
    else
    {
        cJSON_AddNullToObject(diag, "lastErrorMessage");
    }
    /* Diagnosticos adicionais, opcionais no contrato original. */
    cJSON_AddNumberToObject(diag, "uptimeSec",
                            (double)(esp_timer_get_time() / 1000000LL));
    cJSON_AddNumberToObject(diag, "requestsSent", comm.requests_sent);
    cJSON_AddNumberToObject(diag, "validResponses", comm.valid_responses);
    cJSON_AddNumberToObject(diag, "timeouts", comm.response_timeouts);
    cJSON_AddNumberToObject(diag, "crcErrors", comm.crc_errors);
    cJSON_AddNumberToObject(diag, "protocolErrors", comm.protocol_errors);
    cJSON_AddNumberToObject(diag, "latencyMs", comm.last_latency_ms);
    cJSON_AddStringToObject(diag, "parameterSyncState",
                            app_sync_state_to_string(sync.state));
    *json = finish_json(root);
    return *json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static cJSON *create_error_item(const char *id,
                                const char *device_id,
                                const char *code,
                                const char *message,
                                bool recoverable)
{
    char timestamp[32];
    cJSON *item = cJSON_CreateObject();

    if (item == NULL)
    {
        return NULL;
    }
    format_timestamp(timestamp, sizeof(timestamp));
    cJSON_AddStringToObject(item, "id", id);
    cJSON_AddStringToObject(item, "deviceId", device_id);
    cJSON_AddStringToObject(item, "code", code);
    cJSON_AddStringToObject(item, "message", message);
    cJSON_AddStringToObject(item, "createdAt", timestamp);
    cJSON_AddBoolToObject(item, "recoverable", recoverable);
    return item;
}

static esp_err_t build_events(char **json)
{
    remote_runtime_t runtime;
    cJSON *root = create_metadata();
    cJSON *events;

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    take_runtime_snapshot(&runtime);
    events = cJSON_AddArrayToObject(root, "events");
    if (runtime.last_event_message[0] != '\0')
    {
        char timestamp[32];
        char event_id[40];
        cJSON *item = cJSON_CreateObject();

        format_timestamp(timestamp, sizeof(timestamp));
        (void)snprintf(event_id, sizeof(event_id), "event-%08" PRIx32,
                       runtime.event_sequence);
        cJSON_AddStringToObject(item, "id", event_id);
        cJSON_AddStringToObject(item, "deviceId", runtime.device_id);
        cJSON_AddStringToObject(item, "level",
                                runtime.command_status == REMOTE_COMMAND_FAILED ?
                                "error" : "info");
        cJSON_AddStringToObject(item, "title", runtime.last_event_title);
        cJSON_AddStringToObject(item, "message", runtime.last_event_message);
        cJSON_AddStringToObject(item, "createdAt", timestamp);
        if (runtime.last_command_error[0] != '\0')
        {
            cJSON_AddStringToObject(item, "code", runtime.last_command_error);
        }
        cJSON_AddItemToArray(events, item);
    }
    *json = finish_json(root);
    return *json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t build_errors(char **json)
{
    parameter_cache_snapshot_t cache;
    comm_diagnostics_snapshot_t comm;
    ihm_parameter_blob_t parameters;
    remote_runtime_t runtime;
    cJSON *root = create_metadata();
    cJSON *errors;
    char id[40];
    uint16_t code;

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    get_snapshots(&cache, &comm, &parameters, &runtime);
    errors = cJSON_AddArrayToObject(root, "errors");
    for (code = REG_ERROR_DC_BUS_OVERVOLTAGE;
         code <= REG_ERROR_MAINS_UNDERVOLTAGE;
         code++)
    {
        if ((code == 7U) ||
            ((cache.active_fault_mask & (uint16_t)(1U << code)) == 0U))
        {
            continue;
        }
        (void)snprintf(id, sizeof(id), "active-E%02u", code);
        cJSON_AddItemToArray(errors,
                             create_error_item(id, runtime.device_id,
                                               error_name(code),
                                               error_message(code), true));
    }
    if (!comm_is_available(comm.state) &&
        ((cache.active_fault_mask & (uint16_t)(1U << REG_ERROR_COMMUNICATION)) == 0U))
    {
        cJSON_AddItemToArray(errors,
                             create_error_item("active-E08-local", runtime.device_id,
                                               "E08", error_message(REG_ERROR_COMMUNICATION),
                                               true));
    }
    if ((runtime.last_command_error[0] != '\0') &&
        (runtime.command_status == REMOTE_COMMAND_FAILED))
    {
        cJSON_AddItemToArray(errors,
                             create_error_item("last-command-error", runtime.device_id,
                                               runtime.last_command_error,
                                               runtime.last_command_error_message,
                                               true));
    }
    *json = finish_json(root);
    return *json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t build_schedules(char **json)
{
    char *saved_json;
    bool had_saved_json;

    if ((json == NULL) || (s_lock == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    had_saved_json = s_schedules_json != NULL;
    saved_json = had_saved_json ? strdup(s_schedules_json) : NULL;
    (void)xSemaphoreGive(s_lock);
    if (had_saved_json && (saved_json == NULL))
    {
        return ESP_ERR_NO_MEM;
    }
    if (saved_json != NULL)
    {
        *json = saved_json;
    }
    else
    {
        cJSON *root = create_metadata();
        if (root != NULL)
        {
            cJSON_AddArrayToObject(root, "schedules");
            cJSON_AddStringToObject(root, "revision", "0");
            cJSON_AddStringToObject(root, "timezone", "America/Sao_Paulo");
            cJSON_AddNumberToObject(root, "timezoneOffsetMinutes", -180);
        }
        *json = finish_json(root);
    }
    return *json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t remote_protocol_build_json(remote_json_kind_t kind, char **json)
{
    if (json == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    *json = NULL;
    switch (kind)
    {
        case REMOTE_JSON_STATUS: return build_status(json);
        case REMOTE_JSON_STATE: return build_state(json);
        case REMOTE_JSON_CAPABILITIES: return build_capabilities(json);
        case REMOTE_JSON_DIAGNOSTICS: return build_diagnostics(json);
        case REMOTE_JSON_EVENTS: return build_events(json);
        case REMOTE_JSON_ERRORS: return build_errors(json);
        case REMOTE_JSON_SCHEDULES: return build_schedules(json);
        default: return ESP_ERR_INVALID_ARG;
    }
}

static bool valid_device_id(const char *device_id)
{
    size_t index;
    size_t length;

    if (device_id == NULL)
    {
        return false;
    }
    length = strlen(device_id);
    if ((length == 0U) || (length > REMOTE_DEVICE_ID_MAX))
    {
        return false;
    }
    for (index = 0U; index < length; index++)
    {
        const char c = device_id[index];
        if (!(((c >= 'a') && (c <= 'z')) ||
              ((c >= 'A') && (c <= 'Z')) ||
              ((c >= '0') && (c <= '9')) ||
              (c == '-') || (c == '_')))
        {
            return false;
        }
    }
    return true;
}

void remote_protocol_set_device_id(const char *device_id)
{
    if (!valid_device_id(device_id) || (s_lock == NULL))
    {
        return;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    copy_text(s_runtime.device_id, sizeof(s_runtime.device_id), device_id);
    (void)xSemaphoreGive(s_lock);
}

void remote_protocol_get_device_id(char *buffer, size_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }
    if (s_lock == NULL)
    {
        copy_text(buffer, buffer_size, "AXON-IHM");
        return;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    copy_text(buffer, buffer_size, s_runtime.device_id);
    (void)xSemaphoreGive(s_lock);
}

void remote_protocol_set_network_state(bool station_connected,
                                       bool access_point_active,
                                       bool mqtt_connected)
{
    if (s_lock == NULL)
    {
        return;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    s_runtime.station_connected = station_connected;
    s_runtime.access_point_active = access_point_active;
    s_runtime.mqtt_connected = mqtt_connected;
    (void)xSemaphoreGive(s_lock);
}

static uint32_t next_request_id(void)
{
    uint32_t id;

    portENTER_CRITICAL(&s_request_lock);
    s_next_request_id++;
    if (s_next_request_id == 0U)
    {
        s_next_request_id = 1U;
    }
    id = s_next_request_id;
    portEXIT_CRITICAL(&s_request_lock);
    return id;
}

static app_comm_result_status_t submit_write(uint16_t address,
                                             uint16_t value,
                                             uint8_t *exception)
{
    QueueHandle_t response_queue;
    app_comm_request_t request = {0};
    app_comm_result_t result = {0};
    app_comm_result_status_t status = APP_COMM_RESULT_QUEUE_ERROR;

    response_queue = xQueueCreate(1U, sizeof(app_comm_result_t));
    if (response_queue == NULL)
    {
        return APP_COMM_RESULT_QUEUE_ERROR;
    }
    request.request_id = next_request_id();
    request.type = APP_COMM_REQUEST_WRITE_SINGLE;
    request.address = address;
    request.quantity = 1U;
    request.values[0] = value;
    if ((app_submit_request(&request, response_queue,
                            pdMS_TO_TICKS(REMOTE_COMMAND_WAIT_MS)) == ESP_OK) &&
        (xQueueReceive(response_queue, &result,
                       pdMS_TO_TICKS(REMOTE_COMMAND_WAIT_MS)) == pdTRUE) &&
        (result.request_id == request.request_id))
    {
        status = result.status;
        if (exception != NULL)
        {
            *exception = result.exception_code;
        }
    }
    vQueueDelete(response_queue);
    return status;
}

static bool wait_operational(void)
{
    TickType_t start;

    if (ihm_command_service_is_e08_active())
    {
        app_request_e08_recovery();
        start = xTaskGetTickCount();
        while (ihm_command_service_is_e08_active())
        {
            if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(REMOTE_E08_WAIT_MS))
            {
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(50U));
        }
    }
    if (!ihm_command_service_is_handshake_complete())
    {
        app_request_parameter_sync();
        start = xTaskGetTickCount();
        while (!ihm_command_service_is_handshake_complete())
        {
            if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(REMOTE_SYNC_WAIT_MS))
            {
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(50U));
        }
    }
    return true;
}

static void set_command_runtime(const parsed_command_t *command,
                                remote_command_status_t status,
                                const char *error,
                                const char *message)
{
    if (s_lock == NULL)
    {
        return;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    s_runtime.command_status = status;
    copy_text(s_runtime.last_command_id, sizeof(s_runtime.last_command_id),
              command != NULL ? command->id : "");
    copy_text(s_runtime.last_command_type, sizeof(s_runtime.last_command_type),
              command != NULL ? command->type : "");
    copy_text(s_runtime.last_command_error, sizeof(s_runtime.last_command_error),
              error);
    copy_text(s_runtime.last_command_error_message,
              sizeof(s_runtime.last_command_error_message), message);
    copy_text(s_runtime.last_event_title, sizeof(s_runtime.last_event_title),
              status == REMOTE_COMMAND_APPLIED ? "Comando aplicado" :
              (status == REMOTE_COMMAND_FAILED ? "Comando recusado" :
               "Comando recebido"));
    copy_text(s_runtime.last_event_message, sizeof(s_runtime.last_event_message),
              message);
    s_runtime.event_sequence++;
    (void)xSemaphoreGive(s_lock);
}

static bool parse_command(cJSON *root, bool wrapped, parsed_command_t *command)
{
    cJSON *node = root;
    const cJSON *schema;
    const cJSON *device_id;
    const cJSON *id;
    const cJSON *type;
    const cJSON *payload;
    char local_device[REMOTE_DEVICE_ID_MAX + 1U];

    if (wrapped)
    {
        node = cJSON_GetObjectItemCaseSensitive(root, "command");
    }
    if (!cJSON_IsObject(node))
    {
        return false;
    }
    schema = cJSON_GetObjectItemCaseSensitive(node, "schema");
    device_id = cJSON_GetObjectItemCaseSensitive(node, "deviceId");
    id = cJSON_GetObjectItemCaseSensitive(node, "id");
    type = cJSON_GetObjectItemCaseSensitive(node, "type");
    payload = cJSON_GetObjectItemCaseSensitive(node, "payload");
    remote_protocol_get_device_id(local_device, sizeof(local_device));
    if (!cJSON_IsString(schema) ||
        (strcmp(schema->valuestring, REMOTE_PROTOCOL_SCHEMA) != 0) ||
        !cJSON_IsString(device_id) ||
        (strcmp(device_id->valuestring, local_device) != 0) ||
        !cJSON_IsString(id) || (strlen(id->valuestring) == 0U) ||
        (strlen(id->valuestring) > REMOTE_COMMAND_ID_MAX) ||
        !cJSON_IsString(type) ||
        (strlen(type->valuestring) > REMOTE_COMMAND_TYPE_MAX) ||
        !cJSON_IsObject(payload))
    {
        return false;
    }
    (void)memset(command, 0, sizeof(*command));
    copy_text(command->id, sizeof(command->id), id->valuestring);
    copy_text(command->type, sizeof(command->type), type->valuestring);
    command->payload = payload;
    {
        const cJSON *behavior = cJSON_GetObjectItemCaseSensitive(payload, "behavior");
        command->behavior = cJSON_IsString(behavior) ? behavior->valuestring : "normal";
    }
    return true;
}

static bool execute_write(const parsed_command_t *command,
                          uint16_t address,
                          uint16_t value,
                          char *error,
                          size_t error_size,
                          char *message,
                          size_t message_size)
{
    uint8_t exception = 0U;
    app_comm_result_status_t status = submit_write(address, value, &exception);

    if (status == APP_COMM_RESULT_OK)
    {
        return true;
    }
    if (status == APP_COMM_RESULT_EXCEPTION)
    {
        (void)snprintf(error, error_size, "MODBUS_EXCEPTION_%02X", exception);
        (void)snprintf(message, message_size,
                       "O inversor recusou %s (Modbus 0x%02X)",
                       command->type, exception);
    }
    else
    {
        copy_text(error, error_size, "RS485_COMMAND_FAILED");
        (void)snprintf(message, message_size, "Falha RS485 em %s: %s",
                       command->type, app_comm_result_to_string(status));
    }
    return false;
}

static bool execute_command(const parsed_command_t *command,
                            char *error,
                            size_t error_size,
                            char *message,
                            size_t message_size)
{
    const cJSON *enabled;
    const cJSON *frequency;
    uint16_t control = 0U;

    if ((strcmp(command->type, "request-status") == 0) ||
        (strcmp(command->type, "request-capabilities") == 0) ||
        (strcmp(command->type, "request-parameters") == 0))
    {
        copy_text(message, message_size, "Snapshot solicitado");
        return true;
    }
    if (strcmp(command->type, "sync-schedules") == 0)
    {
        const cJSON *schedules = cJSON_GetObjectItemCaseSensitive(command->payload,
                                                                  "schedules");
        const cJSON *revision = cJSON_GetObjectItemCaseSensitive(command->payload,
                                                                 "revision");
        cJSON *envelope = create_metadata();
        cJSON *schedules_copy = cJSON_IsArray(schedules) ?
                                cJSON_Duplicate(schedules, true) : NULL;
        char *schedule_json;
        char *stored_json = NULL;
        bool changed = false;
        esp_err_t schedule_error;

        if ((envelope == NULL) || (schedules_copy == NULL))
        {
            cJSON_Delete(envelope);
            cJSON_Delete(schedules_copy);
            copy_text(error, error_size, "INVALID_SCHEDULES");
            copy_text(message, message_size, "Lista de agendamentos invalida");
            return false;
        }
        cJSON_AddItemToObject(envelope, "schedules", schedules_copy);
        cJSON_AddStringToObject(envelope, "revision",
                                cJSON_IsString(revision) ? revision->valuestring : "1");
        schedule_json = finish_json(envelope);
        schedule_error = schedule_json != NULL ?
                         remote_protocol_handle_schedules(schedule_json,
                                                          &stored_json,
                                                          &changed) :
                         ESP_ERR_NO_MEM;
        free(schedule_json);
        free(stored_json);
        if ((schedule_error != ESP_OK) || !changed)
        {
            copy_text(error, error_size, "INVALID_SCHEDULES");
            copy_text(message, message_size,
                      "Nao foi possivel salvar os agendamentos");
            return false;
        }
        copy_text(message, message_size, "Agendamentos atualizados");
        return true;
    }
    if (!wait_operational())
    {
        copy_text(error, error_size, "PARAMETERS_NOT_SYNCED");
        copy_text(message, message_size,
                  "Comunicacao ou sincronizacao de parametros indisponivel");
        return false;
    }

    if (strcmp(command->type, "power-on") == 0)
    {
        if ((strcmp(command->behavior, "skip-stage") != 0) &&
            (strcmp(command->behavior, "normal") != 0))
        {
            copy_text(error, error_size, "INVALID_BEHAVIOR");
            copy_text(message, message_size, "Modo de partida invalido");
            return false;
        }
        if (strcmp(command->behavior, "skip-stage") == 0)
        {
            control = REG_CONTROL_POWER_ON_SKIP;
            copy_text(message, message_size,
                      "Motor ligado; estado da bomba preservado");
        }
        else
        {
            control = REG_CONTROL_POWER_ON_NORMAL;
            copy_text(message, message_size,
                      "Molhagem iniciada; motor ligara ao concluir");
        }
    }
    else if (strcmp(command->type, "power-off") == 0)
    {
        if ((strcmp(command->behavior, "skip-stage") != 0) &&
            (strcmp(command->behavior, "normal") != 0))
        {
            copy_text(error, error_size, "INVALID_BEHAVIOR");
            copy_text(message, message_size, "Modo de parada invalido");
            return false;
        }
        if (strcmp(command->behavior, "skip-stage") == 0)
        {
            /* Interrompe uma secagem em curso e solicita a parada em rampa. */
            if (!execute_write(command, REG_CONTROL_COMMAND,
                               REG_CONTROL_CYCLE_STOP,
                               error, error_size, message, message_size))
            {
                return false;
            }
            control = REG_CONTROL_SYSTEM_OFF;
            copy_text(message, message_size, "Sistema desligando em rampa");
        }
        else
        {
            control = REG_CONTROL_POWER_OFF_NORMAL;
            copy_text(message, message_size,
                      "Parada, secagem e desligamento iniciados");
        }
    }
    else if (strcmp(command->type, "set-frequency") == 0)
    {
        ihm_parameter_blob_t parameters;
        uint16_t centihz;

        frequency = cJSON_GetObjectItemCaseSensitive(command->payload,
                                                      "freqTargetHz");
        if (!cJSON_IsNumber(frequency) || !isfinite(frequency->valuedouble) ||
            (frequency->valuedouble <= 0.0) ||
            (frequency->valuedouble > 60.0))
        {
            copy_text(error, error_size, "FREQUENCY_OUT_OF_RANGE");
            copy_text(message, message_size, "Frequencia solicitada invalida");
            return false;
        }
        centihz = (uint16_t)lround(frequency->valuedouble * 100.0);
        ihm_command_service_get_parameters(&parameters);
        if ((centihz < parameters.values[IHM_PARAM_P20]) ||
            (centihz > parameters.values[IHM_PARAM_P21]))
        {
            copy_text(error, error_size, "FREQUENCY_OUT_OF_RANGE");
            copy_text(message, message_size, "Frequencia fora dos limites P20/P21");
            return false;
        }
        if (!execute_write(command, REG_MOTOR_TARGET_COMMAND, centihz,
                           error, error_size, message, message_size))
        {
            return false;
        }
        (void)ihm_command_service_remember_motor_frequency(centihz);
        (void)snprintf(message, message_size, "Frequencia ajustada para %.2f Hz",
                       frequency->valuedouble);
        return true;
    }
    else if (strcmp(command->type, "set-parameter") == 0)
    {
        const cJSON *code = cJSON_GetObjectItemCaseSensitive(command->payload,
                                                              "code");
        const cJSON *value = cJSON_GetObjectItemCaseSensitive(command->payload,
                                                               "value");
        ihm_parameter_id_t parameter_id;
        ihm_command_status_t parameter_status;

        if (!cJSON_IsString(code) || !cJSON_IsNumber(value) ||
            !isfinite(value->valuedouble) ||
            (value->valuedouble < 0.0) ||
            (value->valuedouble > 65535.0) ||
            (floor(value->valuedouble) != value->valuedouble) ||
            !ihm_parameters_find(code->valuestring, &parameter_id))
        {
            copy_text(error, error_size, "INVALID_PARAMETER");
            copy_text(message, message_size,
                      "Codigo ou valor de parametro invalido");
            return false;
        }

        /* Usa o mesmo bloqueio e a mesma validacao do comando param set. */
        if (ihm_command_service_is_edit_unlocked())
        {
            copy_text(error, error_size, "PARAMETER_EDIT_BUSY");
            copy_text(message, message_size,
                      "Edicao local de parametros em andamento");
            return false;
        }
        if (ihm_command_service_p00(7U) != IHM_COMMAND_OK)
        {
            copy_text(error, error_size, "PARAMETER_UNLOCK_FAILED");
            copy_text(message, message_size,
                      "Nao foi possivel liberar a edicao de parametros");
            return false;
        }

        parameter_status = ihm_command_service_set_parameter(
            parameter_id, (uint16_t)value->valuedouble);
        (void)ihm_command_service_p00(7U);
        if (parameter_status != IHM_COMMAND_OK)
        {
            copy_text(error, error_size,
                      parameter_status == IHM_COMMAND_INVALID_ARGUMENT ?
                      "PARAMETER_OUT_OF_RANGE" : "PARAMETER_UPDATE_FAILED");
            (void)snprintf(message, message_size,
                           "Parametro %s recusado: %s",
                           code->valuestring,
                           ihm_command_status_to_string(parameter_status));
            return false;
        }

        if ((parameter_id != IHM_PARAM_P91) && !wait_operational())
        {
            copy_text(error, error_size, "PARAMETER_SYNC_FAILED");
            (void)snprintf(message, message_size,
                           "Parametro %s salvo, mas a sincronizacao com o inversor falhou",
                           code->valuestring);
            return false;
        }
        (void)snprintf(message, message_size,
                       "Parametro %s atualizado para %u",
                       code->valuestring, (unsigned int)value->valuedouble);
        return true;
    }
    else if ((strcmp(command->type, "set-pump") == 0) ||
             (strcmp(command->type, "set-swing") == 0))
    {
        enabled = cJSON_GetObjectItemCaseSensitive(command->payload, "enabled");
        if (!cJSON_IsBool(enabled))
        {
            copy_text(error, error_size, "INVALID_PERIPHERAL_STATE");
            copy_text(message, message_size, "Estado do periferico invalido");
            return false;
        }
        if (strcmp(command->type, "set-pump") == 0)
        {
            control = cJSON_IsTrue(enabled) ?
                      REG_CONTROL_PUMP_ON : REG_CONTROL_PUMP_OFF;
            copy_text(message, message_size,
                      cJSON_IsTrue(enabled) ? "Bomba ligada" : "Bomba desligada");
        }
        else
        {
            control = cJSON_IsTrue(enabled) ?
                      REG_CONTROL_SWING_ON : REG_CONTROL_SWING_OFF;
            copy_text(message, message_size,
                      cJSON_IsTrue(enabled) ? "Swing ligado" : "Swing desligado");
        }
    }
    else if (strcmp(command->type, "run-drain") == 0)
    {
        control = REG_CONTROL_DRY_START;
        copy_text(message, message_size, "Rotina de secagem/exaustao iniciada");
    }
    else if (strcmp(command->type, "stop-drain") == 0)
    {
        control = REG_CONTROL_CYCLE_STOP;
        copy_text(message, message_size, "Rotina interrompida");
    }
    else
    {
        copy_text(error, error_size, "UNSUPPORTED_COMMAND");
        copy_text(message, message_size, "Comando nao suportado por este firmware");
        return false;
    }

    return execute_write(command, REG_CONTROL_COMMAND, control,
                         error, error_size, message, message_size);
}

static esp_err_t build_ack(const parsed_command_t *command,
                           bool applied,
                           const char *error_code,
                           const char *error_text,
                           char **ack_json)
{
    parameter_cache_snapshot_t cache;
    comm_diagnostics_snapshot_t comm;
    ihm_parameter_blob_t parameters;
    remote_runtime_t runtime;
    cJSON *root = create_metadata();
    cJSON *state;

    if ((root == NULL) || (ack_json == NULL))
    {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(root, "id", command->id);
    cJSON_AddStringToObject(root, "type", command->type);
    cJSON_AddBoolToObject(root, "accepted", applied);
    cJSON_AddBoolToObject(root, "applied", applied);
    cJSON_AddStringToObject(root, "status", applied ? "applied" : "failed");
    get_snapshots(&cache, &comm, &parameters, &runtime);
    state = create_state_object(&cache, &comm, &parameters, &runtime);
    cJSON_AddItemToObject(root, "state", state);
    if (!applied)
    {
        char error_id[96];
        (void)snprintf(error_id, sizeof(error_id), "error-%s", command->id);
        cJSON_AddItemToObject(root, "error",
                             create_error_item(error_id, runtime.device_id,
                                               error_code, error_text, true));
    }
    else
    {
        cJSON_AddNullToObject(root, "error");
    }
    *ack_json = finish_json(root);
    return *ack_json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t remote_protocol_handle_command(const char *json,
                                         bool wrapped,
                                         char **ack_json,
                                         bool *applied)
{
    cJSON *root;
    parsed_command_t command;
    bool command_applied;
    char error[48] = {0};
    char message[REMOTE_EVENT_TEXT_MAX + 1U] = {0};
    esp_err_t result;

    if ((json == NULL) || (ack_json == NULL) || (applied == NULL) ||
        (s_command_lock == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }
    *ack_json = NULL;
    *applied = false;
    root = cJSON_Parse(json);
    if ((root == NULL) || !parse_command(root, wrapped, &command))
    {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_command_lock,
                       pdMS_TO_TICKS(REMOTE_E08_WAIT_MS +
                                     REMOTE_SYNC_WAIT_MS +
                                     REMOTE_COMMAND_WAIT_MS)) != pdTRUE)
    {
        cJSON_Delete(root);
        return ESP_ERR_TIMEOUT;
    }
    set_command_runtime(&command, REMOTE_COMMAND_SENDING, NULL,
                        "Comando recebido");
    command_applied = execute_command(&command, error, sizeof(error),
                                      message, sizeof(message));
    set_command_runtime(&command,
                        command_applied ? REMOTE_COMMAND_APPLIED :
                                          REMOTE_COMMAND_FAILED,
                        command_applied ? NULL : error, message);
    result = build_ack(&command, command_applied, error, message, ack_json);
    (void)xSemaphoreGive(s_command_lock);
    cJSON_Delete(root);
    *applied = command_applied;
    return result;
}

static bool schedules_payload_valid(cJSON *root)
{
    const cJSON *schema = cJSON_GetObjectItemCaseSensitive(root, "schema");
    const cJSON *schedules = cJSON_GetObjectItemCaseSensitive(root, "schedules");
    const cJSON *item;
    int count;

    if (!cJSON_IsObject(root) || !cJSON_IsString(schema) ||
        (strcmp(schema->valuestring, REMOTE_PROTOCOL_SCHEMA) != 0) ||
        !cJSON_IsArray(schedules))
    {
        return false;
    }
    count = cJSON_GetArraySize(schedules);
    if ((count < 0) || (count > 16))
    {
        return false;
    }
    cJSON_ArrayForEach(item, schedules)
    {
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(item, "id");
        const cJSON *type = cJSON_GetObjectItemCaseSensitive(item, "type");
        const cJSON *recurrence = cJSON_GetObjectItemCaseSensitive(item, "recurrence");
        const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(item, "enabled");
        const cJSON *time_value = cJSON_GetObjectItemCaseSensitive(item, "time");
        const cJSON *days = cJSON_GetObjectItemCaseSensitive(item, "daysOfWeek");
        if (!cJSON_IsObject(item) || !cJSON_IsString(id) ||
            !cJSON_IsString(type) || !cJSON_IsString(recurrence) ||
            !cJSON_IsBool(enabled) || !cJSON_IsString(time_value) ||
            !cJSON_IsArray(days))
        {
            return false;
        }
    }
    return true;
}

bool remote_protocol_is_own_schedules_payload(const char *json)
{
    bool own = false;
    cJSON *root = json != NULL ? cJSON_Parse(json) : NULL;
    const cJSON *source;

    if (root != NULL)
    {
        source = cJSON_GetObjectItemCaseSensitive(root, "source");
        own = cJSON_IsString(source) && (strcmp(source->valuestring, "ihm") == 0);
    }
    cJSON_Delete(root);
    return own;
}

esp_err_t remote_protocol_handle_schedules(const char *json,
                                           char **response_json,
                                           bool *changed)
{
    cJSON *root;
    cJSON *canonical;
    cJSON *schedules;
    cJSON *revision;
    char *serialized;
    char timestamp[32];
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    nvs_handle_t nvs;
    esp_err_t error;

    if ((json == NULL) || (response_json == NULL) || (changed == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }
    *response_json = NULL;
    *changed = false;
    root = cJSON_Parse(json);
    if ((root == NULL) || !schedules_payload_valid(root))
    {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }
    canonical = create_metadata();
    schedules = cJSON_Duplicate(cJSON_GetObjectItemCaseSensitive(root, "schedules"), true);
    revision = cJSON_GetObjectItemCaseSensitive(root, "revision");
    cJSON_AddItemToObject(canonical, "schedules", schedules);
    cJSON_AddStringToObject(canonical, "revision",
                            cJSON_IsString(revision) ? revision->valuestring : "1");
    cJSON_AddStringToObject(canonical, "timezone", "America/Sao_Paulo");
    cJSON_AddNumberToObject(canonical, "timezoneOffsetMinutes", -180);
    serialized = finish_json(canonical);
    cJSON_Delete(root);
    if ((serialized == NULL) || (strlen(serialized) > REMOTE_SCHEDULES_MAX_JSON))
    {
        free(serialized);
        return ESP_ERR_INVALID_SIZE;
    }
    error = nvs_open(REMOTE_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (error == ESP_OK)
    {
        error = nvs_set_str(nvs, REMOTE_NVS_SCHEDULES_KEY, serialized);
        if (error == ESP_OK)
        {
            error = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (error != ESP_OK)
    {
        free(serialized);
        return error;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    free(s_schedules_json);
    s_schedules_json = serialized;
    *response_json = strdup(s_schedules_json);
    (void)xSemaphoreGive(s_lock);
    *changed = true;
    format_timestamp(timestamp, sizeof(timestamp));
    remote_protocol_get_device_id(device_id, sizeof(device_id));
    ESP_LOGI(TAG, "Agendamentos sincronizados para %s em %s",
             device_id, timestamp);
    return *response_json != NULL ? ESP_OK : ESP_ERR_NO_MEM;
}

static void load_schedules(void)
{
    nvs_handle_t nvs = 0U;
    size_t size = 0U;

    if ((nvs_open(REMOTE_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) ||
        (nvs_get_str(nvs, REMOTE_NVS_SCHEDULES_KEY, NULL, &size) != ESP_OK) ||
        (size == 0U) || (size > REMOTE_SCHEDULES_MAX_JSON + 1U))
    {
        if (nvs != 0U)
        {
            nvs_close(nvs);
        }
        return;
    }
    s_schedules_json = calloc(size, 1U);
    if (s_schedules_json != NULL)
    {
        if (nvs_get_str(nvs, REMOTE_NVS_SCHEDULES_KEY,
                        s_schedules_json, &size) != ESP_OK)
        {
            free(s_schedules_json);
            s_schedules_json = NULL;
        }
    }
    nvs_close(nvs);
}

esp_err_t remote_protocol_init(void)
{
    if (s_lock != NULL)
    {
        return ESP_OK;
    }
    s_lock = xSemaphoreCreateMutex();
    s_command_lock = xSemaphoreCreateMutex();
    if ((s_lock == NULL) || (s_command_lock == NULL))
    {
        return ESP_ERR_NO_MEM;
    }
    (void)memset(&s_runtime, 0, sizeof(s_runtime));
    copy_text(s_runtime.device_id, sizeof(s_runtime.device_id), "AXON-IHM");
    copy_text(s_runtime.last_event_title, sizeof(s_runtime.last_event_title),
              "Firmware iniciado");
    copy_text(s_runtime.last_event_message, sizeof(s_runtime.last_event_message),
              "MQTT e API local inicializados");
    s_runtime.event_sequence = esp_random();
    load_schedules();
    return ESP_OK;
}
