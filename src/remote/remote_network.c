/**
 * @file remote_network.c
 * @brief Acesso remoto sem interferir na task Modbus validada.
 */

#include "remote_network.h"

#include "remote_protocol.h"

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define REMOTE_AP_SSID                 "AXON-IHM-SETUP"
#define REMOTE_HTTP_PORT               8080U
#define REMOTE_MQTT_URI                "mqtts://b11f6b00.ala.eu-central-1.emqxsl.com"
#define REMOTE_MQTT_PORT               8883U
#define REMOTE_MQTT_USERNAME           "axon-app"
#define REMOTE_MQTT_PASSWORD           "axon42"
#define REMOTE_MQTT_TOPIC_PREFIX       "axon/ihm"
#define REMOTE_MQTT_KEEPALIVE_SECONDS  15U
#define REMOTE_TOPIC_MAX               192U
#define REMOTE_MQTT_PAYLOAD_MAX        8192U
#define REMOTE_HTTP_BODY_MAX           8192U
#define REMOTE_WIFI_SSID_MAX           32U
#define REMOTE_WIFI_PASSWORD_MAX       64U
#define REMOTE_DEVICE_ID_MAX           48U
#define REMOTE_WIFI_RETRY_MAX          8U
#define REMOTE_PUBLISH_PERIOD_MS       5000U
#define REMOTE_FULL_PUBLISH_PERIOD_MS  60000U
#define REMOTE_NVS_NAMESPACE           "remote_access"
#define REMOTE_NVS_DEVICE_KEY          "device_id"
#define REMOTE_NVS_SSID_KEY            "wifi_ssid"
#define REMOTE_NVS_PASSWORD_KEY        "wifi_pass"

typedef enum
{
    PROVISIONING_IDLE = 0,
    PROVISIONING_PENDING,
    PROVISIONING_SUCCESS,
    PROVISIONING_FAILED
} provisioning_state_t;

typedef struct
{
    char *topic;
    char *payload;
} mqtt_work_item_t;

static const char *TAG = "remote_network";
static bool s_station_connected;
static bool s_access_point_active;
static bool s_mqtt_connected;
static bool s_started;
static uint8_t s_wifi_retries;
static provisioning_state_t s_provisioning_state;
static char s_provisioning_message[160];
static char s_station_ssid[REMOTE_WIFI_SSID_MAX + 1U];
static char s_station_password[REMOTE_WIFI_PASSWORD_MAX + 1U];
static esp_mqtt_client_handle_t s_mqtt_client;
static httpd_handle_t s_http_server;
static QueueHandle_t s_mqtt_work_queue;
static SemaphoreHandle_t s_network_lock;
static char s_fragment_topic[REMOTE_TOPIC_MAX + 1U];
static char s_fragment_payload[REMOTE_MQTT_PAYLOAD_MAX + 1U];
static int s_fragment_total;
static int s_fragment_received;

static void copy_text(char *destination, size_t size, const char *source)
{
    if ((destination == NULL) || (size == 0U))
    {
        return;
    }
    (void)snprintf(destination, size, "%s", source != NULL ? source : "");
}

static void set_provisioning(provisioning_state_t state, const char *message)
{
    (void)xSemaphoreTake(s_network_lock, portMAX_DELAY);
    s_provisioning_state = state;
    copy_text(s_provisioning_message, sizeof(s_provisioning_message), message);
    (void)xSemaphoreGive(s_network_lock);
}

static void update_protocol_network_state(void)
{
    remote_protocol_set_network_state(s_station_connected,
                                      s_access_point_active,
                                      s_mqtt_connected);
}

static bool read_nvs_string(const char *key, char *buffer, size_t buffer_size)
{
    nvs_handle_t nvs;
    size_t size = buffer_size;
    esp_err_t error;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return false;
    }
    error = nvs_open(REMOTE_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (error != ESP_OK)
    {
        return false;
    }
    error = nvs_get_str(nvs, key, buffer, &size);
    nvs_close(nvs);
    if (error != ESP_OK)
    {
        buffer[0] = '\0';
        return false;
    }
    return true;
}

static esp_err_t write_nvs_credentials(const char *device_id,
                                       const char *ssid,
                                       const char *password)
{
    nvs_handle_t nvs;
    esp_err_t error = nvs_open(REMOTE_NVS_NAMESPACE, NVS_READWRITE, &nvs);

    if (error != ESP_OK)
    {
        return error;
    }
    error = nvs_set_str(nvs, REMOTE_NVS_DEVICE_KEY, device_id);
    if (error == ESP_OK)
    {
        error = nvs_set_str(nvs, REMOTE_NVS_SSID_KEY, ssid);
    }
    if (error == ESP_OK)
    {
        error = nvs_set_str(nvs, REMOTE_NVS_PASSWORD_KEY,
                            password != NULL ? password : "");
    }
    if (error == ESP_OK)
    {
        error = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return error;
}

static bool valid_device_id(const char *value)
{
    size_t index;
    size_t length = value != NULL ? strlen(value) : 0U;

    if ((length == 0U) || (length > REMOTE_DEVICE_ID_MAX))
    {
        return false;
    }
    for (index = 0U; index < length; index++)
    {
        const char c = value[index];
        if (!(((c >= 'a') && (c <= 'z')) ||
              ((c >= 'A') && (c <= 'Z')) ||
              ((c >= '0') && (c <= '9')) || (c == '-') || (c == '_')))
        {
            return false;
        }
    }
    return true;
}

static void build_topics(const char *device_id,
                         char *commands,
                         char *schedules,
                         char *status,
                         char *state,
                         char *capabilities,
                         char *events,
                         char *errors)
{
    if (commands != NULL)
    {
        (void)snprintf(commands, REMOTE_TOPIC_MAX + 1U, "%s/%s/commands",
                       REMOTE_MQTT_TOPIC_PREFIX, device_id);
    }
    if (schedules != NULL)
    {
        (void)snprintf(schedules, REMOTE_TOPIC_MAX + 1U, "%s/%s/schedules",
                       REMOTE_MQTT_TOPIC_PREFIX, device_id);
    }
    if (status != NULL)
    {
        (void)snprintf(status, REMOTE_TOPIC_MAX + 1U, "%s/%s/status",
                       REMOTE_MQTT_TOPIC_PREFIX, device_id);
    }
    if (state != NULL)
    {
        (void)snprintf(state, REMOTE_TOPIC_MAX + 1U, "%s/%s/state",
                       REMOTE_MQTT_TOPIC_PREFIX, device_id);
    }
    if (capabilities != NULL)
    {
        (void)snprintf(capabilities, REMOTE_TOPIC_MAX + 1U,
                       "%s/%s/capabilities", REMOTE_MQTT_TOPIC_PREFIX,
                       device_id);
    }
    if (events != NULL)
    {
        (void)snprintf(events, REMOTE_TOPIC_MAX + 1U, "%s/%s/events",
                       REMOTE_MQTT_TOPIC_PREFIX, device_id);
    }
    if (errors != NULL)
    {
        (void)snprintf(errors, REMOTE_TOPIC_MAX + 1U, "%s/%s/errors",
                       REMOTE_MQTT_TOPIC_PREFIX, device_id);
    }
}

static esp_err_t mqtt_publish_topic(const char *topic,
                                    const char *json,
                                    bool retained)
{
    int id;

    if (!s_mqtt_connected || (s_mqtt_client == NULL) ||
        (topic == NULL) || (json == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }
    id = esp_mqtt_client_publish(s_mqtt_client, topic, json, 0, 1,
                                 retained ? 1 : 0);
    return id >= 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t mqtt_publish_kind(remote_json_kind_t kind, bool retained)
{
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    char topic[REMOTE_TOPIC_MAX + 1U];
    char *json = NULL;
    esp_err_t error;

    remote_protocol_get_device_id(device_id, sizeof(device_id));
    switch (kind)
    {
        case REMOTE_JSON_STATUS:
            build_topics(device_id, NULL, NULL, topic, NULL, NULL, NULL, NULL);
            break;
        case REMOTE_JSON_STATE:
            build_topics(device_id, NULL, NULL, NULL, topic, NULL, NULL, NULL);
            break;
        case REMOTE_JSON_CAPABILITIES:
            build_topics(device_id, NULL, NULL, NULL, NULL, topic, NULL, NULL);
            break;
        case REMOTE_JSON_EVENTS:
            build_topics(device_id, NULL, NULL, NULL, NULL, NULL, topic, NULL);
            break;
        case REMOTE_JSON_ERRORS:
            build_topics(device_id, NULL, NULL, NULL, NULL, NULL, NULL, topic);
            break;
        case REMOTE_JSON_SCHEDULES:
            build_topics(device_id, NULL, topic, NULL, NULL, NULL, NULL, NULL);
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
    error = remote_protocol_build_json(kind, &json);
    if (error == ESP_OK)
    {
        error = mqtt_publish_topic(topic, json, retained);
    }
    free(json);
    return error;
}

static void mqtt_publish_operational_snapshot(bool full)
{
    if (!s_mqtt_connected)
    {
        return;
    }
    (void)mqtt_publish_kind(REMOTE_JSON_STATUS, true);
    (void)mqtt_publish_kind(REMOTE_JSON_STATE, true);
    (void)mqtt_publish_kind(REMOTE_JSON_ERRORS, false);
    if (full)
    {
        (void)mqtt_publish_kind(REMOTE_JSON_CAPABILITIES, true);
        (void)mqtt_publish_kind(REMOTE_JSON_EVENTS, false);
        (void)mqtt_publish_kind(REMOTE_JSON_SCHEDULES, true);
    }
}

static void mqtt_queue_message(const char *topic, const char *payload)
{
    mqtt_work_item_t item = {0};

    if ((s_mqtt_work_queue == NULL) || (topic == NULL) || (payload == NULL))
    {
        return;
    }
    item.topic = strdup(topic);
    item.payload = strdup(payload);
    if ((item.topic == NULL) || (item.payload == NULL) ||
        (xQueueSend(s_mqtt_work_queue, &item, 0U) != pdTRUE))
    {
        free(item.topic);
        free(item.payload);
        ESP_LOGW(TAG, "Mensagem MQTT descartada por falta de memoria/fila");
    }
}

static void process_complete_mqtt_message(void)
{
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    char commands[REMOTE_TOPIC_MAX + 1U];
    char schedules[REMOTE_TOPIC_MAX + 1U];

    remote_protocol_get_device_id(device_id, sizeof(device_id));
    build_topics(device_id, commands, schedules, NULL, NULL, NULL, NULL, NULL);
    if ((strcmp(s_fragment_topic, commands) == 0) ||
        (strcmp(s_fragment_topic, schedules) == 0))
    {
        mqtt_queue_message(s_fragment_topic, s_fragment_payload);
    }
}

static void mqtt_data_fragment(esp_mqtt_event_handle_t event)
{
    if (event->current_data_offset == 0)
    {
        if ((event->total_data_len <= 0) ||
            (event->total_data_len > (int)REMOTE_MQTT_PAYLOAD_MAX) ||
            (event->topic_len <= 0) ||
            (event->topic_len > (int)REMOTE_TOPIC_MAX))
        {
            s_fragment_total = 0;
            s_fragment_received = 0;
            ESP_LOGW(TAG, "Mensagem MQTT excede os limites do firmware");
            return;
        }
        (void)memcpy(s_fragment_topic, event->topic, (size_t)event->topic_len);
        s_fragment_topic[event->topic_len] = '\0';
        s_fragment_total = event->total_data_len;
        s_fragment_received = 0;
    }
    if ((s_fragment_total <= 0) ||
        (event->current_data_offset != s_fragment_received) ||
        ((event->current_data_offset + event->data_len) > s_fragment_total))
    {
        s_fragment_total = 0;
        s_fragment_received = 0;
        return;
    }
    (void)memcpy(s_fragment_payload + event->current_data_offset,
                 event->data, (size_t)event->data_len);
    s_fragment_received += event->data_len;
    if (s_fragment_received == s_fragment_total)
    {
        s_fragment_payload[s_fragment_total] = '\0';
        process_complete_mqtt_message();
        s_fragment_total = 0;
        s_fragment_received = 0;
    }
}

static void mqtt_event_handler(void *args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    char commands[REMOTE_TOPIC_MAX + 1U];
    char schedules[REMOTE_TOPIC_MAX + 1U];

    (void)args;
    (void)base;
    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            s_mqtt_connected = true;
            update_protocol_network_state();
            remote_protocol_get_device_id(device_id, sizeof(device_id));
            build_topics(device_id, commands, schedules, NULL, NULL, NULL, NULL, NULL);
            (void)esp_mqtt_client_subscribe(s_mqtt_client, commands, 1);
            (void)esp_mqtt_client_subscribe(s_mqtt_client, schedules, 1);
            ESP_LOGI(TAG, "MQTT conectado; comandos ativos para %s", device_id);
            mqtt_publish_operational_snapshot(true);
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt_connected = false;
            update_protocol_network_state();
            ESP_LOGW(TAG, "MQTT desconectado; AP local permanece disponivel");
            break;
        case MQTT_EVENT_DATA:
            mqtt_data_fragment(event);
            break;
        case MQTT_EVENT_ERROR:
            s_mqtt_connected = false;
            update_protocol_network_state();
            ESP_LOGW(TAG, "Falha no transporte MQTT");
            break;
        default:
            break;
    }
}

static void mqtt_stop(void)
{
    if (s_mqtt_client != NULL)
    {
        (void)esp_mqtt_client_stop(s_mqtt_client);
        (void)esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }
    s_mqtt_connected = false;
    update_protocol_network_state();
}

static esp_err_t mqtt_start(void)
{
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    char client_id[80];
    char status_topic[REMOTE_TOPIC_MAX + 1U];
    char last_will[384];
    esp_mqtt_client_config_t config = {0};

    if (!s_station_connected || (s_mqtt_client != NULL))
    {
        return ESP_OK;
    }
    remote_protocol_get_device_id(device_id, sizeof(device_id));
    (void)snprintf(client_id, sizeof(client_id), "axon-ihm-%s", device_id);
    build_topics(device_id, NULL, NULL, status_topic, NULL, NULL, NULL, NULL);
    (void)snprintf(last_will, sizeof(last_will),
                   "{\"schema\":\"%s\",\"deviceId\":\"%s\","
                   "\"timestamp\":\"1970-01-01T00:00:00Z\",\"source\":\"ihm\","
                   "\"status\":{\"deviceOnline\":false,\"connectionMode\":\"cloud\","
                   "\"lastSeen\":null,\"readyState\":\"offline\"}}",
                   REMOTE_PROTOCOL_SCHEMA, device_id);
    config.broker.address.uri = REMOTE_MQTT_URI;
    config.broker.address.port = REMOTE_MQTT_PORT;
    config.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    config.credentials.client_id = client_id;
    config.credentials.username = REMOTE_MQTT_USERNAME;
    config.credentials.authentication.password = REMOTE_MQTT_PASSWORD;
    config.session.keepalive = REMOTE_MQTT_KEEPALIVE_SECONDS;
    config.session.last_will.topic = status_topic;
    config.session.last_will.msg = last_will;
    config.session.last_will.qos = 1;
    config.session.last_will.retain = 1;
    config.network.reconnect_timeout_ms = 5000;
    s_mqtt_client = esp_mqtt_client_init(&config);
    if (s_mqtt_client == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                       mqtt_event_handler, NULL));
    {
        const esp_err_t error = esp_mqtt_client_start(s_mqtt_client);
        if (error != ESP_OK)
        {
            (void)esp_mqtt_client_destroy(s_mqtt_client);
            s_mqtt_client = NULL;
            return error;
        }
    }
    return ESP_OK;
}

static void wifi_event_handler(void *args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    (void)args;
    (void)base;
    (void)event_data;
    switch (event_id)
    {
        case WIFI_EVENT_STA_START:
            if (s_station_ssid[0] != '\0')
            {
                (void)esp_wifi_connect();
            }
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            s_station_connected = false;
            mqtt_stop();
            update_protocol_network_state();
            if ((s_station_ssid[0] != '\0') &&
                (s_wifi_retries < REMOTE_WIFI_RETRY_MAX))
            {
                s_wifi_retries++;
                (void)esp_wifi_connect();
            }
            else if (s_provisioning_state == PROVISIONING_PENDING)
            {
                set_provisioning(PROVISIONING_FAILED,
                                 "Nao foi possivel conectar ao Wi-Fi informado");
            }
            break;
        case WIFI_EVENT_AP_START:
            s_access_point_active = true;
            update_protocol_network_state();
            ESP_LOGI(TAG, "AP %s ativo em 192.168.4.1", REMOTE_AP_SSID);
            break;
        case WIFI_EVENT_AP_STOP:
            s_access_point_active = false;
            update_protocol_network_state();
            break;
        default:
            break;
    }
}

static void ip_event_handler(void *args,
                             esp_event_base_t base,
                             int32_t event_id,
                             void *event_data)
{
    (void)args;
    (void)base;
    (void)event_data;
    if (event_id == IP_EVENT_STA_GOT_IP)
    {
        s_station_connected = true;
        s_wifi_retries = 0U;
        update_protocol_network_state();
        if (s_provisioning_state == PROVISIONING_PENDING)
        {
            set_provisioning(PROVISIONING_SUCCESS,
                             "Wi-Fi conectado e configuracao salva");
        }
        ESP_LOGI(TAG, "Wi-Fi STA conectado; iniciando MQTT seguro");
        if (mqtt_start() != ESP_OK)
        {
            ESP_LOGW(TAG, "Nao foi possivel iniciar o cliente MQTT");
        }
    }
}

static esp_err_t configure_station(const char *ssid, const char *password)
{
    wifi_config_t station = {0};

    copy_text((char *)station.sta.ssid, sizeof(station.sta.ssid), ssid);
    copy_text((char *)station.sta.password, sizeof(station.sta.password), password);
    station.sta.threshold.authmode = password[0] != '\0' ?
                                     WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    station.sta.pmf_cfg.capable = true;
    station.sta.pmf_cfg.required = false;
    return esp_wifi_set_config(WIFI_IF_STA, &station);
}

static esp_err_t provision_wifi(const char *device_id,
                                const char *ssid,
                                const char *password)
{
    esp_err_t error;

    if (!valid_device_id(device_id) || (ssid == NULL) ||
        (strlen(ssid) == 0U) || (strlen(ssid) > REMOTE_WIFI_SSID_MAX) ||
        ((password != NULL) && (strlen(password) > REMOTE_WIFI_PASSWORD_MAX)))
    {
        return ESP_ERR_INVALID_ARG;
    }
    error = write_nvs_credentials(device_id, ssid,
                                  password != NULL ? password : "");
    if (error != ESP_OK)
    {
        return error;
    }
    mqtt_stop();
    remote_protocol_set_device_id(device_id);
    copy_text(s_station_ssid, sizeof(s_station_ssid), ssid);
    copy_text(s_station_password, sizeof(s_station_password),
              password != NULL ? password : "");
    s_station_connected = false;
    s_wifi_retries = 0U;
    update_protocol_network_state();
    set_provisioning(PROVISIONING_PENDING, "Conectando ao Wi-Fi informado");
    (void)esp_wifi_disconnect();
    error = configure_station(s_station_ssid, s_station_password);
    if (error == ESP_OK)
    {
        error = esp_wifi_connect();
    }
    if (error != ESP_OK)
    {
        set_provisioning(PROVISIONING_FAILED,
                         "Falha ao aplicar a configuracao de Wi-Fi");
    }
    return error;
}

static void mqtt_worker_task(void *argument)
{
    mqtt_work_item_t item;

    (void)argument;
    for (;;)
    {
        if (xQueueReceive(s_mqtt_work_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            char device_id[REMOTE_DEVICE_ID_MAX + 1U];
            char commands[REMOTE_TOPIC_MAX + 1U];
            char schedules[REMOTE_TOPIC_MAX + 1U];

            remote_protocol_get_device_id(device_id, sizeof(device_id));
            build_topics(device_id, commands, schedules, NULL, NULL, NULL, NULL, NULL);
            if (strcmp(item.topic, commands) == 0)
            {
                char *ack = NULL;
                bool applied = false;
                if (remote_protocol_handle_command(item.payload, false,
                                                   &ack, &applied) == ESP_OK)
                {
                    char ack_topic[REMOTE_TOPIC_MAX + 1U];
                    if (applied)
                    {
                        build_topics(device_id, NULL, NULL, NULL, NULL, NULL,
                                     ack_topic, NULL);
                    }
                    else
                    {
                        build_topics(device_id, NULL, NULL, NULL, NULL, NULL,
                                     NULL, ack_topic);
                    }
                    (void)mqtt_publish_topic(ack_topic, ack, false);
                    /* Inclui capabilities/schedules quando o app os solicita. */
                    mqtt_publish_operational_snapshot(true);
                }
                else
                {
                    ESP_LOGW(TAG, "Comando MQTT invalido ou destinado a outra IHM");
                }
                free(ack);
            }
            else if ((strcmp(item.topic, schedules) == 0) &&
                     !remote_protocol_is_own_schedules_payload(item.payload))
            {
                char *response = NULL;
                bool changed = false;
                if ((remote_protocol_handle_schedules(item.payload, &response,
                                                       &changed) == ESP_OK) &&
                    changed)
                {
                    (void)mqtt_publish_topic(schedules, response, true);
                }
                free(response);
            }
            free(item.topic);
            free(item.payload);
        }
    }
}

static void publisher_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t elapsed = 0U;

    (void)argument;
    for (;;)
    {
        const bool full = elapsed >= REMOTE_FULL_PUBLISH_PERIOD_MS;
        mqtt_publish_operational_snapshot(full);
        elapsed = full ? 0U : elapsed + REMOTE_PUBLISH_PERIOD_MS;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(REMOTE_PUBLISH_PERIOD_MS));
    }
}

static void schedule_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;)
    {
        (void)remote_protocol_process_schedules(time(NULL));
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U));
    }
}

static void set_json_headers(httpd_req_t *request)
{
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
}

static esp_err_t send_json(httpd_req_t *request,
                           const char *status,
                           char *json)
{
    esp_err_t error;

    set_json_headers(request);
    httpd_resp_set_status(request, status);
    error = httpd_resp_sendstr(request, json != NULL ? json : "{}");
    free(json);
    return error;
}

static char *read_http_body(httpd_req_t *request)
{
    char *body;
    int received = 0;

    if ((request->content_len <= 0) ||
        (request->content_len > (int)REMOTE_HTTP_BODY_MAX))
    {
        return NULL;
    }
    body = calloc((size_t)request->content_len + 1U, 1U);
    if (body == NULL)
    {
        return NULL;
    }
    while (received < request->content_len)
    {
        const int count = httpd_req_recv(request, body + received,
                                         request->content_len - received);
        if (count <= 0)
        {
            free(body);
            return NULL;
        }
        received += count;
    }
    return body;
}

static esp_err_t simple_error(httpd_req_t *request,
                              const char *status,
                              const char *code,
                              const char *message)
{
    cJSON *root = cJSON_CreateObject();
    char *json;

    cJSON_AddBoolToObject(root, "ok", false);
    cJSON_AddStringToObject(root, "code", code);
    cJSON_AddStringToObject(root, "message", message);
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return send_json(request, status, json);
}

static esp_err_t snapshot_handler(httpd_req_t *request)
{
    remote_json_kind_t kind = (remote_json_kind_t)(uintptr_t)request->user_ctx;
    char *json = NULL;
    esp_err_t error = remote_protocol_build_json(kind, &json);

    return send_json(request,
                     error == ESP_OK ? "200 OK" : "500 Internal Server Error",
                     json);
}

static esp_err_t ping_handler(httpd_req_t *request)
{
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    char timestamp[32];
    time_t now;
    struct tm utc;
    cJSON *root = cJSON_CreateObject();
    char *json;

    remote_protocol_get_device_id(device_id, sizeof(device_id));
    now = time(NULL);
    if ((now > 0) && (gmtime_r(&now, &utc) != NULL))
    {
        (void)strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &utc);
    }
    else
    {
        (void)snprintf(timestamp, sizeof(timestamp), "1970-01-01T00:00:00Z");
    }
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "message", "Servidor local da IHM ativo");
    cJSON_AddStringToObject(root, "timestamp", timestamp);
    cJSON_AddStringToObject(root, "deviceId", device_id);
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return send_json(request, "200 OK", json);
}

static esp_err_t command_handler(httpd_req_t *request)
{
    char *body = read_http_body(request);
    char *ack = NULL;
    bool applied = false;
    esp_err_t error;

    if (body == NULL)
    {
        return simple_error(request, "400 Bad Request", "INVALID_BODY",
                            "Corpo JSON ausente ou muito grande");
    }
    error = remote_protocol_handle_command(body, true, &ack, &applied);
    free(body);
    if (error != ESP_OK)
    {
        free(ack);
        return simple_error(request, "400 Bad Request", "INVALID_COMMAND",
                            "Comando invalido ou destinado a outra IHM");
    }
    return send_json(request, "200 OK", ack);
}

static esp_err_t schedules_post_handler(httpd_req_t *request)
{
    char *body = read_http_body(request);
    char *response = NULL;
    bool changed = false;
    esp_err_t error;

    if (body == NULL)
    {
        return simple_error(request, "400 Bad Request", "INVALID_BODY",
                            "Corpo JSON ausente ou muito grande");
    }
    error = remote_protocol_handle_schedules(body, &response, &changed);
    free(body);
    if (error != ESP_OK)
    {
        free(response);
        return simple_error(request, "400 Bad Request", "INVALID_SCHEDULES",
                            "Agendamentos invalidos");
    }
    if (changed && s_mqtt_connected)
    {
        char device_id[REMOTE_DEVICE_ID_MAX + 1U];
        char topic[REMOTE_TOPIC_MAX + 1U];
        remote_protocol_get_device_id(device_id, sizeof(device_id));
        build_topics(device_id, NULL, topic, NULL, NULL, NULL, NULL, NULL);
        (void)mqtt_publish_topic(topic, response, true);
    }
    return send_json(request, "200 OK", response);
}

static char *build_provisioning_response(provisioning_state_t state,
                                         const char *message)
{
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];
    cJSON *root = cJSON_CreateObject();
    char *json;

    remote_protocol_get_device_id(device_id, sizeof(device_id));
    cJSON_AddBoolToObject(root, "ok", state != PROVISIONING_FAILED);
    cJSON_AddBoolToObject(root, "accepted", state != PROVISIONING_FAILED);
    cJSON_AddBoolToObject(root, "pending", state == PROVISIONING_PENDING);
    cJSON_AddBoolToObject(root, "success", state == PROVISIONING_SUCCESS);
    cJSON_AddBoolToObject(root, "restartRequired", false);
    cJSON_AddStringToObject(root, "status",
                            state == PROVISIONING_PENDING ? "pending" :
                            (state == PROVISIONING_SUCCESS ? "success" :
                             (state == PROVISIONING_FAILED ? "failed" : "idle")));
    cJSON_AddStringToObject(root, "message", message);
    cJSON_AddStringToObject(root, "deviceId", device_id);
    cJSON_AddStringToObject(root, "firmwareVersion", REMOTE_FIRMWARE_VERSION);
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

static esp_err_t provisioning_post_handler(httpd_req_t *request)
{
    char *body = read_http_body(request);
    cJSON *root;
    const cJSON *device_id;
    const cJSON *ssid;
    const cJSON *password;
    esp_err_t error;
    char *response;

    if (body == NULL)
    {
        return simple_error(request, "400 Bad Request", "INVALID_BODY",
                            "Dados de provisionamento ausentes");
    }
    root = cJSON_Parse(body);
    free(body);
    device_id = root != NULL ?
                cJSON_GetObjectItemCaseSensitive(root, "deviceId") : NULL;
    ssid = root != NULL ?
           cJSON_GetObjectItemCaseSensitive(root, "wifiSsid") : NULL;
    password = root != NULL ?
               cJSON_GetObjectItemCaseSensitive(root, "wifiPassword") : NULL;
    if (!cJSON_IsString(device_id) || !cJSON_IsString(ssid) ||
        ((password != NULL) && !cJSON_IsString(password)))
    {
        cJSON_Delete(root);
        return simple_error(request, "400 Bad Request", "INVALID_PROVISIONING",
                            "deviceId e wifiSsid sao obrigatorios");
    }
    error = provision_wifi(device_id->valuestring, ssid->valuestring,
                           cJSON_IsString(password) ? password->valuestring : "");
    cJSON_Delete(root);
    if (error != ESP_OK)
    {
        return simple_error(request, "400 Bad Request", "PROVISIONING_FAILED",
                            "Nao foi possivel aplicar os dados informados");
    }
    response = build_provisioning_response(PROVISIONING_PENDING,
                                           "Configuracao aceita; conectando ao Wi-Fi");
    return send_json(request, "202 Accepted", response);
}

static esp_err_t provisioning_status_handler(httpd_req_t *request)
{
    provisioning_state_t state;
    char message[sizeof(s_provisioning_message)];
    char *response;

    (void)xSemaphoreTake(s_network_lock, portMAX_DELAY);
    state = s_provisioning_state;
    copy_text(message, sizeof(message), s_provisioning_message);
    (void)xSemaphoreGive(s_network_lock);
    response = build_provisioning_response(state, message);
    return send_json(request, "200 OK", response);
}

static esp_err_t options_handler(httpd_req_t *request)
{
    set_json_headers(request);
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

static esp_err_t register_http_routes(void)
{
    const httpd_uri_t routes[] = {
        {.uri = "/api/v1/ping", .method = HTTP_GET, .handler = ping_handler},
        {.uri = "/api/v1/status", .method = HTTP_GET, .handler = snapshot_handler,
         .user_ctx = (void *)(uintptr_t)REMOTE_JSON_STATUS},
        {.uri = "/api/v1/state", .method = HTTP_GET, .handler = snapshot_handler,
         .user_ctx = (void *)(uintptr_t)REMOTE_JSON_STATE},
        {.uri = "/api/v1/capabilities", .method = HTTP_GET, .handler = snapshot_handler,
         .user_ctx = (void *)(uintptr_t)REMOTE_JSON_CAPABILITIES},
        {.uri = "/api/v1/diagnostics", .method = HTTP_GET, .handler = snapshot_handler,
         .user_ctx = (void *)(uintptr_t)REMOTE_JSON_DIAGNOSTICS},
        {.uri = "/api/v1/events", .method = HTTP_GET, .handler = snapshot_handler,
         .user_ctx = (void *)(uintptr_t)REMOTE_JSON_EVENTS},
        {.uri = "/api/v1/errors", .method = HTTP_GET, .handler = snapshot_handler,
         .user_ctx = (void *)(uintptr_t)REMOTE_JSON_ERRORS},
        {.uri = "/api/v1/schedules", .method = HTTP_GET, .handler = snapshot_handler,
         .user_ctx = (void *)(uintptr_t)REMOTE_JSON_SCHEDULES},
        {.uri = "/api/v1/commands", .method = HTTP_POST, .handler = command_handler},
        {.uri = "/api/v1/schedules", .method = HTTP_POST, .handler = schedules_post_handler},
        {.uri = "/api/v1/provisioning", .method = HTTP_POST,
         .handler = provisioning_post_handler},
        {.uri = "/api/v1/wifi/reconfigure", .method = HTTP_POST,
         .handler = provisioning_post_handler},
        {.uri = "/api/v1/provisioning/status", .method = HTTP_GET,
         .handler = provisioning_status_handler},
        {.uri = "/api/v1/*", .method = HTTP_OPTIONS, .handler = options_handler},
    };
    size_t index;

    for (index = 0U; index < sizeof(routes) / sizeof(routes[0]); index++)
    {
        const esp_err_t error = httpd_register_uri_handler(s_http_server,
                                                            &routes[index]);
        if (error != ESP_OK)
        {
            return error;
        }
    }
    return ESP_OK;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    esp_err_t error;

    config.server_port = REMOTE_HTTP_PORT;
    config.max_uri_handlers = 20;
    config.stack_size = 8192;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    config.uri_match_fn = httpd_uri_match_wildcard;
    error = httpd_start(&s_http_server, &config);
    if (error == ESP_OK)
    {
        error = register_http_routes();
    }
    if (error != ESP_OK)
    {
        if (s_http_server != NULL)
        {
            (void)httpd_stop(s_http_server);
            s_http_server = NULL;
        }
        return error;
    }
    ESP_LOGI(TAG, "API local pronta em http://192.168.4.1:%u",
             REMOTE_HTTP_PORT);
    return ESP_OK;
}

static void load_identity_and_wifi(void)
{
    char device_id[REMOTE_DEVICE_ID_MAX + 1U];

    if (!read_nvs_string(REMOTE_NVS_DEVICE_KEY, device_id,
                         sizeof(device_id)) || !valid_device_id(device_id))
    {
        uint8_t mac[6] = {0};
        (void)esp_read_mac(mac, ESP_MAC_WIFI_STA);
        (void)snprintf(device_id, sizeof(device_id), "AXON-%02X%02X%02X",
                       mac[3], mac[4], mac[5]);
    }
    remote_protocol_set_device_id(device_id);
    (void)read_nvs_string(REMOTE_NVS_SSID_KEY, s_station_ssid,
                          sizeof(s_station_ssid));
    (void)read_nvs_string(REMOTE_NVS_PASSWORD_KEY, s_station_password,
                          sizeof(s_station_password));
}

static esp_err_t start_wifi(void)
{
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t access_point = {0};
    esp_err_t error;

    error = esp_netif_init();
    if ((error != ESP_OK) && (error != ESP_ERR_INVALID_STATE))
    {
        return error;
    }
    error = esp_event_loop_create_default();
    if ((error != ESP_OK) && (error != ESP_ERR_INVALID_STATE))
    {
        return error;
    }
    (void)esp_netif_create_default_wifi_sta();
    (void)esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   wifi_event_handler, NULL));
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                   ip_event_handler, NULL));
    error = esp_wifi_init(&init);
    if (error != ESP_OK)
    {
        return error;
    }
    (void)esp_wifi_set_storage(WIFI_STORAGE_RAM);
    (void)esp_wifi_set_ps(WIFI_PS_NONE);
    error = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (error != ESP_OK)
    {
        return error;
    }
    copy_text((char *)access_point.ap.ssid, sizeof(access_point.ap.ssid),
              REMOTE_AP_SSID);
    access_point.ap.ssid_len = strlen(REMOTE_AP_SSID);
    access_point.ap.channel = 1U;
    access_point.ap.max_connection = 4U;
    access_point.ap.authmode = WIFI_AUTH_OPEN;
    error = esp_wifi_set_config(WIFI_IF_AP, &access_point);
    if ((error == ESP_OK) && (s_station_ssid[0] != '\0'))
    {
        error = configure_station(s_station_ssid, s_station_password);
    }
    if (error == ESP_OK)
    {
        error = esp_wifi_start();
    }
    if (error == ESP_OK)
    {
        const esp_sntp_config_t sntp =
            ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        const esp_err_t sntp_error = esp_netif_sntp_init(&sntp);
        if ((sntp_error != ESP_OK) && (sntp_error != ESP_ERR_INVALID_STATE))
        {
            ESP_LOGW(TAG, "SNTP nao iniciado: %s", esp_err_to_name(sntp_error));
        }
    }
    return error;
}

esp_err_t remote_network_start(void)
{
    BaseType_t task_result;
    esp_err_t error;

    if (s_started)
    {
        return ESP_ERR_INVALID_STATE;
    }
    error = remote_protocol_init();
    if (error != ESP_OK)
    {
        return error;
    }
    s_network_lock = xSemaphoreCreateMutex();
    s_mqtt_work_queue = xQueueCreate(4U, sizeof(mqtt_work_item_t));
    if ((s_network_lock == NULL) || (s_mqtt_work_queue == NULL))
    {
        return ESP_ERR_NO_MEM;
    }
    set_provisioning(PROVISIONING_IDLE,
                     "Aguardando configuracao ou usando dados salvos");
    load_identity_and_wifi();
    error = start_wifi();
    if (error != ESP_OK)
    {
        return error;
    }
    error = start_http_server();
    if (error != ESP_OK)
    {
        return error;
    }
    task_result = xTaskCreate(mqtt_worker_task, "mqtt_commands", 9216U,
                              NULL, 4U, NULL);
    if (task_result != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    task_result = xTaskCreate(publisher_task, "app_publisher", 6144U,
                              NULL, 3U, NULL);
    if (task_result != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    task_result = xTaskCreate(schedule_task, "app_schedules", 6144U,
                              NULL, 3U, NULL);
    if (task_result != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    ESP_LOGI(TAG, "Acesso por aplicativo iniciado (AP + MQTT)");
    return ESP_OK;
}
