/**
 * @file app.c
 * @brief Orquestração da comunicação Modbus RTU mestre.
 */

#include "app.h"

#include "comm_config.h"
#include "comm_diagnostics.h"
#include "console.h"
#include "ihm_command_service.h"
#include "ihm_parameters.h"
#include "parameter_cache.h"
#include "protocol/register_map.h"
#include "rs485_master.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    app_comm_request_t request;
    QueueHandle_t response_queue;
} queued_command_t;

static const char *TAG = "app";
static QueueHandle_t s_command_queue;
static portMUX_TYPE s_app_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_polling_enabled = true;
static bool s_started;
static bool s_first_valid_response_logged;
static uint64_t s_last_request_start_ms;
static app_sync_snapshot_t s_sync = {
    .state = APP_SYNC_IDLE,
    .last_error = APP_COMM_RESULT_OK,
};
static bool s_e08_activation_pending;
static bool s_e08_recovery_requested;

static void set_sync_error(app_comm_result_status_t error);

static uint64_t uptime_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000LL);
}

static TickType_t milliseconds_to_ticks_ceil(uint32_t milliseconds)
{
    TickType_t ticks = pdMS_TO_TICKS(milliseconds);

    if ((milliseconds > 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }
    return ticks;
}

static void wait_minimum_request_interval(void)
{
    const uint64_t now_ms = uptime_ms();
    const uint64_t elapsed_ms = now_ms - s_last_request_start_ms;

    if ((s_last_request_start_ms != 0U) &&
        (elapsed_ms < COMM_MIN_REQUEST_INTERVAL_MS))
    {
        vTaskDelay(milliseconds_to_ticks_ceil(
            (uint32_t)(COMM_MIN_REQUEST_INTERVAL_MS - elapsed_ms)));
    }
}

static bool request_is_valid(const app_comm_request_t *request)
{
    uint32_t last_address;

    if (request == NULL)
    {
        return false;
    }

    switch (request->type)
    {
        case APP_COMM_REQUEST_READ:
            if ((request->quantity == 0U) ||
                (request->quantity > MODBUS_READ_MAX_REGISTERS))
            {
                return false;
            }
            last_address = (uint32_t)request->address +
                           (uint32_t)request->quantity - 1U;
            return last_address <= UINT16_MAX;

        case APP_COMM_REQUEST_WRITE_SINGLE:
            return true;

        case APP_COMM_REQUEST_WRITE_MULTIPLE:
            if ((request->quantity == 0U) ||
                (request->quantity > MODBUS_WRITE_MAX_REGISTERS))
            {
                return false;
            }
            last_address = (uint32_t)request->address +
                           (uint32_t)request->quantity - 1U;
            return last_address <= UINT16_MAX;

        case APP_COMM_REQUEST_SET_POLLING:
            return true;

        default:
            return false;
    }
}

static bool prepare_modbus_request(const app_comm_request_t *source,
                                   modbus_request_t *destination)
{
    switch (source->type)
    {
        case APP_COMM_REQUEST_READ:
            return modbus_master_prepare_read(destination,
                                              source->address,
                                              source->quantity);

        case APP_COMM_REQUEST_WRITE_SINGLE:
            return modbus_master_prepare_write_single(destination,
                                                      source->address,
                                                      source->values[0]);

        case APP_COMM_REQUEST_WRITE_MULTIPLE:
            return modbus_master_prepare_write_multiple(destination,
                                                        source->address,
                                                        source->values,
                                                        source->quantity);

        default:
            return false;
    }
}

static app_comm_result_status_t map_transport_error(
    rs485_transfer_status_t status)
{
    if (status == RS485_TRANSFER_TIMEOUT)
    {
        comm_diagnostics_record_timeout();
        return APP_COMM_RESULT_TIMEOUT;
    }

    comm_diagnostics_record_protocol_error();
    return APP_COMM_RESULT_UART_ERROR;
}

static app_comm_result_status_t map_parse_error(
    modbus_parse_status_t status)
{
    if (status == MODBUS_PARSE_CRC_ERROR)
    {
        comm_diagnostics_record_crc_error();
        return APP_COMM_RESULT_CRC_ERROR;
    }

    if (status == MODBUS_PARSE_EXCEPTION)
    {
        comm_diagnostics_record_exception();
        return APP_COMM_RESULT_EXCEPTION;
    }

    if ((status == MODBUS_PARSE_INVALID_LENGTH) ||
        (status == MODBUS_PARSE_INVALID_BYTE_COUNT))
    {
        comm_diagnostics_record_length_error();
    }
    else if (status == MODBUS_PARSE_WRONG_ADDRESS)
    {
        comm_diagnostics_record_address_error();
    }
    else
    {
        comm_diagnostics_record_protocol_error();
    }
    return APP_COMM_RESULT_PROTOCOL_ERROR;
}

static app_comm_result_t execute_modbus_request(
    const app_comm_request_t *source)
{
    uint8_t request_frame[COMM_FRAME_MAX_SIZE];
    uint8_t response_frame[COMM_FRAME_MAX_SIZE];
    modbus_request_t request;
    modbus_response_t response;
    app_comm_result_t result;
    uint16_t request_length = 0U;
    uint8_t attempt;
    bool link_alive = false;

    (void)memset(&result, 0, sizeof(result));
    result.status = APP_COMM_RESULT_INVALID_ARGUMENT;

    if (!request_is_valid(source) ||
        !prepare_modbus_request(source, &request) ||
        (modbus_master_build_request(&request,
                                     request_frame,
                                     sizeof(request_frame),
                                     &request_length) != MODBUS_PARSE_OK))
    {
        return result;
    }

    for (attempt = 0U; attempt <= COMM_MAX_RETRIES; attempt++)
    {
        uint16_t response_length = 0U;
        uint64_t attempt_start_ms;
        rs485_transfer_status_t transfer_status;
        modbus_parse_status_t parse_status;

        if (attempt > 0U)
        {
            comm_diagnostics_record_retry();
        }

        wait_minimum_request_interval();
        s_last_request_start_ms = uptime_ms();
        attempt_start_ms = s_last_request_start_ms;
        comm_diagnostics_record_request_sent();
        ESP_LOG_BUFFER_HEXDUMP(TAG,
                               request_frame,
                               request_length,
                               ESP_LOG_DEBUG);

        transfer_status = rs485_master_transceive(
            request_frame,
            request_length,
            response_frame,
            sizeof(response_frame),
            &response_length,
            COMM_RESPONSE_TIMEOUT_MS);

        if (transfer_status != RS485_TRANSFER_OK)
        {
            result.status = map_transport_error(transfer_status);
            console_trace_submit(request_frame, request_length,
                                 NULL, 0U,
                                 (uint32_t)(uptime_ms() - attempt_start_ms),
                                 app_comm_result_to_string(result.status));
            continue;
        }

        ESP_LOG_BUFFER_HEXDUMP(TAG,
                               response_frame,
                               response_length,
                               ESP_LOG_DEBUG);
        parse_status = modbus_master_parse_response(&request,
                                                    response_frame,
                                                    response_length,
                                                    &response);
        if (parse_status != MODBUS_PARSE_OK)
        {
            result.status = map_parse_error(parse_status);
            console_trace_submit(request_frame, request_length,
                                 response_frame, response_length,
                                 (uint32_t)(uptime_ms() - attempt_start_ms),
                                 app_comm_result_to_string(result.status));
            if (parse_status == MODBUS_PARSE_EXCEPTION)
            {
                /*
                 * Uma excecao Modbus com endereco e CRC validos prova que o
                 * enlace respondeu. Ela e erro da operacao, nao perda RS485.
                 */
                comm_diagnostics_record_valid_response();
                comm_diagnostics_record_latency(
                    (uint32_t)(uptime_ms() - s_last_request_start_ms));
                link_alive = true;
                result.exception_code = response.exception_code;
                break;
            }
            continue;
        }

        comm_diagnostics_record_valid_response();
        comm_diagnostics_record_latency(
            (uint32_t)(uptime_ms() - s_last_request_start_ms));
        result.status = APP_COMM_RESULT_OK;
        result.quantity = response.quantity;
        if (response.quantity > 0U)
        {
            (void)memcpy(result.values,
                         response.values,
                         (size_t)response.quantity * sizeof(uint16_t));
        }
        link_alive = true;
        console_trace_submit(request_frame, request_length,
                             response_frame, response_length,
                             (uint32_t)(uptime_ms() - attempt_start_ms),
                             "OK");

        if (!s_first_valid_response_logged)
        {
            s_first_valid_response_logged = true;
            ESP_LOGI(TAG, "Primeira resposta Modbus válida recebida");
        }
        break;
    }

    comm_diagnostics_record_transaction(link_alive, uptime_ms());
    if (comm_diagnostics_should_activate_e08())
    {
        ihm_command_service_set_e08_active(true);
        s_e08_activation_pending = true;
    }
    return result;
}

static app_comm_result_t perform_read(uint16_t address, uint16_t quantity)
{
    app_comm_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = APP_COMM_REQUEST_READ;
    request.address = address;
    request.quantity = quantity;
    return execute_modbus_request(&request);
}

static app_comm_result_t perform_write(uint16_t address, uint16_t value)
{
    app_comm_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = APP_COMM_REQUEST_WRITE_SINGLE;
    request.address = address;
    request.quantity = 1U;
    request.values[0] = value;
    return execute_modbus_request(&request);
}

static app_comm_result_t perform_write_multiple(uint16_t address,
                                                const uint16_t *values,
                                                uint16_t quantity)
{
    app_comm_request_t request;

    (void)memset(&request, 0, sizeof(request));
    request.type = APP_COMM_REQUEST_WRITE_MULTIPLE;
    request.address = address;
    request.quantity = quantity;
    (void)memcpy(request.values,
                 values,
                 (size_t)quantity * sizeof(uint16_t));
    return execute_modbus_request(&request);
}

static bool validate_identity_register(const char *name,
                                       uint16_t address,
                                       uint16_t expected_value)
{
    const app_comm_result_t result = perform_read(address, 1U);

    if (result.status != APP_COMM_RESULT_OK)
    {
        ESP_LOGW(TAG,
                 "Falha ao ler %s: %s",
                 name,
                 app_comm_result_to_string(result.status));
        set_sync_error(result.status);
        return false;
    }
    else if (result.values[0] != expected_value)
    {
        ESP_LOGW(TAG,
                 "%s incompatível: recebido 0x%04" PRIX16
                 ", esperado 0x%04" PRIX16,
                 name,
                 result.values[0],
                 expected_value);
        set_sync_error(APP_COMM_RESULT_PROTOCOL_ERROR);
        return false;
    }
    return true;
}

static void set_sync_state(app_sync_state_t state, uint8_t step)
{
    portENTER_CRITICAL(&s_app_lock);
    s_sync.state = state;
    s_sync.step = step;
    portEXIT_CRITICAL(&s_app_lock);
}

static void set_sync_error(app_comm_result_status_t error)
{
    portENTER_CRITICAL(&s_app_lock);
    s_sync.last_error = error;
    portEXIT_CRITICAL(&s_app_lock);
}

static app_comm_result_status_t get_sync_error(void)
{
    app_comm_result_status_t error;
    portENTER_CRITICAL(&s_app_lock);
    error = s_sync.last_error;
    portEXIT_CRITICAL(&s_app_lock);
    return error;
}

static bool result_is_ok(app_comm_result_t result)
{
    if (result.status != APP_COMM_RESULT_OK)
    {
        set_sync_error(result.status);
        return false;
    }
    return true;
}

static bool send_parameter_block(const ihm_parameter_blob_t *parameters,
                                 ihm_parameter_id_t first,
                                 uint16_t quantity)
{
    uint16_t values[5];
    uint16_t index;

    for (index = 0U; index < quantity; index++)
    {
        values[index] = parameters->values[first + index];
    }
    return result_is_ok(perform_write_multiple(
        ihm_parameters_register_address(first), values, quantity));
}

static bool send_isolated_parameter(const ihm_parameter_blob_t *parameters,
                                    ihm_parameter_id_t id)
{
    return result_is_ok(perform_write(
        ihm_parameters_register_address(id), parameters->values[id]));
}

static bool send_all_parameters(const ihm_parameter_blob_t *parameters)
{
    return send_parameter_block(parameters, IHM_PARAM_P10, 3U) &&
           send_parameter_block(parameters, IHM_PARAM_P20, 2U) &&
           send_parameter_block(parameters, IHM_PARAM_P30, 4U) &&
           send_isolated_parameter(parameters, IHM_PARAM_P35) &&
           send_parameter_block(parameters, IHM_PARAM_P41, 5U) &&
           send_isolated_parameter(parameters, IHM_PARAM_P51) &&
           send_parameter_block(parameters, IHM_PARAM_P81, 2U) &&
           send_parameter_block(parameters, IHM_PARAM_P85, 2U);
}

static bool verify_parameter_block(const ihm_parameter_blob_t *parameters,
                                   ihm_parameter_id_t first,
                                   uint16_t quantity)
{
    app_comm_result_t result = perform_read(
        ihm_parameters_register_address(first), quantity);
    uint16_t index;

    if (!result_is_ok(result))
    {
        return false;
    }
    for (index = 0U; index < quantity; index++)
    {
        if (result.values[index] != parameters->values[first + index])
        {
            set_sync_error(APP_COMM_RESULT_PROTOCOL_ERROR);
            return false;
        }
    }
    return true;
}

static bool verify_all_parameters(const ihm_parameter_blob_t *parameters)
{
    return verify_parameter_block(parameters, IHM_PARAM_P10, 3U) &&
           verify_parameter_block(parameters, IHM_PARAM_P20, 2U) &&
           verify_parameter_block(parameters, IHM_PARAM_P30, 4U) &&
           verify_parameter_block(parameters, IHM_PARAM_P35, 1U) &&
           verify_parameter_block(parameters, IHM_PARAM_P41, 5U) &&
           verify_parameter_block(parameters, IHM_PARAM_P51, 1U) &&
           verify_parameter_block(parameters, IHM_PARAM_P81, 2U) &&
           verify_parameter_block(parameters, IHM_PARAM_P85, 2U);
}

static bool stm32_set_write_lock(bool unlocked)
{
    app_comm_result_t status = perform_read(REG_STATUS_WORD, 1U);
    bool currently_unlocked;

    if (!result_is_ok(status))
    {
        return false;
    }
    currently_unlocked =
        (status.values[0] & REG_STATUS_WRITE_UNLOCKED_MASK) != 0U;
    if (currently_unlocked == unlocked)
    {
        return true;
    }
    return result_is_ok(perform_write(REG_P00_COMMAND,
                                      REG_P00_TOGGLE_LOCK));
}

static bool run_parameter_handshake(void)
{
    ihm_parameter_blob_t parameters;
    app_comm_result_t result;
    bool success = false;

    portENTER_CRITICAL(&s_app_lock);
    s_sync.attempts++;
    s_sync.last_error = APP_COMM_RESULT_OK;
    portEXIT_CRITICAL(&s_app_lock);
    ihm_command_service_get_parameters(&parameters);

    set_sync_state(APP_SYNC_READING_ID, 1U);
    if (!validate_identity_register("versão do protocolo",
                                    REG_PROTOCOL_VERSION,
                                    REG_PROTOCOL_VERSION_EXPECTED) ||
        !validate_identity_register("identificador do dispositivo",
                                    REG_DEVICE_ID,
                                    REG_DEVICE_ID_EXPECTED))
    {
        goto cleanup;
    }

    set_sync_state(APP_SYNC_UNLOCKING, 2U);
    if (!stm32_set_write_lock(true))
    {
        goto cleanup;
    }

    set_sync_state(APP_SYNC_BEGIN, 3U);
    if (!result_is_ok(perform_write(REG_CONTROL_COMMAND,
                                    REG_CONTROL_PARAM_SYNC_BEGIN)))
    {
        goto cleanup;
    }

    set_sync_state(APP_SYNC_SENDING_PARAMETERS, 4U);
    if (!send_all_parameters(&parameters))
    {
        goto cleanup;
    }

    set_sync_state(APP_SYNC_COMMIT, 5U);
    if (!result_is_ok(perform_write(REG_CONTROL_COMMAND,
                                    REG_CONTROL_PARAM_SYNC_COMMIT)))
    {
        goto cleanup;
    }

    set_sync_state(APP_SYNC_VERIFYING, 6U);
    if (!verify_all_parameters(&parameters))
    {
        goto cleanup;
    }

    set_sync_state(APP_SYNC_LOCKING, 7U);
    if (!stm32_set_write_lock(false))
    {
        goto cleanup;
    }

    result = perform_read(REG_STATUS_WORD, 1U);
    if (!result_is_ok(result) ||
        ((result.values[0] & REG_STATUS_PARAMETERS_SYNCED_MASK) == 0U))
    {
        set_sync_error(APP_COMM_RESULT_PROTOCOL_ERROR);
        goto cleanup;
    }

    success = true;

cleanup:
    if (!success)
    {
        (void)stm32_set_write_lock(false);
        set_sync_state(APP_SYNC_FAILED, 8U);
        ESP_LOGW(TAG,
                 "Handshake falhou: %s",
                 app_comm_result_to_string(get_sync_error()));
    }
    else
    {
        app_comm_result_t comm_test;

        set_sync_state(APP_SYNC_COMPLETED, 8U);
        comm_test = perform_write(REG_COMM_TEST,
                                  REG_COMM_TEST_INITIAL_VALUE);
        if (result_is_ok(comm_test))
        {
            comm_test = perform_read(REG_COMM_TEST, 1U);
        }
        if (!result_is_ok(comm_test) ||
            (comm_test.values[0] != REG_COMM_TEST_INITIAL_VALUE))
        {
            ESP_LOGW(TAG, "Teste inicial de COMM_TEST falhou");
        }
        ESP_LOGI(TAG, "Handshake de parâmetros concluído");
    }
    ihm_command_service_set_sync_result(success);
    return success;
}

static void set_polling_enabled(bool enabled)
{
    portENTER_CRITICAL(&s_app_lock);
    s_polling_enabled = enabled;
    portEXIT_CRITICAL(&s_app_lock);
}

static void answer_queued_command(const queued_command_t *command)
{
    app_comm_result_t result;

    if (command->request.type == APP_COMM_REQUEST_SET_POLLING)
    {
        (void)memset(&result, 0, sizeof(result));
        set_polling_enabled(command->request.polling_enabled);
        result.status = APP_COMM_RESULT_OK;
    }
    else
    {
        result = execute_modbus_request(&command->request);
    }

    if (xQueueSend(command->response_queue, &result, 0U) != pdTRUE)
    {
        ESP_LOGW(TAG, "Fila de resposta do comando está cheia");
    }
}

static void communication_task(void *context)
{
    uint16_t heartbeat_sequence = 0U;
    uint64_t next_poll_ms;
    uint64_t next_heartbeat_ms;
    uint64_t next_sync_attempt_ms;
    comm_state_t previous_comm_state = COMM_STATE_CONNECTING;

    (void)context;

    while (rs485_master_init() != ESP_OK)
    {
        comm_diagnostics_force_offline();
        ESP_LOGW(TAG, "Falha ao iniciar UART1; nova tentativa em 1 s");
        vTaskDelay(milliseconds_to_ticks_ceil(1000U));
    }

    comm_diagnostics_set_connecting();
    vTaskDelay(milliseconds_to_ticks_ceil(COMM_STM32_STARTUP_DELAY_MS));

    next_poll_ms = uptime_ms();
    next_heartbeat_ms = uptime_ms();
    next_sync_attempt_ms = uptime_ms();

    for (;;)
    {
        queued_command_t command;
        uint64_t now_ms;

        if (xQueueReceive(s_command_queue,
                          &command,
                          milliseconds_to_ticks_ceil(10U)) == pdTRUE)
        {
            answer_queued_command(&command);
        }

        now_ms = uptime_ms();
        if (ihm_command_service_is_sync_pending() &&
            !ihm_command_service_is_edit_unlocked() &&
            (now_ms >= next_sync_attempt_ms))
        {
            (void)run_parameter_handshake();
            next_sync_attempt_ms = uptime_ms() + COMM_SYNC_RETRY_PERIOD_MS;
        }

        if (s_e08_activation_pending)
        {
            (void)perform_write(REG_CONTROL_COMMAND,
                                REG_CONTROL_ACTIVATE_E08);
            s_e08_activation_pending = false;
        }

        portENTER_CRITICAL(&s_app_lock);
        bool recovery_requested = s_e08_recovery_requested;
        portEXIT_CRITICAL(&s_app_lock);
        if (recovery_requested &&
            ihm_command_service_is_handshake_complete() &&
            (comm_diagnostics_get_state() == COMM_STATE_ONLINE) &&
            !comm_diagnostics_loss_exceeds_tolerance())
        {
            const app_comm_result_t clear_result =
                perform_write(REG_CONTROL_COMMAND, REG_CONTROL_CLEAR_E08);
            if (clear_result.status == APP_COMM_RESULT_OK)
            {
                ihm_command_service_set_e08_active(false);
                portENTER_CRITICAL(&s_app_lock);
                s_e08_recovery_requested = false;
                portEXIT_CRITICAL(&s_app_lock);
            }
        }

        now_ms = uptime_ms();
        if (app_is_polling_enabled() && (now_ms >= next_poll_ms))
        {
            app_comm_result_t status_result;

            status_result = perform_read(REG_STATUS_WORD, 3U);
            if (status_result.status == APP_COMM_RESULT_OK)
            {
                (void)parameter_cache_update_runtime_snapshot(
                    status_result.values,
                    uptime_ms());
            }
            else
            {
                parameter_cache_record_runtime_failure();
            }
            next_poll_ms = uptime_ms() + COMM_PARAMETER_POLL_PERIOD_MS;
        }
        else if (!app_is_polling_enabled())
        {
            next_poll_ms = now_ms + COMM_PARAMETER_POLL_PERIOD_MS;
        }

        now_ms = uptime_ms();
        if (now_ms >= next_heartbeat_ms)
        {
            const app_comm_result_t heartbeat_result =
                perform_write(REG_HEARTBEAT_SEQUENCE, heartbeat_sequence);

            comm_diagnostics_record_heartbeat(
                heartbeat_result.status == APP_COMM_RESULT_OK);
            if (comm_diagnostics_should_activate_e08())
            {
                ihm_command_service_set_e08_active(true);
                s_e08_activation_pending = true;
            }
            heartbeat_sequence++;
            next_heartbeat_ms = uptime_ms() + COMM_HEARTBEAT_PERIOD_MS;
        }

        {
            const comm_state_t current_state =
                comm_diagnostics_get_state();
            if ((current_state == COMM_STATE_ONLINE) &&
                ((previous_comm_state == COMM_STATE_OFFLINE) ||
                 ((previous_comm_state == COMM_STATE_DEGRADED) &&
                  ihm_command_service_is_e08_active())))
            {
                /*
                 * Ressincroniza somente apos perda real ou E08 ativo. Uma
                 * falha isolada seguida de recuperacao nao reinicia o
                 * handshake nem ocupa a comunicacao desnecessariamente.
                 */
                ihm_command_service_set_e08_active(true);
                ihm_command_service_request_sync();
                portENTER_CRITICAL(&s_app_lock);
                s_e08_recovery_requested = true;
                portEXIT_CRITICAL(&s_app_lock);
            }
            previous_comm_state = current_state;
        }
    }
}

esp_err_t app_start(void)
{
    BaseType_t task_created;
    esp_err_t error;

    if (s_started)
    {
        return ESP_ERR_INVALID_STATE;
    }

    parameter_cache_init();
    comm_diagnostics_init();
    error = ihm_command_service_init();
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar NVS: %s", esp_err_to_name(error));
        return error;
    }

    s_command_queue = xQueueCreate(COMM_COMMAND_QUEUE_LENGTH,
                                   sizeof(queued_command_t));
    if (s_command_queue == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    task_created = xTaskCreate(communication_task,
                               "modbus_master",
                               COMM_TASK_STACK_SIZE,
                               NULL,
                               COMM_TASK_PRIORITY,
                               NULL);
    if (task_created != pdPASS)
    {
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    error = console_start();
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao criar console: %s", esp_err_to_name(error));
        return error;
    }

    s_started = true;
    return ESP_OK;
}

esp_err_t app_submit_request(const app_comm_request_t *request,
                             QueueHandle_t response_queue,
                             TickType_t timeout)
{
    queued_command_t command;

    if (!request_is_valid(request) ||
        (response_queue == NULL) ||
        (s_command_queue == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    command.request = *request;
    command.response_queue = response_queue;

    if (xQueueSend(s_command_queue, &command, timeout) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool app_is_polling_enabled(void)
{
    bool enabled;

    portENTER_CRITICAL(&s_app_lock);
    enabled = s_polling_enabled;
    portEXIT_CRITICAL(&s_app_lock);
    return enabled;
}

void app_request_parameter_sync(void)
{
    ihm_command_service_request_sync();
}

void app_request_e08_recovery(void)
{
    portENTER_CRITICAL(&s_app_lock);
    s_e08_recovery_requested = true;
    portEXIT_CRITICAL(&s_app_lock);
    ihm_command_service_request_sync();
}

void app_get_sync_snapshot(app_sync_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    portENTER_CRITICAL(&s_app_lock);
    *snapshot = s_sync;
    portEXIT_CRITICAL(&s_app_lock);
}

const char *app_sync_state_to_string(app_sync_state_t state)
{
    switch (state)
    {
        case APP_SYNC_IDLE: return "IDLE";
        case APP_SYNC_READING_ID: return "READING_ID";
        case APP_SYNC_UNLOCKING: return "UNLOCKING";
        case APP_SYNC_BEGIN: return "SYNC_BEGIN";
        case APP_SYNC_SENDING_PARAMETERS: return "SENDING_PARAMETERS";
        case APP_SYNC_COMMIT: return "SYNC_COMMIT";
        case APP_SYNC_VERIFYING: return "VERIFYING";
        case APP_SYNC_LOCKING: return "LOCKING";
        case APP_SYNC_COMPLETED: return "COMPLETED";
        case APP_SYNC_FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

const char *app_comm_result_to_string(app_comm_result_status_t status)
{
    switch (status)
    {
        case APP_COMM_RESULT_OK:
            return "OK";
        case APP_COMM_RESULT_TIMEOUT:
            return "timeout";
        case APP_COMM_RESULT_CRC_ERROR:
            return "erro de CRC";
        case APP_COMM_RESULT_PROTOCOL_ERROR:
            return "erro de protocolo";
        case APP_COMM_RESULT_EXCEPTION:
            return "exceção Modbus";
        case APP_COMM_RESULT_UART_ERROR:
            return "erro de UART";
        case APP_COMM_RESULT_INVALID_ARGUMENT:
            return "argumento inválido";
        case APP_COMM_RESULT_QUEUE_ERROR:
            return "erro de fila";
        default:
            return "resultado desconhecido";
    }
}
