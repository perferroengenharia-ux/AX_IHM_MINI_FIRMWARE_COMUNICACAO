/**
 * @file app.h
 * @brief Interface da aplicação e da fila de comandos Modbus.
 */

#ifndef APP_H
#define APP_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "modbus_master.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_COMM_REQUEST_READ = 0,
    APP_COMM_REQUEST_WRITE_SINGLE,
    APP_COMM_REQUEST_WRITE_MULTIPLE,
    APP_COMM_REQUEST_SET_POLLING
} app_comm_request_type_t;

typedef enum
{
    APP_COMM_RESULT_OK = 0,
    APP_COMM_RESULT_TIMEOUT,
    APP_COMM_RESULT_CRC_ERROR,
    APP_COMM_RESULT_PROTOCOL_ERROR,
    APP_COMM_RESULT_EXCEPTION,
    APP_COMM_RESULT_UART_ERROR,
    APP_COMM_RESULT_INVALID_ARGUMENT,
    APP_COMM_RESULT_QUEUE_ERROR
} app_comm_result_status_t;

typedef struct
{
    uint32_t request_id;
    app_comm_request_type_t type;
    uint16_t address;
    uint16_t quantity;
    uint16_t values[MODBUS_WRITE_MAX_REGISTERS];
    bool polling_enabled;
} app_comm_request_t;

typedef struct
{
    uint32_t request_id;
    app_comm_result_status_t status;
    uint16_t values[MODBUS_READ_MAX_REGISTERS];
    uint16_t quantity;
    uint8_t exception_code;
} app_comm_result_t;

typedef enum
{
    APP_SYNC_IDLE = 0,
    APP_SYNC_READING_ID,
    APP_SYNC_UNLOCKING,
    APP_SYNC_BEGIN,
    APP_SYNC_SENDING_PARAMETERS,
    APP_SYNC_COMMIT,
    APP_SYNC_VERIFYING,
    APP_SYNC_LOCKING,
    APP_SYNC_COMPLETED,
    APP_SYNC_FAILED
} app_sync_state_t;

typedef struct
{
    app_sync_state_t state;
    uint8_t step;
    uint32_t attempts;
    app_comm_result_status_t last_error;
} app_sync_snapshot_t;

/** Inicializa serviços, task exclusiva de comunicação e console. */
esp_err_t app_start(void);

/**
 * Enfileira um comando para a task Modbus.
 *
 * A fila indicada em response_queue deve receber objetos app_comm_result_t.
 * O chamador é responsável por manter essa fila válida até a resposta.
 */
esp_err_t app_submit_request(const app_comm_request_t *request,
                             QueueHandle_t response_queue,
                             TickType_t timeout);

/** Indica se a varredura de comunicacao, nivel e perifericos esta habilitada. */
bool app_is_polling_enabled(void);
void app_request_parameter_sync(void);
void app_request_e08_recovery(void);
void app_get_sync_snapshot(app_sync_snapshot_t *snapshot);
const char *app_sync_state_to_string(app_sync_state_t state);

const char *app_comm_result_to_string(app_comm_result_status_t status);

#endif /* APP_H */
