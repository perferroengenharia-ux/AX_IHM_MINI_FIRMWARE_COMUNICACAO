/**
 * @file console.c
 * @brief Terminal enxuto para comunicacao e controle de perifericos.
 */

#include "console.h"

#include "app.h"
#include "comm_config.h"
#include "comm_diagnostics.h"
#include "ihm_command_service.h"
#include "parameter_cache.h"
#include "protocol/register_map.h"

#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IHM_FIRMWARE_VERSION "COMUNICACAO-1.0.1"

static const char *TAG = "ihm_console";

static TickType_t timeout_ticks(void)
{
    TickType_t ticks = pdMS_TO_TICKS(CONSOLE_RESPONSE_TIMEOUT_MS);
    return ticks == 0U ? 1U : ticks;
}

static bool parse_u16(const char *text, uint16_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL))
    {
        return false;
    }
    errno = 0;
    parsed = strtoul(text, &end, 0);
    if ((errno != 0) || (end == text) || (*end != '\0') ||
        (parsed > UINT16_MAX))
    {
        return false;
    }
    *value = (uint16_t)parsed;
    return true;
}

static bool submit(const app_comm_request_t *request, app_comm_result_t *result)
{
    QueueHandle_t queue;
    bool ok = false;

    queue = xQueueCreate(1U, sizeof(*result));
    if (queue == NULL)
    {
        printf("ERR NO_MEMORY\n");
        return false;
    }
    if ((app_submit_request(request, queue, timeout_ticks()) == ESP_OK) &&
        (xQueueReceive(queue, result, timeout_ticks()) == pdTRUE))
    {
        ok = true;
    }
    else
    {
        printf("ERR RESPONSE_TIMEOUT\n");
    }
    vQueueDelete(queue);
    return ok;
}

static bool direct_read(uint16_t address,
                        uint16_t quantity,
                        app_comm_result_t *result)
{
    app_comm_request_t request = {
        .type = APP_COMM_REQUEST_READ,
        .address = address,
        .quantity = quantity,
    };

    return submit(&request, result);
}

static bool direct_write(uint16_t address,
                         uint16_t value,
                         app_comm_result_t *result)
{
    app_comm_request_t request = {
        .type = APP_COMM_REQUEST_WRITE_SINGLE,
        .address = address,
        .quantity = 1U,
        .values = {value},
    };

    return submit(&request, result);
}

static bool result_ok(const app_comm_result_t *result)
{
    if (result->status == APP_COMM_RESULT_OK)
    {
        return true;
    }
    if (result->status == APP_COMM_RESULT_EXCEPTION)
    {
        printf("ERR MODBUS_EXCEPTION code=0x%02X\n", result->exception_code);
    }
    else
    {
        printf("ERR %s\n", app_comm_result_to_string(result->status));
    }
    return false;
}

static const char *pump_block_reason_to_string(uint16_t reason)
{
    switch (reason)
    {
        case REG_BLOCK_NONE:
            return "none";
        case REG_BLOCK_PARAMETERS_NOT_SYNCED:
            return "parameters_not_synced";
        case REG_BLOCK_COMMUNICATION_ERROR:
            return "communication_error";
        case REG_BLOCK_P82_DISABLED:
            return "p82_disabled";
        case REG_BLOCK_LEVEL_SENSOR_DISABLED:
            return "level_sensor_disabled";
        case REG_BLOCK_LEVEL_NOT_STABLE:
            return "level_not_stable";
        case REG_BLOCK_WATER_SHORTAGE:
            return "water_shortage";
        default:
            return "unknown";
    }
}

static bool peripheral_command_ok(bool pump,
                                  bool enabling,
                                  const app_comm_result_t *result)
{
    if (result->status == APP_COMM_RESULT_OK)
    {
        return true;
    }

    if (pump && enabling &&
        (result->status == APP_COMM_RESULT_EXCEPTION) &&
        (result->exception_code == 0x03U))
    {
        app_comm_result_t block;

        if (direct_read(REG_DIAG_PUMP_BLOCK_REASON, 1U, &block) &&
            (block.status == APP_COMM_RESULT_OK))
        {
            printf("ERR PUMP_BLOCKED reason=%s code=%u\n",
                   pump_block_reason_to_string(block.values[0]),
                   block.values[0]);
            return false;
        }
    }

    (void)result_ok(result);
    return false;
}

static int command_version(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("OK firmware=%s protocol=0x%04X scope=communication-peripherals "
           "adc=off motor=off pwm=off\n",
           IHM_FIRMWARE_VERSION, REG_PROTOCOL_VERSION_EXPECTED);
    return 0;
}

static int command_status(int argc, char **argv)
{
    comm_diagnostics_snapshot_t diag;
    app_sync_snapshot_t sync;
    parameter_cache_snapshot_t cache;
    uint16_t peripheral;

    (void)argc;
    (void)argv;
    comm_diagnostics_get_snapshot(&diag);
    app_get_sync_snapshot(&sync);
    parameter_cache_get_snapshot(&cache);
    peripheral = cache.runtime_valid ? cache.peripheral_status : 0U;
    printf("OK comm=%s sync=%s handshake=%s e08=%s cache=%s "
           "peripheral=0x%04X level_valid=%s water=%s shortage=%s "
           "tx=%" PRIu32 " valid=%" PRIu32 " timeout=%" PRIu32
           " crc=%" PRIu32 "\n",
           comm_diagnostics_state_to_string(diag.state),
           app_sync_state_to_string(sync.state),
           ihm_command_service_is_handshake_complete() ? "yes" : "no",
           ihm_command_service_is_e08_active() ? "active" : "inactive",
           parameter_cache_is_fresh() ? "fresh" : "stale",
           peripheral,
           (peripheral & REG_PERIPHERAL_LEVEL_VALID_MASK) != 0U ?
               "yes" : "no",
           (peripheral & REG_PERIPHERAL_WATER_AVAILABLE_MASK) != 0U ?
               "available" : "unavailable",
           (peripheral & REG_PERIPHERAL_WATER_SHORTAGE_MASK) != 0U ?
               "yes" : "no",
           diag.requests_sent, diag.valid_responses,
           diag.response_timeouts, diag.crc_errors);
    return 0;
}

static int command_comm(int argc, char **argv)
{
    app_comm_result_t result;

    if (argc != 2)
    {
        printf("ERR use=\"comm ping|stats|clear-stats\"\n");
        return 1;
    }
    if (strcmp(argv[1], "ping") == 0)
    {
        if (!direct_read(REG_PROTOCOL_VERSION, 2U, &result) ||
            !result_ok(&result))
        {
            return 1;
        }
        printf("OK protocol=0x%04X device=0x%04X match=%s\n",
               result.values[0], result.values[1],
               (result.values[0] == REG_PROTOCOL_VERSION_EXPECTED) &&
               (result.values[1] == REG_DEVICE_ID_EXPECTED) ? "yes" : "no");
        return 0;
    }
    if (strcmp(argv[1], "stats") == 0)
    {
        comm_diagnostics_snapshot_t diag;
        comm_diagnostics_get_snapshot(&diag);
        printf("OK state=%s tx=%" PRIu32 " valid=%" PRIu32
               " timeout=%" PRIu32 " crc=%" PRIu32
               " protocol=%" PRIu32 " retries=%" PRIu32
               " latency=%" PRIu32 "ms\n",
               comm_diagnostics_state_to_string(diag.state),
               diag.requests_sent, diag.valid_responses,
               diag.response_timeouts, diag.crc_errors,
               diag.protocol_errors, diag.retries, diag.last_latency_ms);
        return 0;
    }
    if (strcmp(argv[1], "clear-stats") == 0)
    {
        comm_diagnostics_clear_statistics();
        printf("OK stats=cleared\n");
        return 0;
    }
    printf("ERR use=\"comm ping|stats|clear-stats\"\n");
    return 1;
}

static int command_mb(int argc, char **argv)
{
    uint16_t address;
    uint16_t value_or_quantity;
    app_comm_result_t result;

    if ((argc != 4) || !parse_u16(argv[2], &address) ||
        !parse_u16(argv[3], &value_or_quantity))
    {
        printf("ERR use=\"mb read <address> <quantity> | "
               "mb write <address> <value>\"\n");
        return 1;
    }
    if (strcmp(argv[1], "read") == 0)
    {
        uint16_t index;
        if ((value_or_quantity == 0U) ||
            (value_or_quantity > MODBUS_READ_MAX_REGISTERS) ||
            !direct_read(address, value_or_quantity, &result) ||
            !result_ok(&result))
        {
            return 1;
        }
        printf("OK address=0x%04X values=", address);
        for (index = 0U; index < result.quantity; index++)
        {
            printf("%s0x%04X", index == 0U ? "" : ",", result.values[index]);
        }
        printf("\n");
        return 0;
    }
    if (strcmp(argv[1], "write") == 0)
    {
        if (!direct_write(address, value_or_quantity, &result) ||
            !result_ok(&result))
        {
            return 1;
        }
        printf("OK address=0x%04X value=0x%04X\n",
               address, value_or_quantity);
        return 0;
    }
    printf("ERR use=\"mb read|write ...\"\n");
    return 1;
}

static int command_peripheral(int argc, char **argv)
{
    bool pump;
    uint16_t on_command;
    uint16_t off_command;
    uint16_t requested_mask;
    uint16_t active_mask;
    uint16_t pin_mask;
    app_comm_result_t result;
    bool enabling = false;

    if (argc != 2)
    {
        printf("ERR use=\"bomba|pump|swing on|off|status\"\n");
        return 1;
    }
    pump = (strcmp(argv[0], "bomba") == 0) ||
           (strcmp(argv[0], "pump") == 0);
    on_command = pump ? REG_CONTROL_PUMP_ON : REG_CONTROL_SWING_ON;
    off_command = pump ? REG_CONTROL_PUMP_OFF : REG_CONTROL_SWING_OFF;
    requested_mask = pump ? REG_PERIPHERAL_PUMP_REQUEST_MASK :
                            REG_PERIPHERAL_SWING_REQUEST_MASK;
    active_mask = pump ? REG_PERIPHERAL_PUMP_ACTIVE_MASK :
                         REG_PERIPHERAL_SWING_ACTIVE_MASK;
    pin_mask = pump ? REG_PERIPHERAL_PUMP_PIN_HIGH_MASK :
                      REG_PERIPHERAL_SWING_PIN_HIGH_MASK;

    if ((strcmp(argv[1], "on") == 0) || (strcmp(argv[1], "ligar") == 0))
    {
        enabling = true;
        if (!direct_write(REG_CONTROL_COMMAND, on_command, &result) ||
            !peripheral_command_ok(pump, enabling, &result))
        {
            return 1;
        }
    }
    else if ((strcmp(argv[1], "off") == 0) ||
             (strcmp(argv[1], "desligar") == 0))
    {
        if (!direct_write(REG_CONTROL_COMMAND, off_command, &result) ||
            !peripheral_command_ok(pump, enabling, &result))
        {
            return 1;
        }
    }
    else if ((strcmp(argv[1], "status") != 0) &&
             (strcmp(argv[1], "estado") != 0))
    {
        printf("ERR use=\"%s on|off|status\"\n", argv[0]);
        return 1;
    }

    if (!direct_read(REG_PERIPHERAL_STATUS, 1U, &result) ||
        !result_ok(&result))
    {
        return 1;
    }
    if (pump)
    {
        const uint16_t status = result.values[0];
        app_comm_result_t block;

        if (!direct_read(REG_DIAG_PUMP_BLOCK_REASON, 1U, &block) ||
            !result_ok(&block))
        {
            return 1;
        }
        printf("OK bomba requested=%s active=%s pin=%s block=%s(%u) "
               "status=0x%04X\n",
               (status & requested_mask) != 0U ? "on" : "off",
               (status & active_mask) != 0U ? "on" : "off",
               (status & pin_mask) != 0U ? "high" : "low",
               pump_block_reason_to_string(block.values[0]),
               block.values[0], status);
    }
    else
    {
        printf("OK swing requested=%s active=%s pin=%s status=0x%04X\n",
               (result.values[0] & requested_mask) != 0U ? "on" : "off",
               (result.values[0] & active_mask) != 0U ? "on" : "off",
               (result.values[0] & pin_mask) != 0U ? "high" : "low",
               result.values[0]);
    }
    return 0;
}

static int command_sensor(int argc, char **argv)
{
    app_comm_result_t mode;
    app_comm_result_t level;
    app_comm_result_t status;

    if ((argc != 2) ||
        ((strcmp(argv[1], "status") != 0) &&
         (strcmp(argv[1], "show") != 0)))
    {
        printf("ERR use=\"sensor status\"\n");
        return 1;
    }
    if (!direct_read(REG_P85_LEVEL_SENSOR_MODE, 1U, &mode) ||
        !result_ok(&mode) ||
        !direct_read(REG_DIAG_LEVEL_ELECTRICAL, 3U, &level) ||
        !result_ok(&level) ||
        !direct_read(REG_PERIPHERAL_STATUS, 1U, &status) ||
        !result_ok(&status))
    {
        return 1;
    }
    printf("OK sensor=level mode=%u electrical=%u normal=%u stable_s=%u "
           "valid=%s water=%s shortage=%s\n",
           mode.values[0], level.values[0], level.values[1], level.values[2],
           (status.values[0] & REG_PERIPHERAL_LEVEL_VALID_MASK) != 0U ?
               "yes" : "no",
           (status.values[0] & REG_PERIPHERAL_WATER_AVAILABLE_MASK) != 0U ?
               "available" : "unavailable",
           (status.values[0] & REG_PERIPHERAL_WATER_SHORTAGE_MASK) != 0U ?
               "yes" : "no");
    return 0;
}

static int command_outputs(int argc, char **argv)
{
    app_comm_result_t result;
    uint16_t bits;

    if ((argc != 2) || (strcmp(argv[1], "status") != 0))
    {
        printf("ERR use=\"outputs status\"\n");
        return 1;
    }
    if (!direct_read(REG_PERIPHERAL_STATUS, 1U, &result) ||
        !result_ok(&result))
    {
        return 1;
    }
    bits = result.values[0];
    printf("OK pump_requested=%s pump_allowed=%s pump_active=%s "
           "swing_requested=%s swing_allowed=%s swing_active=%s "
           "motor=off pwm=off bypass=off\n",
           (bits & REG_PERIPHERAL_PUMP_REQUEST_MASK) != 0U ? "on" : "off",
           (bits & REG_PERIPHERAL_PUMP_ALLOWED_MASK) != 0U ? "yes" : "no",
           (bits & REG_PERIPHERAL_PUMP_ACTIVE_MASK) != 0U ? "on" : "off",
           (bits & REG_PERIPHERAL_SWING_REQUEST_MASK) != 0U ? "on" : "off",
           (bits & REG_PERIPHERAL_SWING_ALLOWED_MASK) != 0U ? "yes" : "no",
           (bits & REG_PERIPHERAL_SWING_ACTIVE_MASK) != 0U ? "on" : "off");
    return 0;
}

static int command_sync(int argc, char **argv)
{
    app_sync_snapshot_t sync;

    if (argc != 2)
    {
        printf("ERR use=\"sync run|status\"\n");
        return 1;
    }
    if (strcmp(argv[1], "run") == 0)
    {
        app_request_parameter_sync();
        printf("OK sync=requested\n");
        return 0;
    }
    if (strcmp(argv[1], "status") == 0)
    {
        app_get_sync_snapshot(&sync);
        printf("OK sync=%s step=%u attempts=%" PRIu32 " last=%s\n",
               app_sync_state_to_string(sync.state), sync.step, sync.attempts,
               app_comm_result_to_string(sync.last_error));
        return 0;
    }
    printf("ERR use=\"sync run|status\"\n");
    return 1;
}

static int command_error(int argc, char **argv)
{
    if ((argc == 2) && (strcmp(argv[1], "clear-e08") == 0))
    {
        app_request_e08_recovery();
        printf("OK e08_recovery=requested\n");
        return 0;
    }
    printf("ERR use=\"error clear-e08\"\n");
    return 1;
}

static esp_err_t register_command(const char *name,
                                  const char *help,
                                  esp_console_cmd_func_t function)
{
    const esp_console_cmd_t command = {
        .command = name,
        .help = help,
        .func = function,
    };
    return esp_console_cmd_register(&command);
}

static esp_err_t register_all_commands(void)
{
    esp_err_t error = esp_console_register_help_command();

#define REGISTER(name_, help_, function_)                                    \
    do {                                                                      \
        if (error == ESP_OK) {                                                \
            error = register_command((name_), (help_), (function_));          \
        }                                                                     \
    } while (0)

    REGISTER("version", "Identidade e escopo do firmware", command_version);
    REGISTER("status", "Resumo da comunicacao e perifericos", command_status);
    REGISTER("comm", "comm ping|stats|clear-stats", command_comm);
    REGISTER("mb", "mb read <addr> <qtd> | mb write <addr> <valor>",
             command_mb);
    REGISTER("sync", "sync run|status", command_sync);
    REGISTER("error", "error clear-e08", command_error);
    REGISTER("bomba", "bomba on|off|status", command_peripheral);
    REGISTER("pump", "pump on|off|status", command_peripheral);
    REGISTER("swing", "swing on|off|status", command_peripheral);
    REGISTER("sensor", "sensor status", command_sensor);
    REGISTER("outputs", "outputs status", command_outputs);
#undef REGISTER
    return error;
}

void console_trace_submit(const uint8_t *tx,
                          uint16_t tx_length,
                          const uint8_t *rx,
                          uint16_t rx_length,
                          uint32_t duration_ms,
                          const char *result)
{
    (void)tx;
    (void)tx_length;
    (void)rx;
    (void)rx_length;
    (void)duration_ms;
    (void)result;
}

esp_err_t console_start(void)
{
#if !CONFIG_IHM_TERMINAL_ENABLE
    ESP_LOGI(TAG, "Terminal desabilitado por configuracao");
    return ESP_OK;
#else
    esp_console_repl_config_t repl_config =
        ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_repl_t *repl = NULL;
    esp_err_t error = register_all_commands();

    if (error != ESP_OK)
    {
        return error;
    }
    repl_config.prompt = "comm> ";
    repl_config.max_cmdline_length = CONSOLE_LINE_MAX_SIZE;
    repl_config.task_stack_size = CONSOLE_TASK_STACK_SIZE;
    repl_config.task_priority = CONSOLE_TASK_PRIORITY;

#if CONFIG_IHM_TERMINAL_USB_SERIAL_JTAG && CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    {
        const esp_console_dev_usb_serial_jtag_config_t device_config =
            ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
        error = esp_console_new_repl_usb_serial_jtag(
            &device_config, &repl_config, &repl);
    }
#elif CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    {
        const esp_console_dev_uart_config_t device_config =
            ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
        error = esp_console_new_repl_uart(
            &device_config, &repl_config, &repl);
    }
#else
    error = ESP_ERR_NOT_SUPPORTED;
#endif
    if (error == ESP_OK)
    {
        error = esp_console_start_repl(repl);
    }
    ESP_LOGI(TAG, "Terminal de comunicacao iniciado");
    return error;
#endif
}
