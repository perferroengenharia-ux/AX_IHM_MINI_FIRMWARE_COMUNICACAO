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
#include "freertos/task.h"
#include "sdkconfig.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IHM_FIRMWARE_VERSION "COMUNICACAO-MOTOR-1.1.4"
#define PARAMETER_SYNC_WAIT_MS 6000U
#define E08_RECOVERY_WAIT_MS 12000U

static const char *TAG = "ihm_console";
static QueueHandle_t s_response_queue;
static uint32_t s_next_request_id;

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

static bool parse_hz_centihz(const char *text, uint16_t *value)
{
    char *end = NULL;
    double parsed;

    if ((text == NULL) || (value == NULL))
    {
        return false;
    }
    errno = 0;
    parsed = strtod(text, &end);
    if ((errno != 0) || (end == text) || (*end != '\0') ||
        !isfinite(parsed) || (parsed < 0.01) || (parsed > 60.0))
    {
        return false;
    }
    *value = (uint16_t)(parsed * 100.0 + 0.5);
    return true;
}

static bool submit(const app_comm_request_t *request, app_comm_result_t *result)
{
    app_comm_request_t identified_request;
    TickType_t started;
    TickType_t timeout;
    TickType_t remaining;

    if ((request == NULL) || (result == NULL) || (s_response_queue == NULL))
    {
        printf("ERR CONSOLE_QUEUE_UNAVAILABLE\n");
        return false;
    }

    identified_request = *request;
    s_next_request_id++;
    if (s_next_request_id == 0U)
    {
        s_next_request_id++;
    }
    identified_request.request_id = s_next_request_id;
    timeout = timeout_ticks();
    started = xTaskGetTickCount();

    if (app_submit_request(&identified_request,
                           s_response_queue,
                           timeout) != ESP_OK)
    {
        printf("ERR COMMAND_QUEUE_TIMEOUT\n");
        return false;
    }

    remaining = timeout;
    while (xQueueReceive(s_response_queue, result, remaining) == pdTRUE)
    {
        const TickType_t elapsed = xTaskGetTickCount() - started;

        if (result->request_id == identified_request.request_id)
        {
            return true;
        }
        if (elapsed >= timeout)
        {
            break;
        }
        remaining = timeout - elapsed;
    }

    printf("ERR RESPONSE_TIMEOUT\n");
    return false;
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

static bool wait_for_e08_recovery(void)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(E08_RECOVERY_WAIT_MS);
    const TickType_t interval = pdMS_TO_TICKS(50U) == 0U ?
                                1U : pdMS_TO_TICKS(50U);

    for (;;)
    {
        if (!ihm_command_service_is_e08_active() &&
            ihm_command_service_is_handshake_complete())
        {
            return true;
        }
        if ((xTaskGetTickCount() - start) >= timeout)
        {
            printf("ERR E08_RECOVERY_TIMEOUT use=\"status; sync status; "
                   "comm stats\"\n");
            return false;
        }
        vTaskDelay(interval);
    }
}

/*
 * Um comando operacional pode coincidir com o curto intervalo entre a
 * deteccao do E08 remoto e sua limpeza automatica. Nesse caso, recuperar o
 * enlace e repetir o comando uma unica vez em vez de devolver um 0x03
 * transitorio ao operador.
 */
static bool direct_control_write(uint16_t command,
                                 app_comm_result_t *result)
{
    app_comm_result_t status;

    if (!direct_write(REG_CONTROL_COMMAND, command, result))
    {
        return false;
    }
    if ((result->status != APP_COMM_RESULT_EXCEPTION) ||
        (result->exception_code != 0x03U) ||
        !direct_read(REG_STATUS_WORD, 1U, &status) ||
        (status.status != APP_COMM_RESULT_OK) ||
        ((status.values[0] & REG_STATUS_E08_ACTIVE_MASK) == 0U))
    {
        return true;
    }

    printf("INFO e08=active recovery=automatic\n");
    app_request_e08_recovery();
    if (!wait_for_e08_recovery())
    {
        return false;
    }
    return direct_write(REG_CONTROL_COMMAND, command, result);
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

static const char *peripheral_block_reason_to_string(uint16_t reason)
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
        case REG_BLOCK_P81_DISABLED:
            return "p81_disabled";
        default:
            return "unknown";
    }
}

static bool read_peripheral_block_reason(bool pump, uint16_t *reason)
{
    app_comm_result_t block;

    if ((reason == NULL) ||
        !direct_read(pump ? REG_DIAG_PUMP_BLOCK_REASON :
                            REG_DIAG_SWING_BLOCK_REASON,
                     1U, &block) ||
        (block.status != APP_COMM_RESULT_OK))
    {
        return false;
    }
    *reason = block.values[0];
    return true;
}

static bool wait_for_parameter_sync(void)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(PARAMETER_SYNC_WAIT_MS);
    const TickType_t interval = pdMS_TO_TICKS(50U) == 0U ?
                                1U : pdMS_TO_TICKS(50U);

    for (;;)
    {
        if (!ihm_command_service_is_sync_pending() &&
            ihm_command_service_is_handshake_complete())
        {
            return true;
        }
        if ((xTaskGetTickCount() - start) >= timeout)
        {
            return false;
        }
        vTaskDelay(interval);
    }
}

static void print_sync_failure(void)
{
    app_sync_snapshot_t sync;

    app_get_sync_snapshot(&sync);
    printf("ERR PARAMETER_SYNC state=%s step=%u last=%s\n",
           app_sync_state_to_string(sync.state),
           sync.step,
           app_comm_result_to_string(sync.last_error));
}

static bool peripheral_command_ok(bool pump,
                                  bool enabling,
                                  const app_comm_result_t *result)
{
    if (result->status == APP_COMM_RESULT_OK)
    {
        return true;
    }

    if (enabling &&
        (result->status == APP_COMM_RESULT_EXCEPTION) &&
        (result->exception_code == 0x03U))
    {
        uint16_t reason;

        if (read_peripheral_block_reason(pump, &reason))
        {
            printf("ERR %s_BLOCKED reason=%s code=%u\n",
                   pump ? "PUMP" : "SWING",
                   peripheral_block_reason_to_string(reason), reason);
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
    printf("OK firmware=%s protocol=0x%04X scope=communication-motor-peripherals "
           "adc=off motor_monitor=input bypass=fixed_high pwm=spwm_3phase\n",
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
           "system=%s motor=%s routine=%s peripheral=0x%04X "
           "level_valid=%s water=%s shortage=%s "
           "tx=%" PRIu32 " valid=%" PRIu32 " timeout=%" PRIu32
           " crc=%" PRIu32 "\n",
           comm_diagnostics_state_to_string(diag.state),
           app_sync_state_to_string(sync.state),
           ihm_command_service_is_handshake_complete() ? "yes" : "no",
           ihm_command_service_is_e08_active() ? "active" : "inactive",
           parameter_cache_is_fresh() ? "fresh" : "stale",
           (cache.status_word & REG_STATUS_SYSTEM_ENABLED_MASK) != 0U ?
               "on" : "off",
           (cache.status_word & REG_STATUS_MOTOR_RUNNING_MASK) != 0U ?
               "running" :
               ((cache.status_word & REG_STATUS_MOTOR_READY_MASK) != 0U ?
                "ready" : "stopped"),
           (cache.status_word & REG_STATUS_CYCLE_ACTIVE_MASK) != 0U ?
               "active" : "idle",
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
               " protocol=%" PRIu32 " exception=%" PRIu32
               " failures=%" PRIu32 " retries=%" PRIu32
               " latency=%" PRIu32 "ms\n",
               comm_diagnostics_state_to_string(diag.state),
               diag.requests_sent, diag.valid_responses,
               diag.response_timeouts, diag.crc_errors,
               diag.protocol_errors, diag.exception_responses,
               diag.consecutive_failures, diag.retries,
               diag.last_latency_ms);
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
#if CONSOLE_RAW_WRITE_ENABLED
        if (!direct_write(address, value_or_quantity, &result) ||
            !result_ok(&result))
        {
            return 1;
        }
        printf("OK address=0x%04X value=0x%04X\n",
               address, value_or_quantity);
        return 0;
#else
        printf("ERR RAW_WRITE_DISABLED use=specific_control_or_param_command\n");
        return 1;
#endif
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
        if (!direct_control_write(on_command, &result))
        {
            return 1;
        }
        if ((result.status == APP_COMM_RESULT_EXCEPTION) &&
            (result.exception_code == 0x03U))
        {
            uint16_t reason;

            if (read_peripheral_block_reason(pump, &reason) &&
                (reason == REG_BLOCK_PARAMETERS_NOT_SYNCED))
            {
                if (ihm_command_service_is_edit_unlocked())
                {
                    printf("ERR %s_BLOCKED reason=parameter_edit_unlocked "
                           "action=param_lock\n",
                           pump ? "PUMP" : "SWING");
                    return 1;
                }
                printf("INFO peripheral=resyncing_parameters\n");
                app_request_parameter_sync();
                if (!wait_for_parameter_sync())
                {
                    print_sync_failure();
                    return 1;
                }
                if (!direct_control_write(on_command, &result))
                {
                    return 1;
                }
            }
        }
        if (!peripheral_command_ok(pump, enabling, &result))
        {
            return 1;
        }
    }
    else if ((strcmp(argv[1], "off") == 0) ||
             (strcmp(argv[1], "desligar") == 0))
    {
        if (!direct_control_write(off_command, &result) ||
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
    {
        const uint16_t status = result.values[0];
        app_comm_result_t block;
        const uint16_t block_address = pump ?
            REG_DIAG_PUMP_BLOCK_REASON : REG_DIAG_SWING_BLOCK_REASON;

        if (!direct_read(block_address, 1U, &block) || !result_ok(&block))
        {
            return 1;
        }
        printf("OK %s requested=%s active=%s pin=%s block=%s(%u) "
               "status=0x%04X\n",
               pump ? "bomba" : "swing",
               (status & requested_mask) != 0U ? "on" : "off",
               (status & active_mask) != 0U ? "on" : "off",
               (status & pin_mask) != 0U ? "high" : "low",
               peripheral_block_reason_to_string(block.values[0]),
               block.values[0], status);
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
           "pwm=%s motor_monitor=%s bypass=%s\n",
           (bits & REG_PERIPHERAL_PUMP_REQUEST_MASK) != 0U ? "on" : "off",
           (bits & REG_PERIPHERAL_PUMP_ALLOWED_MASK) != 0U ? "yes" : "no",
           (bits & REG_PERIPHERAL_PUMP_ACTIVE_MASK) != 0U ? "on" : "off",
           (bits & REG_PERIPHERAL_SWING_REQUEST_MASK) != 0U ? "on" : "off",
           (bits & REG_PERIPHERAL_SWING_ALLOWED_MASK) != 0U ? "yes" : "no",
            (bits & REG_PERIPHERAL_SWING_ACTIVE_MASK) != 0U ? "on" : "off",
            (bits & REG_PERIPHERAL_MOTOR_ACTIVE_MASK) != 0U ? "on" : "off",
            (bits & REG_PERIPHERAL_MOTOR_PIN_HIGH_MASK) != 0U ? "high" : "low",
            (bits & REG_PERIPHERAL_BYPASS_PIN_HIGH_MASK) != 0U ? "high" : "low");
    return 0;
}

static bool write_control(uint16_t command)
{
    app_comm_result_t result;

    return direct_control_write(command, &result) &&
           result_ok(&result);
}

static const char *motor_state_to_string(uint16_t state)
{
    switch (state)
    {
        case REG_MOTOR_STATE_DISABLED: return "disabled";
        case REG_MOTOR_STATE_READY: return "ready";
        case REG_MOTOR_STATE_STARTING: return "starting";
        case REG_MOTOR_STATE_RUNNING: return "running";
        case REG_MOTOR_STATE_STOPPING: return "stopping";
        case REG_MOTOR_STATE_FAULT: return "fault";
        default: return "unknown";
    }
}

static const char *cycle_state_to_string(uint16_t state)
{
    switch (state)
    {
        case REG_CYCLE_IDLE: return "idle";
        case REG_CYCLE_WETTING: return "wetting";
        case REG_CYCLE_DRYING: return "drying";
        case REG_CYCLE_DRY_STOPPING: return "dry_stopping";
        case REG_CYCLE_EXHAUST: return "exhaust";
        default: return "unknown";
    }
}

static const char *cycle_block_to_string(uint16_t reason)
{
    switch (reason)
    {
        case 0U: return "none";
        case 1U: return "parameters_not_synced";
        case 2U: return "communication_e08";
        case 3U: return "routine_already_active";
        case 4U: return "pump_blocked";
        case 5U: return "swing_blocked";
        case 6U: return "drying_disabled_p32_zero";
        case 7U: return "motor_not_ready";
        default: return "unknown";
    }
}

static int print_motor_status(bool pwm_only)
{
    app_comm_result_t result;
    uint16_t *motor;
    uint16_t flags;

    if (!direct_read(REG_MOTOR_STATE, REG_MOTOR_DIAGNOSTIC_COUNT, &result) ||
        !result_ok(&result))
    {
        return 1;
    }
    motor = result.values;
    flags = motor[REG_MOTOR_PWM_FLAGS - REG_MOTOR_STATE];
    if (pwm_only)
    {
        printf("OK pwm=%s timer=%s complementary=%s mode=%s carrier=%uHz "
               "ARR=%u PSC=%u deadtime_ns=%u modulation=%.1f%% "
               "motor_monitor=%s phase_step=0x%04X%04X\n",
               (flags & REG_MOTOR_PWM_MOE_ENABLED) != 0U ? "on" : "off",
               (flags & REG_MOTOR_PWM_TIMER_RUNNING) != 0U ?
                   "running" : "stopped",
               (flags & REG_MOTOR_PWM_COMPLEMENTARY) != 0U ? "yes" : "no",
               (flags & REG_MOTOR_PWM_EDGE_ALIGNED) != 0U ? "edge" : "center",
               motor[REG_MOTOR_CARRIER_HZ - REG_MOTOR_STATE],
               motor[REG_MOTOR_ARR - REG_MOTOR_STATE],
               motor[REG_MOTOR_PSC - REG_MOTOR_STATE],
               motor[REG_MOTOR_DEADTIME_NS - REG_MOTOR_STATE],
               (double)motor[REG_MOTOR_MODULATION_PERMILLE -
                             REG_MOTOR_STATE] / 10.0,
               motor[REG_MOTOR_MONITOR_LEVEL - REG_MOTOR_STATE] != 0U ?
                   "high" : "low",
               motor[REG_MOTOR_PHASE_STEP_HIGH - REG_MOTOR_STATE],
               motor[REG_MOTOR_PHASE_STEP_LOW - REG_MOTOR_STATE]);
        return 0;
    }
    printf("OK motor_state=%s target=%.2fHz actual=%.2fHz direction=%s "
           "system=%s pwm=%s carrier=%uHz modulation=%.1f%% "
           "motor_monitor=%s blocks=0x%04X[comm=%u params=%u system=%u "
           "direction=%u target=%u e08=%u safety=%u]\n",
           motor_state_to_string(motor[0]),
           (double)motor[REG_MOTOR_TARGET_FREQUENCY - REG_MOTOR_STATE] / 100.0,
           (double)motor[REG_MOTOR_ACTUAL_FREQUENCY - REG_MOTOR_STATE] / 100.0,
           motor[REG_MOTOR_DIRECTION - REG_MOTOR_STATE] != 0U ?
               "reverse" : "normal",
           motor[REG_MOTOR_SYSTEM_ENABLED - REG_MOTOR_STATE] != 0U ?
               "on" : "off",
           (flags & REG_MOTOR_PWM_MOE_ENABLED) != 0U ? "on" : "off",
           motor[REG_MOTOR_CARRIER_HZ - REG_MOTOR_STATE],
           (double)motor[REG_MOTOR_MODULATION_PERMILLE -
                         REG_MOTOR_STATE] / 10.0,
           motor[REG_MOTOR_MONITOR_LEVEL - REG_MOTOR_STATE] != 0U ?
               "high" : "low",
           motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE],
           (motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE] &
            REG_MOTOR_BLOCK_COMMUNICATION) != 0U,
           (motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE] &
            REG_MOTOR_BLOCK_PARAMETERS) != 0U,
           (motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE] &
            REG_MOTOR_BLOCK_SYSTEM_OFF) != 0U,
           (motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE] &
            REG_MOTOR_BLOCK_DIRECTION) != 0U,
           (motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE] &
            REG_MOTOR_BLOCK_TARGET_FREQUENCY) != 0U,
           (motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE] &
            REG_MOTOR_BLOCK_E08) != 0U,
           (motor[REG_MOTOR_START_BLOCKS - REG_MOTOR_STATE] &
            REG_MOTOR_BLOCK_SAFETY) != 0U);
    return 0;
}

static bool set_motor_frequency(uint16_t centihz)
{
    app_comm_result_t result;

    if (!direct_write(REG_MOTOR_TARGET_COMMAND, centihz, &result) ||
        !result_ok(&result))
    {
        return false;
    }
    if (ihm_command_service_remember_motor_frequency(centihz) !=
        IHM_COMMAND_OK)
    {
        printf("WARN motor_frequency_applied_but_not_persisted\n");
    }
    printf("OK motor_target=%.2fHz\n", (double)centihz / 100.0);
    return true;
}

static int command_system(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("ERR use=\"system on|off|status\"\n");
        return 1;
    }
    if (strcmp(argv[1], "status") == 0)
    {
        return print_motor_status(false);
    }
    if (strcmp(argv[1], "on") == 0)
    {
        if (!write_control(REG_CONTROL_SYSTEM_ON)) { return 1; }
        printf("OK system=on\n");
        return 0;
    }
    if (strcmp(argv[1], "off") == 0)
    {
        if (!write_control(REG_CONTROL_SYSTEM_OFF)) { return 1; }
        printf("OK system=off motor=stopping_in_ramp\n");
        return 0;
    }
    printf("ERR use=\"system on|off|status\"\n");
    return 1;
}

static int command_motor(int argc, char **argv)
{
    uint16_t value;
    app_comm_result_t result;

    if ((argc == 2) && (strcmp(argv[1], "status") == 0))
    {
        return print_motor_status(false);
    }
    if ((argc == 2) && (strcmp(argv[1], "start") == 0))
    {
        if (!write_control(REG_CONTROL_SYSTEM_ON) ||
            !write_control(REG_CONTROL_MOTOR_START))
        {
            printf("ERR motor_start=blocked use=\"motor status\"\n");
            return 1;
        }
        printf("OK motor=start ramp=P10\n");
        return 0;
    }
    if ((argc == 2) && (strcmp(argv[1], "stop") == 0))
    {
        if (!write_control(REG_CONTROL_MOTOR_STOP)) { return 1; }
        printf("OK motor=stop ramp=P11\n");
        return 0;
    }
    if ((argc == 3) && (strcmp(argv[1], "freq") == 0) &&
        parse_hz_centihz(argv[2], &value))
    {
        return set_motor_frequency(value) ? 0 : 1;
    }
    if ((argc >= 2) && (argc <= 3) &&
        ((strcmp(argv[1], "up") == 0) ||
         (strcmp(argv[1], "down") == 0)))
    {
        uint16_t increment = 100U;
        int32_t candidate;
        ihm_parameter_blob_t parameters;

        if ((argc == 3) && !parse_hz_centihz(argv[2], &increment))
        {
            printf("ERR increment=0.01..60.00Hz\n");
            return 1;
        }
        if (!direct_read(REG_MOTOR_TARGET_FREQUENCY, 1U, &result) ||
            !result_ok(&result))
        {
            return 1;
        }
        ihm_command_service_get_parameters(&parameters);
        candidate = (int32_t)result.values[0] +
                    (strcmp(argv[1], "up") == 0 ?
                     (int32_t)increment : -(int32_t)increment);
        if (candidate < (int32_t)parameters.values[IHM_PARAM_P20])
        { candidate = parameters.values[IHM_PARAM_P20]; }
        if (candidate > (int32_t)parameters.values[IHM_PARAM_P21])
        { candidate = parameters.values[IHM_PARAM_P21]; }
        return set_motor_frequency((uint16_t)candidate) ? 0 : 1;
    }
    if ((argc == 3) && (strcmp(argv[1], "dir") == 0))
    {
        uint16_t command;
        if (strcmp(argv[2], "normal") == 0)
        { command = REG_CONTROL_MOTOR_DIR_NORMAL; }
        else if ((strcmp(argv[2], "reverse") == 0) ||
                 (strcmp(argv[2], "reverso") == 0))
        { command = REG_CONTROL_MOTOR_DIR_REVERSE; }
        else
        {
            printf("ERR use=\"motor dir normal|reverse\"\n");
            return 1;
        }
        if (!write_control(command)) { return 1; }
        printf("OK motor_direction=%s\n", argv[2]);
        return 0;
    }
    printf("ERR use=\"motor start|stop|status|freq <Hz>|up [Hz]|"
           "down [Hz]|dir normal|reverse\"\n");
    return 1;
}

static bool remote_configuration_is_idle(void)
{
    app_comm_result_t remote;

    if (!direct_read(REG_STATUS_WORD, 1U, &remote) || !result_ok(&remote))
    {
        return false;
    }
    if ((remote.values[0] &
         (REG_STATUS_MOTOR_RUNNING_MASK | REG_STATUS_CYCLE_ACTIVE_MASK)) != 0U)
    {
        printf("ERR parameter_update_requires_motor_and_routine_idle\n");
        return false;
    }
    return true;
}

static bool update_parameter_and_sync(ihm_parameter_id_t id, uint16_t value)
{
    ihm_command_status_t status;

    if (!remote_configuration_is_idle()) { return false; }
    if (ihm_command_service_is_edit_unlocked())
    {
        printf("ERR parameter_edit_unlocked action=param_lock\n");
        return false;
    }
    status = ihm_command_service_p00(7U);
    if (status == IHM_COMMAND_OK)
    {
        status = ihm_command_service_set_parameter(id, value);
    }
    if (ihm_command_service_is_edit_unlocked())
    {
        (void)ihm_command_service_p00(7U);
    }
    if (status != IHM_COMMAND_OK)
    {
        printf("ERR %s parameter=%s\n",
               ihm_command_status_to_string(status), ihm_parameters_code(id));
        return false;
    }
    app_request_parameter_sync();
    if (!wait_for_parameter_sync())
    {
        print_sync_failure();
        return false;
    }
    printf("OK %s=%u sync=completed\n", ihm_parameters_code(id), value);
    return true;
}

static int command_pwm(int argc, char **argv)
{
    uint16_t carrier;

    if ((argc == 2) && (strcmp(argv[1], "status") == 0))
    {
        return print_motor_status(true);
    }
    if ((argc == 3) && (strcmp(argv[1], "freq") == 0) &&
        parse_u16(argv[2], &carrier) &&
        ((carrier == 5U) || (carrier == 10U)))
    {
        return update_parameter_and_sync(IHM_PARAM_P42, carrier) ? 0 : 1;
    }
    printf("ERR use=\"pwm status|freq 5|10\"\n");
    return 1;
}

static int command_ramp(int argc, char **argv)
{
    uint16_t seconds;
    ihm_parameter_id_t id;

    if ((argc != 3) || !parse_u16(argv[2], &seconds) ||
        (seconds < 5U) || (seconds > 60U))
    {
        printf("ERR use=\"ramp accel|decel <5..60 seconds>\"\n");
        return 1;
    }
    if (strcmp(argv[1], "accel") == 0) { id = IHM_PARAM_P10; }
    else if (strcmp(argv[1], "decel") == 0) { id = IHM_PARAM_P11; }
    else
    {
        printf("ERR use=\"ramp accel|decel <5..60 seconds>\"\n");
        return 1;
    }
    return update_parameter_and_sync(id, seconds) ? 0 : 1;
}

static int command_torque(int argc, char **argv)
{
    uint16_t gain;

    if ((argc != 3) || (strcmp(argv[1], "gain") != 0) ||
        !parse_u16(argv[2], &gain) || (gain > 9U))
    {
        printf("ERR use=\"torque gain <0..9 percent>\"\n");
        return 1;
    }
    return update_parameter_and_sync(IHM_PARAM_P35, gain) ? 0 : 1;
}

static int command_routine(int argc, char **argv)
{
    app_comm_result_t result;
    uint16_t command;

    if ((argc == 2) && (strcmp(argv[1], "status") == 0))
    {
        if (!direct_read(REG_CYCLE_STATE, REG_CYCLE_DIAGNOSTIC_COUNT, &result) ||
            !result_ok(&result))
        {
            return 1;
        }
        printf("OK routine=%s remaining_s=%u block=%s(%u) flags=0x%04X\n",
               cycle_state_to_string(result.values[0]), result.values[1],
               cycle_block_to_string(result.values[2]), result.values[2],
               result.values[3]);
        return 0;
    }
    if ((argc == 3) && (strcmp(argv[2], "start") == 0))
    {
        if ((strcmp(argv[1], "wet") == 0) ||
            (strcmp(argv[1], "molhagem") == 0))
        { command = REG_CONTROL_WET_START; }
        else if ((strcmp(argv[1], "dry") == 0) ||
                 (strcmp(argv[1], "secagem") == 0))
        { command = REG_CONTROL_DRY_START; }
        else
        {
            printf("ERR use=\"routine wet|dry start | routine stop|status\"\n");
            return 1;
        }
        if (!write_control(command))
        {
            (void)command_routine(2, (char *[]){argv[0], "status"});
            return 1;
        }
        printf("OK routine=%s started\n", argv[1]);
        return 0;
    }
    if ((argc == 2) && (strcmp(argv[1], "stop") == 0))
    {
        if (!write_control(REG_CONTROL_CYCLE_STOP)) { return 1; }
        printf("OK routine=stop_requested\n");
        return 0;
    }
    printf("ERR use=\"routine wet|dry start | routine stop|status\"\n");
    return 1;
}

static bool find_parameter(const char *text, ihm_parameter_id_t *id)
{
    char normalized[4];

    if ((text == NULL) || (strlen(text) != 3U))
    {
        return false;
    }
    normalized[0] = 'P';
    normalized[1] = text[1];
    normalized[2] = text[2];
    normalized[3] = '\0';
    return ((text[0] == 'P') || (text[0] == 'p')) &&
           ihm_parameters_find(normalized, id);
}

static bool parameter_is_active(ihm_parameter_id_t id)
{
    return (id == IHM_PARAM_P10) ||
           (id == IHM_PARAM_P11) ||
           (id == IHM_PARAM_P20) ||
           (id == IHM_PARAM_P21) ||
           (id == IHM_PARAM_P30) ||
           (id == IHM_PARAM_P31) ||
           (id == IHM_PARAM_P32) ||
           (id == IHM_PARAM_P33) ||
           (id == IHM_PARAM_P35) ||
           (id == IHM_PARAM_P41) ||
           (id == IHM_PARAM_P42) ||
           (id == IHM_PARAM_P51) ||
           (id == IHM_PARAM_P81) ||
           (id == IHM_PARAM_P82) ||
           (id == IHM_PARAM_P85) ||
           (id == IHM_PARAM_P91);
}

static void print_parameter(const ihm_parameter_blob_t *parameters,
                            ihm_parameter_id_t id)
{
    printf("OK %s raw=%u scope=%s persistent=yes sync=%s\n",
           ihm_parameters_code(id),
           parameters->values[id],
           parameter_is_active(id) ? "active" : "legacy_inert",
           ihm_command_service_is_sync_pending() ? "pending" : "done");
}

static void print_parameter_help(void)
{
    printf("OK use=\"param list | param get <Pxx> | "
           "param unlock | param set <Pxx> <raw> | param lock | "
           "param save | param defaults 101 confirm\"\n");
    printf("ACTIVE motor: P10/P11 ramps_s; P20/P21/P32=0.01Hz; "
           "P35=0..9%% torque; P41=60Hz fixed; P42=5|10kHz; P51=dir\n");
    printf("ACTIVE routines: P30/P31/P86=minutes; P33=dry_reverse; "
           "P81 swing; P82 pump; P85 level_sensor; P91 comm_tolerance\n");
}

static int command_param(int argc, char **argv)
{
    ihm_parameter_blob_t parameters;
    ihm_parameter_id_t id;
    ihm_command_status_t command_status;
    const char *action;

    if (argc < 2)
    {
        print_parameter_help();
        return 1;
    }
    action = argv[1];

    if ((strcmp(action, "help") == 0) && (argc == 2))
    {
        print_parameter_help();
        return 0;
    }
    if ((strcmp(action, "list") == 0) && (argc == 2))
    {
        uint16_t index;

        ihm_command_service_get_parameters(&parameters);
        printf("OK edit=%s sync=%s\n",
               ihm_command_service_is_edit_unlocked() ?
                   "unlocked" : "locked",
               ihm_command_service_is_sync_pending() ?
                   "pending" : "done");
        for (index = 0U; index < (uint16_t)IHM_PARAM_COUNT; index++)
        {
            printf("%s raw=%u scope=%s\n",
                   ihm_parameters_code((ihm_parameter_id_t)index),
                   parameters.values[index],
                   parameter_is_active((ihm_parameter_id_t)index) ?
                       "active" : "legacy_inert");
        }
        return 0;
    }
    if ((strcmp(action, "get") == 0) && (argc == 3) &&
        find_parameter(argv[2], &id))
    {
        ihm_command_service_get_parameters(&parameters);
        print_parameter(&parameters, id);
        return 0;
    }
    if ((strcmp(action, "unlock") == 0) && (argc == 2))
    {
        if (!ihm_command_service_is_edit_unlocked())
        {
            command_status = ihm_command_service_p00(7U);
            if (command_status != IHM_COMMAND_OK)
            {
                printf("ERR %s\n",
                       ihm_command_status_to_string(command_status));
                return 1;
            }
        }
        printf("OK edit=unlocked sync=paused_until_lock\n");
        return 0;
    }
    if ((strcmp(action, "lock") == 0) && (argc == 2))
    {
        if (ihm_command_service_is_sync_pending() &&
            !remote_configuration_is_idle())
        {
            return 1;
        }
        if (ihm_command_service_is_edit_unlocked())
        {
            command_status = ihm_command_service_p00(7U);
            if (command_status != IHM_COMMAND_OK)
            {
                printf("ERR %s\n",
                       ihm_command_status_to_string(command_status));
                return 1;
            }
        }
        if (ihm_command_service_is_sync_pending())
        {
            app_request_parameter_sync();
            printf("INFO sync=running\n");
            if (!wait_for_parameter_sync())
            {
                print_sync_failure();
                return 1;
            }
            printf("OK edit=locked sync=completed\n");
        }
        else
        {
            printf("OK edit=locked sync=unchanged\n");
        }
        return 0;
    }
    if ((strcmp(action, "set") == 0) && (argc == 4) &&
        find_parameter(argv[2], &id))
    {
        uint16_t value;

        if (((strcmp(argv[3], "off") == 0) ||
             (strcmp(argv[3], "OFF") == 0)))
        {
            value = 0U;
        }
        else if (!parse_u16(argv[3], &value))
        {
            printf("ERR INVALID_ARGUMENT value=%s\n", argv[3]);
            return 1;
        }
        command_status = ihm_command_service_set_parameter(id, value);
        if (command_status != IHM_COMMAND_OK)
        {
            printf("ERR %s parameter=%s\n",
                   ihm_command_status_to_string(command_status),
                   ihm_parameters_code(id));
            return 1;
        }
        ihm_command_service_get_parameters(&parameters);
        print_parameter(&parameters, id);
        return 0;
    }
    if ((strcmp(action, "save") == 0) && (argc == 2))
    {
        command_status = ihm_command_service_save();
        printf("%s save=%s\n",
               command_status == IHM_COMMAND_OK ? "OK" : "ERR",
               ihm_command_status_to_string(command_status));
        return command_status == IHM_COMMAND_OK ? 0 : 1;
    }
    if ((strcmp(action, "defaults") == 0) && (argc == 4) &&
        (strcmp(argv[2], "101") == 0) &&
        (strcmp(argv[3], "confirm") == 0))
    {
        if (!remote_configuration_is_idle())
        {
            return 1;
        }
        command_status = ihm_command_service_p00(101U);
        if (command_status != IHM_COMMAND_OK)
        {
            printf("ERR defaults=%s\n",
                   ihm_command_status_to_string(command_status));
            return 1;
        }
        app_request_parameter_sync();
        printf("INFO defaults=restored sync=running\n");
        if (!wait_for_parameter_sync())
        {
            print_sync_failure();
            return 1;
        }
        printf("OK defaults=restored edit=locked sync=completed\n");
        return 0;
    }

    print_parameter_help();
    return 1;
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
        if (!remote_configuration_is_idle())
        {
            return 1;
        }
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
    REGISTER("param", "param list|get|set|unlock|lock|save|defaults",
             command_param);
    REGISTER("sync", "sync run|status", command_sync);
    REGISTER("error", "error clear-e08", command_error);
    REGISTER("bomba", "bomba on|off|status", command_peripheral);
    REGISTER("pump", "pump on|off|status", command_peripheral);
    REGISTER("swing", "swing on|off|status", command_peripheral);
    REGISTER("sensor", "sensor status", command_sensor);
    REGISTER("outputs", "outputs status", command_outputs);
    REGISTER("system", "system on|off|status", command_system);
    REGISTER("motor", "motor start|stop|status|freq|up|down|dir",
             command_motor);
    REGISTER("pwm", "pwm status|freq 5|10", command_pwm);
    REGISTER("ramp", "ramp accel|decel <seconds>", command_ramp);
    REGISTER("torque", "torque gain <0..9 percent>", command_torque);
    REGISTER("routine", "routine wet|dry start | stop|status",
             command_routine);
    REGISTER("rotina", "rotina molhagem|secagem start | stop|status",
             command_routine);
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
    esp_err_t error;

    s_response_queue = xQueueCreate(COMM_COMMAND_QUEUE_LENGTH,
                                    sizeof(app_comm_result_t));
    if (s_response_queue == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    error = register_all_commands();

    if (error != ESP_OK)
    {
        vQueueDelete(s_response_queue);
        s_response_queue = NULL;
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
