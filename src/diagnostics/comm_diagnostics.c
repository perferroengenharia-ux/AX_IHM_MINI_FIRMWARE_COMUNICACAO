/**
 * @file comm_diagnostics.c
 * @brief Diagnósticos locais P90/P91 e estado da comunicação.
 */

#include "comm_diagnostics.h"

#include "freertos/FreeRTOS.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define COMM_LOSS_WINDOW_SIZE           100U
#define COMM_P91_MAX_PERCENT            100U
#define COMM_OFFLINE_FAILURE_THRESHOLD  3U
#define COMM_RECOVERY_SUCCESS_THRESHOLD 2U

typedef struct
{
    comm_diagnostics_snapshot_t public_data;
    uint8_t result_window[COMM_LOSS_WINDOW_SIZE];
    uint8_t window_index;
    uint8_t window_count;
    uint8_t window_failure_count;
    uint8_t consecutive_successes;
    bool was_ever_online;
    bool e08_threshold_latched;
    uint64_t total_latency_ms;
    uint32_t latency_sample_count;
} comm_diagnostics_data_t;

static portMUX_TYPE s_diagnostics_lock = portMUX_INITIALIZER_UNLOCKED;
static comm_diagnostics_data_t s_data;

static uint32_t saturating_increment_u32(uint32_t value)
{
    return (value < UINT32_MAX) ? value + 1U : UINT32_MAX;
}

static void update_p90_window(bool success)
{
    const uint8_t new_failure = success ? 0U : 1U;

    if (s_data.window_count == COMM_LOSS_WINDOW_SIZE)
    {
        s_data.window_failure_count =
            (uint8_t)(s_data.window_failure_count -
                      s_data.result_window[s_data.window_index]);
    }
    else
    {
        s_data.window_count++;
    }

    s_data.result_window[s_data.window_index] = new_failure;
    s_data.window_failure_count =
        (uint8_t)(s_data.window_failure_count + new_failure);
    s_data.window_index =
        (uint8_t)((s_data.window_index + 1U) % COMM_LOSS_WINDOW_SIZE);

    s_data.public_data.p90_loss_percent =
        (uint8_t)(((uint16_t)s_data.window_failure_count * 100U) /
                  (uint16_t)s_data.window_count);
    s_data.public_data.p90_window_full =
        s_data.window_count == COMM_LOSS_WINDOW_SIZE;
    if (s_data.public_data.p90_window_full &&
        ((s_data.public_data.p91_tolerance_percent == 0U) ||
         (s_data.public_data.p90_loss_percent <
          s_data.public_data.p91_tolerance_percent)))
    {
        s_data.e08_threshold_latched = false;
    }
}

void comm_diagnostics_init(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    (void)memset(&s_data, 0, sizeof(s_data));
    s_data.public_data.state = COMM_STATE_INIT;
    s_data.public_data.p91_tolerance_percent = 0U;
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_set_connecting(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.state = COMM_STATE_CONNECTING;
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_force_offline(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.state = COMM_STATE_OFFLINE;
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_request_sent(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.requests_sent =
        saturating_increment_u32(s_data.public_data.requests_sent);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_valid_response(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.valid_responses =
        saturating_increment_u32(s_data.public_data.valid_responses);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_timeout(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.response_timeouts =
        saturating_increment_u32(s_data.public_data.response_timeouts);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_crc_error(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.crc_errors =
        saturating_increment_u32(s_data.public_data.crc_errors);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_protocol_error(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.protocol_errors =
        saturating_increment_u32(s_data.public_data.protocol_errors);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_length_error(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.length_errors =
        saturating_increment_u32(s_data.public_data.length_errors);
    s_data.public_data.protocol_errors =
        saturating_increment_u32(s_data.public_data.protocol_errors);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_address_error(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.address_errors =
        saturating_increment_u32(s_data.public_data.address_errors);
    s_data.public_data.protocol_errors =
        saturating_increment_u32(s_data.public_data.protocol_errors);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_exception(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.exception_responses =
        saturating_increment_u32(s_data.public_data.exception_responses);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_retry(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.retries =
        saturating_increment_u32(s_data.public_data.retries);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_latency(uint32_t latency_ms)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.last_latency_ms = latency_ms;
    if ((s_data.latency_sample_count == 0U) ||
        (latency_ms < s_data.public_data.minimum_latency_ms))
    {
        s_data.public_data.minimum_latency_ms = latency_ms;
    }
    if (latency_ms > s_data.public_data.maximum_latency_ms)
    {
        s_data.public_data.maximum_latency_ms = latency_ms;
    }
    if (s_data.latency_sample_count < UINT32_MAX)
    {
        s_data.latency_sample_count++;
        s_data.total_latency_ms += latency_ms;
    }
    s_data.public_data.average_latency_ms =
        (uint32_t)(s_data.total_latency_ms / s_data.latency_sample_count);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_trace_drop(void)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.trace_logs_dropped =
        saturating_increment_u32(s_data.public_data.trace_logs_dropped);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_transaction(bool success, uint64_t timestamp_ms)
{
    portENTER_CRITICAL(&s_diagnostics_lock);

    if (success)
    {
        s_data.public_data.consecutive_failures = 0U;
        if (s_data.consecutive_successes < UINT8_MAX)
        {
            s_data.consecutive_successes++;
        }
        s_data.public_data.last_success_timestamp_ms = timestamp_ms;

        if (!s_data.was_ever_online &&
            (s_data.public_data.state != COMM_STATE_OFFLINE))
        {
            s_data.was_ever_online = true;
            s_data.public_data.state = COMM_STATE_ONLINE;
        }
        else if ((s_data.public_data.state == COMM_STATE_DEGRADED) ||
                 (s_data.public_data.state == COMM_STATE_OFFLINE))
        {
            if (s_data.consecutive_successes >=
                COMM_RECOVERY_SUCCESS_THRESHOLD)
            {
                s_data.was_ever_online = true;
                s_data.public_data.state = COMM_STATE_ONLINE;
            }
        }
        else
        {
            s_data.public_data.state = COMM_STATE_ONLINE;
        }
    }
    else
    {
        s_data.consecutive_successes = 0U;
        s_data.public_data.consecutive_failures =
            saturating_increment_u32(
                s_data.public_data.consecutive_failures);

        if (!s_data.was_ever_online &&
            (s_data.public_data.consecutive_failures <
             COMM_OFFLINE_FAILURE_THRESHOLD))
        {
            s_data.public_data.state = COMM_STATE_CONNECTING;
        }
        else if (s_data.public_data.consecutive_failures >=
                 COMM_OFFLINE_FAILURE_THRESHOLD)
        {
            s_data.public_data.state = COMM_STATE_OFFLINE;
        }
        else
        {
            s_data.public_data.state = COMM_STATE_DEGRADED;
        }
    }

    portEXIT_CRITICAL(&s_diagnostics_lock);
}

void comm_diagnostics_record_heartbeat(bool success)
{
    portENTER_CRITICAL(&s_diagnostics_lock);
    update_p90_window(success);
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

uint8_t comm_diagnostics_get_p90(void)
{
    uint8_t value;

    portENTER_CRITICAL(&s_diagnostics_lock);
    value = s_data.public_data.p90_loss_percent;
    portEXIT_CRITICAL(&s_diagnostics_lock);
    return value;
}

uint8_t comm_diagnostics_get_p91(void)
{
    uint8_t value;

    portENTER_CRITICAL(&s_diagnostics_lock);
    value = s_data.public_data.p91_tolerance_percent;
    portEXIT_CRITICAL(&s_diagnostics_lock);
    return value;
}

bool comm_diagnostics_set_p91(uint8_t tolerance_percent)
{
    if (tolerance_percent > COMM_P91_MAX_PERCENT)
    {
        return false;
    }

    portENTER_CRITICAL(&s_diagnostics_lock);
    s_data.public_data.p91_tolerance_percent = tolerance_percent;
    if ((tolerance_percent == 0U) ||
        (s_data.public_data.p90_loss_percent < tolerance_percent))
    {
        s_data.e08_threshold_latched = false;
    }
    portEXIT_CRITICAL(&s_diagnostics_lock);
    return true;
}

bool comm_diagnostics_loss_exceeds_tolerance(void)
{
    bool exceeded;

    portENTER_CRITICAL(&s_diagnostics_lock);
    exceeded = s_data.public_data.p90_window_full &&
               (s_data.public_data.p91_tolerance_percent != 0U) &&
               (s_data.public_data.p90_loss_percent >=
                s_data.public_data.p91_tolerance_percent);
    portEXIT_CRITICAL(&s_diagnostics_lock);
    return exceeded;
}

bool comm_diagnostics_should_activate_e08(void)
{
    bool should_activate;

    portENTER_CRITICAL(&s_diagnostics_lock);
    should_activate = !s_data.e08_threshold_latched &&
                      s_data.public_data.p90_window_full &&
                      (s_data.public_data.p91_tolerance_percent != 0U) &&
                      (s_data.public_data.p90_loss_percent >=
                       s_data.public_data.p91_tolerance_percent);
    if (should_activate)
    {
        s_data.e08_threshold_latched = true;
        if (s_data.public_data.state == COMM_STATE_ONLINE)
        {
            s_data.public_data.state = COMM_STATE_DEGRADED;
        }
    }
    portEXIT_CRITICAL(&s_diagnostics_lock);
    return should_activate;
}

void comm_diagnostics_clear_statistics(void)
{
    uint8_t p91;
    comm_state_t state;

    portENTER_CRITICAL(&s_diagnostics_lock);
    p91 = s_data.public_data.p91_tolerance_percent;
    state = s_data.public_data.state;
    (void)memset(&s_data.public_data, 0, sizeof(s_data.public_data));
    (void)memset(s_data.result_window, 0, sizeof(s_data.result_window));
    s_data.public_data.p91_tolerance_percent = p91;
    s_data.public_data.state = state;
    s_data.window_index = 0U;
    s_data.window_count = 0U;
    s_data.window_failure_count = 0U;
    s_data.total_latency_ms = 0U;
    s_data.latency_sample_count = 0U;
    s_data.e08_threshold_latched = false;
    portEXIT_CRITICAL(&s_diagnostics_lock);
}

comm_state_t comm_diagnostics_get_state(void)
{
    comm_state_t state;

    portENTER_CRITICAL(&s_diagnostics_lock);
    state = s_data.public_data.state;
    portEXIT_CRITICAL(&s_diagnostics_lock);
    return state;
}

const char *comm_diagnostics_state_to_string(comm_state_t state)
{
    switch (state)
    {
        case COMM_STATE_INIT:
            return "INIT";
        case COMM_STATE_CONNECTING:
            return "CONNECTING";
        case COMM_STATE_ONLINE:
            return "ONLINE";
        case COMM_STATE_DEGRADED:
            return "DEGRADED";
        case COMM_STATE_OFFLINE:
            return "OFFLINE";
        default:
            return "UNKNOWN";
    }
}

void comm_diagnostics_get_snapshot(comm_diagnostics_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&s_diagnostics_lock);
    *snapshot = s_data.public_data;
    portEXIT_CRITICAL(&s_diagnostics_lock);
}
