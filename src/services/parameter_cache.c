/**
 * @file parameter_cache.c
 * @brief Cache atomico de comunicacao, sensores e falhas.
 */

#include "parameter_cache.h"

#include "comm_config.h"
#include "protocol/register_map.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static portMUX_TYPE s_cache_lock = portMUX_INITIALIZER_UNLOCKED;
static parameter_cache_snapshot_t s_cache;

static uint16_t increment_saturated(uint16_t value)
{
    return value < UINT16_MAX ? (uint16_t)(value + 1U) : UINT16_MAX;
}

static void update_entry(parameter_cache_entry_t *entry,
                         uint16_t raw,
                         float scale,
                         uint64_t timestamp_ms)
{
    entry->raw_value = raw;
    entry->converted_value = (float)raw * scale;
    entry->last_update_ms = timestamp_ms;
    entry->consecutive_failures = 0U;
    entry->valid = true;
}

static bool get_float(const parameter_cache_entry_t *entry, float *value)
{
    bool valid;

    if (value == NULL) { return false; }
    portENTER_CRITICAL(&s_cache_lock);
    valid = entry->valid;
    if (valid) { *value = entry->converted_value; }
    portEXIT_CRITICAL(&s_cache_lock);
    return valid;
}

void parameter_cache_init(void)
{
    portENTER_CRITICAL(&s_cache_lock);
    (void)memset(&s_cache, 0, sizeof(s_cache));
    portEXIT_CRITICAL(&s_cache_lock);
}

bool parameter_cache_update_telemetry_snapshot(const uint16_t values[11],
                                               uint64_t timestamp_ms)
{
    if ((values == NULL) ||
        (values[REG_TELEMETRY_INDEX_SEQUENCE_BEGIN] !=
         values[REG_TELEMETRY_INDEX_SEQUENCE_END]))
    {
        return false;
    }

    portENTER_CRITICAL(&s_cache_lock);
    update_entry(&s_cache.output_frequency,
                 values[REG_TELEMETRY_INDEX_P01], REG_P01_SCALE_HZ,
                 timestamp_ms);
    update_entry(&s_cache.dc_bus_voltage,
                 values[REG_TELEMETRY_INDEX_P02], REG_P02_SCALE_V,
                 timestamp_ms);
    update_entry(&s_cache.output_current,
                 values[REG_TELEMETRY_INDEX_P03], REG_P03_SCALE_A,
                 timestamp_ms);
    update_entry(&s_cache.output_voltage,
                 values[REG_TELEMETRY_INDEX_P04], REG_P04_SCALE_V,
                 timestamp_ms);
    update_entry(&s_cache.igbt_temperature,
                 values[REG_TELEMETRY_INDEX_P05], REG_P05_SCALE_C,
                 timestamp_ms);
    update_entry(&s_cache.last_error,
                 values[REG_TELEMETRY_INDEX_P06], 1.0f,
                 timestamp_ms);
    s_cache.current_error = values[REG_TELEMETRY_INDEX_CURRENT_ERROR];
    s_cache.sensor_status = values[REG_TELEMETRY_INDEX_SENSOR_STATUS];
    s_cache.active_fault_mask = values[REG_TELEMETRY_INDEX_ACTIVE_FAULTS];
    s_cache.telemetry_sequence = values[REG_TELEMETRY_INDEX_SEQUENCE_BEGIN];
    s_cache.last_telemetry_ms = timestamp_ms;
    s_cache.consecutive_snapshot_failures = 0U;
    s_cache.snapshot_valid = true;
    portEXIT_CRITICAL(&s_cache_lock);
    return true;
}

bool parameter_cache_update_runtime_snapshot(
    const uint16_t status_values[3],
    uint64_t timestamp_ms)
{
    if (status_values == NULL) { return false; }

    portENTER_CRITICAL(&s_cache_lock);
    s_cache.status_word = status_values[0];
    s_cache.current_error = status_values[1];
    s_cache.peripheral_status = status_values[2];
    s_cache.last_runtime_ms = timestamp_ms;
    s_cache.consecutive_runtime_failures = 0U;
    s_cache.runtime_valid = true;
    portEXIT_CRITICAL(&s_cache_lock);
    return true;
}

void parameter_cache_record_poll_failure(void)
{
    portENTER_CRITICAL(&s_cache_lock);
    s_cache.output_frequency.consecutive_failures =
        increment_saturated(s_cache.output_frequency.consecutive_failures);
    s_cache.dc_bus_voltage.consecutive_failures =
        increment_saturated(s_cache.dc_bus_voltage.consecutive_failures);
    s_cache.output_current.consecutive_failures =
        increment_saturated(s_cache.output_current.consecutive_failures);
    s_cache.output_voltage.consecutive_failures =
        increment_saturated(s_cache.output_voltage.consecutive_failures);
    s_cache.igbt_temperature.consecutive_failures =
        increment_saturated(s_cache.igbt_temperature.consecutive_failures);
    s_cache.last_error.consecutive_failures =
        increment_saturated(s_cache.last_error.consecutive_failures);
    s_cache.consecutive_snapshot_failures =
        increment_saturated(s_cache.consecutive_snapshot_failures);
    portEXIT_CRITICAL(&s_cache_lock);
}

void parameter_cache_record_runtime_failure(void)
{
    portENTER_CRITICAL(&s_cache_lock);
    s_cache.consecutive_runtime_failures =
        increment_saturated(s_cache.consecutive_runtime_failures);
    portEXIT_CRITICAL(&s_cache_lock);
}

bool parameter_cache_get_frequency_hz(float *value)
{
    return get_float(&s_cache.output_frequency, value);
}

bool parameter_cache_get_dc_bus_voltage_v(float *value)
{
    return get_float(&s_cache.dc_bus_voltage, value);
}

bool parameter_cache_get_output_current_a(float *value)
{
    return get_float(&s_cache.output_current, value);
}

bool parameter_cache_get_output_voltage_v(float *value)
{
    return get_float(&s_cache.output_voltage, value);
}

bool parameter_cache_get_igbt_temperature_c(float *value)
{
    return get_float(&s_cache.igbt_temperature, value);
}

bool parameter_cache_get_last_error(uint16_t *value)
{
    if (value == NULL) { return false; }
    portENTER_CRITICAL(&s_cache_lock);
    const bool valid = s_cache.last_error.valid;
    if (valid) { *value = s_cache.last_error.raw_value; }
    portEXIT_CRITICAL(&s_cache_lock);
    return valid;
}

bool parameter_cache_get_current_error(uint16_t *value)
{
    if (value == NULL) { return false; }
    portENTER_CRITICAL(&s_cache_lock);
    const bool valid = s_cache.snapshot_valid;
    if (valid) { *value = s_cache.current_error; }
    portEXIT_CRITICAL(&s_cache_lock);
    return valid;
}

bool parameter_cache_get_sensor_status(uint16_t *value)
{
    if (value == NULL) { return false; }
    portENTER_CRITICAL(&s_cache_lock);
    const bool valid = s_cache.snapshot_valid;
    if (valid) { *value = s_cache.sensor_status; }
    portEXIT_CRITICAL(&s_cache_lock);
    return valid;
}

bool parameter_cache_get_active_fault_mask(uint16_t *value)
{
    if (value == NULL) { return false; }
    portENTER_CRITICAL(&s_cache_lock);
    const bool valid = s_cache.snapshot_valid;
    if (valid) { *value = s_cache.active_fault_mask; }
    portEXIT_CRITICAL(&s_cache_lock);
    return valid;
}

bool parameter_cache_is_fresh(void)
{
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
    bool fresh;

    portENTER_CRITICAL(&s_cache_lock);
    fresh = s_cache.snapshot_valid && s_cache.runtime_valid &&
            ((now_ms - s_cache.last_telemetry_ms) <= COMM_CACHE_FRESHNESS_MS) &&
            ((now_ms - s_cache.last_runtime_ms) <= COMM_CACHE_FRESHNESS_MS);
    portEXIT_CRITICAL(&s_cache_lock);
    return fresh;
}

void parameter_cache_get_snapshot(parameter_cache_snapshot_t *snapshot)
{
    if (snapshot == NULL) { return; }
    portENTER_CRITICAL(&s_cache_lock);
    *snapshot = s_cache;
    portEXIT_CRITICAL(&s_cache_lock);
}
