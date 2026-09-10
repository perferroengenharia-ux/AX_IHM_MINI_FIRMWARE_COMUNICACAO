/**
 * @file parameter_cache.h
 * @brief Cache tipado de comunicacao, sensores, falhas e perifericos.
 */

#ifndef PARAMETER_CACHE_H
#define PARAMETER_CACHE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t raw_value;
    float converted_value;
    uint64_t last_update_ms;
    uint16_t consecutive_failures;
    bool valid;
} parameter_cache_entry_t;

typedef struct
{
    parameter_cache_entry_t output_frequency;
    parameter_cache_entry_t dc_bus_voltage;
    parameter_cache_entry_t output_current;
    parameter_cache_entry_t output_voltage;
    parameter_cache_entry_t igbt_temperature;
    parameter_cache_entry_t last_error;
    uint16_t sensor_status;
    uint16_t active_fault_mask;
    uint16_t telemetry_sequence;
    uint64_t last_telemetry_ms;
    uint16_t consecutive_snapshot_failures;
    bool snapshot_valid;
    uint16_t status_word;
    uint16_t current_error;
    uint16_t peripheral_status;
    uint64_t last_runtime_ms;
    uint16_t consecutive_runtime_failures;
    bool runtime_valid;
} parameter_cache_snapshot_t;

void parameter_cache_init(void);
bool parameter_cache_update_telemetry_snapshot(
    const uint16_t values[11],
    uint64_t timestamp_ms);
bool parameter_cache_update_runtime_snapshot(
    const uint16_t status_values[3],
    uint64_t timestamp_ms);
void parameter_cache_record_runtime_failure(void);
void parameter_cache_record_poll_failure(void);
bool parameter_cache_get_frequency_hz(float *value);
bool parameter_cache_get_dc_bus_voltage_v(float *value);
bool parameter_cache_get_output_current_a(float *value);
bool parameter_cache_get_output_voltage_v(float *value);
bool parameter_cache_get_igbt_temperature_c(float *value);
bool parameter_cache_get_last_error(uint16_t *value);
bool parameter_cache_get_current_error(uint16_t *value);
bool parameter_cache_get_sensor_status(uint16_t *value);
bool parameter_cache_get_active_fault_mask(uint16_t *value);
bool parameter_cache_is_fresh(void);
void parameter_cache_get_snapshot(parameter_cache_snapshot_t *snapshot);

#endif /* PARAMETER_CACHE_H */
