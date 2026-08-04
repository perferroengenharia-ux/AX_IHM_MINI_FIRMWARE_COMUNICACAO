/**
 * @file parameter_cache.h
 * @brief Cache de comunicacao, sensor de nivel e perifericos.
 */

#ifndef PARAMETER_CACHE_H
#define PARAMETER_CACHE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t status_word;
    uint16_t current_error;
    uint16_t peripheral_status;
    uint64_t last_runtime_ms;
    uint16_t consecutive_runtime_failures;
    bool runtime_valid;
} parameter_cache_snapshot_t;

void parameter_cache_init(void);
bool parameter_cache_update_runtime_snapshot(
    const uint16_t status_values[3],
    uint64_t timestamp_ms);
void parameter_cache_record_runtime_failure(void);
bool parameter_cache_is_fresh(void);
void parameter_cache_get_snapshot(parameter_cache_snapshot_t *snapshot);

#endif /* PARAMETER_CACHE_H */
