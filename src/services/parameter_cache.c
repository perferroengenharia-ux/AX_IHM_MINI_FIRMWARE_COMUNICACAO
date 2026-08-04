/**
 * @file parameter_cache.c
 * @brief Cache atomico sem telemetria analogica.
 */

#include "parameter_cache.h"

#include "comm_config.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static portMUX_TYPE s_cache_lock = portMUX_INITIALIZER_UNLOCKED;
static parameter_cache_snapshot_t s_cache;

void parameter_cache_init(void)
{
    portENTER_CRITICAL(&s_cache_lock);
    (void)memset(&s_cache, 0, sizeof(s_cache));
    portEXIT_CRITICAL(&s_cache_lock);
}

bool parameter_cache_update_runtime_snapshot(
    const uint16_t status_values[3],
    uint64_t timestamp_ms)
{
    if (status_values == NULL)
    {
        return false;
    }

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

void parameter_cache_record_runtime_failure(void)
{
    portENTER_CRITICAL(&s_cache_lock);
    if (s_cache.consecutive_runtime_failures < UINT16_MAX)
    {
        s_cache.consecutive_runtime_failures++;
    }
    portEXIT_CRITICAL(&s_cache_lock);
}

bool parameter_cache_is_fresh(void)
{
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
    bool fresh;

    portENTER_CRITICAL(&s_cache_lock);
    fresh = s_cache.runtime_valid &&
            ((now_ms - s_cache.last_runtime_ms) <= COMM_CACHE_FRESHNESS_MS);
    portEXIT_CRITICAL(&s_cache_lock);
    return fresh;
}

void parameter_cache_get_snapshot(parameter_cache_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&s_cache_lock);
    *snapshot = s_cache;
    portEXIT_CRITICAL(&s_cache_lock);
}
