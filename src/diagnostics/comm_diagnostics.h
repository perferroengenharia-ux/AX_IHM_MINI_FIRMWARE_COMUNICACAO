/**
 * @file comm_diagnostics.h
 * @brief Diagnósticos locais P90/P91 e estado da comunicação.
 */

#ifndef COMM_DIAGNOSTICS_H
#define COMM_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    COMM_STATE_INIT = 0,
    COMM_STATE_CONNECTING,
    COMM_STATE_ONLINE,
    COMM_STATE_DEGRADED,
    COMM_STATE_OFFLINE
} comm_state_t;

typedef struct
{
    uint32_t requests_sent;
    uint32_t valid_responses;
    uint32_t response_timeouts;
    uint32_t crc_errors;
    uint32_t protocol_errors;
    uint32_t length_errors;
    uint32_t address_errors;
    uint32_t exception_responses;
    uint32_t retries;
    uint32_t trace_logs_dropped;
    uint32_t consecutive_failures;
    uint64_t last_success_timestamp_ms;
    uint32_t last_latency_ms;
    uint32_t minimum_latency_ms;
    uint32_t maximum_latency_ms;
    uint32_t average_latency_ms;
    uint8_t p90_loss_percent;
    uint8_t p91_tolerance_percent;
    bool p90_window_full;
    comm_state_t state;
} comm_diagnostics_snapshot_t;

void comm_diagnostics_init(void);
void comm_diagnostics_set_connecting(void);
void comm_diagnostics_force_offline(void);

void comm_diagnostics_record_request_sent(void);
void comm_diagnostics_record_valid_response(void);
void comm_diagnostics_record_timeout(void);
void comm_diagnostics_record_crc_error(void);
void comm_diagnostics_record_protocol_error(void);
void comm_diagnostics_record_length_error(void);
void comm_diagnostics_record_address_error(void);
void comm_diagnostics_record_exception(void);
void comm_diagnostics_record_retry(void);
void comm_diagnostics_record_latency(uint32_t latency_ms);
void comm_diagnostics_record_trace_drop(void);

/** Atualiza a máquina de estados uma vez por transação Modbus lógica. */
void comm_diagnostics_record_transaction(bool success, uint64_t timestamp_ms);

/** Registra o resultado de um periodo de heartbeat na janela movel de P90. */
void comm_diagnostics_record_heartbeat(bool success);

uint8_t comm_diagnostics_get_p90(void);
uint8_t comm_diagnostics_get_p91(void);
bool comm_diagnostics_set_p91(uint8_t tolerance_percent);
bool comm_diagnostics_loss_exceeds_tolerance(void);
bool comm_diagnostics_should_activate_e08(void);
void comm_diagnostics_clear_statistics(void);
comm_state_t comm_diagnostics_get_state(void);
const char *comm_diagnostics_state_to_string(comm_state_t state);
void comm_diagnostics_get_snapshot(comm_diagnostics_snapshot_t *snapshot);

#endif /* COMM_DIAGNOSTICS_H */
