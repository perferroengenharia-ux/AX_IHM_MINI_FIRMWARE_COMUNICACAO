/**
 * @file console.h
 * @brief Console serial de diagnóstico.
 */

#ifndef CONSOLE_H
#define CONSOLE_H

#include "esp_err.h"

#include <stdint.h>

/** Cria a task do console na UART padrão do sistema. */
esp_err_t console_start(void);

/** Enfileira trace sem bloquear a task Modbus; descarta se a fila estiver cheia. */
void console_trace_submit(const uint8_t *tx,
                          uint16_t tx_length,
                          const uint8_t *rx,
                          uint16_t rx_length,
                          uint32_t duration_ms,
                          const char *result);

#endif /* CONSOLE_H */
