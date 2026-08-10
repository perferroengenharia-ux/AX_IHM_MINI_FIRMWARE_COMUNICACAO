/**
 * @file comm_config.h
 * @brief Configuração central da comunicação RS485/Modbus RTU.
 */

#ifndef COMM_CONFIG_H
#define COMM_CONFIG_H

#include "driver/uart.h"

#include <stdint.h>

#define COMM_UART_PORT                    UART_NUM_1
#define COMM_UART_TX_GPIO                 37
#define COMM_UART_RX_GPIO                 36
#define COMM_UART_RTS_GPIO                38
#define COMM_UART_CTS_GPIO                UART_PIN_NO_CHANGE
#define COMM_UART_BAUD_RATE               9600

#define COMM_MODBUS_SLAVE_ADDRESS         ((uint8_t)1U)
#define COMM_FRAME_MAX_SIZE               ((uint16_t)256U)
#define COMM_UART_RX_BUFFER_SIZE          512
#define COMM_UART_EVENT_QUEUE_LENGTH      20
#define COMM_UART_RX_TIMEOUT_SYMBOLS      3U

/*
 * O EL817 usado entre GPIO38 e DE//RE precisa estabilizar antes do primeiro
 * start bit. O driver half-duplex continua desativando RTS automaticamente
 * ao final da transmissao; apenas antecipamos a assercao antes de carregar a
 * FIFO da UART.
 */
#define COMM_UART_DE_PRE_DELAY_US          ((uint32_t)1500U)
/*
 * Depois do ultimo stop bit, aguarda o EL817 de DE//RE liberar o receptor.
 * O STM32 somente fecha a requisicao apos 5 ms de silencio, portanto esta
 * janela pode descartar eco/glitch local sem consumir o inicio da resposta.
 */
#define COMM_UART_DE_RELEASE_SETTLE_US     ((uint32_t)1500U)
/*
 * Parity/framing/break isolado apenas limpa FIFO e fila. Uma reinstalacao
 * completa fica reservada para erros de linha consecutivos, reduzindo a
 * rajada "uart: queue free spaces" e o tempo fora de recepcao.
 */
#define COMM_UART_LINE_ERROR_REINSTALL_THRESHOLD ((uint8_t)3U)
/* Pausa entre remover e reinstalar o driver apos erro persistente/overflow. */
#define COMM_UART_RECOVERY_DELAY_US        ((uint32_t)2000U)

#define COMM_RESPONSE_PROCESSING_MARGIN_MS ((uint32_t)50U)
#define COMM_INTER_FRAME_SILENCE_MS       ((uint32_t)5U)
/* Silencio garantido entre o fim de uma transacao e a proxima requisicao. */
#define COMM_MIN_REQUEST_INTERVAL_MS      ((uint32_t)10U)
#define COMM_MAX_RETRIES                  ((uint8_t)2U)

#define COMM_STM32_STARTUP_DELAY_MS       ((uint32_t)500U)
#define COMM_PARAMETER_POLL_PERIOD_MS     ((uint32_t)500U)
#define COMM_HEARTBEAT_PERIOD_MS          ((uint32_t)1000U)
#define COMM_E08_RECOVERY_HEARTBEATS      ((uint8_t)2U)
#define COMM_CACHE_FRESHNESS_MS           ((uint32_t)1500U)

#define COMM_COMMAND_QUEUE_LENGTH         8U
#define COMM_TASK_STACK_SIZE              10240U
#define COMM_TASK_PRIORITY                6U
#define CONSOLE_TASK_STACK_SIZE           4096U
#define CONSOLE_TASK_PRIORITY             4U
#define CONSOLE_LINE_MAX_SIZE             256U
#define CONSOLE_RESPONSE_TIMEOUT_MS       ((uint32_t)1000U)
#define COMM_SYNC_RETRY_PERIOD_MS          ((uint32_t)1000U)
#define COMM_SYNC_RETRY_MAX_MS             ((uint32_t)30000U)
#define COMM_RESYNC_COOLDOWN_MS            ((uint32_t)30000U)
#define COMM_MONITOR_DEFAULT_PERIOD_MS     ((uint32_t)1000U)
#define COMM_MONITOR_MINIMUM_PERIOD_MS     ((uint32_t)200U)
#define CONSOLE_RAW_WRITE_ENABLED          0

#endif /* COMM_CONFIG_H */
