/**
 * @file rs485_master.c
 * @brief Transporte físico RS485 sobre UART1 do ESP32-S3.
 */

#include "rs485_master.h"

#include "comm_config.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static const char *TAG = "rs485";
static QueueHandle_t s_uart_event_queue;
static bool s_initialized;

static TickType_t milliseconds_to_ticks_ceil(uint32_t milliseconds)
{
    TickType_t ticks = pdMS_TO_TICKS(milliseconds);

    if ((milliseconds > 0U) && (ticks == 0U))
    {
        ticks = 1U;
    }
    return ticks;
}

static void discard_uart_input(void)
{
    (void)uart_flush_input(COMM_UART_PORT);
    if (s_uart_event_queue != NULL)
    {
        (void)xQueueReset(s_uart_event_queue);
    }
}

esp_err_t rs485_master_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = COMM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0U,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = 0U,
            .backup_before_sleep = 0U,
        },
    };
    esp_err_t error;

    if (s_initialized)
    {
        return ESP_OK;
    }

    error = uart_driver_install(COMM_UART_PORT,
                                COMM_UART_RX_BUFFER_SIZE,
                                0,
                                COMM_UART_EVENT_QUEUE_LENGTH,
                                &s_uart_event_queue,
                                0);
    if (error != ESP_OK)
    {
        return error;
    }

    error = uart_param_config(COMM_UART_PORT, &uart_config);
    if (error == ESP_OK)
    {
        error = uart_set_pin(COMM_UART_PORT,
                             COMM_UART_TX_GPIO,
                             COMM_UART_RX_GPIO,
                             COMM_UART_RTS_GPIO,
                             COMM_UART_CTS_GPIO);
    }
    if (error == ESP_OK)
    {
        error = uart_set_mode(COMM_UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
    }
    if (error == ESP_OK)
    {
        /*
         * No modo half-duplex do ESP32-S3, sw_rts=1 mantem o pino RTS fisico
         * em nivel baixo (recepcao). A ISR oficial assume sw_rts=0 ao
         * transmitir e retorna para 1 depois do ultimo stop bit.
         */
        error = uart_set_rts(COMM_UART_PORT, 1);
    }
    if (error == ESP_OK)
    {
        error = uart_set_rx_timeout(COMM_UART_PORT,
                                    COMM_UART_RX_TIMEOUT_SYMBOLS);
    }

    if (error != ESP_OK)
    {
        (void)uart_driver_delete(COMM_UART_PORT);
        s_uart_event_queue = NULL;
        return error;
    }

    discard_uart_input();
    s_initialized = true;
    ESP_LOGI(TAG,
             "UART1 RS485 pronta: RX GPIO%d, TX GPIO%d, RTS/DE GPIO%d, "
             "9600 8E1, pre-DE %" PRIu32 " us",
             COMM_UART_RX_GPIO,
             COMM_UART_TX_GPIO,
             COMM_UART_RTS_GPIO,
             COMM_UART_DE_PRE_DELAY_US);
    return ESP_OK;
}

rs485_transfer_status_t rs485_master_transceive(
    const uint8_t *request,
    uint16_t request_length,
    uint8_t *response,
    uint16_t response_capacity,
    uint16_t *response_length,
    uint32_t timeout_ms)
{
    const int64_t timeout_us = (int64_t)timeout_ms * 1000LL;
    int64_t deadline_us;
    bool received_any_data = false;
    uint16_t received_length = 0U;
    int written;

    if (response_length != NULL)
    {
        *response_length = 0U;
    }

    if (!s_initialized ||
        (request == NULL) ||
        (request_length == 0U) ||
        (request_length > COMM_FRAME_MAX_SIZE) ||
        (response == NULL) ||
        (response_capacity == 0U) ||
        (response_length == NULL) ||
        (timeout_ms == 0U))
    {
        return RS485_TRANSFER_INVALID_ARGUMENT;
    }

    discard_uart_input();

    /*
     * O driver padrao ativa RTS imediatamente antes de alimentar a FIFO. Isso
     * e rapido demais para o EL817 do ENABLE. Antecipar sw_rts=0 cria a mesma
     * margem de 1 ms ja validada no hardware; o TX_DONE do driver continua
     * desativando RTS automaticamente.
     */
    if (uart_set_rts(COMM_UART_PORT, 0) != ESP_OK)
    {
        return RS485_TRANSFER_UART_ERROR;
    }
    esp_rom_delay_us(COMM_UART_DE_PRE_DELAY_US);

    written = uart_write_bytes(COMM_UART_PORT, request, request_length);
    if (written != (int)request_length)
    {
        (void)uart_set_rts(COMM_UART_PORT, 1);
        return RS485_TRANSFER_UART_ERROR;
    }

    if (uart_wait_tx_done(COMM_UART_PORT,
                          milliseconds_to_ticks_ceil(timeout_ms)) != ESP_OK)
    {
        (void)uart_set_rts(COMM_UART_PORT, 1);
        return RS485_TRANSFER_UART_ERROR;
    }

    deadline_us = esp_timer_get_time() + timeout_us;

    while (esp_timer_get_time() < deadline_us)
    {
        uart_event_t event;
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        uint32_t wait_ms;

        if (remaining_us <= 0LL)
        {
            break;
        }

        wait_ms = (uint32_t)((remaining_us + 999LL) / 1000LL);
        if (received_any_data && (wait_ms > COMM_INTER_FRAME_SILENCE_MS))
        {
            wait_ms = COMM_INTER_FRAME_SILENCE_MS;
        }

        if (xQueueReceive(s_uart_event_queue,
                          &event,
                          milliseconds_to_ticks_ceil(wait_ms)) != pdTRUE)
        {
            if (received_any_data)
            {
                *response_length = received_length;
                return RS485_TRANSFER_OK;
            }
            continue;
        }

        switch (event.type)
        {
            case UART_DATA:
            {
                int read_length;

                if (event.size > (size_t)(response_capacity - received_length))
                {
                    discard_uart_input();
                    return RS485_TRANSFER_OVERFLOW;
                }

                read_length = uart_read_bytes(
                    COMM_UART_PORT,
                    &response[received_length],
                    (uint32_t)event.size,
                    milliseconds_to_ticks_ceil(COMM_INTER_FRAME_SILENCE_MS));

                if (read_length < 0)
                {
                    return RS485_TRANSFER_UART_ERROR;
                }

                received_length = (uint16_t)(received_length +
                                             (uint16_t)read_length);
                received_any_data = (received_length > 0U);

                if (received_any_data && event.timeout_flag)
                {
                    *response_length = received_length;
                    return RS485_TRANSFER_OK;
                }
                break;
            }

            case UART_PARITY_ERR:
                discard_uart_input();
                return RS485_TRANSFER_PARITY_ERROR;

            case UART_FRAME_ERR:
            case UART_BREAK:
                discard_uart_input();
                return RS485_TRANSFER_FRAMING_ERROR;

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                discard_uart_input();
                return RS485_TRANSFER_OVERFLOW;

            default:
                ESP_LOGD(TAG, "Evento UART ignorado: %d", (int)event.type);
                break;
        }
    }

    if (received_any_data)
    {
        *response_length = received_length;
        return RS485_TRANSFER_OK;
    }

    return RS485_TRANSFER_TIMEOUT;
}
