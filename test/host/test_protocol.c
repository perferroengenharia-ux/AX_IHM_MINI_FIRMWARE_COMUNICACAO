#include "comm_diagnostics.h"
#include "ihm_command_service.h"
#include "ihm_parameters.h"
#include "mock_parameter_storage.h"
#include "modbus_crc.h"
#include "modbus_master.h"
#include "protocol/register_map.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned int s_failed_checks;
static unsigned int s_executed_checks;

#define CHECK(condition)                                                      \
    do                                                                        \
    {                                                                         \
        s_executed_checks++;                                                  \
        if (!(condition))                                                     \
        {                                                                     \
            s_failed_checks++;                                                \
            printf("FALHA %s:%d: %s\n", __FILE__, __LINE__, #condition);     \
        }                                                                     \
    } while (0)

static void append_crc(uint8_t *frame, uint16_t payload_length)
{
    const uint16_t crc = modbus_crc_calculate(frame, payload_length);

    frame[payload_length] = (uint8_t)(crc & 0xFFU);
    frame[payload_length + 1U] = (uint8_t)(crc >> 8U);
}

static void test_crc_and_request_builders(void)
{
    static const uint8_t crc_input[] = {0x01U, 0x03U, 0x01U,
                                        0x02U, 0x00U, 0x01U};
    static const uint8_t expected_read[] = {0x01U, 0x03U, 0x01U, 0x02U,
                                            0x00U, 0x01U, 0x24U, 0x36U};
    static const uint8_t expected_write[] = {0x01U, 0x06U, 0x01U, 0x00U,
                                             0x12U, 0x34U, 0x85U, 0x41U};
    uint8_t frame[COMM_FRAME_MAX_SIZE];
    uint16_t frame_length;
    uint16_t values[] = {0x1111U, 0x2222U};
    modbus_request_t request;

    CHECK(modbus_crc_calculate(crc_input, sizeof(crc_input)) == 0x3624U);

    CHECK(modbus_master_prepare_read(&request, 0x0102U, 1U));
    CHECK(modbus_master_build_request(&request,
                                      frame,
                                      sizeof(frame),
                                      &frame_length) == MODBUS_PARSE_OK);
    CHECK(frame_length == sizeof(expected_read));
    CHECK(memcmp(frame, expected_read, sizeof(expected_read)) == 0);

    CHECK(modbus_master_prepare_write_single(&request, 0x0100U, 0x1234U));
    CHECK(modbus_master_build_request(&request,
                                      frame,
                                      sizeof(frame),
                                      &frame_length) == MODBUS_PARSE_OK);
    CHECK(frame_length == sizeof(expected_write));
    CHECK(memcmp(frame, expected_write, sizeof(expected_write)) == 0);

    CHECK(modbus_master_prepare_write_multiple(&request,
                                               0x0010U,
                                               values,
                                               2U));
    CHECK(modbus_master_build_request(&request,
                                      frame,
                                      sizeof(frame),
                                      &frame_length) == MODBUS_PARSE_OK);
    CHECK(frame_length == 13U);
    CHECK((frame[1] == 0x10U) && (frame[6] == 4U));
    CHECK((frame[7] == 0x11U) && (frame[8] == 0x11U) &&
          (frame[9] == 0x22U) && (frame[10] == 0x22U));

    CHECK(!modbus_master_prepare_read(&request, 0x0000U, 0U));
    CHECK(!modbus_master_prepare_read(&request, 0xFFFFU, 2U));
    CHECK(!modbus_master_prepare_write_multiple(&request,
                                                0x0000U,
                                                values,
                                                124U));
}

static void test_response_validation(void)
{
    uint8_t expected_response[] = {
        0x01U, 0x03U, 0x02U, 0xC0U, 0x02U, 0x00U, 0x00U};
    uint8_t frame[16];
    modbus_request_t request;
    modbus_response_t response;

    append_crc(expected_response, 5U);
    CHECK(modbus_master_prepare_read(&request, REG_PROTOCOL_VERSION, 1U));
    CHECK(modbus_master_parse_response(&request,
                                       expected_response,
                                       sizeof(expected_response),
                                       &response) == MODBUS_PARSE_OK);
    CHECK((response.quantity == 1U) &&
          (response.values[0] == REG_PROTOCOL_VERSION_EXPECTED));

    (void)memcpy(frame, expected_response, sizeof(expected_response));
    frame[5] ^= 0x01U;
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       sizeof(expected_response),
                                       &response) == MODBUS_PARSE_CRC_ERROR);

    (void)memcpy(frame, expected_response, sizeof(expected_response));
    frame[0] = 0x02U;
    append_crc(frame, 5U);
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       sizeof(expected_response),
                                       &response) ==
          MODBUS_PARSE_WRONG_ADDRESS);

    (void)memcpy(frame, expected_response, sizeof(expected_response));
    frame[1] = 0x04U;
    append_crc(frame, 5U);
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       sizeof(expected_response),
                                       &response) ==
          MODBUS_PARSE_WRONG_FUNCTION);

    frame[0] = 0x01U;
    frame[1] = 0x83U;
    frame[2] = 0x02U;
    append_crc(frame, 3U);
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       5U,
                                       &response) ==
          MODBUS_PARSE_EXCEPTION);
    CHECK(response.exception_code == 0x02U);

    CHECK(modbus_master_parse_response(&request,
                                       expected_response,
                                       4U,
                                       &response) ==
          MODBUS_PARSE_INVALID_LENGTH);

    (void)memcpy(frame, expected_response, sizeof(expected_response));
    frame[2] = 0x04U;
    append_crc(frame, 5U);
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       sizeof(expected_response),
                                       &response) ==
          MODBUS_PARSE_INVALID_BYTE_COUNT);
}

static void test_communication_snapshot_contract(void)
{
    uint8_t frame[3U + (2U * REG_TELEMETRY_SNAPSHOT_COUNT) + 2U];
    const uint16_t values[REG_TELEMETRY_SNAPSHOT_COUNT] = {
        7U, 0U, 0U, 0U, 0U, 0U, 8U, 8U,
        (uint16_t)(REG_SENSOR_LEVEL_VALID_MASK |
                   REG_SENSOR_WATER_SHORTAGE_MASK),
        (uint16_t)(1U << 8U), 7U};
    modbus_request_t request;
    modbus_response_t response;
    uint16_t index;

    CHECK(REG_TELEMETRY_SEQUENCE_END ==
          (REG_TELEMETRY_SEQUENCE_BEGIN +
           REG_TELEMETRY_SNAPSHOT_COUNT - 1U));
    CHECK(REG_DIAG_LEVEL_STABLE_SECONDS ==
          (REG_DIAG_LEVEL_ELECTRICAL + 2U));
    CHECK(REG_PROTOCOL_VERSION_EXPECTED == 0xC002U);

    CHECK(modbus_master_prepare_read(&request,
                                     REG_TELEMETRY_SEQUENCE_BEGIN,
                                     REG_TELEMETRY_SNAPSHOT_COUNT));
    frame[0] = 0x01U;
    frame[1] = 0x03U;
    frame[2] = (uint8_t)(2U * REG_TELEMETRY_SNAPSHOT_COUNT);
    for (index = 0U; index < REG_TELEMETRY_SNAPSHOT_COUNT; index++)
    {
        frame[3U + (2U * index)] = (uint8_t)(values[index] >> 8U);
        frame[4U + (2U * index)] = (uint8_t)values[index];
    }
    append_crc(frame, (uint16_t)(3U + (2U * REG_TELEMETRY_SNAPSHOT_COUNT)));
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       sizeof(frame),
                                       &response) == MODBUS_PARSE_OK);
    CHECK(response.quantity == REG_TELEMETRY_SNAPSHOT_COUNT);
    CHECK(response.values[REG_TELEMETRY_INDEX_SEQUENCE_BEGIN] ==
          response.values[REG_TELEMETRY_INDEX_SEQUENCE_END]);
    CHECK(response.values[REG_TELEMETRY_INDEX_P02] == 0U);
    CHECK(response.values[REG_TELEMETRY_INDEX_P03] == 0U);
    CHECK(response.values[REG_TELEMETRY_INDEX_P05] == 0U);
    CHECK((response.values[REG_TELEMETRY_INDEX_SENSOR_STATUS] &
           REG_SENSOR_WATER_SHORTAGE_MASK) != 0U);
}

static void test_write_response_echoes(void)
{
    uint8_t frame[16];
    uint16_t frame_length;
    uint16_t values[] = {0x1234U, 0x5678U};
    modbus_request_t request;
    modbus_response_t response;

    CHECK(modbus_master_prepare_write_single(&request, 0x0100U, 0x1234U));
    CHECK(modbus_master_build_request(&request,
                                      frame,
                                      sizeof(frame),
                                      &frame_length) == MODBUS_PARSE_OK);
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       frame_length,
                                       &response) == MODBUS_PARSE_OK);

    frame[5] ^= 0x01U;
    append_crc(frame, 6U);
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       frame_length,
                                       &response) ==
          MODBUS_PARSE_INVALID_ECHO);

    CHECK(modbus_master_prepare_write_multiple(&request,
                                               0x0020U,
                                               values,
                                               2U));
    frame[0] = 0x01U;
    frame[1] = 0x10U;
    frame[2] = 0x00U;
    frame[3] = 0x20U;
    frame[4] = 0x00U;
    frame[5] = 0x02U;
    append_crc(frame, 6U);
    CHECK(modbus_master_parse_response(&request,
                                       frame,
                                       8U,
                                       &response) == MODBUS_PARSE_OK);
}

static void test_timeout_disconnect_and_recovery(void)
{
    comm_diagnostics_snapshot_t snapshot;

    comm_diagnostics_init();
    CHECK(comm_diagnostics_get_state() == COMM_STATE_INIT);
    comm_diagnostics_set_connecting();
    CHECK(comm_diagnostics_get_state() == COMM_STATE_CONNECTING);

    comm_diagnostics_record_request_sent();
    comm_diagnostics_record_timeout();
    comm_diagnostics_record_transaction(false, 10U);
    CHECK(comm_diagnostics_get_state() == COMM_STATE_CONNECTING);

    comm_diagnostics_record_request_sent();
    comm_diagnostics_record_timeout();
    comm_diagnostics_record_transaction(false, 20U);
    CHECK(comm_diagnostics_get_state() == COMM_STATE_CONNECTING);

    comm_diagnostics_record_request_sent();
    comm_diagnostics_record_timeout();
    comm_diagnostics_record_transaction(false, 30U);
    CHECK(comm_diagnostics_get_state() == COMM_STATE_OFFLINE);
    CHECK(comm_diagnostics_get_p90() == 0U);
    comm_diagnostics_record_heartbeat(false);
    CHECK(comm_diagnostics_get_p90() == 100U);

    comm_diagnostics_record_valid_response();
    comm_diagnostics_record_transaction(true, 40U);
    CHECK(comm_diagnostics_get_state() == COMM_STATE_OFFLINE);

    comm_diagnostics_record_valid_response();
    comm_diagnostics_record_transaction(true, 50U);
    CHECK(comm_diagnostics_get_state() == COMM_STATE_ONLINE);

    comm_diagnostics_get_snapshot(&snapshot);
    CHECK(snapshot.requests_sent == 3U);
    CHECK(snapshot.response_timeouts == 3U);
    CHECK(snapshot.valid_responses == 2U);
    CHECK(snapshot.consecutive_failures == 0U);
    CHECK(snapshot.last_success_timestamp_ms == 50U);
}

static void test_p90_window_and_p91(void)
{
    unsigned int index;

    comm_diagnostics_init();
    for (index = 0U; index < 25U; index++)
    {
        comm_diagnostics_record_heartbeat(false);
    }
    for (index = 25U; index < 100U; index++)
    {
        comm_diagnostics_record_heartbeat(true);
    }

    CHECK(comm_diagnostics_get_p90() == 25U);
    CHECK(comm_diagnostics_get_p91() == 0U);
    CHECK(!comm_diagnostics_loss_exceeds_tolerance());
    CHECK(comm_diagnostics_set_p91(25U));
    CHECK(comm_diagnostics_loss_exceeds_tolerance());
    CHECK(comm_diagnostics_should_activate_e08());
    CHECK(!comm_diagnostics_should_activate_e08());
    CHECK(comm_diagnostics_set_p91(100U));
    CHECK(!comm_diagnostics_set_p91(101U));

    comm_diagnostics_record_heartbeat(true);
    CHECK(comm_diagnostics_get_p90() == 24U);
}

static void test_parameter_blob_and_local_p00(void)
{
    ihm_parameter_blob_t parameters;
    ihm_parameter_id_t id;

    ihm_parameters_load_defaults(&parameters);
    CHECK(ihm_parameters_blob_is_valid(&parameters));
    CHECK(parameters.values[IHM_PARAM_P10] == 10U);
    CHECK(parameters.values[IHM_PARAM_P12] == 1U);
    CHECK(parameters.values[IHM_PARAM_P20] == 500U);
    CHECK(parameters.values[IHM_PARAM_P21] == 6000U);
    CHECK(parameters.values[IHM_PARAM_P30] == 1U);
    CHECK(parameters.values[IHM_PARAM_P31] == 3U);
    CHECK(parameters.values[IHM_PARAM_P42] == 10U);
    CHECK(!ihm_parameters_set(&parameters, IHM_PARAM_P42, 20U));
    CHECK(!ihm_parameters_set(&parameters, IHM_PARAM_P42, 15U));
    CHECK(!ihm_parameters_set(&parameters, IHM_PARAM_P41, 50U));
    CHECK(!ihm_parameters_set(&parameters, IHM_PARAM_P21, 6001U));
    CHECK(parameters.values[IHM_PARAM_P43] == 600U);
    CHECK(parameters.values[IHM_PARAM_P45] == 180U);
    CHECK(parameters.values[IHM_PARAM_P81] == 1U);
    CHECK(parameters.values[IHM_PARAM_P82] == 1U);
    CHECK(parameters.values[IHM_PARAM_P85] == 1U);
    CHECK(parameters.values[IHM_PARAM_P91] == 47U);
    CHECK(ihm_parameters_set(&parameters, IHM_PARAM_P43, 900U));
    CHECK(!ihm_parameters_set(&parameters, IHM_PARAM_P43, 901U));
    CHECK(ihm_parameters_set(&parameters, IHM_PARAM_P91, 100U));
    CHECK(!ihm_parameters_set(&parameters, IHM_PARAM_P91, 101U));
    CHECK(ihm_parameters_find("P43", &id) && (id == IHM_PARAM_P43));
    CHECK(!ihm_parameters_find("P84", &id));
    CHECK(ihm_parameters_register_address(IHM_PARAM_P91) == UINT16_MAX);

    parameters.crc ^= 1U;
    CHECK(!ihm_parameters_blob_is_valid(&parameters));
    ihm_parameters_load_defaults(&parameters);
    parameters.schema_version++;
    CHECK(!ihm_parameters_blob_is_valid(&parameters));

    mock_parameter_storage_reset();
    comm_diagnostics_init();
    CHECK(ihm_command_service_init() == ESP_OK);
    CHECK(!ihm_command_service_is_edit_unlocked());
    CHECK(ihm_command_service_get_motor_start_frequency() == 500U);
    CHECK(ihm_command_service_remember_motor_frequency(3000U) ==
          IHM_COMMAND_OK);
    CHECK(ihm_command_service_get_motor_start_frequency() == 3000U);
    CHECK(ihm_command_service_set_parameter(IHM_PARAM_P43, 450U) ==
          IHM_COMMAND_PARAMETER_LOCKED);
    CHECK(ihm_command_service_p00(7U) == IHM_COMMAND_OK);
    CHECK(ihm_command_service_is_edit_unlocked());
    CHECK(ihm_command_service_set_parameter(IHM_PARAM_P43, 450U) ==
          IHM_COMMAND_OK);
    ihm_command_service_get_parameters(&parameters);
    CHECK(parameters.values[IHM_PARAM_P43] == 450U);

    /* Uma nova inicializacao recupera a estrutura valida persistida. */
    CHECK(ihm_command_service_init() == ESP_OK);
    ihm_command_service_get_parameters(&parameters);
    CHECK(parameters.values[IHM_PARAM_P43] == 450U);
    CHECK(ihm_command_service_get_motor_start_frequency() == 3000U);

    /* CRC invalido causa retorno seguro aos padroes. */
    mock_parameter_storage_corrupt();
    CHECK(ihm_command_service_init() == ESP_OK);
    ihm_command_service_get_parameters(&parameters);
    CHECK(parameters.values[IHM_PARAM_P43] == 600U);

    CHECK(ihm_command_service_p00(7U) == IHM_COMMAND_OK);
    CHECK(ihm_command_service_p00(101U) == IHM_COMMAND_OK);
    CHECK(!ihm_command_service_is_edit_unlocked());
    ihm_command_service_get_parameters(&parameters);
    CHECK(parameters.values[IHM_PARAM_P43] == 600U);
    CHECK(parameters.values[IHM_PARAM_P91] == 47U);
    CHECK(ihm_command_service_is_sync_pending());

    /* P91 e local; perifericos exigem uma nova sincronizacao. */
    ihm_command_service_set_sync_result(true);
    CHECK(!ihm_command_service_is_sync_pending());
    CHECK(ihm_command_service_p00(7U) == IHM_COMMAND_OK);
    CHECK(ihm_command_service_set_parameter(IHM_PARAM_P91, 25U) ==
          IHM_COMMAND_OK);
    CHECK(!ihm_command_service_is_sync_pending());
    CHECK(ihm_command_service_set_parameter(IHM_PARAM_P82, 2U) ==
          IHM_COMMAND_OK);
    CHECK(ihm_command_service_is_sync_pending());
}

static void test_actuator_diagnostic_contract(void)
{
    CHECK(REG_DIAG_PUMP_BLOCK_REASON == 0x0142U);
    CHECK(REG_DIAG_SWING_BLOCK_REASON == 0x0143U);
    CHECK(REG_DIAG_ELECTRICAL_FAULT_MASK == 0x0144U);
    CHECK(REG_BLOCK_PARAMETERS_NOT_SYNCED == 1U);
    CHECK(REG_BLOCK_COMMUNICATION_ERROR == 2U);
    CHECK(REG_BLOCK_P82_DISABLED == 3U);
    CHECK(REG_BLOCK_LEVEL_SENSOR_DISABLED == 4U);
    CHECK(REG_BLOCK_LEVEL_NOT_STABLE == 5U);
    CHECK(REG_BLOCK_WATER_SHORTAGE == 6U);
    CHECK(REG_BLOCK_P81_DISABLED == 7U);
    CHECK((REG_PERIPHERAL_BYPASS_ACTIVE_MASK &
           REG_PERIPHERAL_MOTOR_ACTIVE_MASK) == 0U);
    CHECK((REG_PERIPHERAL_PUMP_REQUEST_MASK &
           REG_PERIPHERAL_PUMP_ACTIVE_MASK) == 0U);
    CHECK((REG_PERIPHERAL_LEVEL_VALID_MASK &
           REG_PERIPHERAL_WATER_AVAILABLE_MASK) == 0U);
}

int main(void)
{
    test_crc_and_request_builders();
    test_response_validation();
    test_communication_snapshot_contract();
    test_write_response_echoes();
    test_timeout_disconnect_and_recovery();
    test_p90_window_and_p91();
    test_parameter_blob_and_local_p00();
    test_actuator_diagnostic_contract();

    if (s_failed_checks != 0U)
    {
        printf("%u de %u verificações falharam.\n",
               s_failed_checks,
               s_executed_checks);
        return 1;
    }

    printf("OK: %u verificações executadas.\n", s_executed_checks);
    return 0;
}
