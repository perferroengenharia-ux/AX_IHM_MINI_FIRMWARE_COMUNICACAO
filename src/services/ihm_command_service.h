#ifndef IHM_COMMAND_SERVICE_H
#define IHM_COMMAND_SERVICE_H

#include "esp_err.h"
#include "ihm_parameters.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IHM_COMMAND_OK = 0,
    IHM_COMMAND_INVALID_ARGUMENT,
    IHM_COMMAND_PARAMETER_LOCKED,
    IHM_COMMAND_STORAGE_ERROR
} ihm_command_status_t;

esp_err_t ihm_command_service_init(void);
ihm_command_status_t ihm_command_service_p00(uint16_t command);
ihm_command_status_t ihm_command_service_set_parameter(ihm_parameter_id_t id,
                                                       uint16_t value);
ihm_command_status_t ihm_command_service_save(void);
void ihm_command_service_get_parameters(ihm_parameter_blob_t *parameters);
bool ihm_command_service_is_edit_unlocked(void);
bool ihm_command_service_is_sync_pending(void);
bool ihm_command_service_is_handshake_complete(void);
void ihm_command_service_request_sync(void);
void ihm_command_service_set_sync_result(bool success);
void ihm_command_service_set_e08_active(bool active);
bool ihm_command_service_is_e08_active(void);
const char *ihm_command_status_to_string(ihm_command_status_t status);

#endif /* IHM_COMMAND_SERVICE_H */
