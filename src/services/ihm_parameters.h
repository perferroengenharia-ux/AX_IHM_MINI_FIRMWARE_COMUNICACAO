/**
 * @file ihm_parameters.h
 * @brief Estrutura versionada e validacao dos parametros persistentes da IHM.
 */

#ifndef IHM_PARAMETERS_H
#define IHM_PARAMETERS_H

#include <stdbool.h>
#include <stdint.h>

#define IHM_PARAMETER_SCHEMA_VERSION ((uint16_t)2U)

typedef enum
{
    IHM_PARAM_P10 = 0,
    IHM_PARAM_P11,
    IHM_PARAM_P12,
    IHM_PARAM_P20,
    IHM_PARAM_P21,
    IHM_PARAM_P30,
    IHM_PARAM_P31,
    IHM_PARAM_P32,
    IHM_PARAM_P33,
    IHM_PARAM_P35,
    IHM_PARAM_P41,
    IHM_PARAM_P42,
    IHM_PARAM_P43,
    IHM_PARAM_P44,
    IHM_PARAM_P45,
    IHM_PARAM_P51,
    IHM_PARAM_P81,
    IHM_PARAM_P82,
    IHM_PARAM_P85,
    IHM_PARAM_P86,
    IHM_PARAM_P91,
    IHM_PARAM_COUNT
} ihm_parameter_id_t;

typedef struct
{
    uint16_t schema_version;
    uint16_t values[IHM_PARAM_COUNT];
    uint16_t crc;
} ihm_parameter_blob_t;

void ihm_parameters_load_defaults(ihm_parameter_blob_t *parameters);
void ihm_parameters_finalize(ihm_parameter_blob_t *parameters);
bool ihm_parameters_blob_is_valid(const ihm_parameter_blob_t *parameters);
bool ihm_parameters_validate(const ihm_parameter_blob_t *parameters);
bool ihm_parameters_set(ihm_parameter_blob_t *parameters,
                        ihm_parameter_id_t id,
                        uint16_t value);
bool ihm_parameters_get(const ihm_parameter_blob_t *parameters,
                        ihm_parameter_id_t id,
                        uint16_t *value);
bool ihm_parameters_find(const char *code, ihm_parameter_id_t *id);
const char *ihm_parameters_code(ihm_parameter_id_t id);
uint16_t ihm_parameters_register_address(ihm_parameter_id_t id);

#endif /* IHM_PARAMETERS_H */
