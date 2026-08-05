#ifndef PARAMETER_STORAGE_H
#define PARAMETER_STORAGE_H

#include "esp_err.h"
#include "ihm_parameters.h"

#include <stdbool.h>

esp_err_t parameter_storage_init(ihm_parameter_blob_t *parameters,
                                 bool *defaults_loaded);
esp_err_t parameter_storage_save(const ihm_parameter_blob_t *parameters);
esp_err_t parameter_storage_load_motor_frequency(uint16_t *centihz);
esp_err_t parameter_storage_save_motor_frequency(uint16_t centihz);

#endif /* PARAMETER_STORAGE_H */
