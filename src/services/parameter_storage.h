#ifndef PARAMETER_STORAGE_H
#define PARAMETER_STORAGE_H

#include "esp_err.h"
#include "ihm_parameters.h"

#include <stdbool.h>

esp_err_t parameter_storage_init(ihm_parameter_blob_t *parameters,
                                 bool *defaults_loaded);
esp_err_t parameter_storage_save(const ihm_parameter_blob_t *parameters);

#endif /* PARAMETER_STORAGE_H */
