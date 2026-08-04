#ifndef TEST_MOCK_PARAMETER_STORAGE_H
#define TEST_MOCK_PARAMETER_STORAGE_H

#include "ihm_parameters.h"

void mock_parameter_storage_reset(void);
void mock_parameter_storage_set(const ihm_parameter_blob_t *parameters);
void mock_parameter_storage_corrupt(void);

#endif /* TEST_MOCK_PARAMETER_STORAGE_H */
