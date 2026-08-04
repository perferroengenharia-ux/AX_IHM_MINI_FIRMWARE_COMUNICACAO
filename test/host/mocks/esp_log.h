#ifndef TEST_MOCK_ESP_LOG_H
#define TEST_MOCK_ESP_LOG_H

#define ESP_LOGI(tag, format, ...) ((void)(tag))
#define ESP_LOGW(tag, format, ...) ((void)(tag))
#define ESP_LOGE(tag, format, ...) ((void)(tag))

#endif /* TEST_MOCK_ESP_LOG_H */
