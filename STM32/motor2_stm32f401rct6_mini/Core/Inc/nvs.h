int16_t init_nvs();
size_t nvs_get_data(uint32_t version, void* buffer, size_t size);
int16_t nvs_set_data(uint32_t version, void *buffer, size_t size);
