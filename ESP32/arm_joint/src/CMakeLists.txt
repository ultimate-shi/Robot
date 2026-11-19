# 作用：注册关节固件源文件；用法：ESP-IDF构建时自动加载。
idf_component_register(SRCS "main.c" "arm_core.c" INCLUDE_DIRS "../include"
 REQUIRES driver esp_adc esp_timer nvs_flash)
