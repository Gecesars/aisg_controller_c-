#ifndef RET_FLASH_STORAGE_H
#define RET_FLASH_STORAGE_H

#include "ret_stm32_platform.h"

bool ret_stm32_storage_load(void *context, ret_config_t *config);

bool ret_stm32_storage_save(void *context, const ret_config_t *config);

#endif
