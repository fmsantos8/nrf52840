#include "system/hal_system.h"
#include "hal_nrf_connect_private.h"

static bool initialized;

hal_status_t hal_system_init(const hal_system_config_t *config) {
    hal_memory_info_t info;
    if (initialized || hal_memory_get_info(&info) == HAL_MEMORY_OK) {
        return HAL_STATUS_INVALID_STATE;
    }
    if (!config || !config->memory_buffer || config->memory_capacity == 0 ||
        (uintptr_t)config->memory_buffer % _Alignof(max_align_t) != 0) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (k_is_in_isr()) {
        return HAL_STATUS_ERROR;
    }
    // Zephyr has initialized the CPU, clocks, kernel and GPIO devices before main.
    hal_status_t status = hal_nrf_connect_memory_status(hal_memory_init(config->memory_buffer, config->memory_capacity));
    if (status == HAL_STATUS_OK) {
        initialized = true;
    }
    return status;
}

hal_status_t hal_system_seal(void) {
    return initialized ? hal_nrf_connect_memory_status(hal_memory_seal()) : HAL_STATUS_NOT_INITIALIZED;
}
