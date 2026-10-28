#ifndef HAL_NRF_CONNECT_PRIVATE_H
#define HAL_NRF_CONNECT_PRIVATE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/nrf-pinctrl.h>
#include <nrfx.h>

#include "common/hal_pin.h"
#include "common/hal_status.h"
#include "hal_memory.h"

hal_status_t hal_nrf_connect_construction(void);
hal_status_t hal_nrf_connect_memory_status(hal_memory_status_t status);
hal_status_t hal_nrf_connect_sdk_status(nrfx_err_t status);
hal_status_t hal_nrf_connect_pin_validate(hal_pin_t pin);
hal_status_t hal_nrf_connect_claim(hal_pin_t first, const hal_pin_t *second);
void hal_nrf_connect_release(hal_pin_t pin);
const struct device *hal_nrf_connect_gpio_device(uint8_t port);
bool hal_nrf_connect_route(const struct pinctrl_dev_config *routing, uint32_t function, hal_pin_t pin);

static inline uint32_t hal_nrf_connect_pin_number(hal_pin_t pin) {
    return (uint32_t)pin.port * 32U + pin.number;
}

static inline bool hal_nrf_connect_same_pin(hal_pin_t a, hal_pin_t b) {
    return a.port == b.port && a.number == b.number;
}

static inline bool hal_nrf_connect_timeout_valid(uint32_t timeout) {
    return timeout > 0 && timeout <= INT32_MAX;
}

#endif
