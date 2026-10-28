#include "hal_nrf_connect_private.h"

static uint32_t claimed_pins[2];

hal_status_t hal_nrf_connect_memory_status(hal_memory_status_t status) {
    switch (status) {
    case HAL_MEMORY_OK: return HAL_STATUS_OK;
    case HAL_MEMORY_INVALID_ARGUMENT: return HAL_STATUS_INVALID_ARGUMENT;
    case HAL_MEMORY_NOT_INITIALIZED: return HAL_STATUS_NOT_INITIALIZED;
    case HAL_MEMORY_INVALID_STATE: return HAL_STATUS_INVALID_STATE;
    case HAL_MEMORY_NO_MEMORY: return HAL_STATUS_NO_MEMORY;
    default: return HAL_STATUS_ERROR;
    }
}

hal_status_t hal_nrf_connect_sdk_status(nrfx_err_t status) {
    switch (status) {
    case NRFX_SUCCESS: return HAL_STATUS_OK;
    case NRFX_ERROR_NO_MEM: return HAL_STATUS_NO_MEMORY;
    case NRFX_ERROR_BUSY:
    case NRFX_ERROR_ALREADY: return HAL_STATUS_RESOURCE_BUSY;
    case NRFX_ERROR_NOT_SUPPORTED: return HAL_STATUS_NOT_SUPPORTED;
    case NRFX_ERROR_TIMEOUT: return HAL_STATUS_TIMEOUT;
    default: return HAL_STATUS_ERROR;
    }
}

hal_status_t hal_nrf_connect_construction(void) {
    hal_memory_info_t info;
    if (k_is_in_isr()) {
        return HAL_STATUS_INVALID_STATE;
    }
    hal_status_t status = hal_nrf_connect_memory_status(hal_memory_get_info(&info));
    if (status == HAL_STATUS_OK && info.sealed) {
        return HAL_STATUS_INVALID_STATE;
    }
    return status;
}

const struct device *hal_nrf_connect_gpio_device(uint8_t port) {
    return port == 0 ? DEVICE_DT_GET(DT_NODELABEL(gpio0)) : DEVICE_DT_GET(DT_NODELABEL(gpio1));
}

hal_status_t hal_nrf_connect_pin_validate(hal_pin_t pin) {
    if (pin.port > 1 || pin.number >= (pin.port == 0 ? 32 : 16)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    const struct device *device = hal_nrf_connect_gpio_device(pin.port);
    if (!device_is_ready(device)) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    const struct gpio_driver_config *config = device->config;
    if ((config->port_pin_mask & BIT(pin.number)) == 0) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    return HAL_STATUS_OK;
}

hal_status_t hal_nrf_connect_claim(hal_pin_t first, const hal_pin_t *second) {
    if ((claimed_pins[first.port] & BIT(first.number)) ||
        (second && (claimed_pins[second->port] & BIT(second->number)))) {
        return HAL_STATUS_RESOURCE_BUSY;
    }
    claimed_pins[first.port] |= BIT(first.number);
    if (second) {
        claimed_pins[second->port] |= BIT(second->number);
    }
    return HAL_STATUS_OK;
}

void hal_nrf_connect_release(hal_pin_t pin) {
    claimed_pins[pin.port] &= ~BIT(pin.number);
}

bool hal_nrf_connect_route(const struct pinctrl_dev_config *routing, uint32_t function, hal_pin_t pin) {
    const struct pinctrl_state *state;
    if (pinctrl_lookup_state(routing, PINCTRL_STATE_DEFAULT, &state) != 0) {
        return false;
    }
    for (uint8_t index = 0; index < state->pin_cnt; ++index) {
        uint32_t value = state->pins[index];
        if (((value >> NRF_FUN_POS) & NRF_FUN_MSK) == function &&
            ((value >> NRF_PIN_POS) & NRF_PIN_MSK) == hal_nrf_connect_pin_number(pin)) {
            return true;
        }
    }
    return false;
}
