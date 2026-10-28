#include "gpio/hal_gpio.h"
#include "hal_nrf_connect_private.h"

struct hal_gpio {
    const struct device *device;
    gpio_pin_t pin;
    hal_gpio_direction_t direction;
    bool initialized;
};

_Static_assert(sizeof(hal_gpio_t) <= UINT32_MAX, "GPIO object exceeds arena size range");
_Static_assert(_Alignof(hal_gpio_t) <= _Alignof(max_align_t), "Unsupported GPIO alignment");

hal_status_t hal_gpio_config(hal_gpio_t **gpio, const hal_gpio_config_t *config) {
    if (gpio) { *gpio = NULL; }
    if (!gpio || !config || (config->direction != HAL_GPIO_INPUT && config->direction != HAL_GPIO_OUTPUT) ||
        (config->pull != HAL_GPIO_PULL_NONE && config->pull != HAL_GPIO_PULL_UP && config->pull != HAL_GPIO_PULL_DOWN)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    hal_status_t status = hal_nrf_connect_pin_validate(config->pin);
    if (status == HAL_STATUS_OK) { status = hal_nrf_connect_construction(); }
    if (status == HAL_STATUS_OK) { status = hal_nrf_connect_claim(config->pin, NULL); }
    if (status != HAL_STATUS_OK) { return status; }
    void *storage;
    status = hal_nrf_connect_memory_status(hal_memory_alloc(sizeof(hal_gpio_t), &storage));
    if (status != HAL_STATUS_OK) {
        hal_nrf_connect_release(config->pin);
        return status;
    }
    hal_gpio_t *object = storage;
    memset(object, 0, sizeof(*object));
    object->device = hal_nrf_connect_gpio_device(config->pin.port);
    object->pin = config->pin.number;
    object->direction = config->direction;
    gpio_flags_t flags = config->direction == HAL_GPIO_INPUT ? GPIO_INPUT :
                         (config->initial_level ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW);
    if (config->pull == HAL_GPIO_PULL_UP) { flags |= GPIO_PULL_UP; }
    if (config->pull == HAL_GPIO_PULL_DOWN) { flags |= GPIO_PULL_DOWN; }
    if (gpio_pin_configure(object->device, object->pin, flags) != 0) {
        hal_nrf_connect_release(config->pin);
        return HAL_STATUS_ERROR;
    }
    object->initialized = true;
    *gpio = object;
    return HAL_STATUS_OK;
}

hal_status_t hal_gpio_read(hal_gpio_t *gpio, bool *level) {
    if (!gpio || !level) { return HAL_STATUS_INVALID_ARGUMENT; }
    if (!gpio->initialized) { return HAL_STATUS_NOT_INITIALIZED; }
    if (gpio->direction != HAL_GPIO_INPUT) { return HAL_STATUS_NOT_SUPPORTED; }
    int value = gpio_pin_get_raw(gpio->device, gpio->pin);
    if (value < 0) { return HAL_STATUS_ERROR; }
    *level = value != 0;
    return HAL_STATUS_OK;
}

hal_status_t hal_gpio_write(hal_gpio_t *gpio, bool level) {
    if (!gpio) { return HAL_STATUS_INVALID_ARGUMENT; }
    if (!gpio->initialized) { return HAL_STATUS_NOT_INITIALIZED; }
    if (gpio->direction != HAL_GPIO_OUTPUT) { return HAL_STATUS_NOT_SUPPORTED; }
    return gpio_pin_set_raw(gpio->device, gpio->pin, level) == 0 ? HAL_STATUS_OK : HAL_STATUS_ERROR;
}
