#include "uart/hal_uart.h"
#include "hal_nrf_connect_private.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#define UART_NODE DT_ALIAS(hal_uart0)
BUILD_ASSERT(DT_NODE_HAS_COMPAT(UART_NODE, zephyr_cdc_acm_uart), "HAL serial 0 uses USB CDC ACM");
static bool claimed;

struct hal_uart {
    const struct device *device;
    struct k_sem completion;
    struct k_spinlock lock;
    uint32_t timeout;
    uint32_t pending;
    uint32_t accepted;
    bool active;
    bool initialized;
    uint8_t staging[64];
};

_Static_assert(sizeof(hal_uart_t) <= UINT32_MAX, "UART object exceeds arena size range");
_Static_assert(_Alignof(hal_uart_t) <= _Alignof(max_align_t), "Unsupported UART alignment");

static void serial_ready(const struct device *device, void *context) {
    hal_uart_t *uart = context;
    uart_irq_update(device);
    k_spinlock_key_t key = k_spin_lock(&uart->lock);
    if (uart->active && uart_irq_tx_ready(device)) {
        int count = uart_fifo_fill(device, uart->staging + uart->accepted, uart->pending - uart->accepted);
        if (count > 0) { uart->accepted += (uint32_t)count; }
        if (uart->accepted == uart->pending || count < 0) {
            uart->active = false;
            uart_irq_tx_disable(device);
            k_sem_give(&uart->completion);
        }
    } else if (!uart->active) {
        uart_irq_tx_disable(device);
    }
    k_spin_unlock(&uart->lock, key);
}

hal_status_t hal_uart_config(hal_uart_t **uart, const hal_uart_config_t *config) {
    if (uart) { *uart = NULL; }
    if (!uart || !config || config->instance >= 2 || !config->baudrate ||
        !hal_nrf_connect_timeout_valid(config->timeout) ||
        (config->data_width != HAL_UART_DATA_WIDTH_7 && config->data_width != HAL_UART_DATA_WIDTH_8) ||
        (config->parity != HAL_UART_PARITY_NONE && config->parity != HAL_UART_PARITY_EVEN && config->parity != HAL_UART_PARITY_ODD) ||
        (config->stop_count != HAL_UART_STOP_COUNT_1 && config->stop_count != HAL_UART_STOP_COUNT_2)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    bool tx_absent = config->transmit_pin.port == UINT8_MAX && config->transmit_pin.number == UINT8_MAX;
    bool rx_absent = config->receive_pin.port == UINT8_MAX && config->receive_pin.number == UINT8_MAX;
    if (tx_absent != rx_absent) { return HAL_STATUS_INVALID_ARGUMENT; }
    if (!tx_absent) {
        hal_status_t pins = hal_nrf_connect_pin_validate(config->transmit_pin);
        if (pins == HAL_STATUS_OK) { pins = hal_nrf_connect_pin_validate(config->receive_pin); }
        if (pins != HAL_STATUS_OK) { return pins; }
        if (hal_nrf_connect_same_pin(config->transmit_pin, config->receive_pin)) { return HAL_STATUS_INVALID_ARGUMENT; }
        return HAL_STATUS_NOT_SUPPORTED;
    }
    if (config->instance != 0 || config->baudrate != 115200 || config->data_width != HAL_UART_DATA_WIDTH_8 ||
        config->parity != HAL_UART_PARITY_NONE || config->stop_count != HAL_UART_STOP_COUNT_1 ||
        config->transmit_pin.port != UINT8_MAX || config->transmit_pin.number != UINT8_MAX ||
        config->receive_pin.port != UINT8_MAX || config->receive_pin.number != UINT8_MAX) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    hal_status_t status = hal_nrf_connect_construction();
    if (status != HAL_STATUS_OK) { return status; }
    if (claimed) { return HAL_STATUS_RESOURCE_BUSY; }
    const struct device *device = DEVICE_DT_GET(UART_NODE);
    if (!device_is_ready(device)) { return HAL_STATUS_ERROR; }
    void *storage;
    status = hal_nrf_connect_memory_status(hal_memory_alloc(sizeof(hal_uart_t), &storage));
    if (status != HAL_STATUS_OK) { return status; }
    hal_uart_t *object = storage;
    memset(object, 0, sizeof(*object));
    object->device = device;
    object->timeout = config->timeout;
    k_sem_init(&object->completion, 0, 1);
    if (uart_irq_callback_user_data_set(device, serial_ready, object) != 0) { return HAL_STATUS_ERROR; }
    uart_irq_rx_disable(device);
    uart_irq_tx_disable(device);
    // USB starts without waiting for enumeration or DTR; an absent host cannot block initialization.
    if (usb_enable(NULL) != 0) { return HAL_STATUS_ERROR; }
    claimed = true;
    object->initialized = true;
    *uart = object;
    return HAL_STATUS_OK;
}

hal_status_t hal_uart_write(hal_uart_t *uart, const uint8_t *buffer, uint32_t size, uint32_t *transmitted_count) {
    if (transmitted_count) { *transmitted_count = 0; }
    if (!uart || !transmitted_count || (!buffer && size)) { return HAL_STATUS_INVALID_ARGUMENT; }
    if (!uart->initialized) { return HAL_STATUS_NOT_INITIALIZED; }
    if (!size) { return HAL_STATUS_OK; }
    if (k_is_in_isr()) { return HAL_STATUS_NOT_SUPPORTED; }
    const int64_t deadline = k_uptime_get() + uart->timeout;
    uint32_t dtr = 0;
    while (!dtr) {
        if (uart_line_ctrl_get(uart->device, UART_LINE_CTRL_DTR, &dtr) != 0) { return HAL_STATUS_ERROR; }
        if (dtr) { break; }
        if (k_uptime_get() >= deadline) { return HAL_STATUS_TIMEOUT; }
        k_msleep(1);
    }
    while (*transmitted_count < size) {
        int64_t remaining = deadline - k_uptime_get();
        if (remaining <= 0) { return HAL_STATUS_TIMEOUT; }
        uint32_t chunk = MIN(size - *transmitted_count, sizeof(uart->staging));
        k_sem_reset(&uart->completion);
        k_spinlock_key_t key = k_spin_lock(&uart->lock);
        memcpy(uart->staging, buffer + *transmitted_count, chunk);
        uart->pending = chunk;
        uart->accepted = 0;
        uart->active = true;
        k_spin_unlock(&uart->lock, key);
        uart_irq_tx_enable(uart->device);
        int result = k_sem_take(&uart->completion, K_MSEC(remaining));
        key = k_spin_lock(&uart->lock);
        uart->active = false;
        uart_irq_tx_disable(uart->device);
        uint32_t accepted = uart->accepted;
        k_spin_unlock(&uart->lock, key);
        *transmitted_count += accepted;
        if (result != 0) { return HAL_STATUS_TIMEOUT; }
        if (accepted != chunk) { return HAL_STATUS_ERROR; }
    }
    return HAL_STATUS_OK;
}
