#include "uart/hal_uart.h"
#include "hal_nrf_connect_private.h"
#include <nrfx_uarte.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#define UART_NODE             DT_ALIAS(hal_uart0)
#define UART_ABORT_TIMEOUT_MS 10U  /** Bounded wait for STOPTX after a transfer timeout. */
#define UART_CLOCK_TIMEOUT_MS 100U /** Maximum wait for the external high-frequency crystal. */

BUILD_ASSERT(!DT_NODE_HAS_STATUS(UART_NODE, okay), "Reserve UARTE0 for the framework; disable the Zephyr UART driver");
BUILD_ASSERT(DT_SAME_NODE(UART_NODE, DT_NODELABEL(uart0)), "HAL UART 0 maps to physical UARTE0");
PINCTRL_DT_DEFINE(UART_NODE);
static const nrfx_uarte_t device = NRFX_UARTE_INSTANCE(0);
static bool claimed;

struct hal_uart {
    struct k_sem completion;
    struct onoff_client clock_request;
    uint32_t timeout;
    uint32_t pending;
    uint32_t transferred;
    hal_status_t result;
    bool active;
    bool initialized;
    uint8_t staging[64];
};

_Static_assert(sizeof(hal_uart_t) <= UINT32_MAX, "UART object exceeds arena size range");
_Static_assert(_Alignof(hal_uart_t) <= _Alignof(max_align_t), "Unsupported UART alignment");

static bool request_clock(struct onoff_manager *clock, struct onoff_client *request) {
    if (!clock) {
        return false;
    }
    sys_notify_init_spinwait(&request->notify);
    if (onoff_request(clock, request) < 0) {
        return false;
    }
    int result;
    const int64_t deadline = k_uptime_get() + UART_CLOCK_TIMEOUT_MS;
    while (sys_notify_fetch_result(&request->notify, &result) != 0) {
        if (k_uptime_get() >= deadline) {
            (void)onoff_cancel_or_release(clock, request);
            return false;
        }
        k_msleep(1);
    }
    if (result < 0) {
        (void)onoff_cancel_or_release(clock, request);
        return false;
    }
    return true;
}

static void completed(const nrfx_uarte_event_t *event, void *context) {
    hal_uart_t *uart = context;
    if (event->type != NRFX_UARTE_EVT_TX_DONE || event->data.tx.p_buffer != uart->staging) {
        return;
    }
    unsigned int key = irq_lock();
    if (uart->active) {
        uart->transferred = (uint32_t)MIN(event->data.tx.length, uart->pending);
        uart->result = event->data.tx.flags & NRFX_UARTE_TX_DONE_ABORTED ? HAL_STATUS_ERROR : HAL_STATUS_OK;
        uart->active = false;
        k_sem_give(&uart->completion);
    }
    irq_unlock(key);
}

hal_status_t hal_uart_config(hal_uart_t **uart, const hal_uart_config_t *config) {
    if (uart) {
        *uart = NULL;
    }
    if (!uart || !config || config->instance >= 2 || !config->baudrate ||
        !hal_nrf_connect_timeout_valid(config->timeout) ||
        hal_nrf_connect_same_pin(config->transmit_pin, config->receive_pin) ||
        (config->data_width != HAL_UART_DATA_WIDTH_7 && config->data_width != HAL_UART_DATA_WIDTH_8) ||
        (config->parity != HAL_UART_PARITY_NONE && config->parity != HAL_UART_PARITY_EVEN && config->parity != HAL_UART_PARITY_ODD) ||
        (config->stop_count != HAL_UART_STOP_COUNT_1 && config->stop_count != HAL_UART_STOP_COUNT_2)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    hal_status_t status = hal_nrf_connect_pin_validate(config->transmit_pin);
    if (status == HAL_STATUS_OK) {
        status = hal_nrf_connect_pin_validate(config->receive_pin);
    }
    if (status != HAL_STATUS_OK) {
        return status;
    }
    if (config->instance != 0 || config->baudrate != 115200 || config->data_width != HAL_UART_DATA_WIDTH_8 ||
        config->parity != HAL_UART_PARITY_NONE || config->stop_count != HAL_UART_STOP_COUNT_1 ||
        !hal_nrf_connect_route(PINCTRL_DT_DEV_CONFIG_GET(UART_NODE), NRF_FUN_UART_TX, config->transmit_pin) ||
        !hal_nrf_connect_route(PINCTRL_DT_DEV_CONFIG_GET(UART_NODE), NRF_FUN_UART_RX, config->receive_pin)) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    status = hal_nrf_connect_construction();
    if (status != HAL_STATUS_OK) {
        return status;
    }
    if (claimed || nrfx_uarte_init_check(&device)) {
        return HAL_STATUS_RESOURCE_BUSY;
    }
    status = hal_nrf_connect_claim(config->transmit_pin, &config->receive_pin);
    if (status != HAL_STATUS_OK) {
        return status;
    }
    void *storage;
    status = hal_nrf_connect_memory_status(hal_memory_alloc(sizeof(hal_uart_t), &storage));
    if (status != HAL_STATUS_OK) {
        goto release;
    }
    hal_uart_t *object = storage;
    memset(object, 0, sizeof(*object));
    object->timeout = config->timeout;
    k_sem_init(&object->completion, 0, 1);
    // Retain the SDK-managed HFXO reference: HFINT alone does not ensure UART accuracy.
    struct onoff_manager *clock = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!request_clock(clock, &object->clock_request)) {
        status = HAL_STATUS_ERROR;
        goto release;
    }
    if (pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(UART_NODE), PINCTRL_STATE_DEFAULT) != 0) {
        (void)onoff_release(clock);
        status = HAL_STATUS_ERROR;
        goto release;
    }
    nrfx_uarte_config_t hardware = NRFX_UARTE_DEFAULT_CONFIG(hal_nrf_connect_pin_number(config->transmit_pin),
                                                             hal_nrf_connect_pin_number(config->receive_pin));
    hardware.skip_gpio_cfg = true;
    hardware.skip_psel_cfg = true;
    hardware.p_context = object;
    IRQ_CONNECT(DT_IRQN(UART_NODE), DT_IRQ(UART_NODE, priority), nrfx_uarte_0_irq_handler, 0, 0);
    // The SDK may partly initialize hardware on failure; retain ownership until restart.
    claimed = true;
    status = hal_nrf_connect_sdk_status(nrfx_uarte_init(&device, &hardware, completed));
    if (status != HAL_STATUS_OK) {
        return status;
    }
    object->initialized = true;
    *uart = object;
    return HAL_STATUS_OK;
release:
    hal_nrf_connect_release(config->transmit_pin);
    hal_nrf_connect_release(config->receive_pin);
    return status;
}

static void stop_transfer(hal_uart_t *uart) {
    // Asynchronous abort avoids the SDK's unbounded synchronous STOPTX wait.
    (void)nrfx_uarte_tx_abort(&device, false);
    (void)k_sem_take(&uart->completion, K_MSEC(UART_ABORT_TIMEOUT_MS));
    unsigned int key = irq_lock();
    if (uart->active) {
        // Keep permanent DMA storage and pin ownership, but forbid any buffer reuse.
        // A late IRQ can finish the hardware transfer without touching caller memory.
        uart->transferred = (uint32_t)MIN(nrfy_uarte_tx_amount_get(device.p_reg), uart->pending);
        uart->active = false;
        uart->initialized = false;
    }
    irq_unlock(key);
}

hal_status_t hal_uart_write(hal_uart_t *uart, const uint8_t *buffer, uint32_t size, uint32_t *transmitted_count) {
    if (transmitted_count) {
        *transmitted_count = 0;
    }
    if (!uart || !transmitted_count || (!buffer && size)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (!uart->initialized) {
        return HAL_STATUS_NOT_INITIALIZED;
    }
    if (!size) {
        return HAL_STATUS_OK;
    }
    if (k_is_in_isr()) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    const int64_t deadline = k_uptime_get() + uart->timeout;
    while (*transmitted_count < size) {
        int64_t remaining = deadline - k_uptime_get();
        if (remaining <= 0) {
            return HAL_STATUS_TIMEOUT;
        }
        uint32_t chunk = MIN(size - *transmitted_count, sizeof(uart->staging));
        k_sem_reset(&uart->completion);
        unsigned int key = irq_lock();
        memcpy(uart->staging, buffer + *transmitted_count, chunk);
        uart->pending = chunk;
        uart->transferred = 0;
        uart->result = HAL_STATUS_ERROR;
        uart->active = true;
        irq_unlock(key);
        hal_status_t status = hal_nrf_connect_sdk_status(nrfx_uarte_tx(&device, uart->staging, chunk, 0));
        if (status != HAL_STATUS_OK) {
            key = irq_lock();
            uart->active = false;
            irq_unlock(key);
            return status;
        }
        if (k_sem_take(&uart->completion, K_MSEC(remaining)) != 0) {
            stop_transfer(uart);
            *transmitted_count += uart->transferred;
            return HAL_STATUS_TIMEOUT;
        }
        *transmitted_count += uart->transferred;
        if (uart->result != HAL_STATUS_OK || uart->transferred != chunk) {
            return HAL_STATUS_ERROR;
        }
    }
    return HAL_STATUS_OK;
}
