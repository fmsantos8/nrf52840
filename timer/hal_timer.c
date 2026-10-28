#include "timer/hal_timer.h"
#include "hal_nrf_connect_private.h"
#include <nrfx_timer.h>

#define TIMER_NODE DT_NODELABEL(timer2)
BUILD_ASSERT(!DT_NODE_HAS_STATUS(TIMER_NODE, okay), "Reserve TIMER2 for the framework");
static const nrfx_timer_t device = NRFX_TIMER_INSTANCE(2);
static bool claimed;

struct hal_timer {
    hal_timer_callback_t callback;
    bool initialized;
    bool started;
};

_Static_assert(sizeof(hal_timer_t) <= UINT32_MAX, "TIMER object exceeds arena size range");
_Static_assert(_Alignof(hal_timer_t) <= _Alignof(max_align_t), "Unsupported timer alignment");

static void expiration(nrf_timer_event_t event, void *context) {
    hal_timer_t *timer = context;
    if (event == NRF_TIMER_EVENT_COMPARE0 && timer->started) { timer->callback(); }
}

hal_status_t hal_timer_config(hal_timer_t **timer, const hal_timer_config_t *config) {
    if (timer) { *timer = NULL; }
    if (!timer || !config || !config->callback || !config->period || config->instance >= 5) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    if (config->instance != 2 || config->period > UINT32_MAX / 1000U) { return HAL_STATUS_NOT_SUPPORTED; }
    hal_status_t status = hal_nrf_connect_construction();
    if (status != HAL_STATUS_OK) { return status; }
    if (claimed || nrfx_timer_init_check(&device)) { return HAL_STATUS_RESOURCE_BUSY; }
    void *storage;
    status = hal_nrf_connect_memory_status(hal_memory_alloc(sizeof(hal_timer_t), &storage));
    if (status != HAL_STATUS_OK) { return status; }
    hal_timer_t *object = storage;
    memset(object, 0, sizeof(*object));
    object->callback = config->callback;
    nrfx_timer_config_t hardware = NRFX_TIMER_DEFAULT_CONFIG(1000000);
    hardware.bit_width = NRF_TIMER_BIT_WIDTH_32;
    hardware.p_context = object;
    IRQ_CONNECT(DT_IRQN(TIMER_NODE), DT_IRQ(TIMER_NODE, priority), nrfx_timer_2_irq_handler, 0, 0);
    status = hal_nrf_connect_sdk_status(nrfx_timer_init(&device, &hardware, expiration));
    if (status != HAL_STATUS_OK) { return status; }
    nrfx_timer_extended_compare(&device, NRF_TIMER_CC_CHANNEL0, config->period * 1000U, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    object->initialized = true;
    claimed = true;
    *timer = object;
    return HAL_STATUS_OK;
}

hal_status_t hal_timer_start(hal_timer_t *timer) {
    if (!timer) { return HAL_STATUS_INVALID_ARGUMENT; }
    if (!timer->initialized) { return HAL_STATUS_NOT_INITIALIZED; }
    if (timer->started) { return HAL_STATUS_INVALID_STATE; }
    if (k_is_in_isr()) { return HAL_STATUS_NOT_SUPPORTED; }
    unsigned int key = irq_lock();
    nrfx_timer_clear(&device);
    NVIC_ClearPendingIRQ(DT_IRQN(TIMER_NODE));
    timer->started = true;
    nrfx_timer_enable(&device);
    irq_unlock(key);
    return HAL_STATUS_OK;
}
