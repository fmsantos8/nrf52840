#include "i2c/hal_i2c.h"
#include "hal_nrf_connect_private.h"
#include <nrfx_twim.h>

#define BUS_NODE DT_ALIAS(hal_i2c0)
BUILD_ASSERT(!DT_NODE_HAS_STATUS(BUS_NODE, okay), "Reserve TWIM0 for the framework; disable the Zephyr bus driver");
BUILD_ASSERT(DT_SAME_NODE(BUS_NODE, DT_NODELABEL(i2c0)), "HAL bus 0 maps to TWIM0");
PINCTRL_DT_DEFINE(BUS_NODE);
static const nrfx_twim_t device = NRFX_TWIM_INSTANCE(0);
static bool claimed;

struct hal_i2c {
    struct k_sem completion;
    uint32_t timeout;
    hal_status_t result;
    bool initialized;
    _Alignas(4) uint8_t dma_buffer[256];
};

_Static_assert(sizeof(hal_i2c_t) <= UINT32_MAX, "I2C object exceeds arena size range");
_Static_assert(_Alignof(hal_i2c_t) <= _Alignof(max_align_t), "Unsupported I2C alignment");

static void completed(const nrfx_twim_evt_t *event, void *context) {
    hal_i2c_t *bus = context;
    bus->result = event->type == NRFX_TWIM_EVT_DONE ? HAL_STATUS_OK : HAL_STATUS_ERROR;
    k_sem_give(&bus->completion);
}

hal_status_t hal_i2c_config(hal_i2c_t **i2c, const hal_i2c_config_t *config) {
    if (i2c) { *i2c = NULL; }
    if (!i2c || !config || config->instance >= 2 || !config->frequency ||
        !hal_nrf_connect_timeout_valid(config->timeout) || hal_nrf_connect_same_pin(config->clock_pin, config->data_pin)) {
        return HAL_STATUS_INVALID_ARGUMENT;
    }
    hal_status_t status = hal_nrf_connect_pin_validate(config->clock_pin);
    if (status == HAL_STATUS_OK) { status = hal_nrf_connect_pin_validate(config->data_pin); }
    if (status != HAL_STATUS_OK) { return status; }
    if (config->instance != 0 || (config->frequency != 100000 && config->frequency != 250000 && config->frequency != 400000) ||
        !hal_nrf_connect_route(PINCTRL_DT_DEV_CONFIG_GET(BUS_NODE), NRF_FUN_TWIM_SCL, config->clock_pin) ||
        !hal_nrf_connect_route(PINCTRL_DT_DEV_CONFIG_GET(BUS_NODE), NRF_FUN_TWIM_SDA, config->data_pin)) {
        return HAL_STATUS_NOT_SUPPORTED;
    }
    status = hal_nrf_connect_construction();
    if (status != HAL_STATUS_OK) { return status; }
    if (claimed || nrfx_twim_init_check(&device)) { return HAL_STATUS_RESOURCE_BUSY; }
    status = hal_nrf_connect_claim(config->clock_pin, &config->data_pin);
    if (status != HAL_STATUS_OK) { return status; }
    void *storage;
    status = hal_nrf_connect_memory_status(hal_memory_alloc(sizeof(hal_i2c_t), &storage));
    if (status != HAL_STATUS_OK) { goto release; }
    hal_i2c_t *object = storage;
    memset(object, 0, sizeof(*object));
    object->timeout = config->timeout;
    k_sem_init(&object->completion, 0, 1);
    nrfx_twim_config_t hardware = NRFX_TWIM_DEFAULT_CONFIG(hal_nrf_connect_pin_number(config->clock_pin), hal_nrf_connect_pin_number(config->data_pin));
    hardware.frequency = config->frequency == 100000 ? NRF_TWIM_FREQ_100K :
                         config->frequency == 250000 ? NRF_TWIM_FREQ_250K : NRF_TWIM_FREQ_400K;
    hardware.skip_gpio_cfg = true;
    if (pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(BUS_NODE), PINCTRL_STATE_DEFAULT) != 0) {
        status = HAL_STATUS_ERROR;
        goto release;
    }
    IRQ_CONNECT(DT_IRQN(BUS_NODE), DT_IRQ(BUS_NODE, priority), nrfx_twim_0_irq_handler, 0, 0);
    status = hal_nrf_connect_sdk_status(nrfx_twim_init(&device, &hardware, completed, object));
    if (status != HAL_STATUS_OK) { goto release; }
    nrfx_twim_enable(&device);
    claimed = true;
    object->initialized = true;
    *i2c = object;
    return HAL_STATUS_OK;
release:
    hal_nrf_connect_release(config->clock_pin);
    hal_nrf_connect_release(config->data_pin);
    return status;
}

static hal_status_t transfer(hal_i2c_t *i2c, uint8_t address, uint8_t *buffer, uint32_t size, bool read) {
    if (!i2c || !buffer || !size || address < 0x08 || address > 0x77) { return HAL_STATUS_INVALID_ARGUMENT; }
    if (!i2c->initialized) { return HAL_STATUS_NOT_INITIALIZED; }
    if (k_is_in_isr()) { return HAL_STATUS_NOT_SUPPORTED; }
    // One DMA transaction, one STOP, and persistent storage on both directions.
    if (size > sizeof(i2c->dma_buffer)) { return HAL_STATUS_NOT_SUPPORTED; }
    if (!read) { memcpy(i2c->dma_buffer, buffer, size); }
    k_sem_reset(&i2c->completion);
    nrfx_twim_xfer_desc_t request = read ? (nrfx_twim_xfer_desc_t)NRFX_TWIM_XFER_DESC_RX(address, i2c->dma_buffer, size) :
                                          (nrfx_twim_xfer_desc_t)NRFX_TWIM_XFER_DESC_TX(address, i2c->dma_buffer, size);
    hal_status_t status = hal_nrf_connect_sdk_status(nrfx_twim_xfer(&device, &request, 0));
    if (status != HAL_STATUS_OK) { return status; }
    if (k_sem_take(&i2c->completion, K_MSEC(i2c->timeout)) != 0) {
        // Disabling resets the driver's busy state and stops EasyDMA without an unbounded STOP wait.
        unsigned int key = irq_lock();
        nrfx_twim_disable(&device);
        NVIC_ClearPendingIRQ(DT_IRQN(BUS_NODE));
        nrfx_twim_enable(&device);
        irq_unlock(key);
        return HAL_STATUS_TIMEOUT;
    }
    if (i2c->result == HAL_STATUS_OK && read) { memcpy(buffer, i2c->dma_buffer, size); }
    return i2c->result;
}

hal_status_t hal_i2c_write(hal_i2c_t *i2c, uint8_t address, const uint8_t *buffer, uint32_t size) {
    return transfer(i2c, address, (uint8_t *)buffer, size, false);
}

hal_status_t hal_i2c_read(hal_i2c_t *i2c, uint8_t address, uint8_t *buffer, uint32_t size) {
    return transfer(i2c, address, buffer, size, true);
}
