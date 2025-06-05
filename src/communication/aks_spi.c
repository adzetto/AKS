/**
 * @file aks_spi.c
 * @brief SPI communication implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_spi.h"
#include <string.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Private variables */
static aks_spi_status_t g_spi_status[AKS_SPI_MAX_INSTANCES] = {0};
static aks_spi_config_t g_spi_config[AKS_SPI_MAX_INSTANCES] = {0};

/* HAL SPI handles */
#ifdef USE_HAL_DRIVER
static SPI_HandleTypeDef g_hspi[AKS_SPI_MAX_INSTANCES];
#endif

/* Callback functions */
static aks_spi_tx_complete_callback_t g_tx_callbacks[AKS_SPI_MAX_INSTANCES] = {NULL};
static aks_spi_rx_complete_callback_t g_rx_callbacks[AKS_SPI_MAX_INSTANCES] = {NULL};
static aks_spi_txrx_complete_callback_t g_txrx_callbacks[AKS_SPI_MAX_INSTANCES] = {NULL};
static aks_spi_error_callback_t g_error_callbacks[AKS_SPI_MAX_INSTANCES] = {NULL};

/* Private function prototypes */
static aks_result_t aks_spi_validate_instance(aks_spi_instance_t instance);
static void aks_spi_cs_control(const aks_spi_device_t* device, bool active);
static uint32_t aks_spi_get_hal_instance_base(aks_spi_instance_t instance);

/**
 * @brief Initialize SPI instance
 */
aks_result_t aks_spi_init(aks_spi_instance_t instance, const aks_spi_config_t* config)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (g_spi_status[instance].initialized) {
        return AKS_ERROR_BUSY;
    }
    
    /* Copy configuration */
    g_spi_config[instance] = *config;
    
#ifdef USE_HAL_DRIVER
    /* Configure HAL SPI */
    SPI_HandleTypeDef* hspi = &g_hspi[instance];
    
    hspi->Instance = (SPI_TypeDef*)aks_spi_get_hal_instance_base(instance);
    
    /* Mode configuration */
    hspi->Init.Mode = (config->mode == AKS_SPI_MODE_MASTER) ? 
                      SPI_MODE_MASTER : SPI_MODE_SLAVE;
    
    /* Direction and data size */
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize = (config->data_size == AKS_SPI_DATASIZE_8BIT) ? 
                          SPI_DATASIZE_8BIT : SPI_DATASIZE_16BIT;
    
    /* Clock configuration */
    hspi->Init.CLKPolarity = (config->clock_polarity == AKS_SPI_CPOL_LOW) ? 
                             SPI_POLARITY_LOW : SPI_POLARITY_HIGH;
    hspi->Init.CLKPhase = (config->clock_phase == AKS_SPI_CPHA_1EDGE) ? 
                          SPI_PHASE_1EDGE : SPI_PHASE_2EDGE;
    
    /* First bit configuration */
    hspi->Init.FirstBit = (config->first_bit == AKS_SPI_FIRSTBIT_MSB) ? 
                          SPI_FIRSTBIT_MSB : SPI_FIRSTBIT_LSB;
    
    /* NSS configuration */
    switch (config->nss_management) {
        case AKS_SPI_NSS_SOFT:
            hspi->Init.NSS = SPI_NSS_SOFT;
            break;
        case AKS_SPI_NSS_HARD_INPUT:
            hspi->Init.NSS = SPI_NSS_HARD_INPUT;
            break;
        case AKS_SPI_NSS_HARD_OUTPUT:
            hspi->Init.NSS = SPI_NSS_HARD_OUTPUT;
            break;
    }
    
    /* Baudrate prescaler */
    hspi->Init.BaudRatePrescaler = config->baudrate_prescaler;
    
    /* CRC configuration */
    if (config->crc_enabled) {
        hspi->Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
        hspi->Init.CRCPolynomial = 7;
    } else {
        hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    }
    
    /* Initialize HAL SPI */
    if (HAL_SPI_Init(hspi) != HAL_OK) {
        return AKS_ERROR;
    }
#endif
    
    /* Initialize status */
    memset(&g_spi_status[instance], 0, sizeof(aks_spi_status_t));
    g_spi_status[instance].initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize SPI instance
 */
aks_result_t aks_spi_deinit(aks_spi_instance_t instance)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    if (HAL_SPI_DeInit(&g_hspi[instance]) != HAL_OK) {
        return AKS_ERROR;
    }
#endif
    
    /* Clear status */
    memset(&g_spi_status[instance], 0, sizeof(aks_spi_status_t));
    
    /* Clear callbacks */
    g_tx_callbacks[instance] = NULL;
    g_rx_callbacks[instance] = NULL;
    g_txrx_callbacks[instance] = NULL;
    g_error_callbacks[instance] = NULL;
    
    return AKS_OK;
}

/**
 * @brief Transmit data via SPI
 */
aks_result_t aks_spi_transmit(aks_spi_instance_t instance, const uint8_t* data, 
                              uint16_t length, uint32_t timeout_ms)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (g_spi_status[instance].busy) {
        return AKS_ERROR_BUSY;
    }
    
    g_spi_status[instance].busy = true;
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_result = HAL_SPI_Transmit(&g_hspi[instance], (uint8_t*)data, length, timeout_ms);
    
    if (hal_result == HAL_OK) {
        g_spi_status[instance].bytes_transmitted += length;
        result = AKS_OK;
    } else if (hal_result == HAL_TIMEOUT) {
        result = AKS_ERROR_TIMEOUT;
    } else {
        g_spi_status[instance].transfer_errors++;
        result = AKS_ERROR;
    }
#else
    /* Software implementation for testing */
    (void)timeout_ms;
    g_spi_status[instance].bytes_transmitted += length;
    result = AKS_OK;
#endif
    
    g_spi_status[instance].busy = false;
    
    return result;
}

/**
 * @brief Receive data via SPI
 */
aks_result_t aks_spi_receive(aks_spi_instance_t instance, uint8_t* data, 
                             uint16_t length, uint32_t timeout_ms)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (g_spi_status[instance].busy) {
        return AKS_ERROR_BUSY;
    }
    
    g_spi_status[instance].busy = true;
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_result = HAL_SPI_Receive(&g_hspi[instance], data, length, timeout_ms);
    
    if (hal_result == HAL_OK) {
        g_spi_status[instance].bytes_received += length;
        result = AKS_OK;
    } else if (hal_result == HAL_TIMEOUT) {
        result = AKS_ERROR_TIMEOUT;
    } else {
        g_spi_status[instance].transfer_errors++;
        result = AKS_ERROR;
    }
#else
    /* Software implementation for testing */
    (void)timeout_ms;
    memset(data, 0, length);
    g_spi_status[instance].bytes_received += length;
    result = AKS_OK;
#endif
    
    g_spi_status[instance].busy = false;
    
    return result;
}

/**
 * @brief Transmit and receive data via SPI
 */
aks_result_t aks_spi_transmit_receive(aks_spi_instance_t instance, const uint8_t* tx_data, 
                                      uint8_t* rx_data, uint16_t length, uint32_t timeout_ms)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized || tx_data == NULL || rx_data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (g_spi_status[instance].busy) {
        return AKS_ERROR_BUSY;
    }
    
    g_spi_status[instance].busy = true;
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_result = HAL_SPI_TransmitReceive(&g_hspi[instance], 
                                                          (uint8_t*)tx_data, rx_data, 
                                                          length, timeout_ms);
    
    if (hal_result == HAL_OK) {
        g_spi_status[instance].bytes_transmitted += length;
        g_spi_status[instance].bytes_received += length;
        result = AKS_OK;
    } else if (hal_result == HAL_TIMEOUT) {
        result = AKS_ERROR_TIMEOUT;
    } else {
        g_spi_status[instance].transfer_errors++;
        result = AKS_ERROR;
    }
#else
    /* Software implementation for testing */
    (void)timeout_ms;
    memcpy(rx_data, tx_data, length);
    g_spi_status[instance].bytes_transmitted += length;
    g_spi_status[instance].bytes_received += length;
    result = AKS_OK;
#endif
    
    g_spi_status[instance].busy = false;
    
    return result;
}

/**
 * @brief Transmit data via SPI using DMA
 */
aks_result_t aks_spi_transmit_dma(aks_spi_instance_t instance, const uint8_t* data, uint16_t length)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (!g_spi_config[instance].enable_dma) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
    if (g_spi_status[instance].busy) {
        return AKS_ERROR_BUSY;
    }
    
    g_spi_status[instance].busy = true;
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_result = HAL_SPI_Transmit_DMA(&g_hspi[instance], (uint8_t*)data, length);
    
    if (hal_result != HAL_OK) {
        g_spi_status[instance].busy = false;
        g_spi_status[instance].transfer_errors++;
        return AKS_ERROR;
    }
#else
    /* Software fallback */
    g_spi_status[instance].bytes_transmitted += length;
    g_spi_status[instance].busy = false;
#endif
    
    return AKS_OK;
}

/**
 * @brief Receive data via SPI using DMA
 */
aks_result_t aks_spi_receive_dma(aks_spi_instance_t instance, uint8_t* data, uint16_t length)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (!g_spi_config[instance].enable_dma) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
    if (g_spi_status[instance].busy) {
        return AKS_ERROR_BUSY;
    }
    
    g_spi_status[instance].busy = true;
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_result = HAL_SPI_Receive_DMA(&g_hspi[instance], data, length);
    
    if (hal_result != HAL_OK) {
        g_spi_status[instance].busy = false;
        g_spi_status[instance].transfer_errors++;
        return AKS_ERROR;
    }
#else
    /* Software fallback */
    memset(data, 0, length);
    g_spi_status[instance].bytes_received += length;
    g_spi_status[instance].busy = false;
#endif
    
    return AKS_OK;
}

/**
 * @brief Transmit and receive data via SPI using DMA
 */
aks_result_t aks_spi_transmit_receive_dma(aks_spi_instance_t instance, const uint8_t* tx_data, 
                                          uint8_t* rx_data, uint16_t length)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized || tx_data == NULL || rx_data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (!g_spi_config[instance].enable_dma) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
    if (g_spi_status[instance].busy) {
        return AKS_ERROR_BUSY;
    }
    
    g_spi_status[instance].busy = true;
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_result = HAL_SPI_TransmitReceive_DMA(&g_hspi[instance], 
                                                              (uint8_t*)tx_data, rx_data, length);
    
    if (hal_result != HAL_OK) {
        g_spi_status[instance].busy = false;
        g_spi_status[instance].transfer_errors++;
        return AKS_ERROR;
    }
#else
    /* Software fallback */
    memcpy(rx_data, tx_data, length);
    g_spi_status[instance].bytes_transmitted += length;
    g_spi_status[instance].bytes_received += length;
    g_spi_status[instance].busy = false;
#endif
    
    return AKS_OK;
}

/**
 * @brief Device-specific SPI communication
 */
aks_result_t aks_spi_device_transfer(const aks_spi_device_t* device, const uint8_t* tx_data, 
                                     uint8_t* rx_data, uint16_t length, uint32_t timeout_ms)
{
    if (device == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Assert CS */
    aks_spi_cs_control(device, true);
    
    aks_result_t result;
    
    if (tx_data != NULL && rx_data != NULL) {
        /* Full-duplex transfer */
        result = aks_spi_transmit_receive(device->instance, tx_data, rx_data, length, timeout_ms);
    } else if (tx_data != NULL) {
        /* Transmit only */
        result = aks_spi_transmit(device->instance, tx_data, length, timeout_ms);
    } else if (rx_data != NULL) {
        /* Receive only */
        result = aks_spi_receive(device->instance, rx_data, length, timeout_ms);
    } else {
        result = AKS_ERROR_INVALID_PARAM;
    }
    
    /* Deassert CS */
    aks_spi_cs_control(device, false);
    
    return result;
}

/**
 * @brief Write single byte to SPI device
 */
aks_result_t aks_spi_device_write_byte(const aks_spi_device_t* device, uint8_t byte, uint32_t timeout_ms)
{
    return aks_spi_device_transfer(device, &byte, NULL, 1, timeout_ms);
}

/**
 * @brief Read single byte from SPI device
 */
aks_result_t aks_spi_device_read_byte(const aks_spi_device_t* device, uint8_t* byte, uint32_t timeout_ms)
{
    uint8_t dummy = 0xFF;
    return aks_spi_device_transfer(device, &dummy, byte, 1, timeout_ms);
}

/**
 * @brief Write register to SPI device
 */
aks_result_t aks_spi_device_write_register(const aks_spi_device_t* device, uint8_t reg_addr, 
                                           uint8_t data, uint32_t timeout_ms)
{
    uint8_t tx_data[2] = {reg_addr, data};
    return aks_spi_device_transfer(device, tx_data, NULL, 2, timeout_ms);
}

/**
 * @brief Read register from SPI device
 */
aks_result_t aks_spi_device_read_register(const aks_spi_device_t* device, uint8_t reg_addr, 
                                          uint8_t* data, uint32_t timeout_ms)
{
    uint8_t tx_data[2] = {reg_addr, 0xFF};
    uint8_t rx_data[2] = {0};
    
    aks_result_t result = aks_spi_device_transfer(device, tx_data, rx_data, 2, timeout_ms);
    if (result == AKS_OK) {
        *data = rx_data[1];
    }
    
    return result;
}

/**
 * @brief Write multiple bytes to register
 */
aks_result_t aks_spi_device_write_registers(const aks_spi_device_t* device, uint8_t reg_addr, 
                                            const uint8_t* data, uint16_t length, uint32_t timeout_ms)
{
    if (device == NULL || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint8_t tx_buffer[AKS_SPI_MAX_TRANSFER_SIZE];
    
    if (length + 1 > AKS_SPI_MAX_TRANSFER_SIZE) {
        return AKS_ERROR_OVERFLOW;
    }
    
    tx_buffer[0] = reg_addr;
    memcpy(&tx_buffer[1], data, length);
    
    return aks_spi_device_transfer(device, tx_buffer, NULL, length + 1, timeout_ms);
}

/**
 * @brief Read multiple bytes from register
 */
aks_result_t aks_spi_device_read_registers(const aks_spi_device_t* device, uint8_t reg_addr, 
                                           uint8_t* data, uint16_t length, uint32_t timeout_ms)
{
    if (device == NULL || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint8_t tx_buffer[AKS_SPI_MAX_TRANSFER_SIZE];
    uint8_t rx_buffer[AKS_SPI_MAX_TRANSFER_SIZE];
    
    if (length + 1 > AKS_SPI_MAX_TRANSFER_SIZE) {
        return AKS_ERROR_OVERFLOW;
    }
    
    tx_buffer[0] = reg_addr;
    memset(&tx_buffer[1], 0xFF, length);
    
    aks_result_t result = aks_spi_device_transfer(device, tx_buffer, rx_buffer, length + 1, timeout_ms);
    if (result == AKS_OK) {
        memcpy(data, &rx_buffer[1], length);
    }
    
    return result;
}

/**
 * @brief Check if SPI is busy
 */
bool aks_spi_is_busy(aks_spi_instance_t instance)
{
    if (aks_spi_validate_instance(instance) != AKS_OK) {
        return false;
    }
    
    return g_spi_status[instance].busy;
}

/**
 * @brief Abort SPI transfer
 */
aks_result_t aks_spi_abort_transfer(aks_spi_instance_t instance)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    if (HAL_SPI_Abort(&g_hspi[instance]) != HAL_OK) {
        return AKS_ERROR;
    }
#endif
    
    g_spi_status[instance].busy = false;
    
    return AKS_OK;
}

/**
 * @brief Get SPI status
 */
aks_result_t aks_spi_get_status(aks_spi_instance_t instance, aks_spi_status_t* status)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *status = g_spi_status[instance];
    
    return AKS_OK;
}

/**
 * @brief Set baudrate prescaler
 */
aks_result_t aks_spi_set_baudrate_prescaler(aks_spi_instance_t instance, uint32_t prescaler)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    if (!g_spi_status[instance].initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_spi_config[instance].baudrate_prescaler = prescaler;
    
#ifdef USE_HAL_DRIVER
    /* Note: This would require reinitializing the SPI peripheral */
    /* For now, just store the value */
#endif
    
    return AKS_OK;
}

/**
 * @brief Register TX complete callback
 */
aks_result_t aks_spi_register_tx_callback(aks_spi_instance_t instance, 
                                          aks_spi_tx_complete_callback_t callback)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    g_tx_callbacks[instance] = callback;
    
    return AKS_OK;
}

/**
 * @brief Register RX complete callback
 */
aks_result_t aks_spi_register_rx_callback(aks_spi_instance_t instance, 
                                          aks_spi_rx_complete_callback_t callback)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    g_rx_callbacks[instance] = callback;
    
    return AKS_OK;
}

/**
 * @brief Register TX/RX complete callback
 */
aks_result_t aks_spi_register_txrx_callback(aks_spi_instance_t instance, 
                                            aks_spi_txrx_complete_callback_t callback)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    g_txrx_callbacks[instance] = callback;
    
    return AKS_OK;
}

/**
 * @brief Register error callback
 */
aks_result_t aks_spi_register_error_callback(aks_spi_instance_t instance, 
                                             aks_spi_error_callback_t callback)
{
    aks_result_t result = aks_spi_validate_instance(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    g_error_callbacks[instance] = callback;
    
    return AKS_OK;
}

/**
 * @brief Control device chip select
 */
aks_result_t aks_spi_device_set_cs(const aks_spi_device_t* device, bool active)
{
    if (device == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_spi_cs_control(device, active);
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Validate SPI instance
 */
static aks_result_t aks_spi_validate_instance(aks_spi_instance_t instance)
{
    if (instance >= AKS_SPI_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    return AKS_OK;
}

/**
 * @brief Control chip select signal
 */
static void aks_spi_cs_control(const aks_spi_device_t* device, bool active)
{
    if (device == NULL) {
        return;
    }
    
#ifdef USE_HAL_DRIVER
    GPIO_PinState pin_state = (device->cs_active_low) ? 
                              (active ? GPIO_PIN_RESET : GPIO_PIN_SET) :
                              (active ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    HAL_GPIO_WritePin((GPIO_TypeDef*)device->cs_port, device->cs_pin, pin_state);
#else
    (void)active; /* Suppress warning in test builds */
#endif
}

/**
 * @brief Get HAL SPI instance base address
 */
static uint32_t aks_spi_get_hal_instance_base(aks_spi_instance_t instance)
{
#ifdef USE_HAL_DRIVER
    switch (instance) {
        case AKS_SPI_1:
            return (uint32_t)SPI1;
        case AKS_SPI_2:
            return (uint32_t)SPI2;
        case AKS_SPI_3:
            return (uint32_t)SPI3;
        default:
            return 0;
    }
#else
    (void)instance;
    return 0;
#endif
}

/* HAL callback implementations */
#ifdef USE_HAL_DRIVER

/**
 * @brief HAL SPI TX complete callback
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < AKS_SPI_MAX_INSTANCES; i++) {
        if (hspi == &g_hspi[i]) {
            g_spi_status[i].busy = false;
            g_spi_status[i].bytes_transmitted += hspi->TxXferSize;
            
            if (g_tx_callbacks[i] != NULL) {
                g_tx_callbacks[i]((aks_spi_instance_t)i);
            }
            break;
        }
    }
}

/**
 * @brief HAL SPI RX complete callback
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < AKS_SPI_MAX_INSTANCES; i++) {
        if (hspi == &g_hspi[i]) {
            g_spi_status[i].busy = false;
            g_spi_status[i].bytes_received += hspi->RxXferSize;
            
            if (g_rx_callbacks[i] != NULL) {
                g_rx_callbacks[i]((aks_spi_instance_t)i);
            }
            break;
        }
    }
}

/**
 * @brief HAL SPI TX/RX complete callback
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < AKS_SPI_MAX_INSTANCES; i++) {
        if (hspi == &g_hspi[i]) {
            g_spi_status[i].busy = false;
            g_spi_status[i].bytes_transmitted += hspi->TxXferSize;
            g_spi_status[i].bytes_received += hspi->RxXferSize;
            
            if (g_txrx_callbacks[i] != NULL) {
                g_txrx_callbacks[i]((aks_spi_instance_t)i);
            }
            break;
        }
    }
}

/**
 * @brief HAL SPI error callback
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < AKS_SPI_MAX_INSTANCES; i++) {
        if (hspi == &g_hspi[i]) {
            g_spi_status[i].busy = false;
            g_spi_status[i].transfer_errors++;
            
            /* Determine error type */
            uint32_t error = 0;
            if (hspi->ErrorCode & HAL_SPI_ERROR_CRC) {
                g_spi_status[i].crc_errors++;
                error |= 1;
            }
            if (hspi->ErrorCode & HAL_SPI_ERROR_OVR) {
                g_spi_status[i].overrun_errors++;
                error |= 2;
            }
            
            if (g_error_callbacks[i] != NULL) {
                g_error_callbacks[i]((aks_spi_instance_t)i, error);
            }
            break;
        }
    }
}

#endif /* USE_HAL_DRIVER */
