/**
 * @file aks_spi.h
 * @brief SPI communication interface for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_SPI_H
#define AKS_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"

/**
 * @defgroup AKS_SPI SPI Communication
 * @brief SPI communication interface for LoRa and other peripherals
 * @{
 */

/* SPI Configuration */
#define AKS_SPI_MAX_INSTANCES       3
#define AKS_SPI_MAX_TRANSFER_SIZE   256
#define AKS_SPI_DEFAULT_TIMEOUT     1000

/* SPI Instance IDs */
typedef enum {
    AKS_SPI_1 = 0,
    AKS_SPI_2 = 1,
    AKS_SPI_3 = 2
} aks_spi_instance_t;

/* SPI Mode */
typedef enum {
    AKS_SPI_MODE_MASTER = 0,
    AKS_SPI_MODE_SLAVE = 1
} aks_spi_mode_t;

/* SPI Clock Polarity */
typedef enum {
    AKS_SPI_CPOL_LOW = 0,       /**< Clock polarity low */
    AKS_SPI_CPOL_HIGH = 1       /**< Clock polarity high */
} aks_spi_cpol_t;

/* SPI Clock Phase */
typedef enum {
    AKS_SPI_CPHA_1EDGE = 0,     /**< Data capture on first edge */
    AKS_SPI_CPHA_2EDGE = 1      /**< Data capture on second edge */
} aks_spi_cpha_t;

/* SPI Data Size */
typedef enum {
    AKS_SPI_DATASIZE_8BIT = 0,
    AKS_SPI_DATASIZE_16BIT = 1
} aks_spi_datasize_t;

/* SPI First Bit */
typedef enum {
    AKS_SPI_FIRSTBIT_MSB = 0,
    AKS_SPI_FIRSTBIT_LSB = 1
} aks_spi_firstbit_t;

/* SPI NSS Management */
typedef enum {
    AKS_SPI_NSS_SOFT = 0,
    AKS_SPI_NSS_HARD_INPUT = 1,
    AKS_SPI_NSS_HARD_OUTPUT = 2
} aks_spi_nss_t;

/* SPI Configuration */
typedef struct {
    aks_spi_mode_t mode;            /**< Master/Slave mode */
    uint32_t baudrate_prescaler;    /**< Baudrate prescaler */
    aks_spi_cpol_t clock_polarity;  /**< Clock polarity */
    aks_spi_cpha_t clock_phase;     /**< Clock phase */
    aks_spi_datasize_t data_size;   /**< Data size */
    aks_spi_firstbit_t first_bit;   /**< First bit transmission */
    aks_spi_nss_t nss_management;   /**< NSS management */
    bool enable_dma;                /**< DMA enable */
    bool crc_enabled;               /**< CRC enable */
} aks_spi_config_t;

/* SPI Status */
typedef struct {
    bool initialized;           /**< Initialization status */
    bool busy;                 /**< SPI busy flag */
    uint32_t bytes_transmitted; /**< Total bytes transmitted */
    uint32_t bytes_received;   /**< Total bytes received */
    uint32_t transfer_errors;  /**< Transfer errors */
    uint32_t crc_errors;       /**< CRC errors */
    uint32_t overrun_errors;   /**< Overrun errors */
} aks_spi_status_t;

/* SPI Device Handle */
typedef struct {
    aks_spi_instance_t instance;    /**< SPI instance */
    uint8_t cs_pin;                /**< Chip select pin */
    uint8_t cs_port;               /**< Chip select port */
    uint32_t max_speed;            /**< Maximum transfer speed */
    bool cs_active_low;            /**< CS active low flag */
} aks_spi_device_t;

/* SPI Callback Types */
typedef void (*aks_spi_tx_complete_callback_t)(aks_spi_instance_t instance);
typedef void (*aks_spi_rx_complete_callback_t)(aks_spi_instance_t instance);
typedef void (*aks_spi_txrx_complete_callback_t)(aks_spi_instance_t instance);
typedef void (*aks_spi_error_callback_t)(aks_spi_instance_t instance, uint32_t error);

/**
 * @brief Initialize SPI instance
 * @param instance SPI instance
 * @param config Pointer to SPI configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_init(aks_spi_instance_t instance, const aks_spi_config_t* config);

/**
 * @brief Deinitialize SPI instance
 * @param instance SPI instance
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_deinit(aks_spi_instance_t instance);

/**
 * @brief Transmit data via SPI
 * @param instance SPI instance
 * @param data Pointer to data buffer
 * @param length Data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_transmit(aks_spi_instance_t instance, const uint8_t* data, 
                              uint16_t length, uint32_t timeout_ms);

/**
 * @brief Receive data via SPI
 * @param instance SPI instance
 * @param data Pointer to receive buffer
 * @param length Data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_receive(aks_spi_instance_t instance, uint8_t* data, 
                             uint16_t length, uint32_t timeout_ms);

/**
 * @brief Transmit and receive data via SPI
 * @param instance SPI instance
 * @param tx_data Pointer to transmit buffer
 * @param rx_data Pointer to receive buffer
 * @param length Data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_transmit_receive(aks_spi_instance_t instance, const uint8_t* tx_data, 
                                      uint8_t* rx_data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Transmit data via SPI using DMA
 * @param instance SPI instance
 * @param data Pointer to data buffer
 * @param length Data length
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_transmit_dma(aks_spi_instance_t instance, const uint8_t* data, uint16_t length);

/**
 * @brief Receive data via SPI using DMA
 * @param instance SPI instance
 * @param data Pointer to receive buffer
 * @param length Data length
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_receive_dma(aks_spi_instance_t instance, uint8_t* data, uint16_t length);

/**
 * @brief Transmit and receive data via SPI using DMA
 * @param instance SPI instance
 * @param tx_data Pointer to transmit buffer
 * @param rx_data Pointer to receive buffer
 * @param length Data length
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_transmit_receive_dma(aks_spi_instance_t instance, const uint8_t* tx_data, 
                                          uint8_t* rx_data, uint16_t length);

/**
 * @brief Device-specific SPI communication
 * @param device Pointer to SPI device handle
 * @param tx_data Pointer to transmit buffer
 * @param rx_data Pointer to receive buffer (can be NULL for TX only)
 * @param length Data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_transfer(const aks_spi_device_t* device, const uint8_t* tx_data, 
                                     uint8_t* rx_data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Write single byte to SPI device
 * @param device Pointer to SPI device handle
 * @param byte Byte to write
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_write_byte(const aks_spi_device_t* device, uint8_t byte, uint32_t timeout_ms);

/**
 * @brief Read single byte from SPI device
 * @param device Pointer to SPI device handle
 * @param byte Pointer to receive byte
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_read_byte(const aks_spi_device_t* device, uint8_t* byte, uint32_t timeout_ms);

/**
 * @brief Write register to SPI device
 * @param device Pointer to SPI device handle
 * @param reg_addr Register address
 * @param data Data to write
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_write_register(const aks_spi_device_t* device, uint8_t reg_addr, 
                                           uint8_t data, uint32_t timeout_ms);

/**
 * @brief Read register from SPI device
 * @param device Pointer to SPI device handle
 * @param reg_addr Register address
 * @param data Pointer to receive data
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_read_register(const aks_spi_device_t* device, uint8_t reg_addr, 
                                          uint8_t* data, uint32_t timeout_ms);

/**
 * @brief Write multiple bytes to register
 * @param device Pointer to SPI device handle
 * @param reg_addr Register address
 * @param data Pointer to data buffer
 * @param length Data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_write_registers(const aks_spi_device_t* device, uint8_t reg_addr, 
                                            const uint8_t* data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Read multiple bytes from register
 * @param device Pointer to SPI device handle
 * @param reg_addr Register address
 * @param data Pointer to receive buffer
 * @param length Data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_read_registers(const aks_spi_device_t* device, uint8_t reg_addr, 
                                           uint8_t* data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Check if SPI is busy
 * @param instance SPI instance
 * @return true if busy, false otherwise
 */
bool aks_spi_is_busy(aks_spi_instance_t instance);

/**
 * @brief Abort ongoing SPI transfer
 * @param instance SPI instance
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_abort_transfer(aks_spi_instance_t instance);

/**
 * @brief Get SPI status
 * @param instance SPI instance
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_get_status(aks_spi_instance_t instance, aks_spi_status_t* status);

/**
 * @brief Set SPI baudrate prescaler
 * @param instance SPI instance
 * @param prescaler Baudrate prescaler
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_set_baudrate_prescaler(aks_spi_instance_t instance, uint32_t prescaler);

/**
 * @brief Register TX complete callback
 * @param instance SPI instance
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_register_tx_callback(aks_spi_instance_t instance, 
                                          aks_spi_tx_complete_callback_t callback);

/**
 * @brief Register RX complete callback
 * @param instance SPI instance
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_register_rx_callback(aks_spi_instance_t instance, 
                                          aks_spi_rx_complete_callback_t callback);

/**
 * @brief Register TX/RX complete callback
 * @param instance SPI instance
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_register_txrx_callback(aks_spi_instance_t instance, 
                                            aks_spi_txrx_complete_callback_t callback);

/**
 * @brief Register error callback
 * @param instance SPI instance
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_register_error_callback(aks_spi_instance_t instance, 
                                             aks_spi_error_callback_t callback);

/**
 * @brief Manually control chip select
 * @param device Pointer to SPI device handle
 * @param active Chip select state (true = active, false = inactive)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_spi_device_set_cs(const aks_spi_device_t* device, bool active);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_SPI_H */