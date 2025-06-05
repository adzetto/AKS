/**
 * @file aks_uart.h
 * @brief UART communication interface for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_UART_H
#define AKS_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"

/**
 * @defgroup AKS_UART UART Communication
 * @brief UART communication interface
 * @{
 */

/* UART Configuration */
#define AKS_UART_MAX_INSTANCES      4
#define AKS_UART_BUFFER_SIZE        256
#define AKS_UART_TX_TIMEOUT         1000
#define AKS_UART_RX_TIMEOUT         1000

/* UART Instance IDs */
typedef enum {
    AKS_UART_1 = 0,
    AKS_UART_2 = 1,
    AKS_UART_3 = 2,
    AKS_UART_6 = 3
} aks_uart_instance_t;

/* UART Parity */
typedef enum {
    AKS_UART_PARITY_NONE = 0,
    AKS_UART_PARITY_EVEN = 1,
    AKS_UART_PARITY_ODD = 2
} aks_uart_parity_t;

/* UART Stop Bits */
typedef enum {
    AKS_UART_STOPBITS_1 = 0,
    AKS_UART_STOPBITS_2 = 1
} aks_uart_stopbits_t;

/* UART Flow Control */
typedef enum {
    AKS_UART_FLOWCONTROL_NONE = 0,
    AKS_UART_FLOWCONTROL_RTS = 1,
    AKS_UART_FLOWCONTROL_CTS = 2,
    AKS_UART_FLOWCONTROL_RTS_CTS = 3
} aks_uart_flowcontrol_t;

/* UART Configuration */
typedef struct {
    uint32_t baudrate;                      /**< Baudrate */
    uint8_t data_bits;                      /**< Data bits (7 or 8) */
    aks_uart_parity_t parity;               /**< Parity */
    aks_uart_stopbits_t stop_bits;          /**< Stop bits */
    aks_uart_flowcontrol_t flow_control;    /**< Flow control */
    bool enable_dma;                        /**< DMA enable */
    uint16_t rx_buffer_size;                /**< RX buffer size */
    uint16_t tx_buffer_size;                /**< TX buffer size */
} aks_uart_config_t;

/* UART Status */
typedef struct {
    bool initialized;           /**< Initialization status */
    bool transmitting;         /**< Transmission in progress */
    bool receiving;            /**< Reception in progress */
    uint32_t bytes_transmitted; /**< Total bytes transmitted */
    uint32_t bytes_received;   /**< Total bytes received */
    uint32_t tx_errors;        /**< Transmission errors */
    uint32_t rx_errors;        /**< Reception errors */
    uint32_t overrun_errors;   /**< Overrun errors */
    uint32_t frame_errors;     /**< Frame errors */
    uint32_t parity_errors;    /**< Parity errors */
} aks_uart_status_t;

/* UART Callback Types */
typedef void (*aks_uart_tx_complete_callback_t)(aks_uart_instance_t instance);
typedef void (*aks_uart_rx_complete_callback_t)(aks_uart_instance_t instance, uint8_t* data, uint16_t length);
typedef void (*aks_uart_error_callback_t)(aks_uart_instance_t instance, uint32_t error);

/**
 * @brief Initialize UART instance
 * @param instance UART instance
 * @param config Pointer to UART configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_init(aks_uart_instance_t instance, const aks_uart_config_t* config);

/**
 * @brief Deinitialize UART instance
 * @param instance UART instance
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_deinit(aks_uart_instance_t instance);

/**
 * @brief Transmit data via UART
 * @param instance UART instance
 * @param data Pointer to data buffer
 * @param length Data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_transmit(aks_uart_instance_t instance, const uint8_t* data, 
                               uint16_t length, uint32_t timeout_ms);

/**
 * @brief Receive data via UART
 * @param instance UART instance
 * @param data Pointer to receive buffer
 * @param length Expected data length
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_receive(aks_uart_instance_t instance, uint8_t* data, 
                              uint16_t length, uint32_t timeout_ms);

/**
 * @brief Transmit data via UART using DMA
 * @param instance UART instance
 * @param data Pointer to data buffer
 * @param length Data length
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_transmit_dma(aks_uart_instance_t instance, const uint8_t* data, uint16_t length);

/**
 * @brief Receive data via UART using DMA
 * @param instance UART instance
 * @param data Pointer to receive buffer
 * @param length Expected data length
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_receive_dma(aks_uart_instance_t instance, uint8_t* data, uint16_t length);

/**
 * @brief Transmit single byte via UART
 * @param instance UART instance
 * @param byte Byte to transmit
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_transmit_byte(aks_uart_instance_t instance, uint8_t byte, uint32_t timeout_ms);

/**
 * @brief Receive single byte via UART
 * @param instance UART instance
 * @param byte Pointer to receive byte
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_receive_byte(aks_uart_instance_t instance, uint8_t* byte, uint32_t timeout_ms);

/**
 * @brief Transmit string via UART
 * @param instance UART instance
 * @param string Null-terminated string
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_transmit_string(aks_uart_instance_t instance, const char* string, uint32_t timeout_ms);

/**
 * @brief Check if data is available for reception
 * @param instance UART instance
 * @return Number of bytes available
 */
uint16_t aks_uart_get_rx_data_available(aks_uart_instance_t instance);

/**
 * @brief Check if transmission is complete
 * @param instance UART instance
 * @return true if transmission complete, false otherwise
 */
bool aks_uart_is_tx_complete(aks_uart_instance_t instance);

/**
 * @brief Abort ongoing transmission
 * @param instance UART instance
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_abort_transmit(aks_uart_instance_t instance);

/**
 * @brief Abort ongoing reception
 * @param instance UART instance
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_abort_receive(aks_uart_instance_t instance);

/**
 * @brief Flush UART buffers
 * @param instance UART instance
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_flush(aks_uart_instance_t instance);

/**
 * @brief Get UART status
 * @param instance UART instance
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_get_status(aks_uart_instance_t instance, aks_uart_status_t* status);

/**
 * @brief Set UART baudrate
 * @param instance UART instance
 * @param baudrate New baudrate
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_set_baudrate(aks_uart_instance_t instance, uint32_t baudrate);

/**
 * @brief Register TX complete callback
 * @param instance UART instance
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_register_tx_callback(aks_uart_instance_t instance, 
                                           aks_uart_tx_complete_callback_t callback);

/**
 * @brief Register RX complete callback
 * @param instance UART instance
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_register_rx_callback(aks_uart_instance_t instance, 
                                           aks_uart_rx_complete_callback_t callback);

/**
 * @brief Register error callback
 * @param instance UART instance
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_register_error_callback(aks_uart_instance_t instance, 
                                              aks_uart_error_callback_t callback);

/**
 * @brief Enable/disable UART interrupts
 * @param instance UART instance
 * @param enable Enable/disable flag
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_uart_set_interrupt_enable(aks_uart_instance_t instance, bool enable);

/**
 * @brief printf-style formatted output via UART
 * @param instance UART instance
 * @param format Format string
 * @param ... Variable arguments
 * @return Number of characters transmitted, negative on error
 */
int aks_uart_printf(aks_uart_instance_t instance, const char* format, ...);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_UART_H */