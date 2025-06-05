/**
 * @file aks_uart.c
 * @brief UART communication implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* UART Handle Structure */
typedef struct {
    bool initialized;
    UART_HandleTypeDef huart;
    aks_uart_config_t config;
    aks_uart_status_t status;
    uint8_t* rx_buffer;
    uint8_t* tx_buffer;
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t tx_head;
    uint16_t tx_tail;
    aks_uart_tx_complete_callback_t tx_callback;
    aks_uart_rx_complete_callback_t rx_callback;
    aks_uart_error_callback_t error_callback;
} aks_uart_handle_t;

/* Private variables */
static aks_uart_handle_t g_uart_handles[AKS_UART_MAX_INSTANCES];

/* HAL UART instances */
#ifdef USE_HAL_DRIVER
static USART_TypeDef* const g_uart_instances[AKS_UART_MAX_INSTANCES] = {
    USART1, USART2, USART3, USART6
};
#endif

/* Private function prototypes */
static aks_result_t aks_uart_hal_init(aks_uart_instance_t instance);
static void aks_uart_buffer_init(aks_uart_instance_t instance);
static uint16_t aks_uart_buffer_put(aks_uart_instance_t instance, const uint8_t* data, uint16_t length, bool is_tx);
static uint16_t aks_uart_buffer_get(aks_uart_instance_t instance, uint8_t* data, uint16_t length, bool is_tx);
static uint16_t aks_uart_buffer_available(aks_uart_instance_t instance, bool is_tx);

/**
 * @brief Initialize UART instance
 */
aks_result_t aks_uart_init(aks_uart_instance_t instance, const aks_uart_config_t* config)
{
    if (instance >= AKS_UART_MAX_INSTANCES || config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (handle->initialized) {
        return AKS_ERROR_BUSY;
    }
    
    /* Copy configuration */
    handle->config = *config;
    
    /* Initialize buffers */
    aks_uart_buffer_init(instance);
    
    /* Initialize HAL */
    aks_result_t result = aks_uart_hal_init(instance);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize status */
    memset(&handle->status, 0, sizeof(aks_uart_status_t));
    handle->status.initialized = true;
    
    handle->initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize UART instance
 */
aks_result_t aks_uart_deinit(aks_uart_instance_t instance)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    HAL_UART_DeInit(&handle->huart);
#endif
    
    /* Free buffers */
    if (handle->rx_buffer) {
        free(handle->rx_buffer);
        handle->rx_buffer = NULL;
    }
    
    if (handle->tx_buffer) {
        free(handle->tx_buffer);
        handle->tx_buffer = NULL;
    }
    
    /* Reset handle */
    memset(handle, 0, sizeof(aks_uart_handle_t));
    
    return AKS_OK;
}

/**
 * @brief Transmit data via UART
 */
aks_result_t aks_uart_transmit(aks_uart_instance_t instance, const uint8_t* data, 
                               uint16_t length, uint32_t timeout_ms)
{
    if (instance >= AKS_UART_MAX_INSTANCES || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_status = HAL_UART_Transmit(&handle->huart, (uint8_t*)data, length, timeout_ms);
    
    if (hal_status == HAL_OK) {
        handle->status.bytes_transmitted += length;
        return AKS_OK;
    } else if (hal_status == HAL_TIMEOUT) {
        return AKS_ERROR_TIMEOUT;
    } else {
        handle->status.tx_errors++;
        return AKS_ERROR;
    }
#else
    /* Software implementation for testing */
    handle->status.bytes_transmitted += length;
    return AKS_OK;
#endif
}

/**
 * @brief Receive data via UART
 */
aks_result_t aks_uart_receive(aks_uart_instance_t instance, uint8_t* data, 
                              uint16_t length, uint32_t timeout_ms)
{
    if (instance >= AKS_UART_MAX_INSTANCES || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_status = HAL_UART_Receive(&handle->huart, data, length, timeout_ms);
    
    if (hal_status == HAL_OK) {
        handle->status.bytes_received += length;
        return AKS_OK;
    } else if (hal_status == HAL_TIMEOUT) {
        return AKS_ERROR_TIMEOUT;
    } else {
        handle->status.rx_errors++;
        return AKS_ERROR;
    }
#else
    /* Software implementation for testing */
    handle->status.bytes_received += length;
    return AKS_OK;
#endif
}

/**
 * @brief Transmit data via UART using DMA
 */
aks_result_t aks_uart_transmit_dma(aks_uart_instance_t instance, const uint8_t* data, uint16_t length)
{
    if (instance >= AKS_UART_MAX_INSTANCES || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (!handle->config.enable_dma) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(&handle->huart, (uint8_t*)data, length);
    
    if (hal_status == HAL_OK) {
        handle->status.transmitting = true;
        return AKS_OK;
    } else {
        handle->status.tx_errors++;
        return AKS_ERROR;
    }
#else
    /* Software implementation for testing */
    handle->status.transmitting = true;
    return AKS_OK;
#endif
}

/**
 * @brief Receive data via UART using DMA
 */
aks_result_t aks_uart_receive_dma(aks_uart_instance_t instance, uint8_t* data, uint16_t length)
{
    if (instance >= AKS_UART_MAX_INSTANCES || data == NULL || length == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (!handle->config.enable_dma) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
#ifdef USE_HAL_DRIVER
    HAL_StatusTypeDef hal_status = HAL_UART_Receive_DMA(&handle->huart, data, length);
    
    if (hal_status == HAL_OK) {
        handle->status.receiving = true;
        return AKS_OK;
    } else {
        handle->status.rx_errors++;
        return AKS_ERROR;
    }
#else
    /* Software implementation for testing */
    handle->status.receiving = true;
    return AKS_OK;
#endif
}

/**
 * @brief Transmit single byte via UART
 */
aks_result_t aks_uart_transmit_byte(aks_uart_instance_t instance, uint8_t byte, uint32_t timeout_ms)
{
    return aks_uart_transmit(instance, &byte, 1, timeout_ms);
}

/**
 * @brief Receive single byte via UART
 */
aks_result_t aks_uart_receive_byte(aks_uart_instance_t instance, uint8_t* byte, uint32_t timeout_ms)
{
    return aks_uart_receive(instance, byte, 1, timeout_ms);
}

/**
 * @brief Transmit string via UART
 */
aks_result_t aks_uart_transmit_string(aks_uart_instance_t instance, const char* string, uint32_t timeout_ms)
{
    if (string == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint16_t length = strlen(string);
    return aks_uart_transmit(instance, (const uint8_t*)string, length, timeout_ms);
}

/**
 * @brief Check if data is available for reception
 */
uint16_t aks_uart_get_rx_data_available(aks_uart_instance_t instance)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return 0;
    }
    
    return aks_uart_buffer_available(instance, false);
}

/**
 * @brief Check if transmission is complete
 */
bool aks_uart_is_tx_complete(aks_uart_instance_t instance)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return false;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    return !handle->status.transmitting;
}

/**
 * @brief Abort ongoing transmission
 */
aks_result_t aks_uart_abort_transmit(aks_uart_instance_t instance)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    HAL_UART_AbortTransmit(&handle->huart);
#endif
    
    handle->status.transmitting = false;
    return AKS_OK;
}

/**
 * @brief Abort ongoing reception
 */
aks_result_t aks_uart_abort_receive(aks_uart_instance_t instance)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    HAL_UART_AbortReceive(&handle->huart);
#endif
    
    handle->status.receiving = false;
    return AKS_OK;
}

/**
 * @brief Flush UART buffers
 */
aks_result_t aks_uart_flush(aks_uart_instance_t instance)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Reset buffer pointers */
    handle->rx_head = 0;
    handle->rx_tail = 0;
    handle->tx_head = 0;
    handle->tx_tail = 0;
    
    return AKS_OK;
}

/**
 * @brief Get UART status
 */
aks_result_t aks_uart_get_status(aks_uart_instance_t instance, aks_uart_status_t* status)
{
    if (instance >= AKS_UART_MAX_INSTANCES || status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    *status = handle->status;
    return AKS_OK;
}

/**
 * @brief Set UART baudrate
 */
aks_result_t aks_uart_set_baudrate(aks_uart_instance_t instance, uint32_t baudrate)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    handle->config.baudrate = baudrate;
    
#ifdef USE_HAL_DRIVER
    handle->huart.Init.BaudRate = baudrate;
    return (HAL_UART_Init(&handle->huart) == HAL_OK) ? AKS_OK : AKS_ERROR;
#else
    return AKS_OK;
#endif
}

/**
 * @brief Register transmission complete callback
 */
aks_result_t aks_uart_register_tx_callback(aks_uart_instance_t instance, 
                                           aks_uart_tx_complete_callback_t callback)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_uart_handles[instance].tx_callback = callback;
    return AKS_OK;
}

/**
 * @brief Register reception complete callback
 */
aks_result_t aks_uart_register_rx_callback(aks_uart_instance_t instance, 
                                           aks_uart_rx_complete_callback_t callback)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_uart_handles[instance].rx_callback = callback;
    return AKS_OK;
}

/**
 * @brief Register error callback
 */
aks_result_t aks_uart_register_error_callback(aks_uart_instance_t instance, 
                                              aks_uart_error_callback_t callback)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_uart_handles[instance].error_callback = callback;
    return AKS_OK;
}

/**
 * @brief Set interrupt enable
 */
aks_result_t aks_uart_set_interrupt_enable(aks_uart_instance_t instance, bool enable)
{
    if (instance >= AKS_UART_MAX_INSTANCES) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
#ifdef USE_HAL_DRIVER
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    if (!handle->initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (enable) {
        /* Enable UART interrupts */
        
        /* Enable RXNE (Receive Data Register Not Empty) interrupt */
        __HAL_UART_ENABLE_IT(&handle->huart, UART_IT_RXNE);
        
        /* Enable TXE (Transmit Data Register Empty) interrupt */
        __HAL_UART_ENABLE_IT(&handle->huart, UART_IT_TXE);
        
        /* Enable Error interrupts */
        __HAL_UART_ENABLE_IT(&handle->huart, UART_IT_ERR);
        
        /* Enable IDLE line detection interrupt */
        __HAL_UART_ENABLE_IT(&handle->huart, UART_IT_IDLE);
        
        /* Configure NVIC for UART interrupts */
        IRQn_Type irq_number;
        switch (instance) {
            case AKS_UART_INSTANCE_1:
                irq_number = USART1_IRQn;
                break;
            case AKS_UART_INSTANCE_2:
                irq_number = USART2_IRQn;
                break;
            case AKS_UART_INSTANCE_3:
                irq_number = USART3_IRQn;
                break;
            default:
                return AKS_ERROR_INVALID_PARAM;
        }
        
        HAL_NVIC_SetPriority(irq_number, 5, 0);
        HAL_NVIC_EnableIRQ(irq_number);
        
    } else {
        /* Disable UART interrupts */
        
        /* Disable all UART interrupts */
        __HAL_UART_DISABLE_IT(&handle->huart, UART_IT_RXNE);
        __HAL_UART_DISABLE_IT(&handle->huart, UART_IT_TXE);
        __HAL_UART_DISABLE_IT(&handle->huart, UART_IT_ERR);
        __HAL_UART_DISABLE_IT(&handle->huart, UART_IT_IDLE);
        
        /* Disable NVIC for UART interrupts */
        IRQn_Type irq_number;
        switch (instance) {
            case AKS_UART_INSTANCE_1:
                irq_number = USART1_IRQn;
                break;
            case AKS_UART_INSTANCE_2:
                irq_number = USART2_IRQn;
                break;
            case AKS_UART_INSTANCE_3:
                irq_number = USART3_IRQn;
                break;
            default:
                return AKS_ERROR_INVALID_PARAM;
        }
        
        HAL_NVIC_DisableIRQ(irq_number);
    }
    
    return AKS_OK;
#else
    /* For testing without HAL */
    return AKS_OK;
#endif
}

/**
 * @brief Printf function for UART
 */
int aks_uart_printf(aks_uart_instance_t instance, const char* format, ...)
{
    char buffer[256];
    va_list args;
    
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (length > 0) {
        aks_uart_transmit_string(instance, buffer, 1000);
    }
    
    return length;
}

/* Private Functions */

/**
 * @brief Initialize HAL UART
 */
static aks_result_t aks_uart_hal_init(aks_uart_instance_t instance)
{
#ifdef USE_HAL_DRIVER
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    handle->huart.Instance = g_uart_instances[instance];
    handle->huart.Init.BaudRate = handle->config.baudrate;
    handle->huart.Init.WordLength = (handle->config.data_bits == 8) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
    
    switch (handle->config.parity) {
        case AKS_UART_PARITY_NONE:
            handle->huart.Init.Parity = UART_PARITY_NONE;
            break;
        case AKS_UART_PARITY_EVEN:
            handle->huart.Init.Parity = UART_PARITY_EVEN;
            break;
        case AKS_UART_PARITY_ODD:
            handle->huart.Init.Parity = UART_PARITY_ODD;
            break;
    }
    
    handle->huart.Init.StopBits = (handle->config.stop_bits == AKS_UART_STOPBITS_1) ? 
                                  UART_STOPBITS_1 : UART_STOPBITS_2;
    
    switch (handle->config.flow_control) {
        case AKS_UART_FLOWCONTROL_NONE:
            handle->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
            break;
        case AKS_UART_FLOWCONTROL_RTS:
            handle->huart.Init.HwFlowCtl = UART_HWCONTROL_RTS;
            break;
        case AKS_UART_FLOWCONTROL_CTS:
            handle->huart.Init.HwFlowCtl = UART_HWCONTROL_CTS;
            break;
        case AKS_UART_FLOWCONTROL_RTS_CTS:
            handle->huart.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
            break;
    }
    
    handle->huart.Init.Mode = UART_MODE_TX_RX;
    handle->huart.Init.OverSampling = UART_OVERSAMPLING_16;
    
    return (HAL_UART_Init(&handle->huart) == HAL_OK) ? AKS_OK : AKS_ERROR;
#else
    return AKS_OK;
#endif
}

/**
 * @brief Initialize UART buffers
 */
static void aks_uart_buffer_init(aks_uart_instance_t instance)
{
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    
    /* Allocate buffers if not already allocated */
    if (handle->rx_buffer == NULL) {
        handle->rx_buffer = malloc(handle->config.rx_buffer_size);
    }
    
    if (handle->tx_buffer == NULL) {
        handle->tx_buffer = malloc(handle->config.tx_buffer_size);
    }
    
    /* Initialize buffer pointers */
    handle->rx_head = 0;
    handle->rx_tail = 0;
    handle->tx_head = 0;
    handle->tx_tail = 0;
}

/**
 * @brief Put data into buffer
 */
static uint16_t aks_uart_buffer_put(aks_uart_instance_t instance, const uint8_t* data, 
                                    uint16_t length, bool is_tx)
{
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    uint8_t* buffer = is_tx ? handle->tx_buffer : handle->rx_buffer;
    uint16_t buffer_size = is_tx ? handle->config.tx_buffer_size : handle->config.rx_buffer_size;
    uint16_t* head = is_tx ? &handle->tx_head : &handle->rx_head;
    
    uint16_t bytes_written = 0;
    
    for (uint16_t i = 0; i < length; i++) {
        uint16_t next_head = (*head + 1) % buffer_size;
        
        if (next_head != (is_tx ? handle->tx_tail : handle->rx_tail)) {
            buffer[*head] = data[i];
            *head = next_head;
            bytes_written++;
        } else {
            break; // Buffer full
        }
    }
    
    return bytes_written;
}

/**
 * @brief Get data from buffer
 */
static uint16_t aks_uart_buffer_get(aks_uart_instance_t instance, uint8_t* data, 
                                    uint16_t length, bool is_tx)
{
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    uint8_t* buffer = is_tx ? handle->tx_buffer : handle->rx_buffer;
    uint16_t buffer_size = is_tx ? handle->config.tx_buffer_size : handle->config.rx_buffer_size;
    uint16_t* tail = is_tx ? &handle->tx_tail : &handle->rx_tail;
    uint16_t head = is_tx ? handle->tx_head : handle->rx_head;
    
    uint16_t bytes_read = 0;
    
    for (uint16_t i = 0; i < length; i++) {
        if (*tail != head) {
            data[i] = buffer[*tail];
            *tail = (*tail + 1) % buffer_size;
            bytes_read++;
        } else {
            break; // Buffer empty
        }
    }
    
    return bytes_read;
}

/**
 * @brief Get available data in buffer
 */
static uint16_t aks_uart_buffer_available(aks_uart_instance_t instance, bool is_tx)
{
    aks_uart_handle_t* handle = &g_uart_handles[instance];
    uint16_t buffer_size = is_tx ? handle->config.tx_buffer_size : handle->config.rx_buffer_size;
    uint16_t head = is_tx ? handle->tx_head : handle->rx_head;
    uint16_t tail = is_tx ? handle->tx_tail : handle->rx_tail;
    
    if (head >= tail) {
        return head - tail;
    } else {
        return buffer_size - tail + head;
    }
}