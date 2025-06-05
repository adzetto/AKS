/**
 * @file aks_queue.c
 * @brief Queue (ring buffer) implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Queue structure definition */
typedef struct {
    uint8_t* buffer;           /**< Data buffer */
    uint16_t size;             /**< Queue size in bytes */
    uint16_t element_size;     /**< Size of each element */
    uint16_t count;            /**< Number of elements in queue */
    uint16_t capacity;         /**< Maximum number of elements */
    uint16_t head;             /**< Head index */
    uint16_t tail;             /**< Tail index */
    bool overflow_mode;        /**< Overwrite old data on overflow */
    bool initialized;          /**< Initialization flag */
} aks_queue_t;

/* Maximum number of queues */
#define AKS_MAX_QUEUES          8
#define AKS_DEFAULT_QUEUE_SIZE  256

/* Static queue instances */
static aks_queue_t g_queues[AKS_MAX_QUEUES] = {0};
static uint8_t g_queue_buffers[AKS_MAX_QUEUES][AKS_DEFAULT_QUEUE_SIZE] = {0};
static bool g_queue_module_initialized = false;

/* Private function prototypes */
static uint16_t aks_queue_get_free_index(void);
static bool aks_queue_is_valid_handle(uint16_t handle);

/**
 * @brief Initialize queue module
 */
aks_result_t aks_queue_init(void)
{
    if (g_queue_module_initialized) {
        return AKS_OK;
    }
    
    /* Initialize all queue structures */
    for (uint8_t i = 0; i < AKS_MAX_QUEUES; i++) {
        memset(&g_queues[i], 0, sizeof(aks_queue_t));
        memset(g_queue_buffers[i], 0, AKS_DEFAULT_QUEUE_SIZE);
    }
    
    g_queue_module_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Create a new queue
 */
aks_result_t aks_queue_create(uint16_t element_size, uint16_t capacity, 
                              bool overflow_mode, uint16_t* handle)
{
    if (!g_queue_module_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (element_size == 0 || capacity == 0 || handle == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Check if total size fits in default buffer */
    uint16_t total_size = element_size * capacity;
    if (total_size > AKS_DEFAULT_QUEUE_SIZE) {
        return AKS_ERROR_OVERFLOW;
    }
    
    /* Find free queue slot */
    uint16_t index = aks_queue_get_free_index();
    if (index >= AKS_MAX_QUEUES) {
        return AKS_ERROR_NO_MEMORY;
    }
    
    /* Initialize queue */
    aks_queue_t* queue = &g_queues[index];
    queue->buffer = g_queue_buffers[index];
    queue->size = total_size;
    queue->element_size = element_size;
    queue->capacity = capacity;
    queue->count = 0;
    queue->head = 0;
    queue->tail = 0;
    queue->overflow_mode = overflow_mode;
    queue->initialized = true;
    
    *handle = index;
    
    return AKS_OK;
}

/**
 * @brief Destroy queue
 */
aks_result_t aks_queue_destroy(uint16_t handle)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    
    /* Clear queue structure */
    memset(queue, 0, sizeof(aks_queue_t));
    memset(g_queue_buffers[handle], 0, AKS_DEFAULT_QUEUE_SIZE);
    
    return AKS_OK;
}

/**
 * @brief Add element to queue
 */
aks_result_t aks_queue_enqueue(uint16_t handle, const void* element)
{
    if (!aks_queue_is_valid_handle(handle) || element == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    
    /* Check if queue is full */
    if (queue->count >= queue->capacity) {
        if (!queue->overflow_mode) {
            return AKS_ERROR_OVERFLOW;
        }
        
        /* In overflow mode, advance tail to overwrite oldest element */
        queue->tail = (queue->tail + 1) % queue->capacity;
    } else {
        queue->count++;
    }
    
    /* Copy element to queue */
    uint8_t* dest = queue->buffer + (queue->head * queue->element_size);
    memcpy(dest, element, queue->element_size);
    
    /* Advance head */
    queue->head = (queue->head + 1) % queue->capacity;
    
    return AKS_OK;
}

/**
 * @brief Remove element from queue
 */
aks_result_t aks_queue_dequeue(uint16_t handle, void* element)
{
    if (!aks_queue_is_valid_handle(handle) || element == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    
    /* Check if queue is empty */
    if (queue->count == 0) {
        return AKS_ERROR_UNDERFLOW;
    }
    
    /* Copy element from queue */
    uint8_t* src = queue->buffer + (queue->tail * queue->element_size);
    memcpy(element, src, queue->element_size);
    
    /* Advance tail and decrement count */
    queue->tail = (queue->tail + 1) % queue->capacity;
    queue->count--;
    
    return AKS_OK;
}

/**
 * @brief Peek at front element without removing it
 */
aks_result_t aks_queue_peek(uint16_t handle, void* element)
{
    if (!aks_queue_is_valid_handle(handle) || element == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    
    /* Check if queue is empty */
    if (queue->count == 0) {
        return AKS_ERROR_UNDERFLOW;
    }
    
    /* Copy element from queue without removing it */
    uint8_t* src = queue->buffer + (queue->tail * queue->element_size);
    memcpy(element, src, queue->element_size);
    
    return AKS_OK;
}

/**
 * @brief Get queue size (number of elements)
 */
uint16_t aks_queue_size(uint16_t handle)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return 0;
    }
    
    return g_queues[handle].count;
}

/**
 * @brief Get queue capacity
 */
uint16_t aks_queue_capacity(uint16_t handle)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return 0;
    }
    
    return g_queues[handle].capacity;
}

/**
 * @brief Check if queue is empty
 */
bool aks_queue_is_empty(uint16_t handle)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return true;
    }
    
    return (g_queues[handle].count == 0);
}

/**
 * @brief Check if queue is full
 */
bool aks_queue_is_full(uint16_t handle)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return false;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    return (queue->count >= queue->capacity);
}

/**
 * @brief Clear queue
 */
aks_result_t aks_queue_clear(uint16_t handle)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    
    queue->count = 0;
    queue->head = 0;
    queue->tail = 0;
    
    /* Optionally clear buffer data */
    memset(queue->buffer, 0, queue->size);
    
    return AKS_OK;
}

/**
 * @brief Get available space in queue
 */
uint16_t aks_queue_available_space(uint16_t handle)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return 0;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    return (queue->capacity - queue->count);
}

/**
 * @brief Bulk enqueue multiple elements
 */
aks_result_t aks_queue_enqueue_bulk(uint16_t handle, const void* elements, uint16_t count)
{
    if (!aks_queue_is_valid_handle(handle) || elements == NULL || count == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    const uint8_t* src = (const uint8_t*)elements;
    
    for (uint16_t i = 0; i < count; i++) {
        aks_result_t result = aks_queue_enqueue(handle, src + (i * queue->element_size));
        if (result != AKS_OK && result != AKS_ERROR_OVERFLOW) {
            return result;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Bulk dequeue multiple elements
 */
aks_result_t aks_queue_dequeue_bulk(uint16_t handle, void* elements, uint16_t max_count, uint16_t* actual_count)
{
    if (!aks_queue_is_valid_handle(handle) || elements == NULL || actual_count == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    uint8_t* dest = (uint8_t*)elements;
    uint16_t count = 0;
    
    /* Dequeue up to max_count elements or until queue is empty */
    uint16_t to_dequeue = AKS_MIN(max_count, queue->count);
    
    for (uint16_t i = 0; i < to_dequeue; i++) {
        aks_result_t result = aks_queue_dequeue(handle, dest + (i * queue->element_size));
        if (result == AKS_OK) {
            count++;
        } else {
            break;
        }
    }
    
    *actual_count = count;
    
    return (count > 0) ? AKS_OK : AKS_ERROR_UNDERFLOW;
}

/**
 * @brief Set overflow mode
 */
aks_result_t aks_queue_set_overflow_mode(uint16_t handle, bool overflow_mode)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_queues[handle].overflow_mode = overflow_mode;
    
    return AKS_OK;
}

/**
 * @brief Get queue statistics
 */
aks_result_t aks_queue_get_stats(uint16_t handle, uint16_t* size, uint16_t* capacity, 
                                 uint16_t* available, bool* overflow_mode)
{
    if (!aks_queue_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_queue_t* queue = &g_queues[handle];
    
    if (size != NULL) {
        *size = queue->count;
    }
    
    if (capacity != NULL) {
        *capacity = queue->capacity;
    }
    
    if (available != NULL) {
        *available = queue->capacity - queue->count;
    }
    
    if (overflow_mode != NULL) {
        *overflow_mode = queue->overflow_mode;
    }
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Find free queue index
 */
static uint16_t aks_queue_get_free_index(void)
{
    for (uint16_t i = 0; i < AKS_MAX_QUEUES; i++) {
        if (!g_queues[i].initialized) {
            return i;
        }
    }
    
    return AKS_MAX_QUEUES; /* No free slot */
}

/**
 * @brief Validate queue handle
 */
static bool aks_queue_is_valid_handle(uint16_t handle)
{
    if (!g_queue_module_initialized) {
        return false;
    }
    
    if (handle >= AKS_MAX_QUEUES) {
        return false;
    }
    
    return g_queues[handle].initialized;
}
