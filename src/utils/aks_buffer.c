/**
 * @file aks_buffer.c
 * @brief Buffer management implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_config.h"
#include <string.h>
#include <stdatomic.h>

/* Buffer Management Configuration */
#define AKS_BUFFER_MAX_POOLS        8           /**< Maximum memory pools */
#define AKS_BUFFER_MAX_BLOCKS       64          /**< Maximum blocks per pool */
#define AKS_BUFFER_ALIGNMENT        4           /**< Memory alignment */
#define AKS_BUFFER_GUARD_SIZE       8           /**< Guard byte size */
#define AKS_BUFFER_GUARD_PATTERN    0xDEADBEEF  /**< Guard pattern */

/* Buffer Types */
typedef enum {
    AKS_BUFFER_TYPE_CIRCULAR = 0,   /**< Circular buffer */
    AKS_BUFFER_TYPE_LINEAR,         /**< Linear buffer */
    AKS_BUFFER_TYPE_POOL,           /**< Memory pool */
    AKS_BUFFER_TYPE_STACK,          /**< Stack buffer */
    AKS_BUFFER_TYPE_QUEUE           /**< Queue buffer */
} aks_buffer_type_t;

/* Buffer Usage */
typedef enum {
    AKS_BUFFER_USAGE_CAN_TX = 0,    /**< CAN transmission buffer */
    AKS_BUFFER_USAGE_CAN_RX,        /**< CAN receive buffer */
    AKS_BUFFER_USAGE_UART_TX,       /**< UART transmission buffer */
    AKS_BUFFER_USAGE_UART_RX,       /**< UART receive buffer */
    AKS_BUFFER_USAGE_TELEMETRY,     /**< Telemetry data buffer */
    AKS_BUFFER_USAGE_LOGGING,       /**< Logging buffer */
    AKS_BUFFER_USAGE_SENSOR_DATA,   /**< Sensor data buffer */
    AKS_BUFFER_USAGE_GENERAL        /**< General purpose buffer */
} aks_buffer_usage_t;

/* Memory Block */
typedef struct aks_memory_block {
    struct aks_memory_block* next;  /**< Next block in free list */
    uint32_t guard_start;           /**< Start guard pattern */
    uint8_t data[];                 /**< Block data */
} aks_memory_block_t;

/* Memory Pool */
typedef struct {
    uint8_t* pool_memory;           /**< Pool memory base */
    aks_memory_block_t* free_list;  /**< Free block list */
    uint32_t block_size;            /**< Block size */
    uint32_t total_blocks;          /**< Total blocks */
    uint32_t free_blocks;           /**< Free blocks count */
    uint32_t allocated_blocks;      /**< Allocated blocks count */
    uint32_t peak_usage;            /**< Peak usage */
    bool initialized;               /**< Pool initialized flag */
    atomic_flag lock;               /**< Atomic lock */
} aks_memory_pool_t;

/* Circular Buffer */
typedef struct {
    uint8_t* data;                  /**< Buffer data */
    volatile uint32_t head;         /**< Head index */
    volatile uint32_t tail;         /**< Tail index */
    uint32_t size;                  /**< Buffer size */
    uint32_t element_size;          /**< Element size */
    uint32_t count;                 /**< Element count */
    bool overwrite;                 /**< Overwrite mode */
    atomic_flag lock;               /**< Atomic lock */
} aks_circular_buffer_t;

/* Buffer Handle */
typedef struct {
    aks_buffer_type_t type;         /**< Buffer type */
    aks_buffer_usage_t usage;       /**< Buffer usage */
    union {
        aks_circular_buffer_t circular; /**< Circular buffer */
        aks_memory_pool_t pool;     /**< Memory pool */
        struct {
            uint8_t* data;          /**< Linear buffer data */
            uint32_t size;          /**< Buffer size */
            uint32_t used;          /**< Used bytes */
        } linear;
    } buffer;
    bool active;                    /**< Handle active flag */
} aks_buffer_handle_t;

/* Static buffer handles */
static aks_buffer_handle_t g_buffer_handles[AKS_BUFFER_MAX_POOLS] = {0};
static bool g_buffer_initialized = false;

/* Buffer memory pools */
static uint8_t g_can_tx_pool[512] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));
static uint8_t g_can_rx_pool[1024] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));
static uint8_t g_uart_tx_pool[256] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));
static uint8_t g_uart_rx_pool[512] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));
static uint8_t g_telemetry_pool[2048] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));
static uint8_t g_logging_pool[4096] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));
static uint8_t g_sensor_pool[1024] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));
static uint8_t g_general_pool[1024] __attribute__((aligned(AKS_BUFFER_ALIGNMENT)));

/* Statistics */
static struct {
    uint32_t total_allocations;
    uint32_t total_deallocations;
    uint32_t allocation_failures;
    uint32_t buffer_overruns;
    uint32_t peak_memory_usage;
} g_buffer_stats = {0};

/* Private function prototypes */
static aks_result_t aks_buffer_create_pool(uint8_t handle_id, uint8_t* memory, uint32_t total_size, uint32_t block_size);
static aks_result_t aks_buffer_create_circular(uint8_t handle_id, uint8_t* memory, uint32_t size, uint32_t element_size);
static void* aks_buffer_pool_alloc(aks_memory_pool_t* pool);
static aks_result_t aks_buffer_pool_free(aks_memory_pool_t* pool, void* ptr);
static bool aks_buffer_validate_guard(aks_memory_block_t* block, uint32_t block_size);
static void aks_buffer_set_guard(aks_memory_block_t* block, uint32_t block_size);
static uint32_t aks_buffer_align_size(uint32_t size);

/**
 * @brief Initialize buffer management system
 */
aks_result_t aks_buffer_init(void)
{
    if (g_buffer_initialized) {
        return AKS_OK;
    }
    
    /* Clear buffer handles */
    memset(g_buffer_handles, 0, sizeof(g_buffer_handles));
    
    /* Create CAN TX buffer pool */
    aks_buffer_create_pool(0, g_can_tx_pool, sizeof(g_can_tx_pool), 64);
    g_buffer_handles[0].usage = AKS_BUFFER_USAGE_CAN_TX;
    
    /* Create CAN RX buffer pool */
    aks_buffer_create_pool(1, g_can_rx_pool, sizeof(g_can_rx_pool), 64);
    g_buffer_handles[1].usage = AKS_BUFFER_USAGE_CAN_RX;
    
    /* Create UART TX circular buffer */
    aks_buffer_create_circular(2, g_uart_tx_pool, sizeof(g_uart_tx_pool), 1);
    g_buffer_handles[2].usage = AKS_BUFFER_USAGE_UART_TX;
    
    /* Create UART RX circular buffer */
    aks_buffer_create_circular(3, g_uart_rx_pool, sizeof(g_uart_rx_pool), 1);
    g_buffer_handles[3].usage = AKS_BUFFER_USAGE_UART_RX;
    
    /* Create telemetry data pool */
    aks_buffer_create_pool(4, g_telemetry_pool, sizeof(g_telemetry_pool), 128);
    g_buffer_handles[4].usage = AKS_BUFFER_USAGE_TELEMETRY;
    
    /* Create logging circular buffer */
    aks_buffer_create_circular(5, g_logging_pool, sizeof(g_logging_pool), 1);
    g_buffer_handles[5].usage = AKS_BUFFER_USAGE_LOGGING;
    
    /* Create sensor data pool */
    aks_buffer_create_pool(6, g_sensor_pool, sizeof(g_sensor_pool), 32);
    g_buffer_handles[6].usage = AKS_BUFFER_USAGE_SENSOR_DATA;
    
    /* Create general purpose pool */
    aks_buffer_create_pool(7, g_general_pool, sizeof(g_general_pool), 64);
    g_buffer_handles[7].usage = AKS_BUFFER_USAGE_GENERAL;
    
    /* Clear statistics */
    memset(&g_buffer_stats, 0, sizeof(g_buffer_stats));
    
    g_buffer_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Allocate memory from buffer pool
 */
void* aks_buffer_alloc(aks_buffer_usage_t usage, uint32_t size)
{
    if (!g_buffer_initialized) {
        return NULL;
    }
    
    /* Find appropriate buffer */
    for (uint8_t i = 0; i < AKS_BUFFER_MAX_POOLS; i++) {
        if (g_buffer_handles[i].active && 
            g_buffer_handles[i].usage == usage &&
            g_buffer_handles[i].type == AKS_BUFFER_TYPE_POOL) {
            
            void* ptr = aks_buffer_pool_alloc(&g_buffer_handles[i].buffer.pool);
            if (ptr != NULL) {
                g_buffer_stats.total_allocations++;
                return ptr;
            }
        }
    }
    
    g_buffer_stats.allocation_failures++;
    return NULL;
}

/**
 * @brief Free memory to buffer pool
 */
aks_result_t aks_buffer_free(aks_buffer_usage_t usage, void* ptr)
{
    if (!g_buffer_initialized || ptr == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Find appropriate buffer */
    for (uint8_t i = 0; i < AKS_BUFFER_MAX_POOLS; i++) {
        if (g_buffer_handles[i].active && 
            g_buffer_handles[i].usage == usage &&
            g_buffer_handles[i].type == AKS_BUFFER_TYPE_POOL) {
            
            aks_result_t result = aks_buffer_pool_free(&g_buffer_handles[i].buffer.pool, ptr);
            if (result == AKS_OK) {
                g_buffer_stats.total_deallocations++;
                return AKS_OK;
            }
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Write data to circular buffer
 */
aks_result_t aks_buffer_write(aks_buffer_usage_t usage, const void* data, uint32_t size)
{
    if (!g_buffer_initialized || data == NULL || size == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Find circular buffer */
    for (uint8_t i = 0; i < AKS_BUFFER_MAX_POOLS; i++) {
        if (g_buffer_handles[i].active && 
            g_buffer_handles[i].usage == usage &&
            g_buffer_handles[i].type == AKS_BUFFER_TYPE_CIRCULAR) {
            
            aks_circular_buffer_t* cb = &g_buffer_handles[i].buffer.circular;
            
            /* Lock buffer */
            while (atomic_flag_test_and_set(&cb->lock)) {
                /* Spin wait */
            }
            
            uint32_t available_space = (cb->size - cb->count) * cb->element_size;
            
            if (size > available_space && !cb->overwrite) {
                atomic_flag_clear(&cb->lock);
                g_buffer_stats.buffer_overruns++;
                return AKS_ERROR_BUFFER_FULL;
            }
            
            const uint8_t* src = (const uint8_t*)data;
            uint32_t bytes_written = 0;
            
            while (bytes_written < size) {
                if (cb->count >= cb->size && cb->overwrite) {
                    /* Advance tail to make space */
                    cb->tail = (cb->tail + 1) % cb->size;
                    cb->count--;
                }
                
                if (cb->count < cb->size) {
                    cb->data[cb->head * cb->element_size + (bytes_written % cb->element_size)] = src[bytes_written];
                    bytes_written++;
                    
                    if ((bytes_written % cb->element_size) == 0) {
                        cb->head = (cb->head + 1) % cb->size;
                        cb->count++;
                    }
                } else {
                    break;
                }
            }
            
            atomic_flag_clear(&cb->lock);
            return (bytes_written == size) ? AKS_OK : AKS_ERROR_BUFFER_FULL;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Read data from circular buffer
 */
aks_result_t aks_buffer_read(aks_buffer_usage_t usage, void* data, uint32_t size, uint32_t* bytes_read)
{
    if (!g_buffer_initialized || data == NULL || size == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Find circular buffer */
    for (uint8_t i = 0; i < AKS_BUFFER_MAX_POOLS; i++) {
        if (g_buffer_handles[i].active && 
            g_buffer_handles[i].usage == usage &&
            g_buffer_handles[i].type == AKS_BUFFER_TYPE_CIRCULAR) {
            
            aks_circular_buffer_t* cb = &g_buffer_handles[i].buffer.circular;
            
            /* Lock buffer */
            while (atomic_flag_test_and_set(&cb->lock)) {
                /* Spin wait */
            }
            
            uint32_t available_data = cb->count * cb->element_size;
            uint32_t read_size = (size < available_data) ? size : available_data;
            
            uint8_t* dst = (uint8_t*)data;
            uint32_t bytes_read_count = 0;
            
            while (bytes_read_count < read_size && cb->count > 0) {
                dst[bytes_read_count] = cb->data[cb->tail * cb->element_size + (bytes_read_count % cb->element_size)];
                bytes_read_count++;
                
                if ((bytes_read_count % cb->element_size) == 0) {
                    cb->tail = (cb->tail + 1) % cb->size;
                    cb->count--;
                }
            }
            
            if (bytes_read != NULL) {
                *bytes_read = bytes_read_count;
            }
            
            atomic_flag_clear(&cb->lock);
            return AKS_OK;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get available space in buffer
 */
uint32_t aks_buffer_get_free_space(aks_buffer_usage_t usage)
{
    if (!g_buffer_initialized) {
        return 0;
    }
    
    for (uint8_t i = 0; i < AKS_BUFFER_MAX_POOLS; i++) {
        if (g_buffer_handles[i].active && g_buffer_handles[i].usage == usage) {
            
            if (g_buffer_handles[i].type == AKS_BUFFER_TYPE_CIRCULAR) {
                aks_circular_buffer_t* cb = &g_buffer_handles[i].buffer.circular;
                return (cb->size - cb->count) * cb->element_size;
            } else if (g_buffer_handles[i].type == AKS_BUFFER_TYPE_POOL) {
                aks_memory_pool_t* pool = &g_buffer_handles[i].buffer.pool;
                return pool->free_blocks * pool->block_size;
            }
        }
    }
    
    return 0;
}

/**
 * @brief Get used space in buffer
 */
uint32_t aks_buffer_get_used_space(aks_buffer_usage_t usage)
{
    if (!g_buffer_initialized) {
        return 0;
    }
    
    for (uint8_t i = 0; i < AKS_BUFFER_MAX_POOLS; i++) {
        if (g_buffer_handles[i].active && g_buffer_handles[i].usage == usage) {
            
            if (g_buffer_handles[i].type == AKS_BUFFER_TYPE_CIRCULAR) {
                aks_circular_buffer_t* cb = &g_buffer_handles[i].buffer.circular;
                return cb->count * cb->element_size;
            } else if (g_buffer_handles[i].type == AKS_BUFFER_TYPE_POOL) {
                aks_memory_pool_t* pool = &g_buffer_handles[i].buffer.pool;
                return pool->allocated_blocks * pool->block_size;
            }
        }
    }
    
    return 0;
}

/**
 * @brief Clear buffer contents
 */
aks_result_t aks_buffer_clear(aks_buffer_usage_t usage)
{
    if (!g_buffer_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    for (uint8_t i = 0; i < AKS_BUFFER_MAX_POOLS; i++) {
        if (g_buffer_handles[i].active && g_buffer_handles[i].usage == usage) {
            
            if (g_buffer_handles[i].type == AKS_BUFFER_TYPE_CIRCULAR) {
                aks_circular_buffer_t* cb = &g_buffer_handles[i].buffer.circular;
                
                while (atomic_flag_test_and_set(&cb->lock)) {
                    /* Spin wait */
                }
                
                cb->head = 0;
                cb->tail = 0;
                cb->count = 0;
                
                atomic_flag_clear(&cb->lock);
                return AKS_OK;
            }
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get buffer statistics
 */
aks_result_t aks_buffer_get_stats(uint32_t* total_allocations, uint32_t* total_deallocations,
                                  uint32_t* allocation_failures, uint32_t* buffer_overruns)
{
    if (!g_buffer_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (total_allocations != NULL) {
        *total_allocations = g_buffer_stats.total_allocations;
    }
    
    if (total_deallocations != NULL) {
        *total_deallocations = g_buffer_stats.total_deallocations;
    }
    
    if (allocation_failures != NULL) {
        *allocation_failures = g_buffer_stats.allocation_failures;
    }
    
    if (buffer_overruns != NULL) {
        *buffer_overruns = g_buffer_stats.buffer_overruns;
    }
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Create memory pool
 */
static aks_result_t aks_buffer_create_pool(uint8_t handle_id, uint8_t* memory, uint32_t total_size, uint32_t block_size)
{
    if (handle_id >= AKS_BUFFER_MAX_POOLS || memory == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_memory_pool_t* pool = &g_buffer_handles[handle_id].buffer.pool;
    
    /* Align block size */
    uint32_t aligned_block_size = aks_buffer_align_size(block_size + sizeof(aks_memory_block_t) + AKS_BUFFER_GUARD_SIZE);
    uint32_t blocks_count = total_size / aligned_block_size;
    
    if (blocks_count == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    pool->pool_memory = memory;
    pool->block_size = block_size;
    pool->total_blocks = blocks_count;
    pool->free_blocks = blocks_count;
    pool->allocated_blocks = 0;
    pool->peak_usage = 0;
    pool->free_list = NULL;
    atomic_flag_clear(&pool->lock);
    
    /* Initialize free list */
    uint8_t* current = memory;
    for (uint32_t i = 0; i < blocks_count; i++) {
        aks_memory_block_t* block = (aks_memory_block_t*)current;
        aks_buffer_set_guard(block, aligned_block_size);
        
        block->next = pool->free_list;
        pool->free_list = block;
        
        current += aligned_block_size;
    }
    
    pool->initialized = true;
    g_buffer_handles[handle_id].type = AKS_BUFFER_TYPE_POOL;
    g_buffer_handles[handle_id].active = true;
    
    return AKS_OK;
}

/**
 * @brief Create circular buffer
 */
static aks_result_t aks_buffer_create_circular(uint8_t handle_id, uint8_t* memory, uint32_t size, uint32_t element_size)
{
    if (handle_id >= AKS_BUFFER_MAX_POOLS || memory == NULL || element_size == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_circular_buffer_t* cb = &g_buffer_handles[handle_id].buffer.circular;
    
    cb->data = memory;
    cb->size = size / element_size;
    cb->element_size = element_size;
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    cb->overwrite = true; /* Enable overwrite mode */
    atomic_flag_clear(&cb->lock);
    
    g_buffer_handles[handle_id].type = AKS_BUFFER_TYPE_CIRCULAR;
    g_buffer_handles[handle_id].active = true;
    
    return AKS_OK;
}

/**
 * @brief Allocate block from memory pool
 */
static void* aks_buffer_pool_alloc(aks_memory_pool_t* pool)
{
    if (!pool->initialized || pool->free_list == NULL) {
        return NULL;
    }
    
    /* Lock pool */
    while (atomic_flag_test_and_set(&pool->lock)) {
        /* Spin wait */
    }
    
    aks_memory_block_t* block = pool->free_list;
    if (block != NULL) {
        pool->free_list = block->next;
        pool->free_blocks--;
        pool->allocated_blocks++;
        
        if (pool->allocated_blocks > pool->peak_usage) {
            pool->peak_usage = pool->allocated_blocks;
        }
    }
    
    atomic_flag_clear(&pool->lock);
    
    return (block != NULL) ? block->data : NULL;
}

/**
 * @brief Free block to memory pool
 */
static aks_result_t aks_buffer_pool_free(aks_memory_pool_t* pool, void* ptr)
{
    if (!pool->initialized || ptr == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Calculate block address */
    aks_memory_block_t* block = (aks_memory_block_t*)((uint8_t*)ptr - sizeof(aks_memory_block_t));
    
    /* Validate guard bytes */
    uint32_t block_size = aks_buffer_align_size(pool->block_size + sizeof(aks_memory_block_t) + AKS_BUFFER_GUARD_SIZE);
    if (!aks_buffer_validate_guard(block, block_size)) {
        return AKS_ERROR_INVALID_PARAM; /* Memory corruption detected */
    }
    
    /* Lock pool */
    while (atomic_flag_test_and_set(&pool->lock)) {
        /* Spin wait */
    }
    
    block->next = pool->free_list;
    pool->free_list = block;
    pool->free_blocks++;
    pool->allocated_blocks--;
    
    atomic_flag_clear(&pool->lock);
    
    return AKS_OK;
}

/**
 * @brief Validate guard bytes
 */
static bool aks_buffer_validate_guard(aks_memory_block_t* block, uint32_t block_size)
{
    if (block->guard_start != AKS_BUFFER_GUARD_PATTERN) {
        return false;
    }
    
    uint32_t* guard_end = (uint32_t*)((uint8_t*)block + block_size - sizeof(uint32_t));
    if (*guard_end != AKS_BUFFER_GUARD_PATTERN) {
        return false;
    }
    
    return true;
}

/**
 * @brief Set guard bytes
 */
static void aks_buffer_set_guard(aks_memory_block_t* block, uint32_t block_size)
{
    block->guard_start = AKS_BUFFER_GUARD_PATTERN;
    uint32_t* guard_end = (uint32_t*)((uint8_t*)block + block_size - sizeof(uint32_t));
    *guard_end = AKS_BUFFER_GUARD_PATTERN;
}

/**
 * @brief Align size to alignment boundary
 */
static uint32_t aks_buffer_align_size(uint32_t size)
{
    return (size + AKS_BUFFER_ALIGNMENT - 1) & ~(AKS_BUFFER_ALIGNMENT - 1);
}
