/**
 * @file aks_timer.c
 * @brief Timer utilities for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Timer Configuration */
#define AKS_TIMER_MAX_INSTANCES     16          /**< Maximum timer instances */
#define AKS_TIMER_TICK_FREQ_HZ      1000        /**< Timer tick frequency */

/* Timer States */
typedef enum {
    AKS_TIMER_STATE_STOPPED = 0,
    AKS_TIMER_STATE_RUNNING,
    AKS_TIMER_STATE_EXPIRED
} aks_timer_state_t;

/* Timer Types */
typedef enum {
    AKS_TIMER_TYPE_ONE_SHOT = 0,
    AKS_TIMER_TYPE_PERIODIC
} aks_timer_type_t;

/* Timer Structure */
typedef struct {
    uint32_t timeout_ms;            /**< Timeout value in milliseconds */
    uint32_t start_time;            /**< Start time */
    uint32_t elapsed_time;          /**< Elapsed time */
    aks_timer_state_t state;        /**< Timer state */
    aks_timer_type_t type;          /**< Timer type */
    aks_callback_t callback;        /**< Callback function */
    bool initialized;               /**< Initialization flag */
} aks_timer_t;

/* Global timer instances */
static aks_timer_t g_timers[AKS_TIMER_MAX_INSTANCES] = {0};
static bool g_timer_module_initialized = false;
static uint32_t g_system_tick = 0;

/* Private function prototypes */
static uint16_t aks_timer_get_free_index(void);
static bool aks_timer_is_valid_handle(uint16_t handle);
static uint32_t aks_timer_get_tick(void);

/**
 * @brief Initialize timer module
 */
aks_result_t aks_timer_init(void)
{
    if (g_timer_module_initialized) {
        return AKS_OK;
    }
    
    /* Initialize all timers */
    for (uint16_t i = 0; i < AKS_TIMER_MAX_INSTANCES; i++) {
        memset(&g_timers[i], 0, sizeof(aks_timer_t));
    }
    
    g_system_tick = 0;
    g_timer_module_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Create a new timer
 */
aks_result_t aks_timer_create(uint32_t timeout_ms, bool periodic, 
                              aks_callback_t callback, uint16_t* handle)
{
    if (!g_timer_module_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (timeout_ms == 0 || handle == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint16_t index = aks_timer_get_free_index();
    if (index >= AKS_TIMER_MAX_INSTANCES) {
        return AKS_ERROR_NO_MEMORY;
    }
    
    aks_timer_t* timer = &g_timers[index];
    timer->timeout_ms = timeout_ms;
    timer->type = periodic ? AKS_TIMER_TYPE_PERIODIC : AKS_TIMER_TYPE_ONE_SHOT;
    timer->callback = callback;
    timer->state = AKS_TIMER_STATE_STOPPED;
    timer->start_time = 0;
    timer->elapsed_time = 0;
    timer->initialized = true;
    
    *handle = index;
    
    return AKS_OK;
}

/**
 * @brief Destroy timer
 */
aks_result_t aks_timer_destroy(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    memset(&g_timers[handle], 0, sizeof(aks_timer_t));
    
    return AKS_OK;
}

/**
 * @brief Start timer
 */
aks_result_t aks_timer_start(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_timer_t* timer = &g_timers[handle];
    timer->start_time = aks_timer_get_tick();
    timer->elapsed_time = 0;
    timer->state = AKS_TIMER_STATE_RUNNING;
    
    return AKS_OK;
}

/**
 * @brief Stop timer
 */
aks_result_t aks_timer_stop(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_timer_t* timer = &g_timers[handle];
    timer->state = AKS_TIMER_STATE_STOPPED;
    
    return AKS_OK;
}

/**
 * @brief Reset timer
 */
aks_result_t aks_timer_reset(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_timer_t* timer = &g_timers[handle];
    timer->start_time = aks_timer_get_tick();
    timer->elapsed_time = 0;
    timer->state = AKS_TIMER_STATE_RUNNING;
    
    return AKS_OK;
}

/**
 * @brief Check if timer has expired
 */
bool aks_timer_is_expired(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return false;
    }
    
    aks_timer_t* timer = &g_timers[handle];
    
    if (timer->state != AKS_TIMER_STATE_RUNNING) {
        return false;
    }
    
    uint32_t current_time = aks_timer_get_tick();
    timer->elapsed_time = current_time - timer->start_time;
    
    return (timer->elapsed_time >= timer->timeout_ms);
}

/**
 * @brief Get timer elapsed time
 */
uint32_t aks_timer_get_elapsed(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return 0;
    }
    
    aks_timer_t* timer = &g_timers[handle];
    
    if (timer->state == AKS_TIMER_STATE_RUNNING) {
        uint32_t current_time = aks_timer_get_tick();
        timer->elapsed_time = current_time - timer->start_time;
    }
    
    return timer->elapsed_time;
}

/**
 * @brief Get timer remaining time
 */
uint32_t aks_timer_get_remaining(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return 0;
    }
    
    aks_timer_t* timer = &g_timers[handle];
    uint32_t elapsed = aks_timer_get_elapsed(handle);
    
    if (elapsed >= timer->timeout_ms) {
        return 0;
    }
    
    return timer->timeout_ms - elapsed;
}

/**
 * @brief Check if timer is running
 */
bool aks_timer_is_running(uint16_t handle)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return false;
    }
    
    return (g_timers[handle].state == AKS_TIMER_STATE_RUNNING);
}

/**
 * @brief Update all timers (call this periodically)
 */
aks_result_t aks_timer_update(void)
{
    if (!g_timer_module_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    for (uint16_t i = 0; i < AKS_TIMER_MAX_INSTANCES; i++) {
        aks_timer_t* timer = &g_timers[i];
        
        if (!timer->initialized || timer->state != AKS_TIMER_STATE_RUNNING) {
            continue;
        }
        
        /* Check if timer has expired */
        if (aks_timer_is_expired(i)) {
            timer->state = AKS_TIMER_STATE_EXPIRED;
            
            /* Call callback if present */
            if (timer->callback != NULL) {
                timer->callback();
            }
            
            /* Handle periodic timers */
            if (timer->type == AKS_TIMER_TYPE_PERIODIC) {
                timer->start_time = aks_timer_get_tick();
                timer->elapsed_time = 0;
                timer->state = AKS_TIMER_STATE_RUNNING;
            } else {
                timer->state = AKS_TIMER_STATE_STOPPED;
            }
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Set timer timeout
 */
aks_result_t aks_timer_set_timeout(uint16_t handle, uint32_t timeout_ms)
{
    if (!aks_timer_is_valid_handle(handle) || timeout_ms == 0) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_timers[handle].timeout_ms = timeout_ms;
    
    return AKS_OK;
}

/**
 * @brief Set timer callback
 */
aks_result_t aks_timer_set_callback(uint16_t handle, aks_callback_t callback)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_timers[handle].callback = callback;
    
    return AKS_OK;
}

/**
 * @brief Set timer type
 */
aks_result_t aks_timer_set_type(uint16_t handle, bool periodic)
{
    if (!aks_timer_is_valid_handle(handle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_timers[handle].type = periodic ? AKS_TIMER_TYPE_PERIODIC : AKS_TIMER_TYPE_ONE_SHOT;
    
    return AKS_OK;
}

/**
 * @brief Simple delay function
 */
void aks_timer_delay_ms(uint32_t delay_ms)
{
    uint32_t start_time = aks_timer_get_tick();
    
    while ((aks_timer_get_tick() - start_time) < delay_ms) {
        /* Busy wait */
    }
}

/**
 * @brief Non-blocking delay
 */
bool aks_timer_delay_nb(uint32_t* start_time, uint32_t delay_ms)
{
    if (start_time == NULL) {
        return false;
    }
    
    if (*start_time == 0) {
        *start_time = aks_timer_get_tick();
        return false; /* Delay just started */
    }
    
    uint32_t elapsed = aks_timer_get_tick() - *start_time;
    
    if (elapsed >= delay_ms) {
        *start_time = 0; /* Reset for next use */
        return true; /* Delay completed */
    }
    
    return false; /* Delay still in progress */
}

/**
 * @brief Get system uptime in milliseconds
 */
uint32_t aks_timer_get_uptime_ms(void)
{
    return aks_timer_get_tick();
}

/**
 * @brief Get system uptime in seconds
 */
uint32_t aks_timer_get_uptime_s(void)
{
    return aks_timer_get_tick() / 1000;
}

/**
 * @brief Measure execution time
 */
uint32_t aks_timer_measure_start(void)
{
    return aks_timer_get_tick();
}

/**
 * @brief Complete execution time measurement
 */
uint32_t aks_timer_measure_end(uint32_t start_time)
{
    return aks_timer_get_tick() - start_time;
}

/**
 * @brief Get timer statistics
 */
aks_result_t aks_timer_get_stats(uint16_t* active_timers, uint16_t* total_timers)
{
    if (!g_timer_module_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint16_t active_count = 0;
    uint16_t total_count = 0;
    
    for (uint16_t i = 0; i < AKS_TIMER_MAX_INSTANCES; i++) {
        if (g_timers[i].initialized) {
            total_count++;
            if (g_timers[i].state == AKS_TIMER_STATE_RUNNING) {
                active_count++;
            }
        }
    }
    
    if (active_timers != NULL) {
        *active_timers = active_count;
    }
    
    if (total_timers != NULL) {
        *total_timers = total_count;
    }
    
    return AKS_OK;
}

/**
 * @brief Timer tick handler (call from SysTick interrupt)
 */
void aks_timer_tick_handler(void)
{
    g_system_tick++;
}

/* Private function implementations */

/**
 * @brief Get free timer index
 */
static uint16_t aks_timer_get_free_index(void)
{
    for (uint16_t i = 0; i < AKS_TIMER_MAX_INSTANCES; i++) {
        if (!g_timers[i].initialized) {
            return i;
        }
    }
    
    return AKS_TIMER_MAX_INSTANCES; /* No free slot */
}

/**
 * @brief Validate timer handle
 */
static bool aks_timer_is_valid_handle(uint16_t handle)
{
    if (!g_timer_module_initialized) {
        return false;
    }
    
    if (handle >= AKS_TIMER_MAX_INSTANCES) {
        return false;
    }
    
    return g_timers[handle].initialized;
}

/**
 * @brief Get current system tick
 */
static uint32_t aks_timer_get_tick(void)
{
#ifdef USE_HAL_DRIVER
    return HAL_GetTick();
#else
    return g_system_tick; /* Use internal tick counter for testing */
#endif
}
