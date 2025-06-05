/**
 * @file aks_core.c
 * @brief Core system implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include <string.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Private variables */
static aks_config_t g_aks_config;
static aks_state_t g_system_state = AKS_STATE_INIT;
static bool g_system_initialized = false;
static uint32_t g_system_tick = 0;
static aks_error_callback_t g_error_callback = NULL;

/* Private function prototypes */
static void aks_core_tick_handler(void);

/**
 * @brief Initialize the AKS core system
 */
aks_result_t aks_core_init(const aks_config_t* config)
{
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }

    /* Copy configuration */
    g_aks_config = *config;
    
    /* Initialize system state */
    g_system_state = AKS_STATE_INIT;
    g_system_tick = 0;
    
    /* Set initialization flag */
    g_system_initialized = true;
    
    /* Transition to idle state */
    g_system_state = AKS_STATE_IDLE;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize the AKS core system
 */
aks_result_t aks_core_deinit(void)
{
    if (!g_system_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_system_state = AKS_STATE_SHUTDOWN;
    g_system_initialized = false;
    
    return AKS_OK;
}

/**
 * @brief Start the AKS system
 */
aks_result_t aks_core_start(void)
{
    if (!g_system_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_system_state != AKS_STATE_IDLE && g_system_state != AKS_STATE_READY) {
        return AKS_ERROR_BUSY;
    }
    
    g_system_state = AKS_STATE_RUNNING;
    
    return AKS_OK;
}

/**
 * @brief Stop the AKS system
 */
aks_result_t aks_core_stop(void)
{
    if (!g_system_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_system_state = AKS_STATE_IDLE;
    
    return AKS_OK;
}

/**
 * @brief Get current system state
 */
aks_state_t aks_core_get_state(void)
{
    return g_system_state;
}

/**
 * @brief Set system state
 */
aks_result_t aks_core_set_state(aks_state_t state)
{
    if (!g_system_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_system_state = state;
    
    return AKS_OK;
}

/**
 * @brief Main system task
 */
aks_result_t aks_core_task(void)
{
    if (!g_system_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Update system tick */
    aks_core_tick_handler();
    
    /* Process based on current state */
    switch (g_system_state) {
        case AKS_STATE_INIT:
            /* Initialization tasks */
            break;
            
        case AKS_STATE_IDLE:
            /* Idle tasks */
            break;
            
        case AKS_STATE_READY:
            /* Ready tasks */
            break;
            
        case AKS_STATE_RUNNING:
            /* Running tasks */
            break;
            
        case AKS_STATE_FAULT:
            /* Fault handling */
            if (g_error_callback) {
                g_error_callback(AKS_ERROR);
            }
            break;
            
        case AKS_STATE_EMERGENCY:
            /* Emergency handling */
            if (g_error_callback) {
                g_error_callback(AKS_ERROR);
            }
            break;
            
        case AKS_STATE_SHUTDOWN:
            /* Shutdown tasks */
            break;
            
        default:
            return AKS_ERROR;
    }
    
    return AKS_OK;
}

/**
 * @brief Register error callback
 */
aks_result_t aks_core_register_error_callback(aks_error_callback_t callback)
{
    g_error_callback = callback;
    return AKS_OK;
}

/**
 * @brief Get system uptime
 */
uint32_t aks_core_get_uptime(void)
{
    return g_system_tick;
}

/**
 * @brief Get system tick
 */
uint32_t aks_core_get_tick(void)
{
#ifdef USE_HAL_DRIVER
    return HAL_GetTick();
#else
    return g_system_tick;
#endif
}

/**
 * @brief Delay execution
 */
void aks_core_delay(uint32_t ms)
{
#ifdef USE_HAL_DRIVER
    HAL_Delay(ms);
#else
    /* Simple delay implementation */
    volatile uint32_t count = ms * 1000;
    while (count--);
#endif
}

/**
 * @brief Enter critical section
 */
void aks_core_enter_critical(void)
{
#ifdef USE_HAL_DRIVER
    __disable_irq();
#endif
}

/**
 * @brief Exit critical section
 */
void aks_core_exit_critical(void)
{
#ifdef USE_HAL_DRIVER
    __enable_irq();
#endif
}

/**
 * @brief Reset the system
 */
void aks_core_reset(uint8_t reason)
{
    (void)reason; /* Suppress unused parameter warning */
    
#ifdef USE_HAL_DRIVER
    NVIC_SystemReset();
#endif
}

/**
 * @brief Get reset reason
 */
uint8_t aks_core_get_reset_reason(void)
{
    /* Implementation would check reset status registers */
    return 0;
}

/**
 * @brief Refresh watchdog
 */
void aks_core_watchdog_refresh(void)
{
    /* Watchdog refresh implementation */
    /* This would typically refresh the hardware watchdog */
}

/**
 * @brief Set debug mode
 */
void aks_core_set_debug(bool enable)
{
    g_aks_config.debug_enabled = enable;
}

/**
 * @brief Check if debug mode is enabled
 */
bool aks_core_is_debug_enabled(void)
{
    return g_aks_config.debug_enabled;
}

/**
 * @brief Internal tick handler
 */
static void aks_core_tick_handler(void)
{
    g_system_tick++;
}