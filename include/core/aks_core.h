/**
 * @file aks_core.h
 * @brief Core system management for AKS (Vehicle Control System)
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_CORE_H
#define AKS_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"

/**
 * @defgroup AKS_Core Core System
 * @brief Core system initialization and management
 * @{
 */

/**
 * @brief Initialize the AKS core system
 * @param config Pointer to system configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_core_init(const aks_config_t* config);

/**
 * @brief Deinitialize the AKS core system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_core_deinit(void);

/**
 * @brief Start the AKS system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_core_start(void);

/**
 * @brief Stop the AKS system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_core_stop(void);

/**
 * @brief Get current system state
 * @return Current system state
 */
aks_state_t aks_core_get_state(void);

/**
 * @brief Set system state
 * @param state New system state
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_core_set_state(aks_state_t state);

/**
 * @brief Main system task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_core_task(void);

/**
 * @brief Register error callback
 * @param callback Error callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_core_register_error_callback(aks_error_callback_t callback);

/**
 * @brief Get system uptime in milliseconds
 * @return System uptime in ms
 */
uint32_t aks_core_get_uptime(void);

/**
 * @brief Get system tick count
 * @return Current tick count
 */
uint32_t aks_core_get_tick(void);

/**
 * @brief Delay execution for specified milliseconds
 * @param ms Delay time in milliseconds
 */
void aks_core_delay(uint32_t ms);

/**
 * @brief Enter critical section
 */
void aks_core_enter_critical(void);

/**
 * @brief Exit critical section
 */
void aks_core_exit_critical(void);

/**
 * @brief Reset the system
 * @param reason Reset reason code
 */
void aks_core_reset(uint8_t reason);

/**
 * @brief Get reset reason
 * @return Reset reason code
 */
uint8_t aks_core_get_reset_reason(void);

/**
 * @brief Trigger watchdog
 */
void aks_core_watchdog_refresh(void);

/**
 * @brief Enable/disable debug mode
 * @param enable Debug enable flag
 */
void aks_core_set_debug(bool enable);

/**
 * @brief Check if debug mode is enabled
 * @return true if debug is enabled, false otherwise
 */
bool aks_core_is_debug_enabled(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_CORE_H */