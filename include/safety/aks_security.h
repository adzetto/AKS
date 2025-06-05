/**
 * @file aks_security.h
 * @brief AKS Security and Memory Protection Header
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_SECURITY_H
#define AKS_SECURITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup AKS_Security Security and Memory Protection
 * @brief Security features and memory protection
 * @{
 */

/* Security Configuration Constants */
#define AKS_SECURITY_CHECK_INTERVAL_MS  100
#define AKS_SECURITY_MAX_VIOLATIONS     10
#define AKS_SECURITY_LOG_SIZE           32

/* Security Event Types */
typedef enum {
    AKS_SECURITY_EVENT_SYSTEM_INIT = 0,
    AKS_SECURITY_EVENT_MEMORY_FAULT,
    AKS_SECURITY_EVENT_STACK_OVERFLOW,
    AKS_SECURITY_EVENT_HARD_FAULT,
    AKS_SECURITY_EVENT_FLASH_INTEGRITY_FAIL,
    AKS_SECURITY_EVENT_MPU_INIT_FAIL,
    AKS_SECURITY_EVENT_SECURE_BOOT_FAIL,
    AKS_SECURITY_EVENT_HARDWARE_INIT_FAIL,
    AKS_SECURITY_EVENT_WATCHDOG_RESET,
    AKS_SECURITY_EVENT_UNAUTHORIZED_ACCESS,
    AKS_SECURITY_EVENT_TAMPER_DETECTED
} aks_security_event_type_t;

/* Security Configuration */
typedef struct {
    bool memory_protection_enabled;    /**< Enable MPU protection */
    bool stack_protection_enabled;     /**< Enable stack canary */
    bool flash_integrity_check;        /**< Enable flash CRC check */
    bool secure_boot_enabled;          /**< Enable secure boot */
    bool watchdog_enabled;              /**< Enable hardware watchdog */
    bool auto_recovery_enabled;        /**< Enable automatic recovery */
    bool periodic_flash_check;         /**< Periodic flash integrity check */
    uint32_t max_violations;           /**< Maximum security violations */
    uint32_t check_interval_ms;        /**< Security check interval */
} aks_security_config_t;

/* Security Status */
typedef struct {
    bool memory_protection_enabled;    /**< MPU protection status */
    bool flash_integrity_ok;           /**< Flash integrity status */
    bool stack_protection_enabled;     /**< Stack protection status */
    bool watchdog_enabled;              /**< Watchdog status */
    bool secure_boot_verified;         /**< Secure boot verification */
    uint32_t violation_count;          /**< Total violation count */
    uint32_t last_reset_reason;        /**< Last reset reason */
    uint32_t uptime_ms;                /**< System uptime */
} aks_security_status_t;

/* Security Event */
typedef struct {
    aks_security_event_type_t event_type;  /**< Event type */
    uint32_t timestamp;                     /**< Event timestamp */
    uint32_t address;                       /**< Fault address (if applicable) */
    uint32_t violation_count;               /**< Violation count at time of event */
} aks_security_event_t;

/* Memory Region Configuration */
typedef struct {
    uint32_t base_address;              /**< Region base address */
    uint32_t size;                      /**< Region size */
    bool read_enabled;                  /**< Read access */
    bool write_enabled;                 /**< Write access */
    bool execute_enabled;               /**< Execute access */
    bool privileged_only;               /**< Privileged access only */
} aks_memory_region_t;

/**
 * @brief Initialize AKS Security System
 * @param config Pointer to security configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_init(const aks_security_config_t* config);

/**
 * @brief Security monitoring task (call periodically)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_task(void);

/**
 * @brief Get current security status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_get_status(aks_security_status_t* status);

/**
 * @brief Log security violation
 * @param event_type Type of security event
 * @param address Fault address (if applicable)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_log_violation(aks_security_event_type_t event_type, uint32_t address);

/**
 * @brief Get security event log
 * @param log Pointer to log buffer
 * @param count Pointer to receive number of entries
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_get_log(aks_security_event_t* log, uint8_t* count);

/**
 * @brief Clear security event log
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_clear_log(void);

/**
 * @brief Configure memory protection region
 * @param region_id Region identifier (0-7)
 * @param region Pointer to region configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_configure_region(uint8_t region_id, const aks_memory_region_t* region);

/**
 * @brief Enable memory protection
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_enable_protection(void);

/**
 * @brief Disable memory protection (for debugging)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_disable_protection(void);

/**
 * @brief Validate flash integrity
 * @return AKS_OK if integrity valid, error code otherwise
 */
aks_result_t aks_security_validate_flash(void);

/**
 * @brief Reset security violation counter
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_security_reset_violations(void);

/**
 * @brief Get reset reason
 * @return Reset reason flags
 */
uint32_t aks_security_get_reset_reason(void);

/**
 * @brief Secure system reset
 * @return Does not return
 */
void aks_security_system_reset(void) __attribute__((noreturn));

/* Stack canary support */
extern uint32_t __stack_chk_guard;
void __stack_chk_fail(void) __attribute__((noreturn));

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_SECURITY_H */