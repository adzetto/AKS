/**
 * @file aks_main.h
 * @brief Main AKS Controller Header (STM32F407VET6TR)
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_MAIN_H
#define AKS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"
#include "aks_safety.h"

/**
 * @defgroup AKS_Main Main AKS Controller
 * @brief Main vehicle control system
 * @{
 */

/* AKS Main States */
typedef enum {
    AKS_MAIN_STATE_INIT = 0,
    AKS_MAIN_STATE_READY,
    AKS_MAIN_STATE_RUNNING,
    AKS_MAIN_STATE_FAULT,
    AKS_MAIN_STATE_EMERGENCY,
    AKS_MAIN_STATE_SHUTDOWN
} aks_main_state_t;

/* Subsystem Status */
typedef struct {
    bool motor_controller;      /**< Motor controller status */
    bool bms;                  /**< Battery management system status */
    bool charger;              /**< Charger status */
    bool isolation_monitor;    /**< Isolation monitor status */
    bool telemetry;           /**< Telemetry system status */
    bool adas;                /**< ADAS system status */
} aks_subsystem_status_t;

/* AKS Main Configuration */
typedef struct {
    aks_safety_config_t safety_config;     /**< Safety configuration */
    uint32_t heartbeat_interval;           /**< Heartbeat interval in ms */
    uint32_t subsystem_timeout;            /**< Subsystem timeout in ms */
    bool debug_mode;                       /**< Debug mode enable */
} aks_main_config_t;

/* AKS Main Status */
typedef struct {
    aks_main_state_t state;                /**< Current state */
    uint32_t uptime;                       /**< System uptime in ms */
    aks_subsystem_status_t subsystem_status; /**< Subsystem status */
    bool power_12v_enabled;                /**< 12V power status */
    bool power_5v_enabled;                 /**< 5V power status */
    bool motor_relay_enabled;              /**< Motor relay status */
    bool charger_relay_enabled;            /**< Charger relay status */
} aks_main_status_t;

/* Telemetry Data */
typedef struct {
    float vehicle_speed;       /**< Vehicle speed in km/h */
    float battery_voltage;     /**< Battery voltage in V */
    float battery_soc;         /**< Battery SOC in % */
    float motor_temp;          /**< Motor temperature in °C */
    uint32_t timestamp;        /**< Timestamp */
} aks_main_telemetry_t;

/**
 * @brief Initialize AKS Main Controller
 * @param config Pointer to main configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_init(const aks_main_config_t* config);

/**
 * @brief Start AKS Main Controller
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_start(void);

/**
 * @brief Stop AKS Main Controller
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_stop(void);

/**
 * @brief Main task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_task(void);

/**
 * @brief Handle emergency stop
 * @param reason Emergency stop reason
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_emergency_stop(aks_fault_code_t reason);

/**
 * @brief Clear emergency stop condition
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_clear_emergency_stop(void);

/**
 * @brief Get AKS Main status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_get_status(aks_main_status_t* status);

/**
 * @brief Set relay state
 * @param relay_id Relay identifier
 * @param state Relay state (true = on, false = off)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_set_relay(uint8_t relay_id, bool state);

/**
 * @brief Send command to subsystem
 * @param subsystem_id Subsystem identifier
 * @param command Command data
 * @param length Command length
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_send_command(uint8_t subsystem_id, const uint8_t* command, uint8_t length);

/**
 * @brief System clock configuration
 */
void SystemClock_Config(void);

/**
 * @brief Error handler
 */
void Error_Handler(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_MAIN_H */