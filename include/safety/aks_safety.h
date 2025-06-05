/**
 * @file aks_safety.h
 * @brief Safety monitoring and isolation detection for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_SAFETY_H
#define AKS_SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"
#include "aks_security.h"

/**
 * @defgroup AKS_Safety Safety System
 * @brief Safety monitoring and fault detection
 * @{
 */

/* Safety Thresholds */
#define AKS_ISOLATION_MIN_RESISTANCE    50000    /**< Minimum isolation resistance in Ohms */
#define AKS_ISOLATION_WARNING_RESISTANCE 200000  /**< Warning threshold in Ohms */
#define AKS_ISOLATION_GOOD_RESISTANCE   1000000  /**< Good isolation threshold in Ohms */
#define AKS_MAX_BATTERY_TEMP            60.0f    /**< Maximum battery temperature in °C */
#define AKS_MAX_MOTOR_TEMP              85.0f    /**< Maximum motor temperature in °C */
#define AKS_EMERGENCY_STOP_TIMEOUT      5000     /**< Emergency stop timeout in ms */

/* Safety Fault Codes */
typedef enum {
    AKS_FAULT_NONE                  = 0x00,
    AKS_FAULT_ISOLATION_LOW         = 0x01,
    AKS_FAULT_BATTERY_OVERVOLTAGE   = 0x02,
    AKS_FAULT_BATTERY_UNDERVOLTAGE  = 0x03,
    AKS_FAULT_BATTERY_OVERTEMP      = 0x04,
    AKS_FAULT_MOTOR_OVERTEMP        = 0x05,
    AKS_FAULT_OVERCURRENT           = 0x06,
    AKS_FAULT_EMERGENCY_STOP        = 0x07,
    AKS_FAULT_DOOR_OPEN             = 0x08,
    AKS_FAULT_SEATBELT_OPEN         = 0x09,
    AKS_FAULT_COMMUNICATION_LOSS    = 0x0A,
    AKS_FAULT_SENSOR_FAILURE        = 0x0B,
    AKS_FAULT_ACTUATOR_FAILURE      = 0x0C,
    AKS_FAULT_WATCHDOG_TIMEOUT      = 0x0D,
    AKS_FAULT_SYSTEM_OVERLOAD       = 0x0E,
    AKS_FAULT_SECURITY_VIOLATION    = 0x0F,
    AKS_FAULT_UNKNOWN               = 0xFF
} aks_fault_code_t;

/* Safety Actions */
typedef enum {
    AKS_ACTION_NONE                 = 0x00,
    AKS_ACTION_WARNING              = 0x01,
    AKS_ACTION_REDUCE_POWER         = 0x02,
    AKS_ACTION_EMERGENCY_STOP       = 0x03,
    AKS_ACTION_SYSTEM_SHUTDOWN      = 0x04,
    AKS_ACTION_ISOLATE_SYSTEM       = 0x05
} aks_safety_action_t;

/* Isolation Monitoring */
typedef struct {
    float positive_resistance;      /**< Positive line resistance in kΩ */
    float negative_resistance;      /**< Negative line resistance in kΩ */
    float total_resistance;         /**< Total isolation resistance in kΩ */
    bool measurement_valid;         /**< Measurement validity flag */
    uint32_t last_measurement_time; /**< Last measurement timestamp */
    uint8_t measurement_accuracy;   /**< Measurement accuracy in % */
} aks_isolation_status_t;

/* Fault Log Entry */
typedef struct {
    aks_fault_code_t fault_code;    /**< Fault code */
    uint32_t timestamp;             /**< Fault occurrence time */
    aks_safety_action_t action_taken; /**< Action taken */
    float fault_value;              /**< Associated measurement value */
    uint8_t occurrence_count;       /**< Number of occurrences */
    bool resolved;                  /**< Fault resolved flag */
} aks_fault_log_entry_t;

/* Safety Limits */
typedef struct {
    float min_battery_voltage;      /**< Minimum battery voltage */
    float max_battery_voltage;      /**< Maximum battery voltage */
    float max_battery_current;      /**< Maximum battery current */
    float max_battery_temp;         /**< Maximum battery temperature */
    float max_motor_temp;           /**< Maximum motor temperature */
    float min_isolation_resistance; /**< Minimum isolation resistance */
    uint32_t emergency_timeout;     /**< Emergency stop timeout */
} aks_safety_limits_t;

/* Safety Configuration */
typedef struct {
    aks_safety_limits_t limits;     /**< Safety limits */
    bool isolation_monitoring_enabled; /**< Isolation monitoring enable */
    bool temperature_monitoring_enabled; /**< Temperature monitoring enable */
    bool voltage_monitoring_enabled;   /**< Voltage monitoring enable */
    bool current_monitoring_enabled;   /**< Current monitoring enable */
    uint32_t monitoring_interval;      /**< Monitoring interval in ms */
    uint8_t fault_log_size;            /**< Maximum fault log entries */
} aks_safety_config_t;

/**
 * @brief Initialize safety monitoring system
 * @param config Pointer to safety configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_init(const aks_safety_config_t* config);

/**
 * @brief Deinitialize safety monitoring system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_deinit(void);

/**
 * @brief Start safety monitoring
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_start_monitoring(void);

/**
 * @brief Stop safety monitoring
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_stop_monitoring(void);

/**
 * @brief Perform isolation resistance measurement
 * @param status Pointer to isolation status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_measure_isolation(aks_isolation_status_t* status);

/**
 * @brief Check all safety parameters
 * @param battery_status Current battery status
 * @param motor_temp Current motor temperature
 * @return AKS_OK if all parameters OK, fault code otherwise
 */
aks_result_t aks_safety_check_parameters(const aks_battery_status_t* battery_status,
                                          float motor_temp);

/**
 * @brief Trigger emergency stop
 * @param reason Emergency stop reason
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_emergency_stop(aks_fault_code_t reason);

/**
 * @brief Clear emergency stop condition
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_clear_emergency_stop(void);

/**
 * @brief Check if system is in emergency state
 * @return true if in emergency state, false otherwise
 */
bool aks_safety_is_emergency_active(void);

/**
 * @brief Log safety fault
 * @param fault_code Fault code
 * @param fault_value Associated measurement value
 * @param action_taken Action taken in response
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_log_fault(aks_fault_code_t fault_code,
                                  float fault_value,
                                  aks_safety_action_t action_taken);

/**
 * @brief Get fault log entries
 * @param log_buffer Pointer to log buffer
 * @param buffer_size Size of log buffer
 * @param entries_count Pointer to return number of entries
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_get_fault_log(aks_fault_log_entry_t* log_buffer,
                                      uint8_t buffer_size,
                                      uint8_t* entries_count);

/**
 * @brief Clear fault log
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_clear_fault_log(void);

/**
 * @brief Get current safety status
 * @param safety_status Pointer to safety status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_get_status(aks_safety_status_t* safety_status);

/**
 * @brief Set safety limits
 * @param limits Pointer to new safety limits
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_set_limits(const aks_safety_limits_t* limits);

/**
 * @brief Get safety limits
 * @param limits Pointer to safety limits structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_get_limits(aks_safety_limits_t* limits);

/**
 * @brief Enable/disable specific monitoring function
 * @param monitoring_type Type of monitoring to control
 * @param enable Enable/disable flag
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_set_monitoring_enable(uint8_t monitoring_type, bool enable);

/**
 * @brief Test isolation monitoring circuit
 * @param test_resistance Known test resistance value
 * @return AKS_OK if test passed, error code otherwise
 */
aks_result_t aks_safety_test_isolation_circuit(float test_resistance);

/**
 * @brief Calibrate isolation monitoring
 * @param calibration_points Array of calibration points
 * @param point_count Number of calibration points
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_calibrate_isolation(const float* calibration_points,
                                            uint8_t point_count);

/**
 * @brief Handle door/seatbelt status change
 * @param door_open Door status (true = open)
 * @param seatbelt_fastened Seatbelt status (true = fastened)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_update_physical_status(bool door_open, bool seatbelt_fastened);

/**
 * @brief Set buzzer/alarm for safety warnings
 * @param enable Enable/disable alarm
 * @param pattern Alarm pattern (0 = continuous, 1 = intermittent)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_set_alarm(bool enable, uint8_t pattern);

/**
 * @brief Safety monitoring task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_task(void);

/**
 * @brief Get fault code description string
 * @param fault_code Fault code
 * @return Pointer to fault description string
 */
const char* aks_safety_get_fault_description(aks_fault_code_t fault_code);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_SAFETY_H */