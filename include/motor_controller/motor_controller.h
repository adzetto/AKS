/**
 * @file motor_controller.h
 * @brief Motor Controller Header (STM32F4 Compatible)
 * @author AKS Development Team
 * @date 2025
 * @version 2.0.0
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"
#include <math.h>

/* Security and Safety Constants */
#define MOTOR_MAX_TEMP_C                80.0f
#define MOTOR_MIN_VOLTAGE_V             48.0f
#define MOTOR_MAX_VOLTAGE_V             84.0f
#define MOTOR_COMM_TIMEOUT_MS           100
#define MOTOR_WATCHDOG_TIMEOUT_MS       50
#define MOTOR_ISOLATION_MIN_KOHM        100.0f

/**
 * @defgroup Motor_Controller Motor Controller
 * @brief BLDC Motor controller with FOC
 * @{
 */

/* Motor Controller States */
typedef enum {
    MOTOR_STATE_INIT = 0,
    MOTOR_STATE_READY,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
    MOTOR_STATE_EMERGENCY
} motor_controller_state_t;

/* Control Modes */
typedef enum {
    MOTOR_CONTROL_SIX_STEP = 0,
    MOTOR_CONTROL_FOC,
    MOTOR_CONTROL_SENSORLESS
} motor_control_mode_t;

/* Motor Configuration */
typedef struct {
    motor_control_mode_t control_mode;  /**< Control algorithm */
    float max_speed;                    /**< Maximum speed in RPM */
    float max_torque;                   /**< Maximum torque in Nm */
    float max_current;                  /**< Maximum current in A */
    float torque_constant;              /**< Torque constant Kt */
    float back_emf_constant;            /**< Back EMF constant Ke */
    float pole_pairs;                   /**< Number of pole pairs */
    uint16_t pwm_frequency;             /**< PWM frequency in Hz */
} motor_controller_config_t;

/* FOC Parameters */
typedef struct {
    float target_speed;                 /**< Target speed in RPM */
    float actual_speed;                 /**< Actual speed in RPM */
    float target_torque;                /**< Target torque in Nm */
    float id_reference;                 /**< D-axis current reference */
    float iq_reference;                 /**< Q-axis current reference */
    float vd_output;                    /**< D-axis voltage output */
    float vq_output;                    /**< Q-axis voltage output */
    float electrical_angle;             /**< Electrical angle in radians */
} motor_foc_params_t;

/* Motor Measurements */
typedef struct {
    float phase_current_rms;            /**< RMS phase current in A */
    float dc_bus_voltage;               /**< DC bus voltage in V */
    float motor_temperature;            /**< Motor temperature in °C */
    uint32_t timestamp;                 /**< Measurement timestamp */
} motor_measurements_t;

/* Protection Status */
typedef struct {
    bool overcurrent;                   /**< Overcurrent flag */
    bool overvoltage;                   /**< Overvoltage flag */
    bool overtemperature;               /**< Overtemperature flag */
    bool hall_fault;                    /**< Hall sensor fault */
    bool fault_active;                  /**< Any fault active */
} motor_protection_t;

/* Motor Security Status */
typedef struct {
    bool isolation_ok;                  /**< Isolation monitoring OK */
    bool temperature_ok;                /**< Temperature within limits */
    bool current_ok;                    /**< Current within limits */
    bool voltage_ok;                    /**< Voltage within limits */
    bool communication_ok;              /**< Communication timeout status */
    uint32_t watchdog_counter;          /**< Hardware watchdog counter */
    float isolation_resistance;         /**< Isolation resistance in kΩ */
} motor_security_t;

/* Motor Status */
typedef struct {
    motor_controller_state_t state;     /**< Controller state */
    float actual_speed;                 /**< Current speed in RPM */
    float target_speed;                 /**< Target speed in RPM */
    float actual_torque;                /**< Current torque in Nm */
    float dc_bus_voltage;               /**< DC bus voltage in V */
    float motor_temperature;            /**< Motor temperature in °C */
    float phase_current_rms;            /**< RMS current in A */
    uint8_t hall_state;                 /**< Hall sensor state */
    motor_protection_t protection_status; /**< Protection status */
} motor_controller_status_t;

/**
 * @brief Initialize Motor Controller
 * @param config Pointer to motor configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_init(const motor_controller_config_t* config);

/**
 * @brief Start Motor Controller
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_start(void);

/**
 * @brief Stop Motor Controller
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_stop(void);

/**
 * @brief Motor controller task - should be called at 1kHz
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_task(void);

/**
 * @brief Set motor speed reference
 * @param speed_rpm Speed in RPM
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_set_speed(float speed_rpm);

/**
 * @brief Set motor torque reference
 * @param torque_nm Torque in Nm
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_set_torque(float torque_nm);

/**
 * @brief Emergency stop motor
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_emergency_stop(void);

/**
 * @brief Clear motor faults
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_clear_faults(void);

/**
 * @brief Get motor status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_get_status(motor_controller_status_t* status);

/**
 * @brief Set FOC parameters
 * @param params Pointer to FOC parameters
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_set_foc_params(const motor_foc_params_t* params);

/**
 * @brief Get FOC parameters
 * @param params Pointer to FOC parameters structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t motor_controller_get_foc_params(motor_foc_params_t* params);

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

#endif /* MOTOR_CONTROLLER_H */