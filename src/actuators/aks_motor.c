/**
 * @file aks_motor.c
 * @brief Motor control implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_can.h"
#include <string.h>
#include <math.h>

/* Motor Control Constants */
#define AKS_MOTOR_MAX_TORQUE            200.0f      // Nm
#define AKS_MOTOR_MAX_SPEED             8000.0f     // RPM
#define AKS_MOTOR_MAX_CURRENT           300.0f      // A
#define AKS_MOTOR_REGEN_MAX_POWER       50000.0f    // W
#define AKS_MOTOR_EFFICIENCY            0.95f       // 95%
#define AKS_MOTOR_POLE_PAIRS            8           // Pole pairs
#define AKS_MOTOR_KV                    12.0f       // RPM/V
#define AKS_MOTOR_CONTROL_FREQUENCY     10000       // Hz

/* CAN Message IDs */
#define AKS_MOTOR_CAN_ID_COMMAND        0x101
#define AKS_MOTOR_CAN_ID_STATUS         0x102
#define AKS_MOTOR_CAN_ID_TELEMETRY      0x103
#define AKS_MOTOR_CAN_ID_FAULT          0x104

/* Motor States */
typedef enum {
    AKS_MOTOR_STATE_IDLE = 0,
    AKS_MOTOR_STATE_READY,
    AKS_MOTOR_STATE_RUNNING,
    AKS_MOTOR_STATE_BRAKE,
    AKS_MOTOR_STATE_FAULT,
    AKS_MOTOR_STATE_EMERGENCY
} aks_motor_state_t;

/* Motor Control Mode */
typedef enum {
    AKS_MOTOR_MODE_TORQUE = 0,
    AKS_MOTOR_MODE_SPEED,
    AKS_MOTOR_MODE_POSITION,
    AKS_MOTOR_MODE_POWER
} aks_motor_mode_t;

/* Motor Configuration */
typedef struct {
    float max_torque;           /**< Maximum torque in Nm */
    float max_speed;            /**< Maximum speed in RPM */
    float max_current;          /**< Maximum current in A */
    float max_power;            /**< Maximum power in W */
    float regen_max_power;      /**< Maximum regenerative power in W */
    uint16_t pole_pairs;        /**< Number of pole pairs */
    float kv_constant;          /**< Motor KV constant */
    uint16_t control_frequency; /**< Control loop frequency in Hz */
    bool enable_regen;          /**< Enable regenerative braking */
    bool enable_field_weakening; /**< Enable field weakening */
} aks_motor_config_t;

/* Motor Status */
typedef struct {
    aks_motor_state_t state;    /**< Current motor state */
    aks_motor_mode_t mode;      /**< Control mode */
    float torque_actual;        /**< Actual torque in Nm */
    float speed_actual;         /**< Actual speed in RPM */
    float current_actual;       /**< Actual current in A */
    float power_actual;         /**< Actual power in W */
    float efficiency;           /**< Current efficiency */
    float temperature;          /**< Motor temperature in °C */
    uint16_t fault_code;        /**< Fault code */
    bool is_enabled;           /**< Motor enable status */
    bool is_fault;             /**< Fault status */
} aks_motor_status_t;

/* Motor Handle */
typedef struct {
    bool initialized;
    aks_motor_config_t config;
    aks_motor_status_t status;
    aks_motor_control_t control;
    uint32_t last_update_time;
    uint32_t control_period_ms;
    float integral_error;
    float previous_error;
    struct {
        float kp;               /**< Proportional gain */
        float ki;               /**< Integral gain */
        float kd;               /**< Derivative gain */
        float max_output;       /**< Maximum output */
        float min_output;       /**< Minimum output */
    } pid_params;
} aks_motor_handle_t;

/* Private variables */
static aks_motor_handle_t g_motor_handle;

/* Default configuration */
static const aks_motor_config_t g_default_config = {
    .max_torque = AKS_MOTOR_MAX_TORQUE,
    .max_speed = AKS_MOTOR_MAX_SPEED,
    .max_current = AKS_MOTOR_MAX_CURRENT,
    .max_power = AKS_MOTOR_MAX_TORQUE * AKS_MOTOR_MAX_SPEED * 2.0f * M_PI / 60.0f,
    .regen_max_power = AKS_MOTOR_REGEN_MAX_POWER,
    .pole_pairs = AKS_MOTOR_POLE_PAIRS,
    .kv_constant = AKS_MOTOR_KV,
    .control_frequency = AKS_MOTOR_CONTROL_FREQUENCY,
    .enable_regen = true,
    .enable_field_weakening = true
};

/* Private function prototypes */
static aks_result_t aks_motor_send_can_message(uint32_t id, const uint8_t* data, uint8_t length);
static aks_result_t aks_motor_update_control(void);
static float aks_motor_calculate_pid(float setpoint, float actual);
static aks_result_t aks_motor_check_limits(void);
static aks_result_t aks_motor_handle_fault(uint16_t fault_code);

/**
 * @brief Initialize motor control system
 */
aks_result_t aks_motor_init(const aks_motor_config_t* config)
{
    if (g_motor_handle.initialized) {
        return AKS_ERROR_BUSY;
    }
    
    /* Use default config if none provided */
    if (config != NULL) {
        g_motor_handle.config = *config;
    } else {
        g_motor_handle.config = g_default_config;
    }
    
    /* Initialize status */
    memset(&g_motor_handle.status, 0, sizeof(aks_motor_status_t));
    g_motor_handle.status.state = AKS_MOTOR_STATE_IDLE;
    g_motor_handle.status.mode = AKS_MOTOR_MODE_TORQUE;
    
    /* Initialize control */
    memset(&g_motor_handle.control, 0, sizeof(aks_motor_control_t));
    
    /* Initialize PID parameters */
    g_motor_handle.pid_params.kp = 1.0f;
    g_motor_handle.pid_params.ki = 0.1f;
    g_motor_handle.pid_params.kd = 0.01f;
    g_motor_handle.pid_params.max_output = g_motor_handle.config.max_torque;
    g_motor_handle.pid_params.min_output = -g_motor_handle.config.max_torque;
    
    /* Calculate control period */
    g_motor_handle.control_period_ms = 1000 / g_motor_handle.config.control_frequency;
    
    g_motor_handle.initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize motor control system
 */
aks_result_t aks_motor_deinit(void)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Stop motor */
    aks_motor_stop();
    
    /* Reset handle */
    memset(&g_motor_handle, 0, sizeof(aks_motor_handle_t));
    
    return AKS_OK;
}

/**
 * @brief Enable motor
 */
aks_result_t aks_motor_enable(void)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_motor_handle.status.is_fault) {
        return AKS_ERROR;
    }
    
    g_motor_handle.status.is_enabled = true;
    g_motor_handle.status.state = AKS_MOTOR_STATE_READY;
    
    /* Send enable command via CAN */
    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
}

/**
 * @brief Disable motor
 */
aks_result_t aks_motor_disable(void)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_motor_handle.status.is_enabled = false;
    g_motor_handle.status.state = AKS_MOTOR_STATE_IDLE;
    
    /* Clear control commands */
    memset(&g_motor_handle.control, 0, sizeof(aks_motor_control_t));
    
    /* Send disable command via CAN */
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
}

/**
 * @brief Set motor torque command
 */
aks_result_t aks_motor_set_torque(float torque_nm)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (!g_motor_handle.status.is_enabled) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Limit torque */
    if (torque_nm > g_motor_handle.config.max_torque) {
        torque_nm = g_motor_handle.config.max_torque;
    } else if (torque_nm < -g_motor_handle.config.max_torque) {
        torque_nm = -g_motor_handle.config.max_torque;
    }
    
    g_motor_handle.control.torque_request = torque_nm;
    g_motor_handle.status.mode = AKS_MOTOR_MODE_TORQUE;
    
    if (torque_nm != 0.0f) {
        g_motor_handle.status.state = AKS_MOTOR_STATE_RUNNING;
    }
    
    /* Send torque command via CAN */
    uint8_t data[8];
    data[0] = 0x02; // Torque command
    memcpy(&data[1], &torque_nm, sizeof(float));
    data[5] = data[6] = data[7] = 0x00;
    
    return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
}

/**
 * @brief Set motor speed command
 */
aks_result_t aks_motor_set_speed(float speed_rpm)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (!g_motor_handle.status.is_enabled) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Limit speed */
    if (speed_rpm > g_motor_handle.config.max_speed) {
        speed_rpm = g_motor_handle.config.max_speed;
    } else if (speed_rpm < -g_motor_handle.config.max_speed) {
        speed_rpm = -g_motor_handle.config.max_speed;
    }
    
    g_motor_handle.control.speed_request = speed_rpm;
    g_motor_handle.status.mode = AKS_MOTOR_MODE_SPEED;
    
    if (speed_rpm != 0.0f) {
        g_motor_handle.status.state = AKS_MOTOR_STATE_RUNNING;
    }
    
    /* Send speed command via CAN */
    uint8_t data[8];
    data[0] = 0x03; // Speed command
    memcpy(&data[1], &speed_rpm, sizeof(float));
    data[5] = data[6] = data[7] = 0x00;
    
    return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
}

/**
 * @brief Stop motor
 */
aks_result_t aks_motor_stop(void)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Clear control commands */
    g_motor_handle.control.torque_request = 0.0f;
    g_motor_handle.control.speed_request = 0.0f;
    g_motor_handle.control.enable = false;
    
    g_motor_handle.status.state = AKS_MOTOR_STATE_IDLE;
    
    /* Send stop command via CAN */
    uint8_t data[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
}

/**
 * @brief Emergency stop motor
 */
aks_result_t aks_motor_emergency_stop(void)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Immediate stop */
    g_motor_handle.control.torque_request = 0.0f;
    g_motor_handle.control.speed_request = 0.0f;
    g_motor_handle.control.enable = false;
    
    g_motor_handle.status.state = AKS_MOTOR_STATE_EMERGENCY;
    g_motor_handle.status.is_enabled = false;
    
    /* Send emergency stop command via CAN */
    uint8_t data[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
}

/**
 * @brief Enable regenerative braking
 */
aks_result_t aks_motor_enable_regen_brake(float power_w)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (!g_motor_handle.config.enable_regen) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
    /* Limit regenerative power */
    if (power_w > g_motor_handle.config.regen_max_power) {
        power_w = g_motor_handle.config.regen_max_power;
    }
    
    /* Calculate regenerative torque based on current speed */
    float current_speed = g_motor_handle.status.speed_actual;
    if (current_speed > 100.0f) { // Minimum speed for regen
        float regen_torque = -power_w / (current_speed * 2.0f * M_PI / 60.0f);
        
        g_motor_handle.control.torque_request = regen_torque;
        g_motor_handle.status.state = AKS_MOTOR_STATE_BRAKE;
        
        /* Send regen brake command via CAN */
        uint8_t data[8];
        data[0] = 0x05; // Regen brake command
        memcpy(&data[1], &power_w, sizeof(float));
        data[5] = data[6] = data[7] = 0x00;
        
        return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
    }
    
    return AKS_ERROR;
}

/**
 * @brief Get motor status
 */
aks_result_t aks_motor_get_status(aks_motor_status_t* status)
{
    if (!g_motor_handle.initialized || status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *status = g_motor_handle.status;
    return AKS_OK;
}

/**
 * @brief Get motor control values
 */
aks_result_t aks_motor_get_control(aks_motor_control_t* control)
{
    if (!g_motor_handle.initialized || control == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *control = g_motor_handle.control;
    return AKS_OK;
}

/**
 * @brief Update motor control (periodic task)
 */
aks_result_t aks_motor_task(void)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = HAL_GetTick(); // Replace with actual time function
    
    /* Check if it's time for control update */
    if (current_time - g_motor_handle.last_update_time >= g_motor_handle.control_period_ms) {
        g_motor_handle.last_update_time = current_time;
        
        /* Update control */
        aks_result_t result = aks_motor_update_control();
        if (result != AKS_OK) {
            return result;
        }
        
        /* Check limits and safety */
        result = aks_motor_check_limits();
        if (result != AKS_OK) {
            return result;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Set PID parameters
 */
aks_result_t aks_motor_set_pid_params(float kp, float ki, float kd)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_motor_handle.pid_params.kp = kp;
    g_motor_handle.pid_params.ki = ki;
    g_motor_handle.pid_params.kd = kd;
    
    return AKS_OK;
}

/**
 * @brief Clear motor faults
 */
aks_result_t aks_motor_clear_faults(void)
{
    if (!g_motor_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_motor_handle.status.is_fault = false;
    g_motor_handle.status.fault_code = 0;
    
    /* Send clear faults command via CAN */
    uint8_t data[8] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return aks_motor_send_can_message(AKS_MOTOR_CAN_ID_COMMAND, data, 8);
}

/* Private Functions */

/**
 * @brief Send CAN message
 */
static aks_result_t aks_motor_send_can_message(uint32_t id, const uint8_t* data, uint8_t length)
{
    aks_can_frame_t frame;
    frame.id = id;
    frame.length = length;
    frame.is_extended = false;
    frame.is_remote = false;
    memcpy(frame.data, data, length);
    
    return aks_can_transmit(&frame, 100);
}

/**
 * @brief Update motor control
 */
static aks_result_t aks_motor_update_control(void)
{
    /* This would typically read sensor feedback and update control */
    
    switch (g_motor_handle.status.mode) {
        case AKS_MOTOR_MODE_TORQUE:
            /* Direct torque control */
            g_motor_handle.status.torque_actual = g_motor_handle.control.torque_request;
            break;
            
        case AKS_MOTOR_MODE_SPEED:
            /* Speed control with PID */
            float torque_output = aks_motor_calculate_pid(
                g_motor_handle.control.speed_request,
                g_motor_handle.status.speed_actual
            );
            g_motor_handle.control.torque_request = torque_output;
            break;
            
        default:
            break;
    }
    
    /* Calculate power */
    g_motor_handle.status.power_actual = g_motor_handle.status.torque_actual *
                                        g_motor_handle.status.speed_actual * 2.0f * M_PI / 60.0f;
    
    /* Calculate efficiency */
    if (g_motor_handle.status.power_actual > 0.0f) {
        g_motor_handle.status.efficiency = AKS_MOTOR_EFFICIENCY;
    } else {
        g_motor_handle.status.efficiency = 0.0f;
    }
    
    return AKS_OK;
}

/**
 * @brief Calculate PID output
 */
static float aks_motor_calculate_pid(float setpoint, float actual)
{
    float error = setpoint - actual;
    float dt = (float)g_motor_handle.control_period_ms / 1000.0f;
    
    /* Proportional term */
    float p_term = g_motor_handle.pid_params.kp * error;
    
    /* Integral term */
    g_motor_handle.integral_error += error * dt;
    float i_term = g_motor_handle.pid_params.ki * g_motor_handle.integral_error;
    
    /* Derivative term */
    float d_term = g_motor_handle.pid_params.kd * (error - g_motor_handle.previous_error) / dt;
    g_motor_handle.previous_error = error;
    
    /* Calculate output */
    float output = p_term + i_term + d_term;
    
    /* Limit output */
    if (output > g_motor_handle.pid_params.max_output) {
        output = g_motor_handle.pid_params.max_output;
    } else if (output < g_motor_handle.pid_params.min_output) {
        output = g_motor_handle.pid_params.min_output;
    }
    
    return output;
}

/**
 * @brief Check motor limits and safety
 */
static aks_result_t aks_motor_check_limits(void)
{
    /* Check current limit */
    if (fabs(g_motor_handle.status.current_actual) > g_motor_handle.config.max_current) {
        return aks_motor_handle_fault(0x0001); // Overcurrent fault
    }
    
    /* Check speed limit */
    if (fabs(g_motor_handle.status.speed_actual) > g_motor_handle.config.max_speed) {
        return aks_motor_handle_fault(0x0002); // Overspeed fault
    }
    
    /* Check torque limit */
    if (fabs(g_motor_handle.status.torque_actual) > g_motor_handle.config.max_torque) {
        return aks_motor_handle_fault(0x0003); // Over torque fault
    }
    
    /* Check temperature */
    if (g_motor_handle.status.temperature > 85.0f) {
        return aks_motor_handle_fault(0x0004); // Overtemperature fault
    }
    
    return AKS_OK;
}

/**
 * @brief Handle motor fault
 */
static aks_result_t aks_motor_handle_fault(uint16_t fault_code)
{
    g_motor_handle.status.is_fault = true;
    g_motor_handle.status.fault_code = fault_code;
    g_motor_handle.status.state = AKS_MOTOR_STATE_FAULT;
    
    /* Stop motor immediately */
    aks_motor_emergency_stop();
    
    /* Send fault notification via CAN */
    uint8_t data[8];
    data[0] = 0x07; // Fault notification
    memcpy(&data[1], &fault_code, sizeof(uint16_t));
    data[3] = data[4] = data[5] = data[6] = data[7] = 0x00;
    
    aks_motor_send_can_message(AKS_MOTOR_CAN_ID_FAULT, data, 8);
    
    return AKS_ERROR;
}
