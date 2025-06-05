/**
 * @file aks_steering.c
 * @brief Steering control implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include "aks_safety.h"
#include <string.h>
#include <math.h>

/* Steering Configuration */
#define AKS_STEERING_MAX_ANGLE          45.0f    // Maximum steering angle in degrees
#define AKS_STEERING_MIN_ANGLE          -45.0f   // Minimum steering angle in degrees
#define AKS_STEERING_DEADBAND           0.5f     // Deadband in degrees
#define AKS_STEERING_MAX_RATE           30.0f    // Maximum rate in deg/s
#define AKS_STEERING_PWM_FREQUENCY      1000     // PWM frequency in Hz
#define AKS_STEERING_POSITION_SAMPLES   8        // Position averaging samples
#define AKS_STEERING_CONTROL_PERIOD     10       // Control period in ms

/* Steering States */
typedef enum {
    AKS_STEERING_STATE_INIT = 0,
    AKS_STEERING_STATE_READY,
    AKS_STEERING_STATE_ACTIVE,
    AKS_STEERING_STATE_FAULT,
    AKS_STEERING_STATE_EMERGENCY
} aks_steering_state_t;

/* Steering Configuration Structure */
typedef struct {
    float max_angle;                    /**< Maximum steering angle */
    float min_angle;                    /**< Minimum steering angle */
    float max_rate;                     /**< Maximum steering rate */
    float deadband;                     /**< Position deadband */
    uint16_t pwm_frequency;             /**< PWM frequency */
    uint8_t position_samples;           /**< Position averaging samples */
    uint32_t control_period;            /**< Control period in ms */
    bool enable_feedback;               /**< Enable position feedback */
    bool enable_safety_limits;          /**< Enable safety limits */
} aks_steering_config_t;

/* PID Controller Structure */
typedef struct {
    float kp;                          /**< Proportional gain */
    float ki;                          /**< Integral gain */
    float kd;                          /**< Derivative gain */
    float integral;                    /**< Integral accumulator */
    float previous_error;              /**< Previous error */
    float output_min;                  /**< Minimum output */
    float output_max;                  /**< Maximum output */
} aks_steering_pid_t;

/* Steering Handle Structure */
typedef struct {
    bool initialized;                   /**< Initialization flag */
    aks_steering_state_t state;        /**< Current state */
    aks_steering_config_t config;      /**< Configuration */
    aks_steering_pid_t pid;            /**< PID controller */
    
    float target_angle;                /**< Target steering angle */
    float current_angle;               /**< Current steering angle */
    float angle_rate;                  /**< Current angle rate */
    
    float position_buffer[AKS_STEERING_POSITION_SAMPLES]; /**< Position buffer */
    uint8_t position_index;            /**< Position buffer index */
    
    uint32_t last_control_time;        /**< Last control update time */
    uint32_t fault_count;              /**< Fault counter */
    
    bool motor_enabled;                /**< Motor enable state */
    float motor_output;                /**< Motor output (-100 to 100%) */
    
} aks_steering_handle_t;

/* Global Variables */
static aks_steering_handle_t g_steering_handle = {0};

/* Private Function Prototypes */
static aks_result_t aks_steering_hardware_init(void);
static aks_result_t aks_steering_pid_init(void);
static float aks_steering_read_position(void);
static aks_result_t aks_steering_set_motor_output(float output);
static float aks_steering_pid_calculate(float setpoint, float feedback);
static float aks_steering_filter_position(float new_position);
static bool aks_steering_check_limits(float angle);
static aks_result_t aks_steering_handle_fault(void);

/**
 * @brief Initialize steering control system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_init(void)
{
    if (g_steering_handle.initialized) {
        return AKS_ERROR_ALREADY_INITIALIZED;
    }
    
    /* Initialize configuration with default values */
    g_steering_handle.config.max_angle = AKS_STEERING_MAX_ANGLE;
    g_steering_handle.config.min_angle = AKS_STEERING_MIN_ANGLE;
    g_steering_handle.config.max_rate = AKS_STEERING_MAX_RATE;
    g_steering_handle.config.deadband = AKS_STEERING_DEADBAND;
    g_steering_handle.config.pwm_frequency = AKS_STEERING_PWM_FREQUENCY;
    g_steering_handle.config.position_samples = AKS_STEERING_POSITION_SAMPLES;
    g_steering_handle.config.control_period = AKS_STEERING_CONTROL_PERIOD;
    g_steering_handle.config.enable_feedback = true;
    g_steering_handle.config.enable_safety_limits = true;
    
    /* Initialize state */
    g_steering_handle.state = AKS_STEERING_STATE_INIT;
    g_steering_handle.target_angle = 0.0f;
    g_steering_handle.current_angle = 0.0f;
    g_steering_handle.angle_rate = 0.0f;
    g_steering_handle.position_index = 0;
    g_steering_handle.fault_count = 0;
    g_steering_handle.motor_enabled = false;
    g_steering_handle.motor_output = 0.0f;
    
    /* Clear position buffer */
    memset(g_steering_handle.position_buffer, 0, sizeof(g_steering_handle.position_buffer));
    
    /* Initialize hardware */
    aks_result_t result = aks_steering_hardware_init();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize PID controller */
    result = aks_steering_pid_init();
    if (result != AKS_OK) {
        return result;
    }
    
    g_steering_handle.initialized = true;
    g_steering_handle.state = AKS_STEERING_STATE_READY;
    g_steering_handle.last_control_time = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Deinitialize steering control system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_deinit(void)
{
    if (!g_steering_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Disable motor */
    aks_steering_set_motor_output(0.0f);
    g_steering_handle.motor_enabled = false;
    
    /* Reset state */
    g_steering_handle.initialized = false;
    g_steering_handle.state = AKS_STEERING_STATE_INIT;
    
    return AKS_OK;
}

/**
 * @brief Set target steering angle
 * @param angle Target angle in degrees (-45 to +45)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_set_angle(float angle)
{
    if (!g_steering_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_steering_handle.state == AKS_STEERING_STATE_FAULT ||
        g_steering_handle.state == AKS_STEERING_STATE_EMERGENCY) {
        return AKS_ERROR_INVALID_STATE;
    }
    
    /* Check angle limits */
    if (!aks_steering_check_limits(angle)) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Check rate limit */
    float angle_diff = angle - g_steering_handle.target_angle;
    uint32_t current_time = aks_core_get_tick();
    uint32_t time_diff = current_time - g_steering_handle.last_control_time;
    
    if (time_diff > 0) {
        float max_change = (g_steering_handle.config.max_rate * time_diff) / 1000.0f;
        if (fabsf(angle_diff) > max_change) {
            angle = g_steering_handle.target_angle + 
                   (angle_diff > 0 ? max_change : -max_change);
        }
    }
    
    g_steering_handle.target_angle = angle;
    
    return AKS_OK;
}

/**
 * @brief Get current steering angle
 * @param angle Pointer to store current angle
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_get_angle(float* angle)
{
    if (!g_steering_handle.initialized || angle == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *angle = g_steering_handle.current_angle;
    return AKS_OK;
}

/**
 * @brief Enable/disable steering motor
 * @param enable Motor enable state
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_set_enable(bool enable)
{
    if (!g_steering_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_steering_handle.state == AKS_STEERING_STATE_FAULT ||
        g_steering_handle.state == AKS_STEERING_STATE_EMERGENCY) {
        enable = false;
    }
    
    g_steering_handle.motor_enabled = enable;
    
    if (!enable) {
        aks_steering_set_motor_output(0.0f);
        g_steering_handle.state = AKS_STEERING_STATE_READY;
    } else {
        g_steering_handle.state = AKS_STEERING_STATE_ACTIVE;
    }
    
    return AKS_OK;
}

/**
 * @brief Steering control task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_task(void)
{
    if (!g_steering_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_core_get_tick();
    
    /* Check if it's time for control update */
    if ((current_time - g_steering_handle.last_control_time) < g_steering_handle.config.control_period) {
        return AKS_OK;
    }
    
    g_steering_handle.last_control_time = current_time;
    
    /* Read current position */
    float raw_position = aks_steering_read_position();
    g_steering_handle.current_angle = aks_steering_filter_position(raw_position);
    
    /* Calculate angle rate */
    static float previous_angle = 0.0f;
    g_steering_handle.angle_rate = (g_steering_handle.current_angle - previous_angle) * 
                                  (1000.0f / g_steering_handle.config.control_period);
    previous_angle = g_steering_handle.current_angle;
    
    /* Check for faults */
    if (fabsf(g_steering_handle.current_angle) > (g_steering_handle.config.max_angle + 5.0f)) {
        g_steering_handle.fault_count++;
        if (g_steering_handle.fault_count > 5) {
            g_steering_handle.state = AKS_STEERING_STATE_FAULT;
            aks_steering_handle_fault();
            return AKS_ERROR_FAULT;
        }
    } else {
        g_steering_handle.fault_count = 0;
    }
    
    /* Control loop */
    if (g_steering_handle.motor_enabled && 
        g_steering_handle.state == AKS_STEERING_STATE_ACTIVE) {
        
        float pid_output = aks_steering_pid_calculate(g_steering_handle.target_angle,
                                                     g_steering_handle.current_angle);
        
        aks_steering_set_motor_output(pid_output);
        g_steering_handle.motor_output = pid_output;
    } else {
        aks_steering_set_motor_output(0.0f);
        g_steering_handle.motor_output = 0.0f;
    }
    
    return AKS_OK;
}

/**
 * @brief Emergency stop steering system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_emergency_stop(void)
{
    if (!g_steering_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Immediately disable motor */
    aks_steering_set_motor_output(0.0f);
    g_steering_handle.motor_enabled = false;
    g_steering_handle.state = AKS_STEERING_STATE_EMERGENCY;
    
    return AKS_OK;
}

/**
 * @brief Get steering system status
 * @param enabled Pointer to store enable status
 * @param angle Pointer to store current angle
 * @param target Pointer to store target angle
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_steering_get_status(bool* enabled, float* angle, float* target)
{
    if (!g_steering_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (enabled) *enabled = g_steering_handle.motor_enabled;
    if (angle) *angle = g_steering_handle.current_angle;
    if (target) *target = g_steering_handle.target_angle;
    
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Initialize steering hardware
 */
static aks_result_t aks_steering_hardware_init(void)
{
    /* Initialize PWM for motor control */
    /* Initialize ADC for position feedback */
    /* Initialize GPIO for enable/direction */
    
    return AKS_OK;
}

/**
 * @brief Initialize PID controller
 */
static aks_result_t aks_steering_pid_init(void)
{
    g_steering_handle.pid.kp = 2.0f;           // Proportional gain
    g_steering_handle.pid.ki = 0.1f;           // Integral gain
    g_steering_handle.pid.kd = 0.05f;          // Derivative gain
    g_steering_handle.pid.integral = 0.0f;
    g_steering_handle.pid.previous_error = 0.0f;
    g_steering_handle.pid.output_min = -100.0f;
    g_steering_handle.pid.output_max = 100.0f;
    
    return AKS_OK;
}

/**
 * @brief Read steering position from sensor
 */
static float aks_steering_read_position(void)
{
    /* Read position from potentiometer/encoder */
    /* Convert ADC value to angle */
    
    // Placeholder implementation
    static float simulated_position = 0.0f;
    
    // Simulate position following target with some lag
    float error = g_steering_handle.target_angle - simulated_position;
    simulated_position += error * 0.1f;
    
    return simulated_position;
}

/**
 * @brief Set motor output
 */
static aks_result_t aks_steering_set_motor_output(float output)
{
    /* Clamp output to valid range */
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    /* Set PWM duty cycle and direction */
    /* output: -100 to +100 (percentage) */
    
    return AKS_OK;
}

/**
 * @brief Calculate PID output
 */
static float aks_steering_pid_calculate(float setpoint, float feedback)
{
    float error = setpoint - feedback;
    
    /* Check deadband */
    if (fabsf(error) < g_steering_handle.config.deadband) {
        error = 0.0f;
    }
    
    /* Proportional term */
    float p_term = g_steering_handle.pid.kp * error;
    
    /* Integral term */
    g_steering_handle.pid.integral += error;
    
    /* Anti-windup */
    if (g_steering_handle.pid.integral > 100.0f) g_steering_handle.pid.integral = 100.0f;
    if (g_steering_handle.pid.integral < -100.0f) g_steering_handle.pid.integral = -100.0f;
    
    float i_term = g_steering_handle.pid.ki * g_steering_handle.pid.integral;
    
    /* Derivative term */
    float d_term = g_steering_handle.pid.kd * (error - g_steering_handle.pid.previous_error);
    g_steering_handle.pid.previous_error = error;
    
    /* Calculate output */
    float output = p_term + i_term + d_term;
    
    /* Clamp output */
    if (output > g_steering_handle.pid.output_max) output = g_steering_handle.pid.output_max;
    if (output < g_steering_handle.pid.output_min) output = g_steering_handle.pid.output_min;
    
    return output;
}

/**
 * @brief Filter position reading
 */
static float aks_steering_filter_position(float new_position)
{
    /* Add to circular buffer */
    g_steering_handle.position_buffer[g_steering_handle.position_index] = new_position;
    g_steering_handle.position_index = (g_steering_handle.position_index + 1) % 
                                      g_steering_handle.config.position_samples;
    
    /* Calculate average */
    float sum = 0.0f;
    for (uint8_t i = 0; i < g_steering_handle.config.position_samples; i++) {
        sum += g_steering_handle.position_buffer[i];
    }
    
    return sum / g_steering_handle.config.position_samples;
}

/**
 * @brief Check angle limits
 */
static bool aks_steering_check_limits(float angle)
{
    return (angle >= g_steering_handle.config.min_angle && 
            angle <= g_steering_handle.config.max_angle);
}

/**
 * @brief Handle steering fault
 */
static aks_result_t aks_steering_handle_fault(void)
{
    /* Disable motor */
    aks_steering_set_motor_output(0.0f);
    g_steering_handle.motor_enabled = false;
    
    /* Log fault */
    aks_safety_log_fault(AKS_FAULT_ACTUATOR_FAILURE, 
                         g_steering_handle.current_angle,
                         AKS_ACTION_DISABLE_ACTUATOR);
    
    return AKS_OK;
}
