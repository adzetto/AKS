/**
 * @file aks_brake.c
 * @brief Brake control implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_config.h"
#include "aks_adc.h"
#include "aks_current.h"
#include "aks_motor.h"
#include "aks_math.h"
#include <string.h>
#include <math.h>

/* Brake System Configuration */
#define AKS_BRAKE_MAX_WHEELS            4           /**< Maximum number of wheels */
#define AKS_BRAKE_REGEN_MAX_POWER       30000.0f    /**< Maximum regenerative power (W) */
#define AKS_BRAKE_HYDRAULIC_MAX_PRESS   150.0f      /**< Maximum hydraulic pressure (bar) */
#define AKS_BRAKE_ABS_THRESHOLD         0.15f       /**< ABS slip threshold */
#define AKS_BRAKE_UPDATE_RATE_HZ        100         /**< Brake update rate (Hz) */
#define AKS_BRAKE_PEDAL_DEADBAND        5.0f        /**< Brake pedal deadband (%) */
#define AKS_BRAKE_BLEND_TRANSITION      30.0f       /**< Regen to hydraulic transition (%) */

/* Brake Types */
typedef enum {
    AKS_BRAKE_TYPE_REGENERATIVE = 0,    /**< Regenerative braking */
    AKS_BRAKE_TYPE_HYDRAULIC,           /**< Hydraulic braking */
    AKS_BRAKE_TYPE_BLENDED,             /**< Blended braking */
    AKS_BRAKE_TYPE_EMERGENCY            /**< Emergency braking */
} aks_brake_type_t;

/* Brake States */
typedef enum {
    AKS_BRAKE_STATE_IDLE = 0,           /**< Brake idle */
    AKS_BRAKE_STATE_REGEN_ONLY,         /**< Regenerative only */
    AKS_BRAKE_STATE_HYDRAULIC_ONLY,     /**< Hydraulic only */
    AKS_BRAKE_STATE_BLENDED,            /**< Blended braking */
    AKS_BRAKE_STATE_ABS_ACTIVE,         /**< ABS active */
    AKS_BRAKE_STATE_EMERGENCY,          /**< Emergency braking */
    AKS_BRAKE_STATE_FAULT               /**< Fault state */
} aks_brake_state_t;

/* Wheel Brake Information */
typedef struct {
    float pressure;                     /**< Brake pressure (bar) */
    float temperature;                  /**< Brake temperature (°C) */
    float wheel_speed;                  /**< Wheel speed (rad/s) */
    float slip_ratio;                   /**< Slip ratio */
    float torque_demand;                /**< Brake torque demand (Nm) */
    float actual_torque;                /**< Actual brake torque (Nm) */
    bool abs_active;                    /**< ABS active flag */
    bool overheated;                    /**< Overheated flag */
    bool fault_detected;                /**< Fault detected */
    uint32_t abs_cycle_count;           /**< ABS cycle counter */
} aks_brake_wheel_t;

/* Brake System Status */
typedef struct {
    aks_brake_wheel_t wheels[AKS_BRAKE_MAX_WHEELS]; /**< Wheel brake status */
    aks_brake_state_t state;            /**< Brake system state */
    aks_brake_type_t active_type;       /**< Active brake type */
    float brake_pedal_position;         /**< Brake pedal position (%) */
    float brake_pedal_force;            /**< Brake pedal force (N) */
    float total_brake_torque;           /**< Total brake torque (Nm) */
    float regen_power;                  /**< Regenerative power (W) */
    float regen_torque;                 /**< Regenerative torque (Nm) */
    float hydraulic_pressure;           /**< Master cylinder pressure (bar) */
    float vehicle_deceleration;         /**< Vehicle deceleration (m/s²) */
    float energy_recovered;             /**< Energy recovered (Wh) */
    bool abs_system_active;             /**< ABS system active */
    bool emergency_brake_active;        /**< Emergency brake active */
    bool brake_lights_on;               /**< Brake lights status */
    bool system_fault;                  /**< System fault flag */
    uint32_t last_update_time;          /**< Last update timestamp */
} aks_brake_system_t;

/* Brake Control Parameters */
typedef struct {
    float regen_efficiency;             /**< Regenerative brake efficiency */
    float hydraulic_efficiency;         /**< Hydraulic brake efficiency */
    float max_regen_torque;             /**< Maximum regenerative torque */
    float max_hydraulic_torque;         /**< Maximum hydraulic torque */
    float blend_factor;                 /**< Blending factor */
    float abs_threshold;                /**< ABS activation threshold */
    float pressure_gain;                /**< Pressure controller gain */
    float torque_gain;                  /**< Torque controller gain */
} aks_brake_config_t;

/* Static brake system */
static aks_brake_system_t g_brake_system = {0};
static bool g_brake_initialized = false;

/* Brake configuration */
static const aks_brake_config_t g_brake_config = {
    .regen_efficiency = 0.85f,          /* 85% regenerative efficiency */
    .hydraulic_efficiency = 0.90f,      /* 90% hydraulic efficiency */
    .max_regen_torque = 150.0f,         /* 150 Nm max regen torque */
    .max_hydraulic_torque = 800.0f,     /* 800 Nm max hydraulic torque per wheel */
    .blend_factor = 0.7f,               /* 70% regen preference in blend */
    .abs_threshold = AKS_BRAKE_ABS_THRESHOLD,
    .pressure_gain = 2.0f,
    .torque_gain = 1.5f
};

/* Statistics */
static struct {
    uint32_t total_brake_applications;
    uint32_t abs_activations;
    uint32_t emergency_stops;
    float total_energy_recovered;
    float max_deceleration_recorded;
    uint32_t overheating_events;
} g_brake_stats = {0};

/* Private function prototypes */
static void aks_brake_update_pedal_input(void);
static void aks_brake_update_wheel_speeds(void);
static void aks_brake_calculate_slip_ratios(void);
static void aks_brake_control_regenerative(void);
static void aks_brake_control_hydraulic(void);
static void aks_brake_control_blended(void);
static void aks_brake_control_abs(void);
static void aks_brake_update_energy_recovery(float dt);
static float aks_brake_calculate_optimal_torque_distribution(uint8_t wheel_id);
static bool aks_brake_check_thermal_limits(void);
static void aks_brake_emergency_stop(void);

/**
 * @brief Initialize brake control system
 */
aks_result_t aks_brake_init(void)
{
    if (g_brake_initialized) {
        return AKS_OK;
    }
    
    /* Clear brake system structure */
    memset(&g_brake_system, 0, sizeof(g_brake_system));
    
    /* Initialize wheel brake data */
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        aks_brake_wheel_t* wheel = &g_brake_system.wheels[i];
        wheel->pressure = 0.0f;
        wheel->temperature = 25.0f;
        wheel->wheel_speed = 0.0f;
        wheel->slip_ratio = 0.0f;
        wheel->torque_demand = 0.0f;
        wheel->actual_torque = 0.0f;
        wheel->abs_active = false;
        wheel->overheated = false;
        wheel->fault_detected = false;
        wheel->abs_cycle_count = 0;
    }
    
    /* Initialize brake system state */
    g_brake_system.state = AKS_BRAKE_STATE_IDLE;
    g_brake_system.active_type = AKS_BRAKE_TYPE_REGENERATIVE;
    g_brake_system.brake_pedal_position = 0.0f;
    g_brake_system.brake_pedal_force = 0.0f;
    g_brake_system.total_brake_torque = 0.0f;
    g_brake_system.regen_power = 0.0f;
    g_brake_system.regen_torque = 0.0f;
    g_brake_system.hydraulic_pressure = 0.0f;
    g_brake_system.vehicle_deceleration = 0.0f;
    g_brake_system.energy_recovered = 0.0f;
    g_brake_system.abs_system_active = false;
    g_brake_system.emergency_brake_active = false;
    g_brake_system.brake_lights_on = false;
    g_brake_system.system_fault = false;
    
    /* Clear statistics */
    memset(&g_brake_stats, 0, sizeof(g_brake_stats));
    
    g_brake_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Update brake control system
 */
aks_result_t aks_brake_update(float dt)
{
    if (!g_brake_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Update pedal input */
    aks_brake_update_pedal_input();
    
    /* Update wheel speeds */
    aks_brake_update_wheel_speeds();
    
    /* Calculate slip ratios */
    aks_brake_calculate_slip_ratios();
    
    /* Check thermal limits */
    aks_brake_check_thermal_limits();
    
    /* Determine brake control strategy */
    if (g_brake_system.emergency_brake_active) {
        aks_brake_emergency_stop();
    } else if (g_brake_system.abs_system_active) {
        aks_brake_control_abs();
    } else {
        /* Normal brake control based on pedal input */
        if (g_brake_system.brake_pedal_position < AKS_BRAKE_PEDAL_DEADBAND) {
            g_brake_system.state = AKS_BRAKE_STATE_IDLE;
            g_brake_system.active_type = AKS_BRAKE_TYPE_REGENERATIVE;
        } else if (g_brake_system.brake_pedal_position < AKS_BRAKE_BLEND_TRANSITION) {
            g_brake_system.state = AKS_BRAKE_STATE_REGEN_ONLY;
            g_brake_system.active_type = AKS_BRAKE_TYPE_REGENERATIVE;
            aks_brake_control_regenerative();
        } else if (g_brake_system.brake_pedal_position < 80.0f) {
            g_brake_system.state = AKS_BRAKE_STATE_BLENDED;
            g_brake_system.active_type = AKS_BRAKE_TYPE_BLENDED;
            aks_brake_control_blended();
        } else {
            g_brake_system.state = AKS_BRAKE_STATE_HYDRAULIC_ONLY;
            g_brake_system.active_type = AKS_BRAKE_TYPE_HYDRAULIC;
            aks_brake_control_hydraulic();
        }
    }
    
    /* Update energy recovery */
    aks_brake_update_energy_recovery(dt);
    
    /* Update brake lights */
    g_brake_system.brake_lights_on = (g_brake_system.brake_pedal_position > AKS_BRAKE_PEDAL_DEADBAND);
    
    /* Update timestamp */
    g_brake_system.last_update_time = 0; /* Would use HAL_GetTick() */
    
    return AKS_OK;
}

/**
 * @brief Get brake pedal position
 */
float aks_brake_get_pedal_position(void)
{
    return g_brake_system.brake_pedal_position;
}

/**
 * @brief Get total brake torque
 */
float aks_brake_get_total_torque(void)
{
    return g_brake_system.total_brake_torque;
}

/**
 * @brief Get regenerative power
 */
float aks_brake_get_regen_power(void)
{
    return g_brake_system.regen_power;
}

/**
 * @brief Get vehicle deceleration
 */
float aks_brake_get_deceleration(void)
{
    return g_brake_system.vehicle_deceleration;
}

/**
 * @brief Get brake system state
 */
aks_brake_state_t aks_brake_get_state(void)
{
    return g_brake_system.state;
}

/**
 * @brief Check if ABS is active
 */
bool aks_brake_is_abs_active(void)
{
    return g_brake_system.abs_system_active;
}

/**
 * @brief Check if brake lights should be on
 */
bool aks_brake_lights_status(void)
{
    return g_brake_system.brake_lights_on;
}

/**
 * @brief Get energy recovered
 */
float aks_brake_get_energy_recovered(void)
{
    return g_brake_system.energy_recovered;
}

/**
 * @brief Activate emergency brake
 */
aks_result_t aks_brake_emergency_activate(void)
{
    if (!g_brake_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_brake_system.emergency_brake_active = true;
    g_brake_stats.emergency_stops++;
    
    return AKS_OK;
}

/**
 * @brief Get wheel brake information
 */
aks_result_t aks_brake_get_wheel_info(uint8_t wheel_id, aks_brake_wheel_t* wheel_info)
{
    if (!g_brake_initialized || wheel_info == NULL || wheel_id >= AKS_BRAKE_MAX_WHEELS) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    memcpy(wheel_info, &g_brake_system.wheels[wheel_id], sizeof(aks_brake_wheel_t));
    
    return AKS_OK;
}

/**
 * @brief Check brake system faults
 */
bool aks_brake_has_fault(void)
{
    return g_brake_system.system_fault;
}

/**
 * @brief Get brake statistics
 */
aks_result_t aks_brake_get_stats(uint32_t* brake_applications, uint32_t* abs_activations,
                                 uint32_t* emergency_stops, float* total_energy_recovered)
{
    if (!g_brake_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (brake_applications != NULL) {
        *brake_applications = g_brake_stats.total_brake_applications;
    }
    
    if (abs_activations != NULL) {
        *abs_activations = g_brake_stats.abs_activations;
    }
    
    if (emergency_stops != NULL) {
        *emergency_stops = g_brake_stats.emergency_stops;
    }
    
    if (total_energy_recovered != NULL) {
        *total_energy_recovered = g_brake_stats.total_energy_recovered;
    }
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Update brake pedal input
 */
static void aks_brake_update_pedal_input(void)
{
    /* Read brake pedal position from ADC */
    uint16_t adc_raw;
    float adc_voltage;
    
    if (aks_adc_read_channel(AKS_ADC_CH_BRAKE_PEDAL, &adc_raw, &adc_voltage) == AKS_OK) {
        /* Convert ADC reading to pedal position percentage */
        g_brake_system.brake_pedal_position = (adc_voltage / 3.3f) * 100.0f;
        
        /* Apply deadband */
        if (g_brake_system.brake_pedal_position < AKS_BRAKE_PEDAL_DEADBAND) {
            g_brake_system.brake_pedal_position = 0.0f;
        }
        
        /* Calculate brake pedal force (estimated) */
        g_brake_system.brake_pedal_force = g_brake_system.brake_pedal_position * 5.0f; /* 5N per % */
        
        /* Update statistics */
        if (g_brake_system.brake_pedal_position > AKS_BRAKE_PEDAL_DEADBAND) {
            static bool brake_was_applied = false;
            if (!brake_was_applied) {
                g_brake_stats.total_brake_applications++;
                brake_was_applied = true;
            }
        } else {
            static bool brake_was_applied = true;
            brake_was_applied = false;
        }
    }
}

/**
 * @brief Update wheel speeds
 */
static void aks_brake_update_wheel_speeds(void)
{
    /* In real implementation, this would read from wheel speed sensors */
    /* For now, simulate based on motor speed and vehicle dynamics */
    
    float vehicle_speed = 0.0f; /* Would get from vehicle speed sensor */
    
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        aks_brake_wheel_t* wheel = &g_brake_system.wheels[i];
        
        /* Simulate wheel speed with some variation */
        wheel->wheel_speed = vehicle_speed + (float)(rand() % 10 - 5) * 0.1f;
        
        /* Update brake temperature (simplified model) */
        if (wheel->actual_torque > 0.0f) {
            wheel->temperature += 0.1f; /* Heat up during braking */
        } else {
            wheel->temperature -= 0.05f; /* Cool down */
            if (wheel->temperature < 25.0f) {
                wheel->temperature = 25.0f;
            }
        }
        
        /* Check overheating */
        if (wheel->temperature > 200.0f) {
            wheel->overheated = true;
            g_brake_stats.overheating_events++;
        } else if (wheel->temperature < 150.0f) {
            wheel->overheated = false;
        }
    }
}

/**
 * @brief Calculate slip ratios for each wheel
 */
static void aks_brake_calculate_slip_ratios(void)
{
    float vehicle_speed = 0.0f; /* Would get average wheel speed */
    
    /* Calculate average wheel speed */
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        vehicle_speed += g_brake_system.wheels[i].wheel_speed;
    }
    vehicle_speed /= AKS_BRAKE_MAX_WHEELS;
    
    /* Calculate slip ratio for each wheel */
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        aks_brake_wheel_t* wheel = &g_brake_system.wheels[i];
        
        if (vehicle_speed > 0.1f) {
            wheel->slip_ratio = (vehicle_speed - wheel->wheel_speed) / vehicle_speed;
        } else {
            wheel->slip_ratio = 0.0f;
        }
        
        /* Check ABS activation threshold */
        if (fabsf(wheel->slip_ratio) > g_brake_config.abs_threshold) {
            if (!wheel->abs_active) {
                wheel->abs_active = true;
                wheel->abs_cycle_count++;
                g_brake_system.abs_system_active = true;
                g_brake_stats.abs_activations++;
            }
        } else {
            wheel->abs_active = false;
        }
    }
    
    /* Check if any wheel has ABS active */
    bool any_abs_active = false;
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        if (g_brake_system.wheels[i].abs_active) {
            any_abs_active = true;
            break;
        }
    }
    g_brake_system.abs_system_active = any_abs_active;
}

/**
 * @brief Control regenerative braking
 */
static void aks_brake_control_regenerative(void)
{
    /* Calculate desired regenerative torque based on pedal position */
    float torque_demand = (g_brake_system.brake_pedal_position / 100.0f) * g_brake_config.max_regen_torque;
    
    /* Limit by motor capabilities */
    float max_motor_regen = 0.0f; /* Would get from motor controller */
    if (torque_demand > max_motor_regen) {
        torque_demand = max_motor_regen;
    }
    
    g_brake_system.regen_torque = torque_demand;
    g_brake_system.regen_power = torque_demand * 50.0f; /* Approximate, would use actual speed */
    g_brake_system.total_brake_torque = torque_demand;
    
    /* Apply regenerative braking through motor controller */
    /* aks_motor_set_regen_torque(torque_demand); */
    
    /* Calculate vehicle deceleration */
    g_brake_system.vehicle_deceleration = torque_demand / 500.0f; /* Simplified, mass=500kg*r */
}

/**
 * @brief Control hydraulic braking
 */
static void aks_brake_control_hydraulic(void)
{
    /* Calculate hydraulic pressure based on pedal position */
    float pressure_demand = (g_brake_system.brake_pedal_position / 100.0f) * AKS_BRAKE_HYDRAULIC_MAX_PRESS;
    
    g_brake_system.hydraulic_pressure = pressure_demand;
    
    /* Distribute brake torque to wheels */
    float total_torque = 0.0f;
    
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        aks_brake_wheel_t* wheel = &g_brake_system.wheels[i];
        
        if (!wheel->overheated && !wheel->fault_detected) {
            /* Calculate optimal torque distribution */
            wheel->torque_demand = aks_brake_calculate_optimal_torque_distribution(i);
            wheel->actual_torque = wheel->torque_demand * g_brake_config.hydraulic_efficiency;
            wheel->pressure = pressure_demand;
            
            total_torque += wheel->actual_torque;
        } else {
            wheel->torque_demand = 0.0f;
            wheel->actual_torque = 0.0f;
            wheel->pressure = 0.0f;
        }
    }
    
    g_brake_system.total_brake_torque = total_torque;
    g_brake_system.regen_power = 0.0f;
    g_brake_system.regen_torque = 0.0f;
    
    /* Calculate vehicle deceleration */
    g_brake_system.vehicle_deceleration = total_torque / 500.0f; /* Simplified */
}

/**
 * @brief Control blended braking (regenerative + hydraulic)
 */
static void aks_brake_control_blended(void)
{
    float total_torque_demand = (g_brake_system.brake_pedal_position / 100.0f) * 
                               (g_brake_config.max_regen_torque + g_brake_config.max_hydraulic_torque);
    
    /* Split between regenerative and hydraulic */
    float regen_portion = g_brake_config.blend_factor;
    float hydraulic_portion = 1.0f - regen_portion;
    
    /* Regenerative component */
    float regen_demand = total_torque_demand * regen_portion;
    if (regen_demand > g_brake_config.max_regen_torque) {
        regen_demand = g_brake_config.max_regen_torque;
    }
    
    g_brake_system.regen_torque = regen_demand;
    g_brake_system.regen_power = regen_demand * 50.0f; /* Approximate */
    
    /* Hydraulic component */
    float hydraulic_demand = total_torque_demand - regen_demand;
    float pressure_demand = (hydraulic_demand / g_brake_config.max_hydraulic_torque) * AKS_BRAKE_HYDRAULIC_MAX_PRESS;
    
    g_brake_system.hydraulic_pressure = pressure_demand;
    
    /* Distribute hydraulic brake torque to wheels */
    float total_hydraulic_torque = 0.0f;
    
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        aks_brake_wheel_t* wheel = &g_brake_system.wheels[i];
        
        if (!wheel->overheated && !wheel->fault_detected) {
            wheel->torque_demand = aks_brake_calculate_optimal_torque_distribution(i) * hydraulic_portion;
            wheel->actual_torque = wheel->torque_demand * g_brake_config.hydraulic_efficiency;
            wheel->pressure = pressure_demand;
            
            total_hydraulic_torque += wheel->actual_torque;
        } else {
            wheel->torque_demand = 0.0f;
            wheel->actual_torque = 0.0f;
            wheel->pressure = 0.0f;
        }
    }
    
    g_brake_system.total_brake_torque = regen_demand + total_hydraulic_torque;
    
    /* Calculate vehicle deceleration */
    g_brake_system.vehicle_deceleration = g_brake_system.total_brake_torque / 500.0f;
}

/**
 * @brief Control ABS system
 */
static void aks_brake_control_abs(void)
{
    g_brake_system.state = AKS_BRAKE_STATE_ABS_ACTIVE;
    
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        aks_brake_wheel_t* wheel = &g_brake_system.wheels[i];
        
        if (wheel->abs_active) {
            /* Reduce brake pressure to prevent lockup */
            wheel->pressure *= 0.8f; /* Reduce by 20% */
            wheel->torque_demand *= 0.8f;
            wheel->actual_torque = wheel->torque_demand * g_brake_config.hydraulic_efficiency;
        } else {
            /* Normal brake control */
            wheel->torque_demand = aks_brake_calculate_optimal_torque_distribution(i);
            wheel->actual_torque = wheel->torque_demand * g_brake_config.hydraulic_efficiency;
        }
    }
    
    /* Reduce regenerative braking during ABS */
    g_brake_system.regen_torque *= 0.5f;
    g_brake_system.regen_power *= 0.5f;
}

/**
 * @brief Update energy recovery calculations
 */
static void aks_brake_update_energy_recovery(float dt)
{
    if (g_brake_system.regen_power > 0.0f) {
        float energy_delta = g_brake_system.regen_power * dt / 3600.0f; /* Convert to Wh */
        g_brake_system.energy_recovered += energy_delta;
        g_brake_stats.total_energy_recovered += energy_delta;
    }
}

/**
 * @brief Calculate optimal torque distribution for a wheel
 */
static float aks_brake_calculate_optimal_torque_distribution(uint8_t wheel_id)
{
    /* Simple equal distribution for now */
    /* In advanced implementation, this would consider:
     * - Weight distribution
     * - Tire grip
     * - Load transfer
     * - Individual wheel conditions
     */
    
    float base_torque = (g_brake_system.brake_pedal_position / 100.0f) * 
                       (g_brake_config.max_hydraulic_torque / AKS_BRAKE_MAX_WHEELS);
    
    /* Adjust for wheel-specific conditions */
    aks_brake_wheel_t* wheel = &g_brake_system.wheels[wheel_id];
    
    if (wheel->overheated) {
        base_torque *= 0.5f; /* Reduce torque for overheated wheels */
    }
    
    if (wheel->slip_ratio > 0.1f) {
        base_torque *= 0.8f; /* Reduce torque for slipping wheels */
    }
    
    return base_torque;
}

/**
 * @brief Check thermal limits
 */
static bool aks_brake_check_thermal_limits(void)
{
    bool thermal_ok = true;
    
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        if (g_brake_system.wheels[i].overheated) {
            thermal_ok = false;
            g_brake_system.system_fault = true;
        }
    }
    
    return thermal_ok;
}

/**
 * @brief Emergency stop procedure
 */
static void aks_brake_emergency_stop(void)
{
    g_brake_system.state = AKS_BRAKE_STATE_EMERGENCY;
    
    /* Maximum braking force */
    g_brake_system.hydraulic_pressure = AKS_BRAKE_HYDRAULIC_MAX_PRESS;
    g_brake_system.regen_torque = g_brake_config.max_regen_torque;
    g_brake_system.regen_power = AKS_BRAKE_REGEN_MAX_POWER;
    
    /* Apply maximum brake torque to all wheels */
    for (uint8_t i = 0; i < AKS_BRAKE_MAX_WHEELS; i++) {
        aks_brake_wheel_t* wheel = &g_brake_system.wheels[i];
        
        wheel->torque_demand = g_brake_config.max_hydraulic_torque;
        wheel->actual_torque = wheel->torque_demand * g_brake_config.hydraulic_efficiency;
        wheel->pressure = AKS_BRAKE_HYDRAULIC_MAX_PRESS;
    }
    
    g_brake_system.total_brake_torque = g_brake_system.regen_torque + 
                                       (g_brake_config.max_hydraulic_torque * AKS_BRAKE_MAX_WHEELS);
    
    /* Calculate maximum deceleration */
    g_brake_system.vehicle_deceleration = g_brake_system.total_brake_torque / 500.0f;
    
    if (g_brake_system.vehicle_deceleration > g_brake_stats.max_deceleration_recorded) {
        g_brake_stats.max_deceleration_recorded = g_brake_system.vehicle_deceleration;
    }
}