/**
 * @file aks_isolation.c
 * @brief Isolation monitoring core implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include "aks_safety.h"
#include <string.h>
#include <math.h>

/* Isolation Monitoring Configuration */
#define AKS_ISOLATION_MIN_RESISTANCE    50000.0f    // 50kΩ minimum
#define AKS_ISOLATION_WARNING_LEVEL     200000.0f   // 200kΩ warning
#define AKS_ISOLATION_GOOD_LEVEL        1000000.0f  // 1MΩ good
#define AKS_ISOLATION_MEASUREMENT_VOLTAGE 12.0f     // 12V measurement voltage
#define AKS_ISOLATION_FILTER_SAMPLES    8           // Filter samples
#define AKS_ISOLATION_MEASUREMENT_PERIOD 500        // 500ms measurement period
#define AKS_ISOLATION_FAULT_THRESHOLD   3           // 3 consecutive faults

/* Isolation States */
typedef enum {
    AKS_ISOLATION_STATE_INIT = 0,
    AKS_ISOLATION_STATE_READY,
    AKS_ISOLATION_STATE_MEASURING,
    AKS_ISOLATION_STATE_WARNING,
    AKS_ISOLATION_STATE_FAULT,
    AKS_ISOLATION_STATE_EMERGENCY
} aks_isolation_state_t;

/* Isolation Measurement Types */
typedef enum {
    AKS_ISOLATION_MEASURE_POSITIVE = 0,
    AKS_ISOLATION_MEASURE_NEGATIVE,
    AKS_ISOLATION_MEASURE_BOTH
} aks_isolation_measure_type_t;

/* Isolation Configuration Structure */
typedef struct {
    float min_resistance;           /**< Minimum acceptable resistance */
    float warning_level;            /**< Warning level resistance */
    float good_level;               /**< Good isolation level */
    float measurement_voltage;      /**< Measurement voltage */
    uint8_t filter_samples;         /**< Number of filter samples */
    uint32_t measurement_period;    /**< Measurement period in ms */
    uint8_t fault_threshold;        /**< Consecutive fault threshold */
    bool enable_positive_monitoring; /**< Enable positive line monitoring */
    bool enable_negative_monitoring; /**< Enable negative line monitoring */
    bool enable_auto_measurement;   /**< Enable automatic measurement */
} aks_isolation_config_t;

/* Isolation Status Structure */
typedef struct {
    aks_isolation_state_t state;    /**< Current state */
    float positive_resistance;      /**< Positive line resistance */
    float negative_resistance;      /**< Negative line resistance */
    float total_resistance;         /**< Total isolation resistance */
    float measurement_voltage_pos;  /**< Measured voltage positive */
    float measurement_voltage_neg;  /**< Measured voltage negative */
    bool isolation_good;            /**< Isolation status good */
    bool isolation_warning;         /**< Isolation warning */
    bool isolation_fault;           /**< Isolation fault */
    uint8_t consecutive_faults;     /**< Consecutive fault count */
    uint32_t last_measurement_time; /**< Last measurement timestamp */
    uint32_t fault_count;           /**< Total fault count */
} aks_isolation_status_t;

/* Isolation Handle Structure */
typedef struct {
    bool initialized;               /**< Initialization flag */
    aks_isolation_config_t config;  /**< Configuration */
    aks_isolation_status_t status;  /**< Current status */
    
    float pos_resistance_buffer[AKS_ISOLATION_FILTER_SAMPLES]; /**< Positive resistance filter */
    float neg_resistance_buffer[AKS_ISOLATION_FILTER_SAMPLES]; /**< Negative resistance filter */
    uint8_t filter_index;           /**< Filter buffer index */
    
    uint32_t last_measurement_time; /**< Last measurement time */
    aks_isolation_measure_type_t current_measurement; /**< Current measurement type */
    
    bool relay_positive_active;     /**< Positive measurement relay active */
    bool relay_negative_active;     /**< Negative measurement relay active */
    
} aks_isolation_handle_t;

/* Global Variables */
static aks_isolation_handle_t g_isolation_handle = {0};

/* Private Function Prototypes */
static aks_result_t aks_isolation_hardware_init(void);
static aks_result_t aks_isolation_set_measurement_relay(aks_isolation_measure_type_t type, bool active);
static float aks_isolation_read_measurement_voltage(aks_isolation_measure_type_t type);
static float aks_isolation_calculate_resistance(float voltage, aks_isolation_measure_type_t type);
static float aks_isolation_filter_resistance(float new_resistance, aks_isolation_measure_type_t type);
static aks_result_t aks_isolation_evaluate_status(void);
static aks_result_t aks_isolation_handle_fault(void);

/**
 * @brief Initialize isolation monitoring system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_init(void)
{
    if (g_isolation_handle.initialized) {
        return AKS_ERROR_ALREADY_INITIALIZED;
    }
    
    /* Initialize configuration with default values */
    g_isolation_handle.config.min_resistance = AKS_ISOLATION_MIN_RESISTANCE;
    g_isolation_handle.config.warning_level = AKS_ISOLATION_WARNING_LEVEL;
    g_isolation_handle.config.good_level = AKS_ISOLATION_GOOD_LEVEL;
    g_isolation_handle.config.measurement_voltage = AKS_ISOLATION_MEASUREMENT_VOLTAGE;
    g_isolation_handle.config.filter_samples = AKS_ISOLATION_FILTER_SAMPLES;
    g_isolation_handle.config.measurement_period = AKS_ISOLATION_MEASUREMENT_PERIOD;
    g_isolation_handle.config.fault_threshold = AKS_ISOLATION_FAULT_THRESHOLD;
    g_isolation_handle.config.enable_positive_monitoring = true;
    g_isolation_handle.config.enable_negative_monitoring = true;
    g_isolation_handle.config.enable_auto_measurement = true;
    
    /* Initialize status */
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_INIT;
    g_isolation_handle.status.positive_resistance = 0.0f;
    g_isolation_handle.status.negative_resistance = 0.0f;
    g_isolation_handle.status.total_resistance = 0.0f;
    g_isolation_handle.status.isolation_good = false;
    g_isolation_handle.status.isolation_warning = false;
    g_isolation_handle.status.isolation_fault = false;
    g_isolation_handle.status.consecutive_faults = 0;
    g_isolation_handle.status.fault_count = 0;
    
    /* Clear filter buffers */
    memset(g_isolation_handle.pos_resistance_buffer, 0, sizeof(g_isolation_handle.pos_resistance_buffer));
    memset(g_isolation_handle.neg_resistance_buffer, 0, sizeof(g_isolation_handle.neg_resistance_buffer));
    g_isolation_handle.filter_index = 0;
    
    /* Initialize measurement state */
    g_isolation_handle.current_measurement = AKS_ISOLATION_MEASURE_POSITIVE;
    g_isolation_handle.relay_positive_active = false;
    g_isolation_handle.relay_negative_active = false;
    
    /* Initialize hardware */
    aks_result_t result = aks_isolation_hardware_init();
    if (result != AKS_OK) {
        return result;
    }
    
    g_isolation_handle.initialized = true;
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_READY;
    g_isolation_handle.last_measurement_time = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Deinitialize isolation monitoring system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_deinit(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Deactivate all measurement relays */
    aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_POSITIVE, false);
    aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_NEGATIVE, false);
    
    /* Reset state */
    g_isolation_handle.initialized = false;
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_INIT;
    
    return AKS_OK;
}

/**
 * @brief Start isolation monitoring
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_start(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_MEASURING;
    g_isolation_handle.last_measurement_time = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Stop isolation monitoring
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_stop(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Deactivate measurement relays */
    aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_POSITIVE, false);
    aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_NEGATIVE, false);
    
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_READY;
    
    return AKS_OK;
}

/**
 * @brief Isolation monitoring task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_task(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_core_get_tick();
    
    /* Check if it's time for measurement */
    if (g_isolation_handle.status.state == AKS_ISOLATION_STATE_MEASURING &&
        g_isolation_handle.config.enable_auto_measurement &&
        (current_time - g_isolation_handle.last_measurement_time) >= g_isolation_handle.config.measurement_period) {
        
        g_isolation_handle.last_measurement_time = current_time;
        
        /* Perform measurement based on current type */
        if (g_isolation_handle.current_measurement == AKS_ISOLATION_MEASURE_POSITIVE &&
            g_isolation_handle.config.enable_positive_monitoring) {
            
            /* Activate positive measurement relay */
            aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_POSITIVE, true);
            
            /* Wait for circuit settling time (real hardware timing) */
            static uint32_t relay_activation_time = 0;
            uint32_t current_time = aks_get_system_time();
            
            if (relay_activation_time == 0) {
                relay_activation_time = current_time;
                /* Return without measurement to allow settling */
                return AKS_OK;
            }
            
            /* Check if enough settling time has passed */
            if ((current_time - relay_activation_time) < AKS_ISOLATION_SETTLING_TIME_MS) {
                /* Still settling, return and wait */
                return AKS_OK;
            }
            
            /* Reset timing for next measurement */
            relay_activation_time = 0;
            
            /* Read measurement voltage */
            float voltage = aks_isolation_read_measurement_voltage(AKS_ISOLATION_MEASURE_POSITIVE);
            g_isolation_handle.status.measurement_voltage_pos = voltage;
            
            /* Calculate resistance */
            float resistance = aks_isolation_calculate_resistance(voltage, AKS_ISOLATION_MEASURE_POSITIVE);
            
            /* Filter resistance */
            g_isolation_handle.status.positive_resistance = 
                aks_isolation_filter_resistance(resistance, AKS_ISOLATION_MEASURE_POSITIVE);
            
            /* Deactivate relay */
            aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_POSITIVE, false);
            
            /* Switch to negative measurement */
            g_isolation_handle.current_measurement = AKS_ISOLATION_MEASURE_NEGATIVE;
            
        } else if (g_isolation_handle.current_measurement == AKS_ISOLATION_MEASURE_NEGATIVE &&
                   g_isolation_handle.config.enable_negative_monitoring) {
            
            /* Activate negative measurement relay */
            aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_NEGATIVE, true);
            
            /* Wait for circuit settling time (real hardware timing) */
            static uint32_t neg_relay_activation_time = 0;
            current_time = aks_get_system_time();
            
            if (neg_relay_activation_time == 0) {
                neg_relay_activation_time = current_time;
                /* Return without measurement to allow settling */
                return AKS_OK;
            }
            
            /* Check if enough settling time has passed */
            if ((current_time - neg_relay_activation_time) < AKS_ISOLATION_SETTLING_TIME_MS) {
                /* Still settling, return and wait */
                return AKS_OK;
            }
            
            /* Reset timing for next measurement */
            neg_relay_activation_time = 0;
            
            /* Read measurement voltage */
            float voltage = aks_isolation_read_measurement_voltage(AKS_ISOLATION_MEASURE_NEGATIVE);
            g_isolation_handle.status.measurement_voltage_neg = voltage;
            
            /* Calculate resistance */
            float resistance = aks_isolation_calculate_resistance(voltage, AKS_ISOLATION_MEASURE_NEGATIVE);
            
            /* Filter resistance */
            g_isolation_handle.status.negative_resistance = 
                aks_isolation_filter_resistance(resistance, AKS_ISOLATION_MEASURE_NEGATIVE);
            
            /* Deactivate relay */
            aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_NEGATIVE, false);
            
            /* Switch back to positive measurement */
            g_isolation_handle.current_measurement = AKS_ISOLATION_MEASURE_POSITIVE;
            
            /* Calculate total isolation resistance */
            if (g_isolation_handle.status.positive_resistance > 0.0f && 
                g_isolation_handle.status.negative_resistance > 0.0f) {
                
                /* Parallel resistance calculation */
                float pos_res = g_isolation_handle.status.positive_resistance;
                float neg_res = g_isolation_handle.status.negative_resistance;
                g_isolation_handle.status.total_resistance = (pos_res * neg_res) / (pos_res + neg_res);
            } else {
                /* Use minimum of the two */
                g_isolation_handle.status.total_resistance = 
                    fminf(g_isolation_handle.status.positive_resistance, 
                          g_isolation_handle.status.negative_resistance);
            }
            
            /* Evaluate isolation status */
            aks_isolation_evaluate_status();
        }
        
        g_isolation_handle.status.last_measurement_time = current_time;
    }
    
    return AKS_OK;
}

/**
 * @brief Perform manual isolation measurement
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_measure(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Force immediate measurement */
    g_isolation_handle.last_measurement_time = 0;
    
    /* Perform measurement task */
    return aks_isolation_task();
}

/**
 * @brief Get isolation status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_get_status(aks_isolation_status_t* status)
{
    if (!g_isolation_handle.initialized || status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *status = g_isolation_handle.status;
    return AKS_OK;
}

/**
 * @brief Get total isolation resistance
 * @param resistance Pointer to store resistance value
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_get_resistance(float* resistance)
{
    if (!g_isolation_handle.initialized || resistance == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *resistance = g_isolation_handle.status.total_resistance;
    return AKS_OK;
}

/**
 * @brief Emergency stop isolation monitoring
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_isolation_emergency_stop(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Immediately deactivate all relays */
    aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_POSITIVE, false);
    aks_isolation_set_measurement_relay(AKS_ISOLATION_MEASURE_NEGATIVE, false);
    
    /* Set emergency state */
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_EMERGENCY;
    
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Initialize isolation monitoring hardware
 */
static aks_result_t aks_isolation_hardware_init(void)
{
    /* Initialize GPIO for measurement relays */
    /* Initialize ADC for voltage measurement */
    /* Initialize measurement circuit */
    
    return AKS_OK;
}

/**
 * @brief Set measurement relay state
 */
static aks_result_t aks_isolation_set_measurement_relay(aks_isolation_measure_type_t type, bool active)
{
    switch (type) {
        case AKS_ISOLATION_MEASURE_POSITIVE:
            g_isolation_handle.relay_positive_active = active;
            /* Control GPIO for positive measurement relay */
            break;
            
        case AKS_ISOLATION_MEASURE_NEGATIVE:
            g_isolation_handle.relay_negative_active = active;
            /* Control GPIO for negative measurement relay */
            break;
            
        default:
            return AKS_ERROR_INVALID_PARAM;
    }
    
    return AKS_OK;
}

/**
 * @brief Read measurement voltage
 */
static float aks_isolation_read_measurement_voltage(aks_isolation_measure_type_t type)
{
    float voltage = 0.0f;
    
    switch (type) {
        case AKS_ISOLATION_MEASURE_POSITIVE:
            /* Read ADC channel for positive measurement */
            voltage = aks_adc_read_voltage(AKS_ADC_CHANNEL_ISOLATION_POS);
            break;
            
        case AKS_ISOLATION_MEASURE_NEGATIVE:
            /* Read ADC channel for negative measurement */
            voltage = aks_adc_read_voltage(AKS_ADC_CHANNEL_ISOLATION_NEG);
            break;
            
        default:
            voltage = 0.0f;
            break;
    }
    
    return voltage;
}

/**
 * @brief Calculate resistance from measured voltage
 */
static float aks_isolation_calculate_resistance(float voltage, aks_isolation_measure_type_t type)
{
    float resistance = 0.0f;
    
    /* Avoid division by zero */
    if (voltage > 0.01f && voltage < (g_isolation_handle.config.measurement_voltage - 0.01f)) {
        
        /* Calculate resistance using voltage divider formula */
        /* R_isolation = R_known * (V_supply - V_measured) / V_measured */
        float r_known = 1000000.0f; // 1MΩ known resistance
        float v_supply = g_isolation_handle.config.measurement_voltage;
        
        resistance = r_known * (v_supply - voltage) / voltage;
        
        /* Limit to reasonable range */
        if (resistance > 10000000.0f) resistance = 10000000.0f; // 10MΩ max
        if (resistance < 1000.0f) resistance = 1000.0f;         // 1kΩ min
        
    } else if (voltage <= 0.01f) {
        /* Very low voltage indicates very high resistance */
        resistance = 10000000.0f; // 10MΩ
    } else {
        /* Very high voltage indicates very low resistance */
        resistance = 1000.0f; // 1kΩ
    }
    
    return resistance;
}

/**
 * @brief Filter resistance measurement
 */
static float aks_isolation_filter_resistance(float new_resistance, aks_isolation_measure_type_t type)
{
    float* buffer;
    
    /* Select appropriate buffer */
    if (type == AKS_ISOLATION_MEASURE_POSITIVE) {
        buffer = g_isolation_handle.pos_resistance_buffer;
    } else {
        buffer = g_isolation_handle.neg_resistance_buffer;
    }
    
    /* Add new sample to circular buffer */
    buffer[g_isolation_handle.filter_index] = new_resistance;
    g_isolation_handle.filter_index = (g_isolation_handle.filter_index + 1) % 
                                     g_isolation_handle.config.filter_samples;
    
    /* Calculate filtered value (moving average) */
    float sum = 0.0f;
    for (uint8_t i = 0; i < g_isolation_handle.config.filter_samples; i++) {
        sum += buffer[i];
    }
    
    return sum / g_isolation_handle.config.filter_samples;
}

/**
 * @brief Evaluate isolation status
 */
static aks_result_t aks_isolation_evaluate_status(void)
{
    float total_resistance = g_isolation_handle.status.total_resistance;
    
    /* Reset status flags */
    g_isolation_handle.status.isolation_good = false;
    g_isolation_handle.status.isolation_warning = false;
    g_isolation_handle.status.isolation_fault = false;
    
    /* Evaluate resistance levels */
    if (total_resistance >= g_isolation_handle.config.good_level) {
        /* Good isolation */
        g_isolation_handle.status.isolation_good = true;
        g_isolation_handle.status.state = AKS_ISOLATION_STATE_MEASURING;
        g_isolation_handle.status.consecutive_faults = 0;
        
    } else if (total_resistance >= g_isolation_handle.config.warning_level) {
        /* Warning level */
        g_isolation_handle.status.isolation_warning = true;
        g_isolation_handle.status.state = AKS_ISOLATION_STATE_WARNING;
        g_isolation_handle.status.consecutive_faults = 0;
        
        /* Log warning */
        aks_safety_log_fault(AKS_FAULT_ISOLATION_LOW, total_resistance, AKS_ACTION_WARNING);
        
    } else if (total_resistance >= g_isolation_handle.config.min_resistance) {
        /* Low isolation but still acceptable */
        g_isolation_handle.status.isolation_warning = true;
        g_isolation_handle.status.state = AKS_ISOLATION_STATE_WARNING;
        g_isolation_handle.status.consecutive_faults++;
        
        /* Log warning */
        aks_safety_log_fault(AKS_FAULT_ISOLATION_LOW, total_resistance, AKS_ACTION_WARNING);
        
    } else {
        /* Isolation fault */
        g_isolation_handle.status.isolation_fault = true;
        g_isolation_handle.status.state = AKS_ISOLATION_STATE_FAULT;
        g_isolation_handle.status.consecutive_faults++;
        g_isolation_handle.status.fault_count++;
        
        /* Handle fault */
        aks_isolation_handle_fault();
    }
    
    return AKS_OK;
}

/**
 * @brief Handle isolation fault
 */
static aks_result_t aks_isolation_handle_fault(void)
{
    /* Log fault */
    aks_safety_log_fault(AKS_FAULT_ISOLATION_LOW, 
                         g_isolation_handle.status.total_resistance,
                         AKS_ACTION_EMERGENCY_STOP);
    
    /* Check if consecutive faults exceed threshold */
    if (g_isolation_handle.status.consecutive_faults >= g_isolation_handle.config.fault_threshold) {
        /* Trigger emergency stop */
        aks_safety_emergency_stop(AKS_FAULT_ISOLATION_LOW);
    }
    
    return AKS_OK;
} 