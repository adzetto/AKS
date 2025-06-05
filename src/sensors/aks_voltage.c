/**
 * @file aks_voltage.c
 * @brief Voltage sensor implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_adc.h"
#include "aks_math.h"
#include "aks_config.h"
#include <string.h>
#include <math.h>

/* Voltage Sensor Configuration */
#define AKS_VOLTAGE_MAX_SENSORS     12          /**< Maximum voltage sensors */
#define AKS_VOLTAGE_FILTER_SAMPLES  16          /**< Moving average filter samples */
#define AKS_VOLTAGE_UPDATE_RATE_HZ  50          /**< Voltage update rate */
#define AKS_VOLTAGE_FAULT_COUNT     5           /**< Fault threshold count */

/* Voltage Sensor Types */
typedef enum {
    AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER = 0,  /**< Resistive voltage divider */
    AKS_VOLTAGE_TYPE_ISOLATION_AMPLIFIER,    /**< Isolation amplifier */
    AKS_VOLTAGE_TYPE_DIFFERENTIAL,           /**< Differential measurement */
    AKS_VOLTAGE_TYPE_DIRECT                  /**< Direct measurement */
} aks_voltage_sensor_type_t;

/* Voltage Sensor Locations */
typedef enum {
    AKS_VOLTAGE_LOCATION_BATTERY_PACK = 0,   /**< Main battery pack voltage */
    AKS_VOLTAGE_LOCATION_DC_LINK,            /**< DC link voltage */
    AKS_VOLTAGE_LOCATION_CHARGER_INPUT,      /**< Charger input voltage */
    AKS_VOLTAGE_LOCATION_CHARGER_OUTPUT,     /**< Charger output voltage */
    AKS_VOLTAGE_LOCATION_INVERTER_INPUT,     /**< Inverter input voltage */
    AKS_VOLTAGE_LOCATION_12V_AUXILIARY,      /**< 12V auxiliary system */
    AKS_VOLTAGE_LOCATION_5V_LOGIC,           /**< 5V logic supply */
    AKS_VOLTAGE_LOCATION_3V3_LOGIC,          /**< 3.3V logic supply */
    AKS_VOLTAGE_LOCATION_MOTOR_PHASE_A,      /**< Motor phase A voltage */
    AKS_VOLTAGE_LOCATION_MOTOR_PHASE_B,      /**< Motor phase B voltage */
    AKS_VOLTAGE_LOCATION_MOTOR_PHASE_C,      /**< Motor phase C voltage */
    AKS_VOLTAGE_LOCATION_ISOLATED_SENSOR     /**< Isolated sensor voltage */
} aks_voltage_location_t;

/* Voltage Sensor Configuration */
typedef struct {
    aks_voltage_sensor_type_t type;     /**< Sensor type */
    aks_voltage_location_t location;    /**< Sensor location */
    uint8_t adc_channel;                /**< ADC channel */
    float divider_ratio;                /**< Voltage divider ratio */
    float amplifier_gain;               /**< Amplifier gain */
    float offset_voltage;               /**< Offset voltage */
    float reference_voltage;            /**< ADC reference voltage */
    float max_voltage;                  /**< Maximum voltage */
    float min_voltage;                  /**< Minimum voltage */
    float resolution;                   /**< Voltage resolution */
    bool enabled;                       /**< Sensor enabled flag */
} aks_voltage_sensor_config_t;

/* Voltage Sensor Data */
typedef struct {
    float voltage;                      /**< Voltage reading (V) */
    float raw_adc;                      /**< Raw ADC value */
    float filtered_voltage;             /**< Filtered voltage */
    float rms_voltage;                  /**< RMS voltage */
    float peak_voltage;                 /**< Peak voltage */
    float min_voltage_recorded;         /**< Minimum voltage recorded */
    float max_voltage_recorded;         /**< Maximum voltage recorded */
    float voltage_history[AKS_VOLTAGE_FILTER_SAMPLES]; /**< Voltage history */
    uint8_t history_index;              /**< History circular buffer index */
    uint32_t last_update_time;          /**< Last update timestamp */
    uint32_t fault_count;               /**< Fault counter */
    bool fault_active;                  /**< Fault status */
    bool data_valid;                    /**< Data validity flag */
    bool overvoltage;                   /**< Overvoltage flag */
    bool undervoltage;                  /**< Undervoltage flag */
} aks_voltage_sensor_data_t;

/* Static sensor configurations */
static const aks_voltage_sensor_config_t g_voltage_sensor_configs[AKS_VOLTAGE_MAX_SENSORS] = {
    /* Battery Pack Voltage - High Voltage Divider */
    {
        .type = AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER,
        .location = AKS_VOLTAGE_LOCATION_BATTERY_PACK,
        .adc_channel = AKS_ADC_CH_BATTERY_VOLTAGE,
        .divider_ratio = 30.0f,         /* 100kΩ / 3.3kΩ = 30.3 */
        .amplifier_gain = 1.0f,
        .offset_voltage = 0.0f,
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = 0.0f,
        .resolution = 0.1f,
        .enabled = true
    },
    /* DC Link Voltage - High Voltage Divider */
    {
        .type = AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER,
        .location = AKS_VOLTAGE_LOCATION_DC_LINK,
        .adc_channel = 8, /* Additional ADC channel */
        .divider_ratio = 30.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 0.0f,
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = 0.0f,
        .resolution = 0.1f,
        .enabled = true
    },
    /* Charger Input Voltage */
    {
        .type = AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER,
        .location = AKS_VOLTAGE_LOCATION_CHARGER_INPUT,
        .adc_channel = 9, /* Additional ADC channel */
        .divider_ratio = 100.0f,        /* 220V AC peak = 311V */
        .amplifier_gain = 1.0f,
        .offset_voltage = 1.65f,        /* Biased at mid-rail */
        .reference_voltage = 3.3f,
        .max_voltage = 400.0f,
        .min_voltage = -400.0f,
        .resolution = 1.0f,
        .enabled = true
    },
    /* Charger Output Voltage */
    {
        .type = AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER,
        .location = AKS_VOLTAGE_LOCATION_CHARGER_OUTPUT,
        .adc_channel = 10, /* Additional ADC channel */
        .divider_ratio = 30.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 0.0f,
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = 0.0f,
        .resolution = 0.1f,
        .enabled = true
    },
    /* Inverter Input Voltage */
    {
        .type = AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER,
        .location = AKS_VOLTAGE_LOCATION_INVERTER_INPUT,
        .adc_channel = 11, /* Additional ADC channel */
        .divider_ratio = 30.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 0.0f,
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = 0.0f,
        .resolution = 0.1f,
        .enabled = true
    },
    /* 12V Auxiliary System */
    {
        .type = AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER,
        .location = AKS_VOLTAGE_LOCATION_12V_AUXILIARY,
        .adc_channel = 12, /* Additional ADC channel */
        .divider_ratio = 5.0f,          /* 16.5kΩ / 3.3kΩ = 5 */
        .amplifier_gain = 1.0f,
        .offset_voltage = 0.0f,
        .reference_voltage = 3.3f,
        .max_voltage = 16.0f,
        .min_voltage = 0.0f,
        .resolution = 0.01f,
        .enabled = true
    },
    /* 5V Logic Supply */
    {
        .type = AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER,
        .location = AKS_VOLTAGE_LOCATION_5V_LOGIC,
        .adc_channel = 13, /* Additional ADC channel */
        .divider_ratio = 2.0f,          /* 3.3kΩ / 3.3kΩ = 2 */
        .amplifier_gain = 1.0f,
        .offset_voltage = 0.0f,
        .reference_voltage = 3.3f,
        .max_voltage = 6.0f,
        .min_voltage = 0.0f,
        .resolution = 0.005f,
        .enabled = true
    },
    /* 3.3V Logic Supply - Direct */
    {
        .type = AKS_VOLTAGE_TYPE_DIRECT,
        .location = AKS_VOLTAGE_LOCATION_3V3_LOGIC,
        .adc_channel = 14, /* Additional ADC channel */
        .divider_ratio = 1.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 0.0f,
        .reference_voltage = 3.3f,
        .max_voltage = 3.6f,
        .min_voltage = 0.0f,
        .resolution = 0.001f,
        .enabled = true
    },
    /* Motor Phase A Voltage */
    {
        .type = AKS_VOLTAGE_TYPE_DIFFERENTIAL,
        .location = AKS_VOLTAGE_LOCATION_MOTOR_PHASE_A,
        .adc_channel = 15, /* Additional ADC channel */
        .divider_ratio = 50.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 1.65f,        /* Biased at mid-rail */
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = -100.0f,
        .resolution = 0.5f,
        .enabled = true
    },
    /* Motor Phase B Voltage */
    {
        .type = AKS_VOLTAGE_TYPE_DIFFERENTIAL,
        .location = AKS_VOLTAGE_LOCATION_MOTOR_PHASE_B,
        .adc_channel = AKS_ADC_CH_MOTOR_VOLTAGE, /* Using existing channel */
        .divider_ratio = 50.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 1.65f,
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = -100.0f,
        .resolution = 0.5f,
        .enabled = true
    },
    /* Motor Phase C Voltage */
    {
        .type = AKS_VOLTAGE_TYPE_DIFFERENTIAL,
        .location = AKS_VOLTAGE_LOCATION_MOTOR_PHASE_C,
        .adc_channel = AKS_ADC_CH_ISOLATION_VOLTAGE, /* Using available channel */
        .divider_ratio = 50.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 1.65f,
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = -100.0f,
        .resolution = 0.5f,
        .enabled = true
    },
    /* Isolated Sensor Voltage */
    {
        .type = AKS_VOLTAGE_TYPE_ISOLATION_AMPLIFIER,
        .location = AKS_VOLTAGE_LOCATION_ISOLATED_SENSOR,
        .adc_channel = AKS_ADC_CH_MISC_1,
        .divider_ratio = 1.0f,
        .amplifier_gain = 0.5f,         /* Isolation amplifier gain */
        .offset_voltage = 2.5f,         /* Isolation amplifier offset */
        .reference_voltage = 3.3f,
        .max_voltage = 100.0f,
        .min_voltage = -100.0f,
        .resolution = 0.2f,
        .enabled = true
    }
};

/* Static sensor data */
static aks_voltage_sensor_data_t g_voltage_sensor_data[AKS_VOLTAGE_MAX_SENSORS] = {0};
static bool g_voltage_initialized = false;
static uint16_t g_lpf_handles[AKS_VOLTAGE_MAX_SENSORS] = {0};

/* Statistics */
static struct {
    uint32_t total_readings;
    uint32_t fault_count;
    uint32_t overvoltage_events;
    uint32_t undervoltage_events;
    float max_voltage_recorded;
} g_voltage_stats = {0};

/* Private function prototypes */
static float aks_voltage_convert_reading(const aks_voltage_sensor_config_t* config, float adc_voltage);
static bool aks_voltage_validate_reading(const aks_voltage_sensor_config_t* config, float voltage);
static void aks_voltage_update_filter(aks_voltage_sensor_data_t* data, float new_voltage);
static float aks_voltage_calculate_rms(const aks_voltage_sensor_data_t* data);
static void aks_voltage_check_limits(const aks_voltage_sensor_config_t* config, aks_voltage_sensor_data_t* data);

/**
 * @brief Initialize voltage sensor system
 */
aks_result_t aks_voltage_init(void)
{
    if (g_voltage_initialized) {
        return AKS_OK;
    }
    
    /* Initialize ADC if not already done */
    aks_result_t result = aks_adc_init();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize sensor data structures */
    for (uint8_t i = 0; i < AKS_VOLTAGE_MAX_SENSORS; i++) {
        memset(&g_voltage_sensor_data[i], 0, sizeof(aks_voltage_sensor_data_t));
        g_voltage_sensor_data[i].data_valid = false;
        g_voltage_sensor_data[i].min_voltage_recorded = 1000.0f;
        g_voltage_sensor_data[i].max_voltage_recorded = -1000.0f;
        
        /* Initialize filter */
        if (g_voltage_sensor_configs[i].enabled) {
            aks_math_lpf_create(0.1f, AKS_VOLTAGE_UPDATE_RATE_HZ, &g_lpf_handles[i]);
        }
    }
    
    /* Clear statistics */
    memset(&g_voltage_stats, 0, sizeof(g_voltage_stats));
    
    g_voltage_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Update all voltage sensors
 */
aks_result_t aks_voltage_update(void)
{
    if (!g_voltage_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    for (uint8_t i = 0; i < AKS_VOLTAGE_MAX_SENSORS; i++) {
        const aks_voltage_sensor_config_t* config = &g_voltage_sensor_configs[i];
        aks_voltage_sensor_data_t* data = &g_voltage_sensor_data[i];
        
        if (!config->enabled) {
            continue;
        }
        
        /* Read ADC value */
        uint16_t adc_raw;
        float adc_voltage;
        aks_result_t result = aks_adc_read_channel(config->adc_channel, &adc_raw, &adc_voltage);
        
        if (result != AKS_OK) {
            data->fault_active = true;
            data->fault_count++;
            g_voltage_stats.fault_count++;
            continue;
        }
        
        data->raw_adc = adc_voltage;
        
        /* Convert to actual voltage */
        float voltage = aks_voltage_convert_reading(config, adc_voltage);
        
        /* Validate reading */
        if (!aks_voltage_validate_reading(config, voltage)) {
            data->fault_active = true;
            data->fault_count++;
            continue;
        }
        
        /* Check voltage limits */
        aks_voltage_check_limits(config, data);
        
        /* Update filter */
        aks_voltage_update_filter(data, voltage);
        
        /* Apply low-pass filter */
        data->filtered_voltage = aks_math_lpf_process(g_lpf_handles[i], voltage);
        
        /* Calculate RMS voltage */
        data->rms_voltage = aks_voltage_calculate_rms(data);
        
        /* Update peak voltage */
        if (fabsf(voltage) > fabsf(data->peak_voltage)) {
            data->peak_voltage = voltage;
        }
        
        /* Update min/max recorded voltages */
        if (voltage < data->min_voltage_recorded) {
            data->min_voltage_recorded = voltage;
        }
        if (voltage > data->max_voltage_recorded) {
            data->max_voltage_recorded = voltage;
        }
        
        /* Update data */
        data->voltage = voltage;
        data->data_valid = true;
        data->fault_active = false;
        data->last_update_time = 0; /* Would use HAL_GetTick() */
        
        /* Update statistics */
        g_voltage_stats.total_readings++;
        if (fabsf(voltage) > g_voltage_stats.max_voltage_recorded) {
            g_voltage_stats.max_voltage_recorded = fabsf(voltage);
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Get voltage reading for specific location
 */
aks_result_t aks_voltage_get_reading(aks_voltage_location_t location, float* voltage)
{
    if (!g_voltage_initialized || voltage == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_VOLTAGE_MAX_SENSORS; i++) {
        if (g_voltage_sensor_configs[i].location == location && 
            g_voltage_sensor_configs[i].enabled) {
            
            if (!g_voltage_sensor_data[i].data_valid) {
                return AKS_ERROR_NOT_READY;
            }
            
            *voltage = g_voltage_sensor_data[i].filtered_voltage;
            return AKS_OK;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get RMS voltage for specific location
 */
aks_result_t aks_voltage_get_rms(aks_voltage_location_t location, float* rms_voltage)
{
    if (!g_voltage_initialized || rms_voltage == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_VOLTAGE_MAX_SENSORS; i++) {
        if (g_voltage_sensor_configs[i].location == location && 
            g_voltage_sensor_configs[i].enabled) {
            
            if (!g_voltage_sensor_data[i].data_valid) {
                return AKS_ERROR_NOT_READY;
            }
            
            *rms_voltage = g_voltage_sensor_data[i].rms_voltage;
            return AKS_OK;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get all voltage readings
 */
aks_result_t aks_voltage_get_all_readings(float voltages[AKS_VOLTAGE_MAX_SENSORS], bool* valid_flags)
{
    if (!g_voltage_initialized || voltages == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_VOLTAGE_MAX_SENSORS; i++) {
        voltages[i] = g_voltage_sensor_data[i].filtered_voltage;
        
        if (valid_flags != NULL) {
            valid_flags[i] = g_voltage_sensor_data[i].data_valid && !g_voltage_sensor_data[i].fault_active;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Check if any voltage sensor has overvoltage
 */
bool aks_voltage_has_overvoltage(void)
{
    if (!g_voltage_initialized) {
        return false;
    }
    
    for (uint8_t i = 0; i < AKS_VOLTAGE_MAX_SENSORS; i++) {
        if (g_voltage_sensor_configs[i].enabled && g_voltage_sensor_data[i].overvoltage) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Check if any voltage sensor has undervoltage
 */
bool aks_voltage_has_undervoltage(void)
{
    if (!g_voltage_initialized) {
        return false;
    }
    
    for (uint8_t i = 0; i < AKS_VOLTAGE_MAX_SENSORS; i++) {
        if (g_voltage_sensor_configs[i].enabled && g_voltage_sensor_data[i].undervoltage) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Get voltage statistics
 */
aks_result_t aks_voltage_get_stats(uint32_t* total_readings, uint32_t* fault_count, 
                                   uint32_t* overvoltage_events, uint32_t* undervoltage_events)
{
    if (!g_voltage_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (total_readings != NULL) {
        *total_readings = g_voltage_stats.total_readings;
    }
    
    if (fault_count != NULL) {
        *fault_count = g_voltage_stats.fault_count;
    }
    
    if (overvoltage_events != NULL) {
        *overvoltage_events = g_voltage_stats.overvoltage_events;
    }
    
    if (undervoltage_events != NULL) {
        *undervoltage_events = g_voltage_stats.undervoltage_events;
    }
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Convert ADC reading to actual voltage
 */
static float aks_voltage_convert_reading(const aks_voltage_sensor_config_t* config, float adc_voltage)
{
    float actual_voltage = 0.0f;
    
    switch (config->type) {
        case AKS_VOLTAGE_TYPE_RESISTIVE_DIVIDER:
            actual_voltage = (adc_voltage - config->offset_voltage) * config->divider_ratio / config->amplifier_gain;
            break;
            
        case AKS_VOLTAGE_TYPE_ISOLATION_AMPLIFIER:
            actual_voltage = (adc_voltage - config->offset_voltage) / config->amplifier_gain;
            break;
            
        case AKS_VOLTAGE_TYPE_DIFFERENTIAL:
            actual_voltage = (adc_voltage - config->offset_voltage) * config->divider_ratio / config->amplifier_gain;
            break;
            
        case AKS_VOLTAGE_TYPE_DIRECT:
            actual_voltage = adc_voltage;
            break;
            
        default:
            actual_voltage = 0.0f;
            break;
    }
    
    return actual_voltage;
}

/**
 * @brief Validate voltage reading
 */
static bool aks_voltage_validate_reading(const aks_voltage_sensor_config_t* config, float voltage)
{
    return (voltage >= config->min_voltage && voltage <= config->max_voltage);
}

/**
 * @brief Update moving average filter
 */
static void aks_voltage_update_filter(aks_voltage_sensor_data_t* data, float new_voltage)
{
    data->voltage_history[data->history_index] = new_voltage;
    data->history_index = (data->history_index + 1) % AKS_VOLTAGE_FILTER_SAMPLES;
}

/**
 * @brief Calculate RMS voltage
 */
static float aks_voltage_calculate_rms(const aks_voltage_sensor_data_t* data)
{
    float sum_squares = 0.0f;
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < AKS_VOLTAGE_FILTER_SAMPLES; i++) {
        sum_squares += data->voltage_history[i] * data->voltage_history[i];
        count++;
    }
    
    return sqrtf(sum_squares / count);
}

/**
 * @brief Check voltage limits and set flags
 */
static void aks_voltage_check_limits(const aks_voltage_sensor_config_t* config, aks_voltage_sensor_data_t* data)
{
    /* Check overvoltage */
    if (data->voltage > config->max_voltage * 0.95f) {
        data->overvoltage = true;
        g_voltage_stats.overvoltage_events++;
    } else {
        data->overvoltage = false;
    }
    
    /* Check undervoltage */
    if (data->voltage < config->min_voltage * 1.05f) {
        data->undervoltage = true;
        g_voltage_stats.undervoltage_events++;
    } else {
        data->undervoltage = false;
    }
}
