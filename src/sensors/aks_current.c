/**
 * @file aks_current.c
 * @brief Current sensor implementation for AKS
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

/* Current Sensor Configuration */
#define AKS_CURRENT_MAX_SENSORS     8           /**< Maximum current sensors */
#define AKS_CURRENT_FILTER_SAMPLES  32          /**< Moving average filter samples */
#define AKS_CURRENT_UPDATE_RATE_HZ  100         /**< Current update rate */
#define AKS_CURRENT_FAULT_COUNT     10          /**< Fault threshold count */

/* Current Sensor Types */
typedef enum {
    AKS_CURRENT_TYPE_SHUNT = 0,     /**< Current shunt resistor */
    AKS_CURRENT_TYPE_HALL_EFFECT,   /**< Hall effect sensor */
    AKS_CURRENT_TYPE_CT,            /**< Current transformer */
    AKS_CURRENT_TYPE_ROGOWSKI       /**< Rogowski coil */
} aks_current_sensor_type_t;

/* Current Sensor Locations */
typedef enum {
    AKS_CURRENT_LOCATION_MOTOR = 0,      /**< Motor current */
    AKS_CURRENT_LOCATION_BATTERY,        /**< Battery current */
    AKS_CURRENT_LOCATION_CHARGER,        /**< Charger current */
    AKS_CURRENT_LOCATION_INVERTER,       /**< Inverter current */
    AKS_CURRENT_LOCATION_DC_LINK,        /**< DC link current */
    AKS_CURRENT_LOCATION_AUXILIARY,      /**< Auxiliary systems */
    AKS_CURRENT_LOCATION_BRAKE_REGEN,    /**< Regenerative brake current */
    AKS_CURRENT_LOCATION_TOTAL_VEHICLE   /**< Total vehicle current */
} aks_current_location_t;

/* Current Sensor Configuration */
typedef struct {
    aks_current_sensor_type_t type;     /**< Sensor type */
    aks_current_location_t location;    /**< Sensor location */
    uint8_t adc_channel;                /**< ADC channel */
    float shunt_resistance;             /**< Shunt resistance (mΩ) */
    float amplifier_gain;               /**< Amplifier gain */
    float offset_voltage;               /**< Zero current offset voltage */
    float scale_factor;                 /**< Scale factor (A/V) */
    float max_current;                  /**< Maximum current (A) */
    float min_current;                  /**< Minimum current (A) */
    float resolution;                   /**< Current resolution (A) */
    bool bidirectional;                 /**< Bidirectional measurement */
    bool enabled;                       /**< Sensor enabled flag */
} aks_current_sensor_config_t;

/* Current Sensor Data */
typedef struct {
    float current;                      /**< Current reading (A) */
    float raw_adc;                      /**< Raw ADC value */
    float filtered_current;             /**< Filtered current */
    float rms_current;                  /**< RMS current */
    float peak_current;                 /**< Peak current */
    float current_history[AKS_CURRENT_FILTER_SAMPLES]; /**< Current history */
    uint8_t history_index;              /**< History circular buffer index */
    uint32_t last_update_time;          /**< Last update timestamp */
    uint32_t fault_count;               /**< Fault counter */
    float total_energy;                 /**< Total energy (Wh) */
    float power;                        /**< Instantaneous power (W) */
    bool fault_active;                  /**< Fault status */
    bool data_valid;                    /**< Data validity flag */
    bool overcurrent;                   /**< Overcurrent flag */
} aks_current_sensor_data_t;

/* Static sensor configurations */
static const aks_current_sensor_config_t g_current_sensor_configs[AKS_CURRENT_MAX_SENSORS] = {
    /* Motor Current - Hall Effect 300A */
    {
        .type = AKS_CURRENT_TYPE_HALL_EFFECT,
        .location = AKS_CURRENT_LOCATION_MOTOR,
        .adc_channel = AKS_ADC_CH_MOTOR_CURRENT,
        .shunt_resistance = 0.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 2.5f,     /* 2.5V @ 0A */
        .scale_factor = 120.0f,     /* 300A / 2.5V = 120 A/V */
        .max_current = 300.0f,
        .min_current = -300.0f,
        .resolution = 0.1f,
        .bidirectional = true,
        .enabled = true
    },
    /* Battery Current - Hall Effect 200A */
    {
        .type = AKS_CURRENT_TYPE_HALL_EFFECT,
        .location = AKS_CURRENT_LOCATION_BATTERY,
        .adc_channel = AKS_ADC_CH_BATTERY_CURRENT,
        .shunt_resistance = 0.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 2.5f,     /* 2.5V @ 0A */
        .scale_factor = 80.0f,      /* 200A / 2.5V = 80 A/V */
        .max_current = 200.0f,
        .min_current = -200.0f,
        .resolution = 0.05f,
        .bidirectional = true,
        .enabled = true
    },
    /* Charger Current - Shunt 50A */
    {
        .type = AKS_CURRENT_TYPE_SHUNT,
        .location = AKS_CURRENT_LOCATION_CHARGER,
        .adc_channel = 8, /* Additional ADC channel */
        .shunt_resistance = 0.5f,   /* 0.5mΩ shunt */
        .amplifier_gain = 1000.0f,  /* 1000x amplifier */
        .offset_voltage = 0.0f,
        .scale_factor = 4000.0f,    /* 50A / 0.0125V = 4000 A/V */
        .max_current = 50.0f,
        .min_current = 0.0f,
        .resolution = 0.01f,
        .bidirectional = false,
        .enabled = true
    },
    /* Inverter Current - Hall Effect 250A */
    {
        .type = AKS_CURRENT_TYPE_HALL_EFFECT,
        .location = AKS_CURRENT_LOCATION_INVERTER,
        .adc_channel = 9, /* Additional ADC channel */
        .shunt_resistance = 0.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 2.5f,
        .scale_factor = 100.0f,     /* 250A / 2.5V = 100 A/V */
        .max_current = 250.0f,
        .min_current = -250.0f,
        .resolution = 0.1f,
        .bidirectional = true,
        .enabled = true
    },
    /* DC Link Current - Hall Effect 400A */
    {
        .type = AKS_CURRENT_TYPE_HALL_EFFECT,
        .location = AKS_CURRENT_LOCATION_DC_LINK,
        .adc_channel = 10, /* Additional ADC channel */
        .shunt_resistance = 0.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 2.5f,
        .scale_factor = 160.0f,     /* 400A / 2.5V = 160 A/V */
        .max_current = 400.0f,
        .min_current = -400.0f,
        .resolution = 0.2f,
        .bidirectional = true,
        .enabled = true
    },
    /* Auxiliary Current - Shunt 20A */
    {
        .type = AKS_CURRENT_TYPE_SHUNT,
        .location = AKS_CURRENT_LOCATION_AUXILIARY,
        .adc_channel = 11, /* Additional ADC channel */
        .shunt_resistance = 2.0f,   /* 2mΩ shunt */
        .amplifier_gain = 100.0f,   /* 100x amplifier */
        .offset_voltage = 0.0f,
        .scale_factor = 100.0f,     /* 20A / 0.2V = 100 A/V */
        .max_current = 20.0f,
        .min_current = 0.0f,
        .resolution = 0.005f,
        .bidirectional = false,
        .enabled = true
    },
    /* Brake Regen Current - Hall Effect 150A */
    {
        .type = AKS_CURRENT_TYPE_HALL_EFFECT,
        .location = AKS_CURRENT_LOCATION_BRAKE_REGEN,
        .adc_channel = 12, /* Additional ADC channel */
        .shunt_resistance = 0.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 2.5f,
        .scale_factor = 60.0f,      /* 150A / 2.5V = 60 A/V */
        .max_current = 150.0f,
        .min_current = -150.0f,
        .resolution = 0.05f,
        .bidirectional = true,
        .enabled = true
    },
    /* Total Vehicle Current - CT 500A */
    {
        .type = AKS_CURRENT_TYPE_CT,
        .location = AKS_CURRENT_LOCATION_TOTAL_VEHICLE,
        .adc_channel = 13, /* Additional ADC channel */
        .shunt_resistance = 0.0f,
        .amplifier_gain = 1.0f,
        .offset_voltage = 2.5f,
        .scale_factor = 200.0f,     /* 500A / 2.5V = 200 A/V */
        .max_current = 500.0f,
        .min_current = -500.0f,
        .resolution = 0.25f,
        .bidirectional = true,
        .enabled = true
    }
};

/* Static sensor data */
static aks_current_sensor_data_t g_current_sensor_data[AKS_CURRENT_MAX_SENSORS] = {0};
static bool g_current_initialized = false;
static uint16_t g_lpf_handles[AKS_CURRENT_MAX_SENSORS] = {0};

/* Statistics */
static struct {
    uint32_t total_readings;
    uint32_t fault_count;
    uint32_t overcurrent_events;
    float max_current_recorded;
    float total_energy_consumed;
} g_current_stats = {0};

/* Private function prototypes */
static float aks_current_convert_shunt(const aks_current_sensor_config_t* config, float adc_voltage);
static float aks_current_convert_hall_effect(const aks_current_sensor_config_t* config, float adc_voltage);
static float aks_current_convert_ct(const aks_current_sensor_config_t* config, float adc_voltage);
static bool aks_current_validate_reading(const aks_current_sensor_config_t* config, float current);
static void aks_current_update_filter(aks_current_sensor_data_t* data, float new_current);
static float aks_current_calculate_rms(const aks_current_sensor_data_t* data);
static void aks_current_update_energy(aks_current_sensor_data_t* data, float voltage, float dt);

/**
 * @brief Initialize current sensor system
 */
aks_result_t aks_current_init(void)
{
    if (g_current_initialized) {
        return AKS_OK;
    }
    
    /* Initialize ADC if not already done */
    aks_result_t result = aks_adc_init();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize sensor data structures */
    for (uint8_t i = 0; i < AKS_CURRENT_MAX_SENSORS; i++) {
        memset(&g_current_sensor_data[i], 0, sizeof(aks_current_sensor_data_t));
        g_current_sensor_data[i].data_valid = false;
        
        /* Initialize filter */
        if (g_current_sensor_configs[i].enabled) {
            aks_math_lpf_create(0.05f, AKS_CURRENT_UPDATE_RATE_HZ, &g_lpf_handles[i]);
        }
    }
    
    /* Clear statistics */
    memset(&g_current_stats, 0, sizeof(g_current_stats));
    
    g_current_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Update all current sensors
 */
aks_result_t aks_current_update(float system_voltage, float dt)
{
    if (!g_current_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    for (uint8_t i = 0; i < AKS_CURRENT_MAX_SENSORS; i++) {
        const aks_current_sensor_config_t* config = &g_current_sensor_configs[i];
        aks_current_sensor_data_t* data = &g_current_sensor_data[i];
        
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
            g_current_stats.fault_count++;
            continue;
        }
        
        data->raw_adc = adc_voltage;
        
        /* Convert to current based on sensor type */
        float current = 0.0f;
        
        switch (config->type) {
            case AKS_CURRENT_TYPE_SHUNT:
                current = aks_current_convert_shunt(config, adc_voltage);
                break;
                
            case AKS_CURRENT_TYPE_HALL_EFFECT:
                current = aks_current_convert_hall_effect(config, adc_voltage);
                break;
                
            case AKS_CURRENT_TYPE_CT:
            case AKS_CURRENT_TYPE_ROGOWSKI:
                current = aks_current_convert_ct(config, adc_voltage);
                break;
                
            default:
                data->fault_active = true;
                continue;
        }
        
        /* Validate reading */
        if (!aks_current_validate_reading(config, current)) {
            data->fault_active = true;
            data->fault_count++;
            continue;
        }
        
        /* Check for overcurrent */
        if (fabsf(current) > config->max_current * 0.9f) {
            data->overcurrent = true;
            g_current_stats.overcurrent_events++;
        } else {
            data->overcurrent = false;
        }
        
        /* Update filter */
        aks_current_update_filter(data, current);
        
        /* Apply low-pass filter */
        data->filtered_current = aks_math_lpf_process(g_lpf_handles[i], current);
        
        /* Calculate RMS current */
        data->rms_current = aks_current_calculate_rms(data);
        
        /* Update peak current */
        if (fabsf(current) > fabsf(data->peak_current)) {
            data->peak_current = current;
        }
        
        /* Calculate instantaneous power */
        data->power = current * system_voltage;
        
        /* Update energy calculation */
        aks_current_update_energy(data, system_voltage, dt);
        
        /* Update data */
        data->current = current;
        data->data_valid = true;
        data->fault_active = false;
        data->last_update_time = 0; /* Would use HAL_GetTick() */
        
        /* Update statistics */
        g_current_stats.total_readings++;
        if (fabsf(current) > g_current_stats.max_current_recorded) {
            g_current_stats.max_current_recorded = fabsf(current);
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Get current reading for specific location
 */
aks_result_t aks_current_get_reading(aks_current_location_t location, float* current)
{
    if (!g_current_initialized || current == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_CURRENT_MAX_SENSORS; i++) {
        if (g_current_sensor_configs[i].location == location && 
            g_current_sensor_configs[i].enabled) {
            
            if (!g_current_sensor_data[i].data_valid) {
                return AKS_ERROR_NOT_READY;
            }
            
            *current = g_current_sensor_data[i].filtered_current;
            return AKS_OK;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get RMS current for specific location
 */
aks_result_t aks_current_get_rms(aks_current_location_t location, float* rms_current)
{
    if (!g_current_initialized || rms_current == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_CURRENT_MAX_SENSORS; i++) {
        if (g_current_sensor_configs[i].location == location && 
            g_current_sensor_configs[i].enabled) {
            
            if (!g_current_sensor_data[i].data_valid) {
                return AKS_ERROR_NOT_READY;
            }
            
            *rms_current = g_current_sensor_data[i].rms_current;
            return AKS_OK;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get power for specific location
 */
aks_result_t aks_current_get_power(aks_current_location_t location, float* power)
{
    if (!g_current_initialized || power == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_CURRENT_MAX_SENSORS; i++) {
        if (g_current_sensor_configs[i].location == location && 
            g_current_sensor_configs[i].enabled) {
            
            if (!g_current_sensor_data[i].data_valid) {
                return AKS_ERROR_NOT_READY;
            }
            
            *power = g_current_sensor_data[i].power;
            return AKS_OK;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get total energy consumption
 */
float aks_current_get_total_energy(void)
{
    return g_current_stats.total_energy_consumed;
}

/**
 * @brief Check if any current sensor has overcurrent
 */
bool aks_current_has_overcurrent(void)
{
    if (!g_current_initialized) {
        return false;
    }
    
    for (uint8_t i = 0; i < AKS_CURRENT_MAX_SENSORS; i++) {
        if (g_current_sensor_configs[i].enabled && g_current_sensor_data[i].overcurrent) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Get current statistics
 */
aks_result_t aks_current_get_stats(uint32_t* total_readings, uint32_t* fault_count, 
                                   uint32_t* overcurrent_events, float* max_current)
{
    if (!g_current_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (total_readings != NULL) {
        *total_readings = g_current_stats.total_readings;
    }
    
    if (fault_count != NULL) {
        *fault_count = g_current_stats.fault_count;
    }
    
    if (overcurrent_events != NULL) {
        *overcurrent_events = g_current_stats.overcurrent_events;
    }
    
    if (max_current != NULL) {
        *max_current = g_current_stats.max_current_recorded;
    }
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Convert shunt sensor ADC reading to current
 */
static float aks_current_convert_shunt(const aks_current_sensor_config_t* config, float adc_voltage)
{
    /* Current = (ADC_Voltage - Offset) / (Shunt_Resistance * Amplifier_Gain) */
    float voltage_drop = (adc_voltage - config->offset_voltage) / config->amplifier_gain;
    return voltage_drop / (config->shunt_resistance / 1000.0f); /* Convert mΩ to Ω */
}

/**
 * @brief Convert Hall effect sensor ADC reading to current
 */
static float aks_current_convert_hall_effect(const aks_current_sensor_config_t* config, float adc_voltage)
{
    /* Current = (ADC_Voltage - Offset_Voltage) * Scale_Factor */
    return (adc_voltage - config->offset_voltage) * config->scale_factor;
}

/**
 * @brief Convert CT sensor ADC reading to current
 */
static float aks_current_convert_ct(const aks_current_sensor_config_t* config, float adc_voltage)
{
    /* Similar to Hall effect but may have different scaling */
    return (adc_voltage - config->offset_voltage) * config->scale_factor;
}

/**
 * @brief Validate current reading
 */
static bool aks_current_validate_reading(const aks_current_sensor_config_t* config, float current)
{
    return (current >= config->min_current && current <= config->max_current);
}

/**
 * @brief Update moving average filter
 */
static void aks_current_update_filter(aks_current_sensor_data_t* data, float new_current)
{
    data->current_history[data->history_index] = new_current;
    data->history_index = (data->history_index + 1) % AKS_CURRENT_FILTER_SAMPLES;
}

/**
 * @brief Calculate RMS current
 */
static float aks_current_calculate_rms(const aks_current_sensor_data_t* data)
{
    float sum_squares = 0.0f;
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < AKS_CURRENT_FILTER_SAMPLES; i++) {
        sum_squares += data->current_history[i] * data->current_history[i];
        count++;
    }
    
    return sqrtf(sum_squares / count);
}

/**
 * @brief Update energy calculation
 */
static void aks_current_update_energy(aks_current_sensor_data_t* data, float voltage, float dt)
{
    /* Energy = Power * Time */
    float energy_increment = fabsf(data->power) * dt / 3600.0f; /* Convert to Wh */
    data->total_energy += energy_increment;
    g_current_stats.total_energy_consumed += energy_increment;
}
