/**
 * @file aks_temperature.c
 * @brief Temperature sensor implementation for AKS
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

/* Temperature Sensor Configuration */
#define AKS_TEMP_MAX_SENSORS        8           /**< Maximum temperature sensors */
#define AKS_TEMP_FILTER_SAMPLES     16          /**< Moving average filter samples */
#define AKS_TEMP_UPDATE_RATE_HZ     10          /**< Temperature update rate */
#define AKS_TEMP_FAULT_THRESHOLD    5.0f        /**< Fault threshold in °C */

/* Temperature Sensor Types */
typedef enum {
    AKS_TEMP_TYPE_NTC_10K = 0,      /**< NTC 10kΩ thermistor */
    AKS_TEMP_TYPE_NTC_100K,         /**< NTC 100kΩ thermistor */
    AKS_TEMP_TYPE_PT100,            /**< PT100 RTD */
    AKS_TEMP_TYPE_PT1000,           /**< PT1000 RTD */
    AKS_TEMP_TYPE_LM35,             /**< LM35 IC sensor */
    AKS_TEMP_TYPE_DS18B20           /**< DS18B20 digital sensor */
} aks_temp_sensor_type_t;

/* Temperature Sensor Locations */
typedef enum {
    AKS_TEMP_LOCATION_MOTOR = 0,    /**< Motor temperature */
    AKS_TEMP_LOCATION_CONTROLLER,   /**< Controller temperature */
    AKS_TEMP_LOCATION_BATTERY_1,    /**< Battery pack 1 */
    AKS_TEMP_LOCATION_BATTERY_2,    /**< Battery pack 2 */
    AKS_TEMP_LOCATION_AMBIENT,      /**< Ambient temperature */
    AKS_TEMP_LOCATION_COOLANT,      /**< Coolant temperature */
    AKS_TEMP_LOCATION_CHARGER,      /**< Charger temperature */
    AKS_TEMP_LOCATION_INVERTER      /**< Inverter temperature */
} aks_temp_location_t;

/* Temperature Sensor Configuration */
typedef struct {
    aks_temp_sensor_type_t type;    /**< Sensor type */
    aks_temp_location_t location;   /**< Sensor location */
    uint8_t adc_channel;            /**< ADC channel */
    float r_series;                 /**< Series resistor value (Ω) */
    float beta;                     /**< Beta coefficient for NTC */
    float r_nominal;                /**< Nominal resistance at 25°C */
    float temp_nominal;             /**< Nominal temperature (°C) */
    float offset;                   /**< Temperature offset (°C) */
    float scale;                    /**< Temperature scale factor */
    float min_temp;                 /**< Minimum valid temperature */
    float max_temp;                 /**< Maximum valid temperature */
    bool enabled;                   /**< Sensor enabled flag */
} aks_temp_sensor_config_t;

/* Temperature Sensor Data */
typedef struct {
    float temperature;              /**< Current temperature (°C) */
    float raw_adc;                  /**< Raw ADC value */
    float filtered_temp;            /**< Filtered temperature */
    float temp_history[AKS_TEMP_FILTER_SAMPLES]; /**< Temperature history */
    uint8_t history_index;          /**< History circular buffer index */
    uint32_t last_update_time;      /**< Last update timestamp */
    uint32_t fault_count;           /**< Fault counter */
    bool fault_active;              /**< Fault status */
    bool data_valid;                /**< Data validity flag */
} aks_temp_sensor_data_t;

/* Static sensor configurations */
static const aks_temp_sensor_config_t g_temp_sensor_configs[AKS_TEMP_MAX_SENSORS] = {
    /* Motor Temperature - NTC 10kΩ */
    {
        .type = AKS_TEMP_TYPE_NTC_10K,
        .location = AKS_TEMP_LOCATION_MOTOR,
        .adc_channel = AKS_ADC_CH_MOTOR_TEMP,
        .r_series = 10000.0f,
        .beta = 3950.0f,
        .r_nominal = 10000.0f,
        .temp_nominal = 25.0f,
        .offset = 0.0f,
        .scale = 1.0f,
        .min_temp = -40.0f,
        .max_temp = 150.0f,
        .enabled = true
    },
    /* Controller Temperature - NTC 10kΩ */
    {
        .type = AKS_TEMP_TYPE_NTC_10K,
        .location = AKS_TEMP_LOCATION_CONTROLLER,
        .adc_channel = AKS_ADC_CH_CONTROLLER_TEMP,
        .r_series = 10000.0f,
        .beta = 3950.0f,
        .r_nominal = 10000.0f,
        .temp_nominal = 25.0f,
        .offset = 0.0f,
        .scale = 1.0f,
        .min_temp = -40.0f,
        .max_temp = 125.0f,
        .enabled = true
    },
    /* Battery Temperature 1 - NTC 10kΩ */
    {
        .type = AKS_TEMP_TYPE_NTC_10K,
        .location = AKS_TEMP_LOCATION_BATTERY_1,
        .adc_channel = 8, /* Additional ADC channel */
        .r_series = 10000.0f,
        .beta = 3950.0f,
        .r_nominal = 10000.0f,
        .temp_nominal = 25.0f,
        .offset = 0.0f,
        .scale = 1.0f,
        .min_temp = -20.0f,
        .max_temp = 60.0f,
        .enabled = true
    },
    /* Battery Temperature 2 - NTC 10kΩ */
    {
        .type = AKS_TEMP_TYPE_NTC_10K,
        .location = AKS_TEMP_LOCATION_BATTERY_2,
        .adc_channel = 9, /* Additional ADC channel */
        .r_series = 10000.0f,
        .beta = 3950.0f,
        .r_nominal = 10000.0f,
        .temp_nominal = 25.0f,
        .offset = 0.0f,
        .scale = 1.0f,
        .min_temp = -20.0f,
        .max_temp = 60.0f,
        .enabled = true
    },
    /* Ambient Temperature - LM35 */
    {
        .type = AKS_TEMP_TYPE_LM35,
        .location = AKS_TEMP_LOCATION_AMBIENT,
        .adc_channel = 10, /* Additional ADC channel */
        .r_series = 0.0f,
        .beta = 0.0f,
        .r_nominal = 0.0f,
        .temp_nominal = 0.0f,
        .offset = 0.0f,
        .scale = 0.01f, /* 10mV/°C */
        .min_temp = -40.0f,
        .max_temp = 85.0f,
        .enabled = true
    },
    /* Coolant Temperature - NTC 10kΩ */
    {
        .type = AKS_TEMP_TYPE_NTC_10K,
        .location = AKS_TEMP_LOCATION_COOLANT,
        .adc_channel = 11, /* Additional ADC channel */
        .r_series = 10000.0f,
        .beta = 3950.0f,
        .r_nominal = 10000.0f,
        .temp_nominal = 25.0f,
        .offset = 0.0f,
        .scale = 1.0f,
        .min_temp = -20.0f,
        .max_temp = 100.0f,
        .enabled = true
    },
    /* Charger Temperature - NTC 10kΩ */
    {
        .type = AKS_TEMP_TYPE_NTC_10K,
        .location = AKS_TEMP_LOCATION_CHARGER,
        .adc_channel = 12, /* Additional ADC channel */
        .r_series = 10000.0f,
        .beta = 3950.0f,
        .r_nominal = 10000.0f,
        .temp_nominal = 25.0f,
        .offset = 0.0f,
        .scale = 1.0f,
        .min_temp = -20.0f,
        .max_temp = 85.0f,
        .enabled = true
    },
    /* Inverter Temperature - NTC 10kΩ */
    {
        .type = AKS_TEMP_TYPE_NTC_10K,
        .location = AKS_TEMP_LOCATION_INVERTER,
        .adc_channel = 13, /* Additional ADC channel */
        .r_series = 10000.0f,
        .beta = 3950.0f,
        .r_nominal = 10000.0f,
        .temp_nominal = 25.0f,
        .offset = 0.0f,
        .scale = 1.0f,
        .min_temp = -20.0f,
        .max_temp = 125.0f,
        .enabled = true
    }
};

/* Static sensor data */
static aks_temp_sensor_data_t g_temp_sensor_data[AKS_TEMP_MAX_SENSORS] = {0};
static bool g_temp_initialized = false;
static uint16_t g_lpf_handles[AKS_TEMP_MAX_SENSORS] = {0};

/* Statistics */
static struct {
    uint32_t total_readings;
    uint32_t fault_count;
    uint32_t out_of_range_count;
} g_temp_stats = {0};

/* Private function prototypes */
static float aks_temp_convert_ntc(const aks_temp_sensor_config_t* config, float adc_voltage);
static float aks_temp_convert_lm35(float adc_voltage);
static float aks_temp_convert_pt100(float resistance);
static bool aks_temp_validate_reading(const aks_temp_sensor_config_t* config, float temperature);
static void aks_temp_update_filter(aks_temp_sensor_data_t* data, float new_temp);
static float aks_temp_get_filtered_value(const aks_temp_sensor_data_t* data);

/**
 * @brief Initialize temperature sensor system
 */
aks_result_t aks_temp_init(void)
{
    if (g_temp_initialized) {
        return AKS_OK;
    }
    
    /* Initialize ADC if not already done */
    aks_result_t result = aks_adc_init();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize math module for filters */
    aks_math_lpf_create(1.0f, AKS_TEMP_UPDATE_RATE_HZ, &g_lpf_handles[0]);
    
    /* Initialize sensor data structures */
    for (uint8_t i = 0; i < AKS_TEMP_MAX_SENSORS; i++) {
        memset(&g_temp_sensor_data[i], 0, sizeof(aks_temp_sensor_data_t));
        g_temp_sensor_data[i].data_valid = false;
        
        /* Initialize filter */
        if (g_temp_sensor_configs[i].enabled) {
            aks_math_lpf_create(0.1f, AKS_TEMP_UPDATE_RATE_HZ, &g_lpf_handles[i]);
        }
    }
    
    /* Clear statistics */
    memset(&g_temp_stats, 0, sizeof(g_temp_stats));
    
    g_temp_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Update all temperature sensors
 */
aks_result_t aks_temp_update(void)
{
    if (!g_temp_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    for (uint8_t i = 0; i < AKS_TEMP_MAX_SENSORS; i++) {
        const aks_temp_sensor_config_t* config = &g_temp_sensor_configs[i];
        aks_temp_sensor_data_t* data = &g_temp_sensor_data[i];
        
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
            continue;
        }
        
        data->raw_adc = adc_voltage;
        
        /* Convert to temperature based on sensor type */
        float temperature = 0.0f;
        
        switch (config->type) {
            case AKS_TEMP_TYPE_NTC_10K:
            case AKS_TEMP_TYPE_NTC_100K:
                temperature = aks_temp_convert_ntc(config, adc_voltage);
                break;
                
            case AKS_TEMP_TYPE_LM35:
                temperature = aks_temp_convert_lm35(adc_voltage);
                break;
                
            case AKS_TEMP_TYPE_PT100:
            case AKS_TEMP_TYPE_PT1000:
                /* PT100/PT1000 requires resistance measurement */
                temperature = aks_temp_convert_pt100(adc_voltage * 1000.0f); /* Assuming voltage to resistance conversion */
                break;
                
            default:
                data->fault_active = true;
                continue;
        }
        
        /* Apply offset and scale */
        temperature = (temperature + config->offset) * config->scale;
        
        /* Validate reading */
        if (!aks_temp_validate_reading(config, temperature)) {
            data->fault_active = true;
            data->fault_count++;
            g_temp_stats.out_of_range_count++;
            continue;
        }
        
        /* Update filter */
        aks_temp_update_filter(data, temperature);
        
        /* Apply low-pass filter */
        data->filtered_temp = aks_math_lpf_process(g_lpf_handles[i], temperature);
        
        /* Update data */
        data->temperature = temperature;
        data->data_valid = true;
        data->fault_active = false;
        data->last_update_time = 0; /* Would use HAL_GetTick() */
        
        g_temp_stats.total_readings++;
    }
    
    return AKS_OK;
}

/**
 * @brief Get temperature reading for specific location
 */
aks_result_t aks_temp_get_reading(aks_temp_location_t location, float* temperature)
{
    if (!g_temp_initialized || temperature == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_TEMP_MAX_SENSORS; i++) {
        if (g_temp_sensor_configs[i].location == location && 
            g_temp_sensor_configs[i].enabled) {
            
            if (!g_temp_sensor_data[i].data_valid) {
                return AKS_ERROR_NOT_READY;
            }
            
            *temperature = g_temp_sensor_data[i].filtered_temp;
            return AKS_OK;
        }
    }
    
    return AKS_ERROR_INVALID_PARAM;
}

/**
 * @brief Get all temperature readings
 */
aks_result_t aks_temp_get_all_readings(float temperatures[AKS_TEMP_MAX_SENSORS], bool* valid_flags)
{
    if (!g_temp_initialized || temperatures == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    for (uint8_t i = 0; i < AKS_TEMP_MAX_SENSORS; i++) {
        temperatures[i] = g_temp_sensor_data[i].filtered_temp;
        
        if (valid_flags != NULL) {
            valid_flags[i] = g_temp_sensor_data[i].data_valid && !g_temp_sensor_data[i].fault_active;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Check if any temperature sensor has a fault
 */
bool aks_temp_has_fault(void)
{
    if (!g_temp_initialized) {
        return true;
    }
    
    for (uint8_t i = 0; i < AKS_TEMP_MAX_SENSORS; i++) {
        if (g_temp_sensor_configs[i].enabled && g_temp_sensor_data[i].fault_active) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Get maximum temperature across all sensors
 */
float aks_temp_get_max_temperature(void)
{
    float max_temp = -1000.0f;
    
    for (uint8_t i = 0; i < AKS_TEMP_MAX_SENSORS; i++) {
        if (g_temp_sensor_configs[i].enabled && 
            g_temp_sensor_data[i].data_valid && 
            !g_temp_sensor_data[i].fault_active) {
            
            if (g_temp_sensor_data[i].filtered_temp > max_temp) {
                max_temp = g_temp_sensor_data[i].filtered_temp;
            }
        }
    }
    
    return max_temp;
}

/**
 * @brief Get minimum temperature across all sensors
 */
float aks_temp_get_min_temperature(void)
{
    float min_temp = 1000.0f;
    
    for (uint8_t i = 0; i < AKS_TEMP_MAX_SENSORS; i++) {
        if (g_temp_sensor_configs[i].enabled && 
            g_temp_sensor_data[i].data_valid && 
            !g_temp_sensor_data[i].fault_active) {
            
            if (g_temp_sensor_data[i].filtered_temp < min_temp) {
                min_temp = g_temp_sensor_data[i].filtered_temp;
            }
        }
    }
    
    return min_temp;
}

/**
 * @brief Get temperature statistics
 */
aks_result_t aks_temp_get_stats(uint32_t* total_readings, uint32_t* fault_count, uint32_t* out_of_range_count)
{
    if (!g_temp_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (total_readings != NULL) {
        *total_readings = g_temp_stats.total_readings;
    }
    
    if (fault_count != NULL) {
        *fault_count = g_temp_stats.fault_count;
    }
    
    if (out_of_range_count != NULL) {
        *out_of_range_count = g_temp_stats.out_of_range_count;
    }
    
    return AKS_OK;
}

/**
 * @brief Reset temperature statistics
 */
aks_result_t aks_temp_reset_stats(void)
{
    if (!g_temp_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    memset(&g_temp_stats, 0, sizeof(g_temp_stats));
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Convert NTC thermistor ADC reading to temperature
 */
static float aks_temp_convert_ntc(const aks_temp_sensor_config_t* config, float adc_voltage)
{
    /* Calculate resistance from voltage divider */
    float vcc = 3.3f; /* ADC reference voltage */
    float resistance = config->r_series * adc_voltage / (vcc - adc_voltage);
    
    /* Steinhart-Hart equation approximation */
    float temp_k = 1.0f / (1.0f / (config->temp_nominal + 273.15f) + 
                          logf(resistance / config->r_nominal) / config->beta);
    
    /* Convert to Celsius */
    return temp_k - 273.15f;
}

/**
 * @brief Convert LM35 ADC reading to temperature
 */
static float aks_temp_convert_lm35(float adc_voltage)
{
    /* LM35: 10mV/°C, 0°C = 0V */
    return adc_voltage / 0.01f;
}

/**
 * @brief Convert PT100 resistance to temperature
 */
static float aks_temp_convert_pt100(float resistance)
{
    /* Simplified PT100 equation: R(T) = R0(1 + αT) */
    float r0 = 100.0f; /* Resistance at 0°C */
    float alpha = 0.003851f; /* Temperature coefficient */
    
    return (resistance - r0) / (r0 * alpha);
}

/**
 * @brief Validate temperature reading
 */
static bool aks_temp_validate_reading(const aks_temp_sensor_config_t* config, float temperature)
{
    return (temperature >= config->min_temp && temperature <= config->max_temp);
}

/**
 * @brief Update moving average filter
 */
static void aks_temp_update_filter(aks_temp_sensor_data_t* data, float new_temp)
{
    data->temp_history[data->history_index] = new_temp;
    data->history_index = (data->history_index + 1) % AKS_TEMP_FILTER_SAMPLES;
}

/**
 * @brief Get filtered temperature value
 */
static float aks_temp_get_filtered_value(const aks_temp_sensor_data_t* data)
{
    float sum = 0.0f;
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < AKS_TEMP_FILTER_SAMPLES; i++) {
        if (data->temp_history[i] != 0.0f) { /* Assuming 0 means no data */
            sum += data->temp_history[i];
            count++;
        }
    }
    
    return (count > 0) ? (sum / count) : 0.0f;
}
