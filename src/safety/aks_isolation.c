/**
 * @file aks_isolation.c
 * @brief Isolation monitoring implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_can.h"
#include <string.h>
#include <math.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Isolation Monitoring Constants */
#define AKS_ISOLATION_MIN_RESISTANCE        50000.0f    // 50 kΩ minimum
#define AKS_ISOLATION_WARNING_LEVEL         200000.0f   // 200 kΩ warning
#define AKS_ISOLATION_GOOD_LEVEL           1000000.0f   // 1 MΩ good
#define AKS_ISOLATION_MEASUREMENT_VOLTAGE   12.0f       // Test voltage
#define AKS_ISOLATION_ADC_RESOLUTION        4096        // 12-bit ADC
#define AKS_ISOLATION_VREF                  3.3f        // Reference voltage
#define AKS_ISOLATION_FILTER_ALPHA          0.1f        // Low-pass filter
#define AKS_ISOLATION_MEASUREMENT_PERIOD    100         // ms

/* CAN Message IDs */
#define AKS_ISOLATION_CAN_ID_STATUS         0x400
#define AKS_ISOLATION_CAN_ID_FAULT          0x401
#define AKS_ISOLATION_CAN_ID_CALIBRATION    0x402

/* Isolation Status */
typedef enum {
    AKS_ISOLATION_STATUS_UNKNOWN = 0,
    AKS_ISOLATION_STATUS_GOOD,
    AKS_ISOLATION_STATUS_WARNING,
    AKS_ISOLATION_STATUS_FAULT,
    AKS_ISOLATION_STATUS_CRITICAL
} aks_isolation_status_level_t;

/* Isolation Measurement State */
typedef enum {
    AKS_ISOLATION_STATE_IDLE = 0,
    AKS_ISOLATION_STATE_MEASURING_POSITIVE,
    AKS_ISOLATION_STATE_MEASURING_NEGATIVE,
    AKS_ISOLATION_STATE_CALCULATING,
    AKS_ISOLATION_STATE_FAULT
} aks_isolation_measurement_state_t;

/* Isolation Configuration */
typedef struct {
    float min_resistance;           /**< Minimum acceptable resistance in Ω */
    float warning_resistance;       /**< Warning level resistance in Ω */
    float good_resistance;          /**< Good level resistance in Ω */
    float test_voltage;            /**< Test voltage in V */
    uint16_t measurement_period_ms; /**< Measurement period in ms */
    float filter_alpha;            /**< Low-pass filter coefficient */
    bool enable_auto_test;         /**< Enable automatic testing */
    bool enable_can_reporting;     /**< Enable CAN reporting */
} aks_isolation_config_t;

/* Isolation Measurements */
typedef struct {
    float positive_resistance;      /**< Positive rail resistance in Ω */
    float negative_resistance;      /**< Negative rail resistance in Ω */
    float total_resistance;         /**< Total isolation resistance in Ω */
    float filtered_resistance;      /**< Filtered resistance value in Ω */
    uint16_t positive_adc_raw;      /**< Raw ADC value for positive */
    uint16_t negative_adc_raw;      /**< Raw ADC value for negative */
    uint32_t measurement_count;     /**< Total measurement count */
    uint32_t fault_count;          /**< Fault occurrence count */
} aks_isolation_measurements_t;

/* Isolation Status Structure */
typedef struct {
    aks_isolation_status_level_t status_level;      /**< Current status level */
    aks_isolation_measurement_state_t state;        /**< Measurement state */
    aks_isolation_measurements_t measurements;      /**< Current measurements */
    bool fault_detected;                           /**< Fault detection flag */
    uint16_t fault_code;                           /**< Current fault code */
    uint32_t last_measurement_time;                /**< Last measurement timestamp */
    uint32_t last_good_measurement_time;           /**< Last good measurement timestamp */
    bool calibration_needed;                       /**< Calibration required flag */
} aks_isolation_status_struct_t;

/* Isolation Handle */
typedef struct {
    bool initialized;
    aks_isolation_config_t config;
    aks_isolation_status_struct_t status;
    ADC_HandleTypeDef* hadc;
    uint32_t adc_channel_positive;
    uint32_t adc_channel_negative;
    float calibration_offset_positive;
    float calibration_offset_negative;
    float calibration_gain_positive;
    float calibration_gain_negative;
} aks_isolation_handle_t;

/* Private variables */
static aks_isolation_handle_t g_isolation_handle;

/* Default configuration */
static const aks_isolation_config_t g_default_config = {
    .min_resistance = AKS_ISOLATION_MIN_RESISTANCE,
    .warning_resistance = AKS_ISOLATION_WARNING_LEVEL,
    .good_resistance = AKS_ISOLATION_GOOD_LEVEL,
    .test_voltage = AKS_ISOLATION_MEASUREMENT_VOLTAGE,
    .measurement_period_ms = AKS_ISOLATION_MEASUREMENT_PERIOD,
    .filter_alpha = AKS_ISOLATION_FILTER_ALPHA,
    .enable_auto_test = true,
    .enable_can_reporting = true
};

/* Private function prototypes */
static aks_result_t aks_isolation_perform_measurement(void);
static aks_result_t aks_isolation_read_adc(uint32_t channel, uint16_t* value);
static float aks_isolation_calculate_resistance(uint16_t adc_value, bool is_positive);
static aks_result_t aks_isolation_update_status(void);
static aks_result_t aks_isolation_send_can_status(void);
static aks_result_t aks_isolation_handle_fault(uint16_t fault_code);
static float aks_isolation_apply_filter(float new_value, float old_value);

/**
 * @brief Initialize isolation monitoring system
 */
aks_result_t aks_isolation_init(const aks_isolation_config_t* config, ADC_HandleTypeDef* hadc,
                               uint32_t channel_positive, uint32_t channel_negative)
{
    if (g_isolation_handle.initialized) {
        return AKS_ERROR_BUSY;
    }
    
    if (hadc == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Use default config if none provided */
    if (config != NULL) {
        g_isolation_handle.config = *config;
    } else {
        g_isolation_handle.config = g_default_config;
    }
    
    /* Store ADC configuration */
    g_isolation_handle.hadc = hadc;
    g_isolation_handle.adc_channel_positive = channel_positive;
    g_isolation_handle.adc_channel_negative = channel_negative;
    
    /* Initialize status */
    memset(&g_isolation_handle.status, 0, sizeof(aks_isolation_status_struct_t));
    g_isolation_handle.status.status_level = AKS_ISOLATION_STATUS_UNKNOWN;
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_IDLE;
    
    /* Initialize calibration values */
    g_isolation_handle.calibration_offset_positive = 0.0f;
    g_isolation_handle.calibration_offset_negative = 0.0f;
    g_isolation_handle.calibration_gain_positive = 1.0f;
    g_isolation_handle.calibration_gain_negative = 1.0f;
    
    g_isolation_handle.initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize isolation monitoring system
 */
aks_result_t aks_isolation_deinit(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Reset handle */
    memset(&g_isolation_handle, 0, sizeof(aks_isolation_handle_t));
    
    return AKS_OK;
}

/**
 * @brief Start isolation monitoring
 */
aks_result_t aks_isolation_start(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_MEASURING_POSITIVE;
    g_isolation_handle.status.last_measurement_time = HAL_GetTick();
    
    return AKS_OK;
}

/**
 * @brief Stop isolation monitoring
 */
aks_result_t aks_isolation_stop(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_IDLE;
    
    return AKS_OK;
}

/**
 * @brief Measure isolation resistance
 */
aks_result_t aks_isolation_measure(aks_isolation_status_t* status)
{
    if (!g_isolation_handle.initialized || status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Perform measurement */
    aks_result_t result = aks_isolation_perform_measurement();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Update status */
    result = aks_isolation_update_status();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Fill output structure */
    status->emergency_stop = g_isolation_handle.status.fault_detected;
    status->isolation_fault = g_isolation_handle.status.fault_detected;
    status->isolation_resistance = g_isolation_handle.status.measurements.filtered_resistance / 1000.0f; // Convert to kΩ
    status->fault_code = g_isolation_handle.status.fault_code;
    
    /* Additional status fields can be filled here */
    status->door_open = false;
    status->seatbelt = false;
    
    return AKS_OK;
}

/**
 * @brief Get isolation measurements
 */
aks_result_t aks_isolation_get_measurements(aks_isolation_measurements_t* measurements)
{
    if (!g_isolation_handle.initialized || measurements == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *measurements = g_isolation_handle.status.measurements;
    return AKS_OK;
}

/**
 * @brief Get isolation status level
 */
aks_isolation_status_level_t aks_isolation_get_status_level(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ISOLATION_STATUS_UNKNOWN;
    }
    
    return g_isolation_handle.status.status_level;
}

/**
 * @brief Check if isolation fault is present
 */
bool aks_isolation_is_fault_detected(void)
{
    if (!g_isolation_handle.initialized) {
        return true; // Safe default
    }
    
    return g_isolation_handle.status.fault_detected;
}

/**
 * @brief Clear isolation faults
 */
aks_result_t aks_isolation_clear_faults(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_isolation_handle.status.fault_detected = false;
    g_isolation_handle.status.fault_code = 0;
    g_isolation_handle.status.measurements.fault_count = 0;
    
    return AKS_OK;
}

/**
 * @brief Perform isolation calibration
 */
aks_result_t aks_isolation_calibrate(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Perform calibration measurements */
    uint16_t positive_cal, negative_cal;
    
    aks_result_t result = aks_isolation_read_adc(g_isolation_handle.adc_channel_positive, &positive_cal);
    if (result != AKS_OK) {
        return result;
    }
    
    result = aks_isolation_read_adc(g_isolation_handle.adc_channel_negative, &negative_cal);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Calculate calibration offsets (assuming known reference) */
    g_isolation_handle.calibration_offset_positive = 0.0f; // Would be calculated from known reference
    g_isolation_handle.calibration_offset_negative = 0.0f;
    g_isolation_handle.calibration_gain_positive = 1.0f;
    g_isolation_handle.calibration_gain_negative = 1.0f;
    
    g_isolation_handle.status.calibration_needed = false;
    
    return AKS_OK;
}

/**
 * @brief Isolation monitoring task (periodic)
 */
aks_result_t aks_isolation_task(void)
{
    if (!g_isolation_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = HAL_GetTick();
    
    /* Check if it's time for measurement */
    if (current_time - g_isolation_handle.status.last_measurement_time >= 
        g_isolation_handle.config.measurement_period_ms) {
        
        if (g_isolation_handle.config.enable_auto_test) {
            aks_isolation_status_t status;
            aks_result_t result = aks_isolation_measure(&status);
            
            if (result != AKS_OK) {
                return result;
            }
            
            /* Send CAN status if enabled */
            if (g_isolation_handle.config.enable_can_reporting) {
                aks_isolation_send_can_status();
            }
        }
    }
    
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Perform isolation measurement
 */
static aks_result_t aks_isolation_perform_measurement(void)
{
    aks_result_t result;
    
    /* Measure positive rail */
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_MEASURING_POSITIVE;
    result = aks_isolation_read_adc(g_isolation_handle.adc_channel_positive,
                                   &g_isolation_handle.status.measurements.positive_adc_raw);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Measure negative rail */
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_MEASURING_NEGATIVE;
    result = aks_isolation_read_adc(g_isolation_handle.adc_channel_negative,
                                   &g_isolation_handle.status.measurements.negative_adc_raw);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Calculate resistances */
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_CALCULATING;
    
    g_isolation_handle.status.measurements.positive_resistance = 
        aks_isolation_calculate_resistance(g_isolation_handle.status.measurements.positive_adc_raw, true);
    
    g_isolation_handle.status.measurements.negative_resistance = 
        aks_isolation_calculate_resistance(g_isolation_handle.status.measurements.negative_adc_raw, false);
    
    /* Calculate total isolation resistance (parallel combination) */
    float pos_res = g_isolation_handle.status.measurements.positive_resistance;
    float neg_res = g_isolation_handle.status.measurements.negative_resistance;
    
    if (pos_res > 0.0f && neg_res > 0.0f) {
        g_isolation_handle.status.measurements.total_resistance = 
            (pos_res * neg_res) / (pos_res + neg_res);
    } else {
        g_isolation_handle.status.measurements.total_resistance = 0.0f;
    }
    
    /* Apply low-pass filter */
    g_isolation_handle.status.measurements.filtered_resistance = 
        aks_isolation_apply_filter(g_isolation_handle.status.measurements.total_resistance,
                                  g_isolation_handle.status.measurements.filtered_resistance);
    
    /* Update counters */
    g_isolation_handle.status.measurements.measurement_count++;
    g_isolation_handle.status.last_measurement_time = HAL_GetTick();
    
    return AKS_OK;
}

/**
 * @brief Read ADC value
 */
static aks_result_t aks_isolation_read_adc(uint32_t channel, uint16_t* value)
{
    if (value == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
#ifdef USE_HAL_DRIVER
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    
    if (HAL_ADC_ConfigChannel(g_isolation_handle.hadc, &sConfig) != HAL_OK) {
        return AKS_ERROR;
    }
    
    if (HAL_ADC_Start(g_isolation_handle.hadc) != HAL_OK) {
        return AKS_ERROR;
    }
    
    if (HAL_ADC_PollForConversion(g_isolation_handle.hadc, 100) != HAL_OK) {
        return AKS_ERROR_TIMEOUT;
    }
    
    *value = HAL_ADC_GetValue(g_isolation_handle.hadc);
    
    HAL_ADC_Stop(g_isolation_handle.hadc);
    
    return AKS_OK;
#else
    /* Software simulation for testing */
    *value = 2048; // Mid-scale value
    return AKS_OK;
#endif
}

/**
 * @brief Calculate resistance from ADC value
 */
static float aks_isolation_calculate_resistance(uint16_t adc_value, bool is_positive)
{
    /* Convert ADC value to voltage */
    float voltage = ((float)adc_value / AKS_ISOLATION_ADC_RESOLUTION) * AKS_ISOLATION_VREF;
    
    /* Apply calibration */
    float calibration_offset = is_positive ? g_isolation_handle.calibration_offset_positive :
                                            g_isolation_handle.calibration_offset_negative;
    float calibration_gain = is_positive ? g_isolation_handle.calibration_gain_positive :
                                          g_isolation_handle.calibration_gain_negative;
    
    voltage = (voltage + calibration_offset) * calibration_gain;
    
    /* Calculate resistance using voltage divider principle */
    /* R_isolation = R_known * (V_ref - V_measured) / V_measured */
    float known_resistance = 100000.0f; // 100kΩ known resistor
    
    if (voltage > 0.001f && voltage < (AKS_ISOLATION_VREF - 0.001f)) {
        float resistance = known_resistance * (AKS_ISOLATION_VREF - voltage) / voltage;
        return resistance;
    }
    
    return 0.0f; // Invalid measurement
}

/**
 * @brief Update isolation status
 */
static aks_result_t aks_isolation_update_status(void)
{
    float resistance = g_isolation_handle.status.measurements.filtered_resistance;
    
    /* Determine status level */
    if (resistance < g_isolation_handle.config.min_resistance) {
        g_isolation_handle.status.status_level = AKS_ISOLATION_STATUS_CRITICAL;
        g_isolation_handle.status.fault_detected = true;
        g_isolation_handle.status.fault_code = 0x0001; // Critical isolation fault
        g_isolation_handle.status.measurements.fault_count++;
        return aks_isolation_handle_fault(0x0001);
    } else if (resistance < g_isolation_handle.config.warning_resistance) {
        g_isolation_handle.status.status_level = AKS_ISOLATION_STATUS_FAULT;
        g_isolation_handle.status.fault_detected = true;
        g_isolation_handle.status.fault_code = 0x0002; // Isolation fault
        g_isolation_handle.status.measurements.fault_count++;
        return aks_isolation_handle_fault(0x0002);
    } else if (resistance < g_isolation_handle.config.good_resistance) {
        g_isolation_handle.status.status_level = AKS_ISOLATION_STATUS_WARNING;
        g_isolation_handle.status.fault_detected = false;
        g_isolation_handle.status.fault_code = 0x0003; // Isolation warning
    } else {
        g_isolation_handle.status.status_level = AKS_ISOLATION_STATUS_GOOD;
        g_isolation_handle.status.fault_detected = false;
        g_isolation_handle.status.fault_code = 0;
        g_isolation_handle.status.last_good_measurement_time = HAL_GetTick();
    }
    
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_IDLE;
    
    return AKS_OK;
}

/**
 * @brief Send CAN status message
 */
static aks_result_t aks_isolation_send_can_status(void)
{
    aks_can_frame_t frame;
    frame.id = AKS_ISOLATION_CAN_ID_STATUS;
    frame.length = 8;
    frame.is_extended = false;
    frame.is_remote = false;
    
    /* Pack status data */
    frame.data[0] = (uint8_t)g_isolation_handle.status.status_level;
    frame.data[1] = g_isolation_handle.status.fault_detected ? 1 : 0;
    
    uint32_t resistance_kohm = (uint32_t)(g_isolation_handle.status.measurements.filtered_resistance / 1000.0f);
    frame.data[2] = (resistance_kohm >> 8) & 0xFF;
    frame.data[3] = resistance_kohm & 0xFF;
    
    frame.data[4] = (g_isolation_handle.status.fault_code >> 8) & 0xFF;
    frame.data[5] = g_isolation_handle.status.fault_code & 0xFF;
    
    frame.data[6] = (g_isolation_handle.status.measurements.measurement_count >> 8) & 0xFF;
    frame.data[7] = g_isolation_handle.status.measurements.measurement_count & 0xFF;
    
    return aks_can_transmit(&frame, 100);
}

/**
 * @brief Handle isolation fault
 */
static aks_result_t aks_isolation_handle_fault(uint16_t fault_code)
{
    g_isolation_handle.status.state = AKS_ISOLATION_STATE_FAULT;
    
    /* Send fault notification via CAN */
    aks_can_frame_t frame;
    frame.id = AKS_ISOLATION_CAN_ID_FAULT;
    frame.length = 8;
    frame.is_extended = false;
    frame.is_remote = false;
    
    frame.data[0] = 0x01; // Fault notification
    frame.data[1] = (fault_code >> 8) & 0xFF;
    frame.data[2] = fault_code & 0xFF;
    
    uint32_t resistance = (uint32_t)g_isolation_handle.status.measurements.filtered_resistance;
    frame.data[3] = (resistance >> 24) & 0xFF;
    frame.data[4] = (resistance >> 16) & 0xFF;
    frame.data[5] = (resistance >> 8) & 0xFF;
    frame.data[6] = resistance & 0xFF;
    
    frame.data[7] = 0x00;
    
    aks_can_transmit(&frame, 100);
    
    return AKS_ERROR;
}

/**
 * @brief Apply low-pass filter
 */
static float aks_isolation_apply_filter(float new_value, float old_value)
{
    return g_isolation_handle.config.filter_alpha * new_value + 
           (1.0f - g_isolation_handle.config.filter_alpha) * old_value;
}
