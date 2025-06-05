/**
 * @file aks_adc.c
 * @brief ADC sensor reading implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include <string.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* ADC Configuration */
#define AKS_ADC_RESOLUTION          4096        // 12-bit ADC
#define AKS_ADC_VREF                3.3f        // Reference voltage
#define AKS_ADC_MAX_CHANNELS        16          // Maximum ADC channels
#define AKS_ADC_FILTER_ALPHA        0.1f        // Low-pass filter coefficient
#define AKS_ADC_CALIBRATION_SAMPLES 100         // Calibration sample count

/* ADC Channel Configuration */
typedef struct {
    uint32_t channel;               /**< ADC channel number */
    float scale_factor;             /**< Scaling factor */
    float offset;                   /**< Offset value */
    float min_value;                /**< Minimum expected value */
    float max_value;                /**< Maximum expected value */
    bool enabled;                   /**< Channel enabled flag */
    const char* name;               /**< Channel name */
} aks_adc_channel_config_t;

/* ADC Channel Data */
typedef struct {
    uint16_t raw_value;             /**< Raw ADC value */
    float voltage;                  /**< Converted voltage */
    float scaled_value;             /**< Scaled physical value */
    float filtered_value;           /**< Filtered value */
    uint32_t sample_count;          /**< Total samples */
    bool fault_detected;            /**< Fault detection flag */
} aks_adc_channel_data_t;

/* ADC Handle */
typedef struct {
    bool initialized;
    ADC_HandleTypeDef* hadc;
    aks_adc_channel_config_t channels[AKS_ADC_MAX_CHANNELS];
    aks_adc_channel_data_t data[AKS_ADC_MAX_CHANNELS];
    uint8_t active_channels;
    float filter_alpha;
    bool calibration_mode;
} aks_adc_handle_t;

/* Private variables */
static aks_adc_handle_t g_adc_handle;

/* Default channel configurations */
static const aks_adc_channel_config_t g_default_channels[8] = {
    {ADC_CHANNEL_0, 16.5f, 0.0f, 0.0f, 80.0f, true, "Battery Voltage"},      // 0-80V range
    {ADC_CHANNEL_1, 100.0f, -50.0f, -50.0f, 50.0f, true, "Battery Current"}, // ±50A range
    {ADC_CHANNEL_2, 50.0f, -40.0f, -40.0f, 85.0f, true, "Battery Temp"},     // -40 to 85°C
    {ADC_CHANNEL_3, 50.0f, -40.0f, -40.0f, 85.0f, true, "Motor Temp"},       // -40 to 85°C
    {ADC_CHANNEL_4, 1000.0f, 0.0f, 0.0f, 2000.0f, true, "Isolation Pos"},    // 0-2MΩ
    {ADC_CHANNEL_5, 1000.0f, 0.0f, 0.0f, 2000.0f, true, "Isolation Neg"},    // 0-2MΩ
    {ADC_CHANNEL_6, 1.0f, 0.0f, 0.0f, 3.3f, true, "Aux Voltage"},            // 0-3.3V
    {ADC_CHANNEL_7, 1.0f, 0.0f, 0.0f, 3.3f, true, "Spare"}                   // 0-3.3V
};

/* Private function prototypes */
static aks_result_t aks_adc_read_channel(uint8_t channel_index, uint16_t* raw_value);
static float aks_adc_convert_to_voltage(uint16_t raw_value);
static float aks_adc_apply_scaling(uint8_t channel_index, float voltage);
static float aks_adc_apply_filter(float new_value, float old_value, float alpha);
static aks_result_t aks_adc_check_limits(uint8_t channel_index);

/**
 * @brief Initialize ADC system
 */
aks_result_t aks_adc_init(ADC_HandleTypeDef* hadc)
{
    if (g_adc_handle.initialized) {
        return AKS_ERROR_BUSY;
    }
    
    if (hadc == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_adc_handle.hadc = hadc;
    g_adc_handle.filter_alpha = AKS_ADC_FILTER_ALPHA;
    g_adc_handle.calibration_mode = false;
    
    /* Initialize channels with default configuration */
    for (int i = 0; i < 8; i++) {
        g_adc_handle.channels[i] = g_default_channels[i];
        memset(&g_adc_handle.data[i], 0, sizeof(aks_adc_channel_data_t));
        if (g_adc_handle.channels[i].enabled) {
            g_adc_handle.active_channels++;
        }
    }
    
    /* Disable remaining channels */
    for (int i = 8; i < AKS_ADC_MAX_CHANNELS; i++) {
        g_adc_handle.channels[i].enabled = false;
    }
    
    g_adc_handle.initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize ADC system
 */
aks_result_t aks_adc_deinit(void)
{
    if (!g_adc_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    memset(&g_adc_handle, 0, sizeof(aks_adc_handle_t));
    
    return AKS_OK;
}

/**
 * @brief Read single ADC channel
 */
aks_result_t aks_adc_read_single(uint8_t channel_index, float* value)
{
    if (!g_adc_handle.initialized || channel_index >= AKS_ADC_MAX_CHANNELS || value == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (!g_adc_handle.channels[channel_index].enabled) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
    uint16_t raw_value;
    aks_result_t result = aks_adc_read_channel(channel_index, &raw_value);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Convert and scale */
    float voltage = aks_adc_convert_to_voltage(raw_value);
    float scaled = aks_adc_apply_scaling(channel_index, voltage);
    
    /* Apply filter */
    float filtered = aks_adc_apply_filter(scaled, 
                                         g_adc_handle.data[channel_index].filtered_value,
                                         g_adc_handle.filter_alpha);
    
    /* Update data */
    g_adc_handle.data[channel_index].raw_value = raw_value;
    g_adc_handle.data[channel_index].voltage = voltage;
    g_adc_handle.data[channel_index].scaled_value = scaled;
    g_adc_handle.data[channel_index].filtered_value = filtered;
    g_adc_handle.data[channel_index].sample_count++;
    
    /* Check limits */
    aks_adc_check_limits(channel_index);
    
    *value = filtered;
    
    return AKS_OK;
}

/**
 * @brief Read all enabled ADC channels
 */
aks_result_t aks_adc_read_all(void)
{
    if (!g_adc_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    for (int i = 0; i < AKS_ADC_MAX_CHANNELS; i++) {
        if (g_adc_handle.channels[i].enabled) {
            float value;
            aks_adc_read_single(i, &value);
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Get channel data
 */
aks_result_t aks_adc_get_channel_data(uint8_t channel_index, aks_adc_channel_data_t* data)
{
    if (!g_adc_handle.initialized || channel_index >= AKS_ADC_MAX_CHANNELS || data == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *data = g_adc_handle.data[channel_index];
    
    return AKS_OK;
}

/**
 * @brief Configure ADC channel
 */
aks_result_t aks_adc_configure_channel(uint8_t channel_index, const aks_adc_channel_config_t* config)
{
    if (!g_adc_handle.initialized || channel_index >= AKS_ADC_MAX_CHANNELS || config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_adc_handle.channels[channel_index] = *config;
    
    return AKS_OK;
}

/**
 * @brief Calibrate ADC channel
 */
aks_result_t aks_adc_calibrate_channel(uint8_t channel_index, float reference_value)
{
    if (!g_adc_handle.initialized || channel_index >= AKS_ADC_MAX_CHANNELS) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (!g_adc_handle.channels[channel_index].enabled) {
        return AKS_ERROR_UNSUPPORTED;
    }
    
    /* Take multiple samples for calibration */
    float sum = 0.0f;
    uint16_t valid_samples = 0;
    
    for (int i = 0; i < AKS_ADC_CALIBRATION_SAMPLES; i++) {
        uint16_t raw_value;
        if (aks_adc_read_channel(channel_index, &raw_value) == AKS_OK) {
            float voltage = aks_adc_convert_to_voltage(raw_value);
            sum += voltage;
            valid_samples++;
        }
    }
    
    if (valid_samples < AKS_ADC_CALIBRATION_SAMPLES / 2) {
        return AKS_ERROR;
    }
    
    float average_voltage = sum / valid_samples;
    
    /* Calculate new offset */
    float expected_voltage = reference_value / g_adc_handle.channels[channel_index].scale_factor;
    g_adc_handle.channels[channel_index].offset = expected_voltage - average_voltage;
    
    return AKS_OK;
}

/**
 * @brief Set filter coefficient
 */
aks_result_t aks_adc_set_filter_alpha(float alpha)
{
    if (!g_adc_handle.initialized || alpha < 0.0f || alpha > 1.0f) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_adc_handle.filter_alpha = alpha;
    
    return AKS_OK;
}

/**
 * @brief Get battery voltage
 */
float aks_adc_get_battery_voltage(void)
{
    if (!g_adc_handle.initialized) {
        return 0.0f;
    }
    
    return g_adc_handle.data[0].filtered_value; // Channel 0 is battery voltage
}

/**
 * @brief Get battery current
 */
float aks_adc_get_battery_current(void)
{
    if (!g_adc_handle.initialized) {
        return 0.0f;
    }
    
    return g_adc_handle.data[1].filtered_value; // Channel 1 is battery current
}

/**
 * @brief Get battery temperature
 */
float aks_adc_get_battery_temperature(void)
{
    if (!g_adc_handle.initialized) {
        return 0.0f;
    }
    
    return g_adc_handle.data[2].filtered_value; // Channel 2 is battery temperature
}

/**
 * @brief Get motor temperature
 */
float aks_adc_get_motor_temperature(void)
{
    if (!g_adc_handle.initialized) {
        return 0.0f;
    }
    
    return g_adc_handle.data[3].filtered_value; // Channel 3 is motor temperature
}

/* Private Functions */

/**
 * @brief Read ADC channel
 */
static aks_result_t aks_adc_read_channel(uint8_t channel_index, uint16_t* raw_value)
{
    if (raw_value == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
#ifdef USE_HAL_DRIVER
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = g_adc_handle.channels[channel_index].channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    
    if (HAL_ADC_ConfigChannel(g_adc_handle.hadc, &sConfig) != HAL_OK) {
        return AKS_ERROR;
    }
    
    if (HAL_ADC_Start(g_adc_handle.hadc) != HAL_OK) {
        return AKS_ERROR;
    }
    
    if (HAL_ADC_PollForConversion(g_adc_handle.hadc, 100) != HAL_OK) {
        return AKS_ERROR_TIMEOUT;
    }
    
    *raw_value = HAL_ADC_GetValue(g_adc_handle.hadc);
    
    HAL_ADC_Stop(g_adc_handle.hadc);
    
    return AKS_OK;
#else
    /* Software simulation for testing */
    *raw_value = 2048; // Mid-scale value
    return AKS_OK;
#endif
}

/**
 * @brief Convert raw ADC value to voltage
 */
static float aks_adc_convert_to_voltage(uint16_t raw_value)
{
    return ((float)raw_value / AKS_ADC_RESOLUTION) * AKS_ADC_VREF;
}

/**
 * @brief Apply scaling to voltage
 */
static float aks_adc_apply_scaling(uint8_t channel_index, float voltage)
{
    float adjusted_voltage = voltage + g_adc_handle.channels[channel_index].offset;
    return adjusted_voltage * g_adc_handle.channels[channel_index].scale_factor;
}

/**
 * @brief Apply low-pass filter
 */
static float aks_adc_apply_filter(float new_value, float old_value, float alpha)
{
    return alpha * new_value + (1.0f - alpha) * old_value;
}

/**
 * @brief Check channel limits
 */
static aks_result_t aks_adc_check_limits(uint8_t channel_index)
{
    float value = g_adc_handle.data[channel_index].scaled_value;
    float min_val = g_adc_handle.channels[channel_index].min_value;
    float max_val = g_adc_handle.channels[channel_index].max_value;
    
    if (value < min_val || value > max_val) {
        g_adc_handle.data[channel_index].fault_detected = true;
        return AKS_ERROR;
    }
    
    g_adc_handle.data[channel_index].fault_detected = false;
    return AKS_OK;
}
