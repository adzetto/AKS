/**
 * @file aks_math.c
 * @brief Mathematical utilities for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* Mathematical constants */
#ifndef M_PI
#define M_PI                3.14159265358979323846
#endif

#define AKS_MATH_E          2.71828182845904523536
#define AKS_MATH_SQRT2      1.41421356237309504880
#define AKS_MATH_SQRT3      1.73205080756887729353

/* Fixed-point arithmetic constants */
#define AKS_FIXED_POINT_SHIFT   16
#define AKS_FIXED_POINT_SCALE   (1 << AKS_FIXED_POINT_SHIFT)

/* Filter structures */
typedef struct {
    float alpha;            /**< Filter coefficient */
    float output;           /**< Last output value */
    bool initialized;       /**< Initialization flag */
} aks_lpf_t;

typedef struct {
    float a0, a1, a2;      /**< Denominator coefficients */
    float b0, b1, b2;      /**< Numerator coefficients */
    float x1, x2;          /**< Input history */
    float y1, y2;          /**< Output history */
    bool initialized;       /**< Initialization flag */
} aks_biquad_t;

typedef struct {
    float kp, ki, kd;      /**< PID gains */
    float integral;        /**< Integral accumulator */
    float last_error;     /**< Last error for derivative */
    float output_min;     /**< Minimum output */
    float output_max;     /**< Maximum output */
    bool initialized;     /**< Initialization flag */
} aks_pid_t;

/* Maximum number of filters */
#define AKS_MAX_FILTERS     16

/* Static filter instances */
static aks_lpf_t g_lpf_filters[AKS_MAX_FILTERS] = {0};
static aks_biquad_t g_biquad_filters[AKS_MAX_FILTERS] = {0};
static aks_pid_t g_pid_controllers[AKS_MAX_FILTERS] = {0};

/* Private function prototypes */
static uint16_t aks_math_get_free_filter_index(void* filters, size_t filter_size);
static bool aks_math_is_valid_filter_handle(uint16_t handle, void* filters, size_t filter_size);

/**
 * @brief Clamp value between min and max
 */
float aks_math_clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Linear interpolation
 */
float aks_math_lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

/**
 * @brief Map value from one range to another
 */
float aks_math_map(float value, float in_min, float in_max, float out_min, float out_max)
{
    float normalized = (value - in_min) / (in_max - in_min);
    return out_min + normalized * (out_max - out_min);
}

/**
 * @brief Fast square root using Newton-Raphson method
 */
float aks_math_sqrt_fast(float x)
{
    if (x <= 0.0f) return 0.0f;
    
    /* Initial guess using bit manipulation */
    union { float f; uint32_t i; } val = {x};
    val.i -= 1 << 23; /* Subtract 2^m. */
    val.i >>= 1; /* Divide by 2. */
    val.i += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */
    
    /* One iteration of Newton-Raphson */
    val.f = 0.5f * (val.f + x / val.f);
    
    return val.f;
}

/**
 * @brief Fast inverse square root (Quake algorithm)
 */
float aks_math_inv_sqrt_fast(float x)
{
    if (x <= 0.0f) return 0.0f;
    
    union { float f; uint32_t i; } val = {x};
    val.i = 0x5f3759df - (val.i >> 1); /* Magic number */
    val.f *= 1.5f - 0.5f * x * val.f * val.f; /* One Newton iteration */
    
    return val.f;
}

/**
 * @brief Calculate moving average
 */
float aks_math_moving_average(float new_value, float old_average, uint16_t count)
{
    if (count == 0) return new_value;
    return old_average + (new_value - old_average) / count;
}

/**
 * @brief Calculate RMS (Root Mean Square)
 */
float aks_math_rms(const float* values, uint16_t count)
{
    if (values == NULL || count == 0) return 0.0f;
    
    float sum_squares = 0.0f;
    for (uint16_t i = 0; i < count; i++) {
        sum_squares += values[i] * values[i];
    }
    
    return aks_math_sqrt_fast(sum_squares / count);
}

/**
 * @brief Convert degrees to radians
 */
float aks_math_deg_to_rad(float degrees)
{
    return degrees * (M_PI / 180.0f);
}

/**
 * @brief Convert radians to degrees
 */
float aks_math_rad_to_deg(float radians)
{
    return radians * (180.0f / M_PI);
}

/**
 * @brief Convert float to fixed-point
 */
int32_t aks_math_float_to_fixed(float value)
{
    return (int32_t)(value * AKS_FIXED_POINT_SCALE);
}

/**
 * @brief Convert fixed-point to float
 */
float aks_math_fixed_to_float(int32_t fixed_value)
{
    return (float)fixed_value / AKS_FIXED_POINT_SCALE;
}

/**
 * @brief Fixed-point multiplication
 */
int32_t aks_math_fixed_multiply(int32_t a, int32_t b)
{
    int64_t result = ((int64_t)a * b) >> AKS_FIXED_POINT_SHIFT;
    return (int32_t)result;
}

/**
 * @brief Fixed-point division
 */
int32_t aks_math_fixed_divide(int32_t a, int32_t b)
{
    if (b == 0) return 0;
    int64_t result = ((int64_t)a << AKS_FIXED_POINT_SHIFT) / b;
    return (int32_t)result;
}

/**
 * @brief Create low-pass filter
 */
aks_result_t aks_math_lpf_create(float cutoff_freq, float sample_freq, uint16_t* handle)
{
    if (handle == NULL || cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint16_t index = aks_math_get_free_filter_index(g_lpf_filters, sizeof(aks_lpf_t));
    if (index >= AKS_MAX_FILTERS) {
        return AKS_ERROR_NO_MEMORY;
    }
    
    aks_lpf_t* filter = &g_lpf_filters[index];
    
    /* Calculate filter coefficient */
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    float dt = 1.0f / sample_freq;
    filter->alpha = dt / (rc + dt);
    filter->output = 0.0f;
    filter->initialized = true;
    
    *handle = index;
    
    return AKS_OK;
}

/**
 * @brief Process low-pass filter
 */
float aks_math_lpf_process(uint16_t handle, float input)
{
    if (!aks_math_is_valid_filter_handle(handle, g_lpf_filters, sizeof(aks_lpf_t))) {
        return input;
    }
    
    aks_lpf_t* filter = &g_lpf_filters[handle];
    
    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;
    
    return filter->output;
}

/**
 * @brief Reset low-pass filter
 */
aks_result_t aks_math_lpf_reset(uint16_t handle, float initial_value)
{
    if (!aks_math_is_valid_filter_handle(handle, g_lpf_filters, sizeof(aks_lpf_t))) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_lpf_filters[handle].output = initial_value;
    
    return AKS_OK;
}

/**
 * @brief Create PID controller
 */
aks_result_t aks_math_pid_create(float kp, float ki, float kd, 
                                 float output_min, float output_max, uint16_t* handle)
{
    if (handle == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint16_t index = aks_math_get_free_filter_index(g_pid_controllers, sizeof(aks_pid_t));
    if (index >= AKS_MAX_FILTERS) {
        return AKS_ERROR_NO_MEMORY;
    }
    
    aks_pid_t* pid = &g_pid_controllers[index];
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->initialized = true;
    
    *handle = index;
    
    return AKS_OK;
}

/**
 * @brief Process PID controller
 */
float aks_math_pid_process(uint16_t handle, float setpoint, float process_value, float dt)
{
    if (!aks_math_is_valid_filter_handle(handle, g_pid_controllers, sizeof(aks_pid_t))) {
        return 0.0f;
    }
    
    aks_pid_t* pid = &g_pid_controllers[handle];
    
    float error = setpoint - process_value;
    
    /* Proportional term */
    float proportional = pid->kp * error;
    
    /* Integral term */
    pid->integral += error * dt;
    float integral = pid->ki * pid->integral;
    
    /* Derivative term */
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = pid->kd * (error - pid->last_error) / dt;
    }
    pid->last_error = error;
    
    /* Calculate output */
    float output = proportional + integral + derivative;
    
    /* Clamp output */
    output = aks_math_clamp(output, pid->output_min, pid->output_max);
    
    /* Anti-windup: limit integral if output is saturated */
    if ((output >= pid->output_max && error > 0.0f) || 
        (output <= pid->output_min && error < 0.0f)) {
        pid->integral -= error * dt; /* Undo integral accumulation */
    }
    
    return output;
}

/**
 * @brief Reset PID controller
 */
aks_result_t aks_math_pid_reset(uint16_t handle)
{
    if (!aks_math_is_valid_filter_handle(handle, g_pid_controllers, sizeof(aks_pid_t))) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_pid_t* pid = &g_pid_controllers[handle];
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    
    return AKS_OK;
}

/**
 * @brief Set PID gains
 */
aks_result_t aks_math_pid_set_gains(uint16_t handle, float kp, float ki, float kd)
{
    if (!aks_math_is_valid_filter_handle(handle, g_pid_controllers, sizeof(aks_pid_t))) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_pid_t* pid = &g_pid_controllers[handle];
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    return AKS_OK;
}

/**
 * @brief Set PID output limits
 */
aks_result_t aks_math_pid_set_limits(uint16_t handle, float output_min, float output_max)
{
    if (!aks_math_is_valid_filter_handle(handle, g_pid_controllers, sizeof(aks_pid_t))) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_pid_t* pid = &g_pid_controllers[handle];
    pid->output_min = output_min;
    pid->output_max = output_max;
    
    return AKS_OK;
}

/**
 * @brief Calculate distance between two points
 */
float aks_math_distance_2d(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return aks_math_sqrt_fast(dx * dx + dy * dy);
}

/**
 * @brief Calculate angle between two points
 */
float aks_math_angle_2d(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return atan2f(dy, dx);
}

/**
 * @brief Normalize angle to [-PI, PI] range
 */
float aks_math_normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * @brief Calculate median of array
 */
float aks_math_median(float* values, uint16_t count)
{
    if (values == NULL || count == 0) return 0.0f;
    
    /* Simple bubble sort for small arrays */
    for (uint16_t i = 0; i < count - 1; i++) {
        for (uint16_t j = 0; j < count - i - 1; j++) {
            if (values[j] > values[j + 1]) {
                float temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }
    
    if (count % 2 == 0) {
        return (values[count / 2 - 1] + values[count / 2]) / 2.0f;
    } else {
        return values[count / 2];
    }
}

/* Private function implementations */

/**
 * @brief Get free filter index
 */
static uint16_t aks_math_get_free_filter_index(void* filters, size_t filter_size)
{
    uint8_t* filter_array = (uint8_t*)filters;
    
    for (uint16_t i = 0; i < AKS_MAX_FILTERS; i++) {
        bool* initialized = (bool*)(filter_array + i * filter_size + filter_size - sizeof(bool));
        if (!(*initialized)) {
            return i;
        }
    }
    
    return AKS_MAX_FILTERS;
}

/**
 * @brief Validate filter handle
 */
static bool aks_math_is_valid_filter_handle(uint16_t handle, void* filters, size_t filter_size)
{
    if (handle >= AKS_MAX_FILTERS) {
        return false;
    }
    
    uint8_t* filter_array = (uint8_t*)filters;
    bool* initialized = (bool*)(filter_array + handle * filter_size + filter_size - sizeof(bool));
    
    return *initialized;
}
