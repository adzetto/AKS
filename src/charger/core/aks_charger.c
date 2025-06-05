/**
 * @file aks_charger.c
 * @brief Onboard charger core implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include "aks_safety.h"
#include <string.h>
#include <math.h>

/* Charger Configuration */
#define AKS_CHARGER_MAX_POWER           3300.0f     // 3.3kW max power
#define AKS_CHARGER_MAX_VOLTAGE         84.0f       // 84V max output voltage
#define AKS_CHARGER_MIN_VOLTAGE         60.0f       // 60V min output voltage
#define AKS_CHARGER_MAX_CURRENT         20.0f       // 20A max output current
#define AKS_CHARGER_AC_VOLTAGE_MIN      200.0f      // 200V AC minimum
#define AKS_CHARGER_AC_VOLTAGE_MAX      250.0f      // 250V AC maximum
#define AKS_CHARGER_EFFICIENCY          0.92f       // 92% efficiency
#define AKS_CHARGER_CONTROL_PERIOD      100         // 100ms control period
#define AKS_CHARGER_PRECHARGE_TIME      5000        // 5s precharge time

/* Charger States */
typedef enum {
    AKS_CHARGER_STATE_INIT = 0,
    AKS_CHARGER_STATE_STANDBY,
    AKS_CHARGER_STATE_PRECHARGE,
    AKS_CHARGER_STATE_CHARGING,
    AKS_CHARGER_STATE_BALANCING,
    AKS_CHARGER_STATE_COMPLETE,
    AKS_CHARGER_STATE_FAULT,
    AKS_CHARGER_STATE_EMERGENCY
} aks_charger_state_t;

/* Charging Modes */
typedef enum {
    AKS_CHARGER_MODE_CC = 0,        // Constant Current
    AKS_CHARGER_MODE_CV,            // Constant Voltage
    AKS_CHARGER_MODE_TRICKLE        // Trickle charge
} aks_charger_mode_t;

/* Charger Configuration Structure */
typedef struct {
    float max_power;                /**< Maximum charging power */
    float max_voltage;              /**< Maximum output voltage */
    float min_voltage;              /**< Minimum output voltage */
    float max_current;              /**< Maximum output current */
    float ac_voltage_min;           /**< Minimum AC input voltage */
    float ac_voltage_max;           /**< Maximum AC input voltage */
    float efficiency;               /**< Charger efficiency */
    uint32_t control_period;        /**< Control loop period */
    uint32_t precharge_time;        /**< Precharge time */
    bool enable_balancing;          /**< Enable balancing during charge */
    bool enable_temperature_control; /**< Enable temperature control */
} aks_charger_config_t;

/* Charger Status Structure */
typedef struct {
    aks_charger_state_t state;      /**< Current charger state */
    aks_charger_mode_t mode;        /**< Current charging mode */
    
    float ac_voltage;               /**< AC input voltage */
    float ac_current;               /**< AC input current */
    float ac_power;                 /**< AC input power */
    
    float dc_voltage;               /**< DC output voltage */
    float dc_current;               /**< DC output current */
    float dc_power;                 /**< DC output power */
    
    float target_voltage;           /**< Target charging voltage */
    float target_current;           /**< Target charging current */
    
    float charger_temperature;      /**< Charger temperature */
    float efficiency;               /**< Current efficiency */
    
    bool ac_connected;              /**< AC input connected */
    bool charging_enabled;          /**< Charging enabled */
    bool fault_active;              /**< Fault active */
    
    uint32_t charging_time;         /**< Total charging time */
    float energy_delivered;         /**< Total energy delivered */
    
    uint32_t fault_flags;           /**< Charger fault flags */
    
} aks_charger_status_t;

/* PID Controller Structure */
typedef struct {
    float kp;                       /**< Proportional gain */
    float ki;                       /**< Integral gain */
    float kd;                       /**< Derivative gain */
    float integral;                 /**< Integral accumulator */
    float previous_error;           /**< Previous error */
    float output_min;               /**< Minimum output */
    float output_max;               /**< Maximum output */
} aks_charger_pid_t;

/* Charger Handle Structure */
typedef struct {
    bool initialized;               /**< Initialization flag */
    aks_charger_config_t config;    /**< Configuration */
    aks_charger_status_t status;    /**< Current status */
    
    aks_charger_pid_t voltage_pid;  /**< Voltage control PID */
    aks_charger_pid_t current_pid;  /**< Current control PID */
    
    uint32_t last_control_time;     /**< Last control update time */
    uint32_t charging_start_time;   /**< Charging start time */
    uint32_t precharge_start_time;  /**< Precharge start time */
    
    float pwm_duty_cycle;           /**< PWM duty cycle */
    bool contactors_closed;         /**< Charger contactors closed */
    bool precharge_complete;        /**< Precharge complete */
    
} aks_charger_handle_t;

/* Global Variables */
static aks_charger_handle_t g_charger_handle = {0};

/* Private Function Prototypes */
static aks_result_t aks_charger_hardware_init(void);
static aks_result_t aks_charger_read_measurements(void);
static aks_result_t aks_charger_control_loop(void);
static aks_result_t aks_charger_set_output(float voltage, float current);
static aks_result_t aks_charger_control_contactors(bool close);
static aks_result_t aks_charger_check_faults(void);
static aks_result_t aks_charger_handle_fault(void);
static float aks_charger_pid_calculate(aks_charger_pid_t* pid, float setpoint, float feedback);
static aks_result_t aks_charger_pid_init(aks_charger_pid_t* pid, float kp, float ki, float kd);

/**
 * @brief Initialize charger system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_charger_init(void)
{
    if (g_charger_handle.initialized) {
        return AKS_ERROR_ALREADY_INITIALIZED;
    }
    
    /* Initialize configuration with default values */
    g_charger_handle.config.max_power = AKS_CHARGER_MAX_POWER;
    g_charger_handle.config.max_voltage = AKS_CHARGER_MAX_VOLTAGE;
    g_charger_handle.config.min_voltage = AKS_CHARGER_MIN_VOLTAGE;
    g_charger_handle.config.max_current = AKS_CHARGER_MAX_CURRENT;
    g_charger_handle.config.ac_voltage_min = AKS_CHARGER_AC_VOLTAGE_MIN;
    g_charger_handle.config.ac_voltage_max = AKS_CHARGER_AC_VOLTAGE_MAX;
    g_charger_handle.config.efficiency = AKS_CHARGER_EFFICIENCY;
    g_charger_handle.config.control_period = AKS_CHARGER_CONTROL_PERIOD;
    g_charger_handle.config.precharge_time = AKS_CHARGER_PRECHARGE_TIME;
    g_charger_handle.config.enable_balancing = true;
    g_charger_handle.config.enable_temperature_control = true;
    
    /* Initialize status */
    g_charger_handle.status.state = AKS_CHARGER_STATE_INIT;
    g_charger_handle.status.mode = AKS_CHARGER_MODE_CC;
    g_charger_handle.status.ac_connected = false;
    g_charger_handle.status.charging_enabled = false;
    g_charger_handle.status.fault_active = false;
    g_charger_handle.status.charging_time = 0;
    g_charger_handle.status.energy_delivered = 0.0f;
    g_charger_handle.status.fault_flags = 0;
    
    /* Initialize PID controllers */
    aks_charger_pid_init(&g_charger_handle.voltage_pid, 0.5f, 0.1f, 0.01f);
    aks_charger_pid_init(&g_charger_handle.current_pid, 1.0f, 0.2f, 0.02f);
    
    /* Initialize control variables */
    g_charger_handle.pwm_duty_cycle = 0.0f;
    g_charger_handle.contactors_closed = false;
    g_charger_handle.precharge_complete = false;
    
    /* Initialize hardware */
    aks_result_t result = aks_charger_hardware_init();
    if (result != AKS_OK) {
        return result;
    }
    
    g_charger_handle.initialized = true;
    g_charger_handle.status.state = AKS_CHARGER_STATE_STANDBY;
    g_charger_handle.last_control_time = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Deinitialize charger system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_charger_deinit(void)
{
    if (!g_charger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Stop charging */
    aks_charger_stop();
    
    /* Open contactors */
    aks_charger_control_contactors(false);
    
    /* Reset state */
    g_charger_handle.initialized = false;
    g_charger_handle.status.state = AKS_CHARGER_STATE_INIT;
    
    return AKS_OK;
}

/**
 * @brief Start charging process
 * @param target_voltage Target charging voltage
 * @param target_current Target charging current
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_charger_start(float target_voltage, float target_current)
{
    if (!g_charger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_charger_handle.status.state == AKS_CHARGER_STATE_FAULT) {
        return AKS_ERROR_INVALID_STATE;
    }
    
    /* Validate parameters */
    if (target_voltage > g_charger_handle.config.max_voltage ||
        target_voltage < g_charger_handle.config.min_voltage ||
        target_current > g_charger_handle.config.max_current ||
        target_current <= 0.0f) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Check AC connection */
    if (!g_charger_handle.status.ac_connected) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Set targets */
    g_charger_handle.status.target_voltage = target_voltage;
    g_charger_handle.status.target_current = target_current;
    
    /* Start precharge sequence */
    g_charger_handle.status.state = AKS_CHARGER_STATE_PRECHARGE;
    g_charger_handle.status.charging_enabled = true;
    g_charger_handle.precharge_start_time = aks_core_get_tick();
    g_charger_handle.charging_start_time = aks_core_get_tick();
    
    /* Reset energy counter */
    g_charger_handle.status.energy_delivered = 0.0f;
    
    return AKS_OK;
}

/**
 * @brief Stop charging process
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_charger_stop(void)
{
    if (!g_charger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Disable charging */
    g_charger_handle.status.charging_enabled = false;
    
    /* Set output to zero */
    aks_charger_set_output(0.0f, 0.0f);
    
    /* Open contactors */
    aks_charger_control_contactors(false);
    
    /* Reset state */
    g_charger_handle.status.state = AKS_CHARGER_STATE_STANDBY;
    g_charger_handle.precharge_complete = false;
    
    return AKS_OK;
}

/**
 * @brief Charger control task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_charger_task(void)
{
    if (!g_charger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_core_get_tick();
    
    /* Check if it's time for control update */
    if ((current_time - g_charger_handle.last_control_time) >= g_charger_handle.config.control_period) {
        g_charger_handle.last_control_time = current_time;
        
        /* Read measurements */
        aks_charger_read_measurements();
        
        /* Check for faults */
        aks_charger_check_faults();
        
        /* State machine */
        switch (g_charger_handle.status.state) {
            case AKS_CHARGER_STATE_STANDBY:
                /* Check for AC connection */
                if (g_charger_handle.status.ac_voltage > g_charger_handle.config.ac_voltage_min) {
                    g_charger_handle.status.ac_connected = true;
                } else {
                    g_charger_handle.status.ac_connected = false;
                }
                break;
                
            case AKS_CHARGER_STATE_PRECHARGE:
                /* Precharge sequence */
                if ((current_time - g_charger_handle.precharge_start_time) >= g_charger_handle.config.precharge_time) {
                    /* Precharge complete, close main contactors */
                    aks_charger_control_contactors(true);
                    g_charger_handle.precharge_complete = true;
                    g_charger_handle.status.state = AKS_CHARGER_STATE_CHARGING;
                    g_charger_handle.status.mode = AKS_CHARGER_MODE_CC; // Start with constant current
                }
                break;
                
            case AKS_CHARGER_STATE_CHARGING:
                /* Charging control loop */
                aks_charger_control_loop();
                
                /* Check for mode transitions */
                if (g_charger_handle.status.dc_voltage >= (g_charger_handle.status.target_voltage * 0.95f)) {
                    /* Switch to constant voltage mode */
                    g_charger_handle.status.mode = AKS_CHARGER_MODE_CV;
                }
                
                /* Check for charge completion */
                if (g_charger_handle.status.dc_current < 0.5f && 
                    g_charger_handle.status.dc_voltage >= (g_charger_handle.status.target_voltage * 0.98f)) {
                    g_charger_handle.status.state = AKS_CHARGER_STATE_COMPLETE;
                }
                
                /* Update charging time and energy */
                g_charger_handle.status.charging_time = current_time - g_charger_handle.charging_start_time;
                g_charger_handle.status.energy_delivered += 
                    (g_charger_handle.status.dc_power * g_charger_handle.config.control_period) / 3600000.0f; // Wh
                break;
                
            case AKS_CHARGER_STATE_COMPLETE:
                /* Charging complete - trickle charge */
                g_charger_handle.status.mode = AKS_CHARGER_MODE_TRICKLE;
                aks_charger_set_output(g_charger_handle.status.target_voltage, 0.1f); // 100mA trickle
                break;
                
            case AKS_CHARGER_STATE_FAULT:
                /* Fault state - disable output */
                aks_charger_set_output(0.0f, 0.0f);
                aks_charger_handle_fault();
                break;
                
            default:
                break;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Get charger status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_charger_get_status(aks_charger_status_t* status)
{
    if (!g_charger_handle.initialized || status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *status = g_charger_handle.status;
    return AKS_OK;
}

/**
 * @brief Emergency stop charger
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_charger_emergency_stop(void)
{
    if (!g_charger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Immediately disable output */
    aks_charger_set_output(0.0f, 0.0f);
    
    /* Open contactors */
    aks_charger_control_contactors(false);
    
    /* Set emergency state */
    g_charger_handle.status.state = AKS_CHARGER_STATE_EMERGENCY;
    g_charger_handle.status.charging_enabled = false;
    
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Initialize charger hardware
 */
static aks_result_t aks_charger_hardware_init(void)
{
    aks_result_t result = AKS_OK;
    
    /* Initialize PWM for power control (switching frequency ~100kHz) */
    aks_pwm_config_t pwm_config = {
        .frequency = 100000,            /* 100 kHz switching frequency */
        .duty_cycle = 0.0f,             /* Start with 0% duty cycle */
        .polarity = AKS_PWM_POLARITY_HIGH, /* Active high */
        .dead_time_ns = 100,            /* 100ns dead time for safety */
        .enable_complementary = true    /* Enable complementary output */
    };
    
    result = aks_pwm_init(AKS_PWM_CHARGER_PRIMARY, &pwm_config);
    if (result != AKS_OK) {
        aks_logger_error("Failed to initialize charger primary PWM: %d", result);
        return result;
    }
    
    result = aks_pwm_init(AKS_PWM_CHARGER_SECONDARY, &pwm_config);
    if (result != AKS_OK) {
        aks_logger_error("Failed to initialize charger secondary PWM: %d", result);
        return result;
    }
    
    /* Initialize ADC channels for measurements */
    aks_adc_channel_config_t adc_channels[] = {
        {AKS_ADC_CH_CHARGER_AC_VOLTAGE, AKS_ADC_SAMPLING_TIME_15_CYCLES, true},
        {AKS_ADC_CH_CHARGER_AC_CURRENT, AKS_ADC_SAMPLING_TIME_15_CYCLES, true},
        {AKS_ADC_CH_CHARGER_DC_VOLTAGE, AKS_ADC_SAMPLING_TIME_15_CYCLES, true},
        {AKS_ADC_CH_CHARGER_DC_CURRENT, AKS_ADC_SAMPLING_TIME_15_CYCLES, true},
        {AKS_ADC_CH_CHARGER_TEMPERATURE, AKS_ADC_SAMPLING_TIME_480_CYCLES, false}
    };
    
    for (uint8_t i = 0; i < 5; i++) {
        result = aks_adc_configure_channel(&adc_channels[i]);
        if (result != AKS_OK) {
            aks_logger_error("Failed to configure ADC channel %d: %d", adc_channels[i].channel, result);
            return result;
        }
    }
    
    /* Initialize GPIO for contactors and control signals */
    aks_gpio_config_t gpio_config = {
        .mode = AKS_GPIO_MODE_OUTPUT_PP,
        .pull = AKS_GPIO_PULL_NONE,
        .speed = AKS_GPIO_SPEED_LOW
    };
    
    /* Main charger contactors */
    result = aks_gpio_init(AKS_GPIO_CHARGER_MAIN_CONTACTOR, &gpio_config);
    if (result != AKS_OK) return result;
    
    result = aks_gpio_init(AKS_GPIO_CHARGER_PRECHARGE_CONTACTOR, &gpio_config);
    if (result != AKS_OK) return result;
    
    /* AC input monitoring and control */
    result = aks_gpio_init(AKS_GPIO_CHARGER_AC_ENABLE, &gpio_config);
    if (result != AKS_OK) return result;
    
    result = aks_gpio_init(AKS_GPIO_CHARGER_FAN_CONTROL, &gpio_config);
    if (result != AKS_OK) return result;
    
    /* Configure AC input monitoring GPIOs as inputs */
    gpio_config.mode = AKS_GPIO_MODE_INPUT;
    gpio_config.pull = AKS_GPIO_PULL_UP;
    
    result = aks_gpio_init(AKS_GPIO_CHARGER_AC_DETECT, &gpio_config);
    if (result != AKS_OK) return result;
    
    result = aks_gpio_init(AKS_GPIO_CHARGER_FAULT_INPUT, &gpio_config);
    if (result != AKS_OK) return result;
    
    /* Initialize all outputs to safe state */
    aks_gpio_write(AKS_GPIO_CHARGER_MAIN_CONTACTOR, false);     /* Open main contactor */
    aks_gpio_write(AKS_GPIO_CHARGER_PRECHARGE_CONTACTOR, false); /* Open precharge */
    aks_gpio_write(AKS_GPIO_CHARGER_AC_ENABLE, false);          /* Disable AC input */
    aks_gpio_write(AKS_GPIO_CHARGER_FAN_CONTROL, false);        /* Fan off */
    
    /* Initialize timer for control loop */
    aks_timer_config_t timer_config = {
        .period_us = 10000,             /* 10ms control period (100Hz) */
        .auto_reload = true,
        .enable_interrupt = true
    };
    
    result = aks_timer_init(AKS_TIMER_CHARGER_CONTROL, &timer_config);
    if (result != AKS_OK) {
        aks_logger_error("Failed to initialize charger control timer: %d", result);
        return result;
    }
    
    /* Initialize isolation monitoring for charger safety */
    result = aks_gpio_init(AKS_GPIO_CHARGER_ISOLATION_CHECK, &gpio_config);
    if (result != AKS_OK) return result;
    
    /* Configure emergency stop input */
    gpio_config.pull = AKS_GPIO_PULL_UP;
    result = aks_gpio_init(AKS_GPIO_CHARGER_EMERGENCY_STOP, &gpio_config);
    if (result != AKS_OK) return result;
    
    /* Enable interrupt for emergency stop */
    result = aks_gpio_configure_interrupt(AKS_GPIO_CHARGER_EMERGENCY_STOP, 
                                         AKS_GPIO_INTERRUPT_FALLING_EDGE,
                                         aks_charger_emergency_stop);
    if (result != AKS_OK) {
        aks_logger_warning("Failed to configure emergency stop interrupt: %d", result);
    }
    
    /* Initialize status LEDs */
    result = aks_gpio_init(AKS_GPIO_CHARGER_STATUS_LED_GREEN, &gpio_config);
    if (result != AKS_OK) return result;
    
    result = aks_gpio_init(AKS_GPIO_CHARGER_STATUS_LED_RED, &gpio_config);
    if (result != AKS_OK) return result;
    
    /* Start with red LED on (not charging) */
    aks_gpio_write(AKS_GPIO_CHARGER_STATUS_LED_GREEN, false);
    aks_gpio_write(AKS_GPIO_CHARGER_STATUS_LED_RED, true);
    
    aks_logger_info("Charger hardware initialized successfully");
    
    return AKS_OK;
}

/**
 * @brief Read charger measurements
 */
static aks_result_t aks_charger_read_measurements(void)
{
    /* Read AC input measurements */
    g_charger_handle.status.ac_voltage = aks_adc_read_voltage(AKS_ADC_CHANNEL_AC_VOLTAGE);
    g_charger_handle.status.ac_current = aks_current_read(AKS_CURRENT_SENSOR_CHARGER_AC);
    g_charger_handle.status.ac_power = g_charger_handle.status.ac_voltage * g_charger_handle.status.ac_current;
    
    /* Read DC output measurements */
    g_charger_handle.status.dc_voltage = aks_voltage_read(AKS_VOLTAGE_SENSOR_CHARGER_OUTPUT);
    g_charger_handle.status.dc_current = aks_current_read(AKS_CURRENT_SENSOR_CHARGER);
    g_charger_handle.status.dc_power = g_charger_handle.status.dc_voltage * g_charger_handle.status.dc_current;
    
    /* Read charger temperature */
    g_charger_handle.status.charger_temperature = aks_temperature_read(AKS_TEMP_SENSOR_CHARGER);
    
    /* Calculate efficiency */
    if (g_charger_handle.status.ac_power > 0.1f) {
        g_charger_handle.status.efficiency = g_charger_handle.status.dc_power / g_charger_handle.status.ac_power;
    } else {
        g_charger_handle.status.efficiency = 0.0f;
    }
    
    return AKS_OK;
}

/**
 * @brief Charger control loop
 */
static aks_result_t aks_charger_control_loop(void)
{
    float control_output = 0.0f;
    
    switch (g_charger_handle.status.mode) {
        case AKS_CHARGER_MODE_CC:
            /* Constant current mode */
            control_output = aks_charger_pid_calculate(&g_charger_handle.current_pid,
                                                      g_charger_handle.status.target_current,
                                                      g_charger_handle.status.dc_current);
            break;
            
        case AKS_CHARGER_MODE_CV:
            /* Constant voltage mode */
            control_output = aks_charger_pid_calculate(&g_charger_handle.voltage_pid,
                                                      g_charger_handle.status.target_voltage,
                                                      g_charger_handle.status.dc_voltage);
            break;
            
        case AKS_CHARGER_MODE_TRICKLE:
            /* Trickle charge mode */
            control_output = 5.0f; // Low duty cycle
            break;
            
        default:
            control_output = 0.0f;
            break;
    }
    
    /* Limit control output */
    if (control_output > 95.0f) control_output = 95.0f;
    if (control_output < 0.0f) control_output = 0.0f;
    
    /* Set PWM duty cycle */
    g_charger_handle.pwm_duty_cycle = control_output;
    
    /* Apply control output to hardware */
    // Set PWM duty cycle for power control
    
    return AKS_OK;
}

/**
 * @brief Set charger output
 */
static aks_result_t aks_charger_set_output(float voltage, float current)
{
    /* Set target values */
    g_charger_handle.status.target_voltage = voltage;
    g_charger_handle.status.target_current = current;
    
    /* If both are zero, disable output */
    if (voltage == 0.0f && current == 0.0f) {
        g_charger_handle.pwm_duty_cycle = 0.0f;
        // Disable PWM output
    }
    
    return AKS_OK;
}

/**
 * @brief Control charger contactors
 */
static aks_result_t aks_charger_control_contactors(bool close)
{
    if (close && !g_charger_handle.status.fault_active) {
        /* Close contactors */
        g_charger_handle.contactors_closed = true;
        // Control GPIO for contactors
    } else {
        /* Open contactors */
        g_charger_handle.contactors_closed = false;
        // Control GPIO for contactors
    }
    
    return AKS_OK;
}

/**
 * @brief Check for charger faults
 */
static aks_result_t aks_charger_check_faults(void)
{
    uint32_t fault_flags = 0;
    
    /* Check AC input voltage */
    if (g_charger_handle.status.ac_voltage < g_charger_handle.config.ac_voltage_min ||
        g_charger_handle.status.ac_voltage > g_charger_handle.config.ac_voltage_max) {
        fault_flags |= (1 << 0); // AC voltage fault
    }
    
    /* Check output voltage */
    if (g_charger_handle.status.dc_voltage > (g_charger_handle.config.max_voltage + 5.0f)) {
        fault_flags |= (1 << 1); // Output overvoltage
    }
    
    /* Check output current */
    if (g_charger_handle.status.dc_current > (g_charger_handle.config.max_current + 2.0f)) {
        fault_flags |= (1 << 2); // Output overcurrent
    }
    
    /* Check temperature */
    if (g_charger_handle.status.charger_temperature > 85.0f) {
        fault_flags |= (1 << 3); // Overtemperature
    }
    
    /* Check efficiency */
    if (g_charger_handle.status.efficiency < 0.7f && g_charger_handle.status.dc_power > 100.0f) {
        fault_flags |= (1 << 4); // Low efficiency
    }
    
    g_charger_handle.status.fault_flags = fault_flags;
    g_charger_handle.status.fault_active = (fault_flags != 0);
    
    if (g_charger_handle.status.fault_active) {
        g_charger_handle.status.state = AKS_CHARGER_STATE_FAULT;
    }
    
    return AKS_OK;
}

/**
 * @brief Handle charger fault
 */
static aks_result_t aks_charger_handle_fault(void)
{
    /* Log fault */
    aks_safety_log_fault(AKS_FAULT_SYSTEM_OVERLOAD, 
                         g_charger_handle.status.charger_temperature,
                         AKS_ACTION_DISABLE_CHARGER);
    
    /* Disable charging */
    g_charger_handle.status.charging_enabled = false;
    
    /* Open contactors */
    aks_charger_control_contactors(false);
    
    return AKS_OK;
}

/**
 * @brief Calculate PID output
 */
static float aks_charger_pid_calculate(aks_charger_pid_t* pid, float setpoint, float feedback)
{
    if (pid == NULL) {
        return 0.0f;
    }
    
    float error = setpoint - feedback;
    
    /* Proportional term */
    float p_term = pid->kp * error;
    
    /* Integral term */
    pid->integral += error;
    
    /* Anti-windup */
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;
    
    float i_term = pid->ki * pid->integral;
    
    /* Derivative term */
    float d_term = pid->kd * (error - pid->previous_error);
    pid->previous_error = error;
    
    /* Calculate output */
    float output = p_term + i_term + d_term;
    
    /* Clamp output */
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}

/**
 * @brief Initialize PID controller
 */
static aks_result_t aks_charger_pid_init(aks_charger_pid_t* pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_min = 0.0f;
    pid->output_max = 100.0f;
    
    return AKS_OK;
} 