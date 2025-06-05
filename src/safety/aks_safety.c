/**
 * @file aks_safety.c
 * @brief Safety monitoring and isolation detection implementation
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_safety.h"
#include "aks_core.h"

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Private variables */
static aks_safety_config_t safety_config;
static aks_fault_log_entry_t fault_log[32];
static uint8_t fault_log_count = 0;
static bool safety_initialized = false;
static bool monitoring_active = false;
static bool emergency_active = false;
static uint32_t last_monitoring_time = 0;

/* Private function prototypes */
static void safety_monitor_isolation(void);
static void safety_monitor_temperature(void);
static void safety_monitor_voltage(void);
static void safety_monitor_current(void);
static uint8_t find_fault_log_entry(aks_fault_code_t fault_code);

/**
 * @brief Initialize safety monitoring system
 * @param config Pointer to safety configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_init(const aks_safety_config_t* config)
{
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }

    /* Copy configuration */
    safety_config = *config;
    
    /* Reset fault log */
    fault_log_count = 0;
    
    /* Reset state */
    emergency_active = false;
    monitoring_active = false;
    last_monitoring_time = 0;
    
    safety_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize safety monitoring system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_deinit(void)
{
    if (!safety_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    monitoring_active = false;
    emergency_active = false;
    safety_initialized = false;
    
    return AKS_OK;
}

/**
 * @brief Start safety monitoring
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_start_monitoring(void)
{
    if (!safety_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    monitoring_active = true;
    last_monitoring_time = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Stop safety monitoring
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_stop_monitoring(void)
{
    monitoring_active = false;
    return AKS_OK;
}

/**
 * @brief Trigger emergency stop
 * @param reason Emergency stop reason
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_emergency_stop(aks_fault_code_t reason)
{
    if (!safety_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    emergency_active = true;
    
    /* Log the emergency stop */
    aks_safety_log_fault(reason, 0.0f, AKS_ACTION_EMERGENCY_STOP);
    
    /* Implement actual emergency stop procedures */
    
    /* 1. Disable motor output immediately */
    aks_result_t motor_result = aks_motor_emergency_stop();
    
    /* 2. Open safety contactors */
    aks_main_set_relay(0, false); // Motor relay
    aks_main_set_relay(1, false); // Charger relay
    
    /* 3. Trigger visual and audible alarms */
    // Set emergency LED
    // Activate buzzer
    
    /* 4. Notify telemetry system */
    aks_telemetry_data_t emergency_data = {
        .vehicle_speed = 0.0f,
        .motor_torque = 0.0f,
        .motor_speed = 0.0f,
        .battery_voltage = 0.0f,
        .battery_current = 0.0f,
        .battery_temperature = 0.0f,
        .battery_soc = 0.0f,
        .gps_latitude = 0.0,
        .gps_longitude = 0.0,
        .gps_speed = 0.0f,
        .isolation_resistance = 0.0f,
        .system_state = 0xFF, // Emergency state
        .fault_codes = (uint16_t)reason
    };
    
    /* Send emergency telemetry */
    aks_logger_telemetry(&emergency_data);
    
    /* 5. Set system to safe state */
    aks_main_emergency_stop(reason);
    
    return AKS_OK;
}

/**
 * @brief Clear emergency stop condition
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_clear_emergency_stop(void)
{
    if (!safety_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    emergency_active = false;
    return AKS_OK;
}

/**
 * @brief Check if system is in emergency state
 * @return true if in emergency state, false otherwise
 */
bool aks_safety_is_emergency_active(void)
{
    return emergency_active;
}

/**
 * @brief Log safety fault
 * @param fault_code Fault code
 * @param fault_value Associated measurement value
 * @param action_taken Action taken in response
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_log_fault(aks_fault_code_t fault_code,
                                  float fault_value,
                                  aks_safety_action_t action_taken)
{
    if (!safety_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Check if this fault already exists */
    uint8_t existing_index = find_fault_log_entry(fault_code);
    
    if (existing_index < fault_log_count) {
        /* Update existing entry */
        fault_log[existing_index].occurrence_count++;
        fault_log[existing_index].fault_value = fault_value;
        fault_log[existing_index].action_taken = action_taken;
        fault_log[existing_index].timestamp = aks_core_get_tick();
        fault_log[existing_index].resolved = false;
    } else {
        /* Add new entry if log not full */
        if (fault_log_count < safety_config.fault_log_size) {
            fault_log[fault_log_count].fault_code = fault_code;
            fault_log[fault_log_count].timestamp = aks_core_get_tick();
            fault_log[fault_log_count].action_taken = action_taken;
            fault_log[fault_log_count].fault_value = fault_value;
            fault_log[fault_log_count].occurrence_count = 1;
            fault_log[fault_log_count].resolved = false;
            fault_log_count++;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Safety monitoring task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_safety_task(void)
{
    if (!safety_initialized || !monitoring_active) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_core_get_tick();
    
    /* Check if it's time for monitoring */
    if ((current_time - last_monitoring_time) >= safety_config.monitoring_interval) {
        last_monitoring_time = current_time;
        
        /* Perform monitoring tasks */
        if (safety_config.isolation_monitoring_enabled) {
            safety_monitor_isolation();
        }
        
        if (safety_config.temperature_monitoring_enabled) {
            safety_monitor_temperature();
        }
        
        if (safety_config.voltage_monitoring_enabled) {
            safety_monitor_voltage();
        }
        
        if (safety_config.current_monitoring_enabled) {
            safety_monitor_current();
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Get fault code description string
 * @param fault_code Fault code
 * @return Pointer to fault description string
 */
const char* aks_safety_get_fault_description(aks_fault_code_t fault_code)
{
    switch (fault_code) {
        case AKS_FAULT_NONE:
            return "No fault";
        case AKS_FAULT_ISOLATION_LOW:
            return "Low isolation resistance";
        case AKS_FAULT_BATTERY_OVERVOLTAGE:
            return "Battery overvoltage";
        case AKS_FAULT_BATTERY_UNDERVOLTAGE:
            return "Battery undervoltage";
        case AKS_FAULT_BATTERY_OVERTEMP:
            return "Battery overtemperature";
        case AKS_FAULT_MOTOR_OVERTEMP:
            return "Motor overtemperature";
        case AKS_FAULT_OVERCURRENT:
            return "Overcurrent detected";
        case AKS_FAULT_EMERGENCY_STOP:
            return "Emergency stop active";
        case AKS_FAULT_DOOR_OPEN:
            return "Door open";
        case AKS_FAULT_SEATBELT_OPEN:
            return "Seatbelt not fastened";
        case AKS_FAULT_COMMUNICATION_LOSS:
            return "Communication loss";
        case AKS_FAULT_SENSOR_FAILURE:
            return "Sensor failure";
        case AKS_FAULT_ACTUATOR_FAILURE:
            return "Actuator failure";
        case AKS_FAULT_WATCHDOG_TIMEOUT:
            return "Watchdog timeout";
        case AKS_FAULT_SYSTEM_OVERLOAD:
            return "System overload";
        default:
            return "Unknown fault";
    }
}

/* Private functions */

/**
 * @brief Find fault log entry by fault code
 * @param fault_code Fault code to search for
 * @return Index of entry if found, fault_log_count if not found
 */
static uint8_t find_fault_log_entry(aks_fault_code_t fault_code)
{
    for (uint8_t i = 0; i < fault_log_count; i++) {
        if (fault_log[i].fault_code == fault_code && !fault_log[i].resolved) {
            return i;
        }
    }
    return fault_log_count;
}

/**
 * @brief Monitor isolation resistance
 */
static void safety_monitor_isolation(void)
{
    /* Implement actual isolation monitoring */
    
    /* 1. Switch isolation monitoring circuit to positive line */
    // GPIO control for isolation monitoring relay
    
    /* 2. Measure positive line resistance */
    float pos_voltage = aks_adc_read_voltage(AKS_ADC_CHANNEL_ISOLATION_POS);
    float pos_resistance = (pos_voltage * 1000000.0f) / (3.3f - pos_voltage); // Calculate resistance
    
    /* 3. Switch to negative line */
    // GPIO control for isolation monitoring relay
    
    /* 4. Measure negative line resistance */
    float neg_voltage = aks_adc_read_voltage(AKS_ADC_CHANNEL_ISOLATION_NEG);
    float neg_resistance = (neg_voltage * 1000000.0f) / (3.3f - neg_voltage); // Calculate resistance
    
    /* 5. Calculate total isolation resistance */
    float total_resistance = (pos_resistance * neg_resistance) / (pos_resistance + neg_resistance);
    
    /* 6. Check against thresholds */
    if (total_resistance < safety_config.min_isolation_resistance) {
        /* Low isolation detected */
        aks_safety_log_fault(AKS_FAULT_ISOLATION_LOW, total_resistance, AKS_ACTION_WARNING);
        
        if (total_resistance < (safety_config.min_isolation_resistance * 0.5f)) {
            /* Critical isolation failure */
            aks_safety_emergency_stop(AKS_FAULT_ISOLATION_LOW);
        }
    }
    
    /* 7. Store measurement for telemetry */
    // Store in global variable for telemetry access
}

/**
 * @brief Monitor temperature sensors
 */
static void safety_monitor_temperature(void)
{
    /* Implement temperature monitoring */
    
    /* 1. Read battery temperature sensors */
    float battery_temp = aks_temperature_read(AKS_TEMP_SENSOR_BATTERY);
    
    /* 2. Read motor temperature sensors */
    float motor_temp = aks_temperature_read(AKS_TEMP_SENSOR_MOTOR);
    
    /* 3. Read controller temperature */
    float controller_temp = aks_temperature_read(AKS_TEMP_SENSOR_CONTROLLER);
    
    /* 4. Check battery temperature limits */
    if (battery_temp > safety_config.max_battery_temp) {
        aks_safety_log_fault(AKS_FAULT_BATTERY_OVERTEMP, battery_temp, AKS_ACTION_REDUCE_POWER);
        
        if (battery_temp > (safety_config.max_battery_temp + 10.0f)) {
            /* Critical battery overtemperature */
            aks_safety_emergency_stop(AKS_FAULT_BATTERY_OVERTEMP);
        }
    }
    
    /* 5. Check motor temperature limits */
    if (motor_temp > safety_config.max_motor_temp) {
        aks_safety_log_fault(AKS_FAULT_MOTOR_OVERTEMP, motor_temp, AKS_ACTION_REDUCE_POWER);
        
        if (motor_temp > (safety_config.max_motor_temp + 15.0f)) {
            /* Critical motor overtemperature */
            aks_safety_emergency_stop(AKS_FAULT_MOTOR_OVERTEMP);
        }
    }
    
    /* 6. Check controller temperature */
    if (controller_temp > 85.0f) { // STM32 max operating temperature
        aks_safety_log_fault(AKS_FAULT_SYSTEM_OVERLOAD, controller_temp, AKS_ACTION_REDUCE_POWER);
        
        if (controller_temp > 95.0f) {
            /* Critical controller overtemperature */
            aks_safety_emergency_stop(AKS_FAULT_SYSTEM_OVERLOAD);
        }
    }
}

/**
 * @brief Monitor voltage levels
 */
static void safety_monitor_voltage(void)
{
    /* Implement voltage monitoring */
    
    /* 1. Read battery pack voltage */
    float battery_voltage = aks_voltage_read(AKS_VOLTAGE_SENSOR_BATTERY_PACK);
    
    /* 2. Read DC link voltage */
    float dc_link_voltage = aks_voltage_read(AKS_VOLTAGE_SENSOR_DC_LINK);
    
    /* 3. Read 12V supply voltage */
    float supply_12v = aks_voltage_read(AKS_VOLTAGE_SENSOR_12V_SUPPLY);
    
    /* 4. Check battery voltage limits */
    if (battery_voltage > safety_config.max_battery_voltage) {
        aks_safety_log_fault(AKS_FAULT_BATTERY_OVERVOLTAGE, battery_voltage, AKS_ACTION_DISCONNECT_CHARGER);
        
        if (battery_voltage > (safety_config.max_battery_voltage + 5.0f)) {
            /* Critical overvoltage */
            aks_safety_emergency_stop(AKS_FAULT_BATTERY_OVERVOLTAGE);
        }
    }
    
    if (battery_voltage < safety_config.min_battery_voltage) {
        aks_safety_log_fault(AKS_FAULT_BATTERY_UNDERVOLTAGE, battery_voltage, AKS_ACTION_REDUCE_POWER);
        
        if (battery_voltage < (safety_config.min_battery_voltage - 5.0f)) {
            /* Critical undervoltage */
            aks_safety_emergency_stop(AKS_FAULT_BATTERY_UNDERVOLTAGE);
        }
    }
    
    /* 5. Check DC link voltage */
    if (dc_link_voltage > (battery_voltage + 10.0f) || dc_link_voltage < (battery_voltage - 10.0f)) {
        /* DC link voltage mismatch */
        aks_safety_log_fault(AKS_FAULT_SYSTEM_OVERLOAD, dc_link_voltage, AKS_ACTION_WARNING);
    }
    
    /* 6. Check 12V supply */
    if (supply_12v < 10.5f || supply_12v > 14.5f) {
        /* 12V supply out of range */
        aks_safety_log_fault(AKS_FAULT_SYSTEM_OVERLOAD, supply_12v, AKS_ACTION_WARNING);
    }
}

/**
 * @brief Monitor current levels
 */
static void safety_monitor_current(void)
{
    /* Implement current monitoring */
    
    /* 1. Read battery current */
    float battery_current = aks_current_read(AKS_CURRENT_SENSOR_BATTERY);
    
    /* 2. Read motor current */
    float motor_current = aks_current_read(AKS_CURRENT_SENSOR_MOTOR);
    
    /* 3. Read charger current */
    float charger_current = aks_current_read(AKS_CURRENT_SENSOR_CHARGER);
    
    /* 4. Check battery current limits */
    if (fabsf(battery_current) > safety_config.max_battery_current) {
        aks_safety_log_fault(AKS_FAULT_OVERCURRENT, battery_current, AKS_ACTION_REDUCE_POWER);
        
        if (fabsf(battery_current) > (safety_config.max_battery_current * 1.2f)) {
            /* Critical overcurrent */
            aks_safety_emergency_stop(AKS_FAULT_OVERCURRENT);
        }
    }
    
    /* 5. Check motor current limits */
    if (fabsf(motor_current) > 300.0f) { // 300A max motor current
        aks_safety_log_fault(AKS_FAULT_OVERCURRENT, motor_current, AKS_ACTION_REDUCE_POWER);
        
        if (fabsf(motor_current) > 350.0f) {
            /* Critical motor overcurrent */
            aks_safety_emergency_stop(AKS_FAULT_OVERCURRENT);
        }
    }
    
    /* 6. Check charger current limits */
    if (charger_current > 20.0f) { // 20A max charging current
        aks_safety_log_fault(AKS_FAULT_OVERCURRENT, charger_current, AKS_ACTION_REDUCE_CHARGING);
        
        if (charger_current > 25.0f) {
            /* Critical charging overcurrent */
            aks_main_set_relay(1, false); // Disconnect charger
        }
    }
    
    /* 7. Check for current imbalance */
    if (fabsf(battery_current - motor_current) > 50.0f) {
        /* Significant current imbalance detected */
        aks_safety_log_fault(AKS_FAULT_SYSTEM_OVERLOAD, 
                           fabsf(battery_current - motor_current), 
                           AKS_ACTION_WARNING);
    }
}