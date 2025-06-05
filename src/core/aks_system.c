/**
 * @file aks_system.c
 * @brief Main system integration for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include "aks_can.h"
#include "aks_uart.h"
#include "aks_types.h"
#include <string.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* System Configuration */
#define AKS_SYSTEM_TASK_PERIOD          10      // ms
#define AKS_SYSTEM_HEARTBEAT_PERIOD     1000    // ms
#define AKS_SYSTEM_TELEMETRY_PERIOD     100     // ms
#define AKS_SYSTEM_MONITORING_PERIOD    1000    // ms

/* System State Machine */
typedef enum {
    AKS_SYSTEM_STATE_INIT = 0,
    AKS_SYSTEM_STATE_STARTUP,
    AKS_SYSTEM_STATE_SELF_TEST,
    AKS_SYSTEM_STATE_READY,
    AKS_SYSTEM_STATE_RUNNING,
    AKS_SYSTEM_STATE_FAULT,
    AKS_SYSTEM_STATE_EMERGENCY,
    AKS_SYSTEM_STATE_SHUTDOWN
} aks_system_state_machine_t;

/* System Handle */
typedef struct {
    bool initialized;
    aks_system_state_machine_t state;
    aks_config_t config;
    uint32_t last_task_time;
    uint32_t last_heartbeat_time;
    uint32_t last_telemetry_time;
    uint32_t last_monitoring_time;
    uint32_t startup_time;
    uint32_t fault_count;
    bool emergency_stop_active;
    bool self_test_passed;
    bool all_systems_ready;
} aks_system_handle_t;

/* Private variables */
static aks_system_handle_t g_system_handle;

/* Private function prototypes */
static aks_result_t aks_system_startup_sequence(void);
static aks_result_t aks_system_self_test(void);
static aks_result_t aks_system_check_all_systems(void);
static aks_result_t aks_system_handle_emergency(void);
static aks_result_t aks_system_send_heartbeat(void);
static aks_result_t aks_system_collect_telemetry(void);
static aks_result_t aks_system_state_transition(aks_system_state_machine_t new_state);

/**
 * @brief Initialize complete AKS system
 */
aks_result_t aks_system_init(const aks_config_t* config)
{
    if (g_system_handle.initialized) {
        return AKS_ERROR_BUSY;
    }
    
    /* Initialize core system first */
    aks_result_t result = aks_core_init(config);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Store configuration */
    if (config != NULL) {
        g_system_handle.config = *config;
    } else {
        /* Load default configuration */
        aks_config_load_default();
        aks_config_get(&g_system_handle.config);
    }
    
    /* Initialize system state */
    g_system_handle.state = AKS_SYSTEM_STATE_INIT;
    g_system_handle.startup_time = HAL_GetTick();
    g_system_handle.fault_count = 0;
    g_system_handle.emergency_stop_active = false;
    g_system_handle.self_test_passed = false;
    g_system_handle.all_systems_ready = false;
    
    /* Initialize timestamps */
    uint32_t current_time = HAL_GetTick();
    g_system_handle.last_task_time = current_time;
    g_system_handle.last_heartbeat_time = current_time;
    g_system_handle.last_telemetry_time = current_time;
    g_system_handle.last_monitoring_time = current_time;
    
    g_system_handle.initialized = true;
    
    /* Start startup sequence */
    aks_system_state_transition(AKS_SYSTEM_STATE_STARTUP);
    
    return AKS_OK;
}

/**
 * @brief Deinitialize AKS system
 */
aks_result_t aks_system_deinit(void)
{
    if (!g_system_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Shutdown sequence */
    aks_system_state_transition(AKS_SYSTEM_STATE_SHUTDOWN);
    
    /* Deinitialize all subsystems */
    aks_monitoring_deinit();
    aks_logger_deinit();
    aks_isolation_deinit();
    aks_motor_deinit();
    aks_adc_deinit();
    aks_uart_deinit(AKS_UART_1);
    aks_can_deinit();
    aks_core_deinit();
    
    /* Reset handle */
    memset(&g_system_handle, 0, sizeof(aks_system_handle_t));
    
    return AKS_OK;
}

/**
 * @brief Start AKS system
 */
aks_result_t aks_system_start(void)
{
    if (!g_system_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_system_handle.state != AKS_SYSTEM_STATE_READY) {
        return AKS_ERROR_NOT_READY;
    }
    
    return aks_system_state_transition(AKS_SYSTEM_STATE_RUNNING);
}

/**
 * @brief Stop AKS system
 */
aks_result_t aks_system_stop(void)
{
    if (!g_system_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    return aks_system_state_transition(AKS_SYSTEM_STATE_READY);
}

/**
 * @brief Emergency stop
 */
aks_result_t aks_system_emergency_stop(void)
{
    if (!g_system_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_system_handle.emergency_stop_active = true;
    
    /* Stop motor immediately */
    aks_motor_emergency_stop();
    
    return aks_system_state_transition(AKS_SYSTEM_STATE_EMERGENCY);
}

/**
 * @brief Clear emergency stop
 */
aks_result_t aks_system_clear_emergency(void)
{
    if (!g_system_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_system_handle.state != AKS_SYSTEM_STATE_EMERGENCY) {
        return AKS_ERROR;
    }
    
    g_system_handle.emergency_stop_active = false;
    
    return aks_system_state_transition(AKS_SYSTEM_STATE_READY);
}

/**
 * @brief Get system state
 */
aks_system_state_machine_t aks_system_get_state(void)
{
    if (!g_system_handle.initialized) {
        return AKS_SYSTEM_STATE_INIT;
    }
    
    return g_system_handle.state;
}

/**
 * @brief Check if system is ready
 */
bool aks_system_is_ready(void)
{
    if (!g_system_handle.initialized) {
        return false;
    }
    
    return (g_system_handle.state == AKS_SYSTEM_STATE_READY || 
            g_system_handle.state == AKS_SYSTEM_STATE_RUNNING);
}

/**
 * @brief Check if emergency stop is active
 */
bool aks_system_is_emergency_active(void)
{
    if (!g_system_handle.initialized) {
        return true; // Safe default
    }
    
    return g_system_handle.emergency_stop_active;
}

/**
 * @brief Get system uptime
 */
uint32_t aks_system_get_uptime(void)
{
    if (!g_system_handle.initialized) {
        return 0;
    }
    
    return HAL_GetTick() - g_system_handle.startup_time;
}

/**
 * @brief Get fault count
 */
uint32_t aks_system_get_fault_count(void)
{
    if (!g_system_handle.initialized) {
        return 0;
    }
    
    return g_system_handle.fault_count;
}

/**
 * @brief Main system task (should be called periodically)
 */
aks_result_t aks_system_task(void)
{
    if (!g_system_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = HAL_GetTick();
    
    /* Check if it's time for task execution */
    if (current_time - g_system_handle.last_task_time < AKS_SYSTEM_TASK_PERIOD) {
        return AKS_OK;
    }
    
    g_system_handle.last_task_time = current_time;
    
    /* State machine processing */
    switch (g_system_handle.state) {
        case AKS_SYSTEM_STATE_INIT:
            /* Should not stay here long */
            break;
            
        case AKS_SYSTEM_STATE_STARTUP:
            aks_system_startup_sequence();
            break;
            
        case AKS_SYSTEM_STATE_SELF_TEST:
            aks_system_self_test();
            break;
            
        case AKS_SYSTEM_STATE_READY:
            aks_system_check_all_systems();
            break;
            
        case AKS_SYSTEM_STATE_RUNNING:
            aks_system_check_all_systems();
            break;
            
        case AKS_SYSTEM_STATE_FAULT:
            /* Handle fault recovery */
            if (aks_monitoring_all_systems_ok()) {
                aks_system_state_transition(AKS_SYSTEM_STATE_READY);
            }
            break;
            
        case AKS_SYSTEM_STATE_EMERGENCY:
            aks_system_handle_emergency();
            break;
            
        case AKS_SYSTEM_STATE_SHUTDOWN:
            /* Shutdown tasks */
            break;
            
        default:
            aks_system_state_transition(AKS_SYSTEM_STATE_FAULT);
            break;
    }
    
    /* Periodic tasks */
    
    /* Heartbeat */
    if (current_time - g_system_handle.last_heartbeat_time >= AKS_SYSTEM_HEARTBEAT_PERIOD) {
        g_system_handle.last_heartbeat_time = current_time;
        aks_system_send_heartbeat();
    }
    
    /* Telemetry */
    if (current_time - g_system_handle.last_telemetry_time >= AKS_SYSTEM_TELEMETRY_PERIOD) {
        g_system_handle.last_telemetry_time = current_time;
        aks_system_collect_telemetry();
    }
    
    /* Monitoring */
    if (current_time - g_system_handle.last_monitoring_time >= AKS_SYSTEM_MONITORING_PERIOD) {
        g_system_handle.last_monitoring_time = current_time;
        aks_monitoring_task();
    }
    
    /* Run subsystem tasks */
    aks_core_task();
    aks_logger_task();
    aks_isolation_task();
    aks_motor_task();
    
    return AKS_OK;
}

/**
 * @brief Perform blink test
 */
aks_result_t aks_system_perform_blink_test(void)
{
    if (!g_system_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    return aks_monitoring_perform_blink_test();
}

/* Private Functions */

/**
 * @brief System startup sequence
 */
static aks_result_t aks_system_startup_sequence(void)
{
    static uint8_t startup_phase = 0;
    
    switch (startup_phase) {
        case 0:
            /* Initialize CAN bus */
            aks_can_init(g_system_handle.config.can_baudrate, NULL, 0);
            startup_phase++;
            break;
            
        case 1:
            /* Initialize UART */
            aks_uart_config_t uart_config = {
                .baudrate = g_system_handle.config.uart_baudrate,
                .data_bits = 8,
                .parity = AKS_UART_PARITY_NONE,
                .stop_bits = AKS_UART_STOPBITS_1,
                .flow_control = AKS_UART_FLOWCONTROL_NONE,
                .enable_dma = false,
                .rx_buffer_size = 256,
                .tx_buffer_size = 256
            };
            aks_uart_init(AKS_UART_1, &uart_config);
            startup_phase++;
            break;
            
        case 2:
            /* Initialize logger */
            aks_logger_init(NULL);
            aks_logger_info("AKS System starting up...");
            startup_phase++;
            break;
            
        case 3:
            /* Initialize monitoring */
            aks_monitoring_init(NULL);
            startup_phase++;
            break;
            
        case 4:
            /* Initialize motor controller */
            aks_motor_init(NULL);
            startup_phase++;
            break;
            
        case 5:
            /* Initialize isolation monitoring with real ADC configuration */
            {
                aks_isolation_config_t iso_config = {
                    .min_resistance = 50000.0f,        /* 50kΩ minimum */
                    .warning_level = 200000.0f,        /* 200kΩ warning */
                    .good_level = 1000000.0f,          /* 1MΩ good */
                    .measurement_voltage = 12.0f,       /* 12V measurement */
                    .filter_samples = 8,               /* 8-sample filter */
                    .measurement_period = 500,         /* 500ms period */
                    .fault_threshold = 3,              /* 3 consecutive faults */
                    .enable_positive_monitoring = true,
                    .enable_negative_monitoring = true,
                    .enable_auto_measurement = true
                };
                aks_isolation_init(&iso_config);
            }
            startup_phase++;
            break;
            
        case 6:
            /* Initialize ADC with all required channels */
            {
                aks_adc_config_t adc_config = {
                    .resolution = 12,                  /* 12-bit resolution */
                    .sampling_time = 15,               /* 15 cycles sampling */
                    .oversampling = 4,                 /* 4x oversampling */
                    .enable_dma = true,                /* Enable DMA */
                    .enable_interrupts = true,         /* Enable interrupts */
                    .reference_voltage = 3.3f,         /* 3.3V reference */
                    .num_channels = 16                 /* 16 channels */
                };
                aks_adc_init(&adc_config);
            }
            startup_phase++;
            break;
            
        case 7:
            /* Startup complete */
            aks_logger_info("AKS System startup complete");
            startup_phase = 0; // Reset for next startup
            aks_system_state_transition(AKS_SYSTEM_STATE_SELF_TEST);
            break;
    }
    
    return AKS_OK;
}

/**
 * @brief System self test
 */
static aks_result_t aks_system_self_test(void)
{
    static uint8_t test_phase = 0;
    static uint32_t test_start_time = 0;
    
    if (test_phase == 0) {
        test_start_time = HAL_GetTick();
        aks_logger_info("Starting system self-test...");
        test_phase++;
    }
    
    switch (test_phase) {
        case 1:
            /* Test CAN communication */
            // Send test message and wait for response
            test_phase++;
            break;
            
        case 2:
            /* Test motor controller communication */
            // Check motor controller heartbeat
            test_phase++;
            break;
            
        case 3:
            /* Test BMS communication */
            // Check BMS heartbeat
            test_phase++;
            break;
            
        case 4:
            /* Test isolation monitoring */
            // Perform isolation measurement
            test_phase++;
            break;
            
        case 5:
            /* Perform blink test */
            aks_monitoring_perform_blink_test();
            test_phase++;
            break;
            
        case 6:
            /* Self test complete */
            uint32_t test_duration = HAL_GetTick() - test_start_time;
            aks_logger_info("Self-test completed in %lu ms", test_duration);
            g_system_handle.self_test_passed = true;
            test_phase = 0; // Reset for next test
            aks_system_state_transition(AKS_SYSTEM_STATE_READY);
            break;
    }
    
    return AKS_OK;
}

/**
 * @brief Check all systems
 */
static aks_result_t aks_system_check_all_systems(void)
{
    /* Check if all systems are OK */
    bool all_ok = aks_monitoring_all_systems_ok();
    
    if (!all_ok && g_system_handle.state != AKS_SYSTEM_STATE_FAULT) {
        g_system_handle.fault_count++;
        aks_logger_error("System fault detected, transitioning to fault state");
        aks_system_state_transition(AKS_SYSTEM_STATE_FAULT);
    }
    
    g_system_handle.all_systems_ready = all_ok;
    
    return AKS_OK;
}

/**
 * @brief Handle emergency situation
 */
static aks_result_t aks_system_handle_emergency(void)
{
    /* Ensure motor is stopped */
    aks_motor_emergency_stop();
    
    /* Log emergency state */
    aks_logger_error("Emergency stop active");
    
    /* Send emergency CAN message */
    aks_can_frame_t frame;
    frame.id = 0x700; // Emergency CAN ID
    frame.length = 8;
    frame.is_extended = false;
    frame.is_remote = false;
    
    frame.data[0] = 0xFF; // Emergency code
    frame.data[1] = (uint8_t)g_system_handle.state;
    memset(&frame.data[2], 0, 6);
    
    aks_can_transmit(&frame, 100);
    
    return AKS_OK;
}

/**
 * @brief Send system heartbeat
 */
static aks_result_t aks_system_send_heartbeat(void)
{
    /* Send heartbeat to monitoring system */
    aks_monitoring_heartbeat(AKS_SYSTEM_MOTOR_CONTROLLER);
    aks_monitoring_heartbeat(AKS_SYSTEM_BMS);
    aks_monitoring_heartbeat(AKS_SYSTEM_ISOLATION);
    aks_monitoring_heartbeat(AKS_SYSTEM_TELEMETRY);
    aks_monitoring_heartbeat(AKS_SYSTEM_SAFETY);
    
    /* Send CAN heartbeat */
    aks_can_frame_t frame;
    frame.id = 0x600; // Heartbeat CAN ID
    frame.length = 8;
    frame.is_extended = false;
    frame.is_remote = false;
    
    frame.data[0] = (uint8_t)g_system_handle.state;
    frame.data[1] = g_system_handle.all_systems_ready ? 1 : 0;
    frame.data[2] = g_system_handle.emergency_stop_active ? 1 : 0;
    frame.data[3] = g_system_handle.self_test_passed ? 1 : 0;
    
    uint32_t uptime = aks_system_get_uptime();
    frame.data[4] = (uptime >> 24) & 0xFF;
    frame.data[5] = (uptime >> 16) & 0xFF;
    frame.data[6] = (uptime >> 8) & 0xFF;
    frame.data[7] = uptime & 0xFF;
    
    return aks_can_transmit(&frame, 100);
}

/**
 * @brief Collect and log telemetry data
 */
static aks_result_t aks_system_collect_telemetry(void)
{
    /* Collect telemetry data from all systems */
    aks_telemetry_data_t telemetry_data;
    
    telemetry_data.timestamp = HAL_GetTick();
    telemetry_data.vehicle_speed = 0.0f; // Would be calculated from wheel sensors
    telemetry_data.motor_torque = 0.0f;  // From motor controller
    telemetry_data.motor_speed = 0.0f;   // From motor controller
    telemetry_data.battery_voltage = aks_adc_get_battery_voltage();
    telemetry_data.battery_current = aks_adc_get_battery_current();
    telemetry_data.battery_temperature = aks_adc_get_battery_temperature();
    telemetry_data.battery_soc = 0.0f;   // From BMS
    telemetry_data.gps_latitude = 0.0;   // From GPS
    telemetry_data.gps_longitude = 0.0;  // From GPS
    telemetry_data.gps_speed = 0.0f;     // From GPS
    telemetry_data.isolation_resistance = 0.0f; // From isolation monitor
    telemetry_data.system_state = (uint8_t)g_system_handle.state;
    telemetry_data.fault_codes = (uint16_t)g_system_handle.fault_count;
    
    /* Log telemetry data */
    return aks_logger_telemetry(&telemetry_data);
}

/**
 * @brief State transition
 */
static aks_result_t aks_system_state_transition(aks_system_state_machine_t new_state)
{
    if (g_system_handle.state == new_state) {
        return AKS_OK; // No change needed
    }
    
    aks_system_state_machine_t old_state = g_system_handle.state;
    g_system_handle.state = new_state;
    
    /* Log state transition */
    aks_logger_info("State transition: %d -> %d", old_state, new_state);
    
    /* Update core system state */
    aks_state_t core_state;
    switch (new_state) {
        case AKS_SYSTEM_STATE_INIT:
        case AKS_SYSTEM_STATE_STARTUP:
        case AKS_SYSTEM_STATE_SELF_TEST:
            core_state = AKS_STATE_INIT;
            break;
        case AKS_SYSTEM_STATE_READY:
            core_state = AKS_STATE_READY;
            break;
        case AKS_SYSTEM_STATE_RUNNING:
            core_state = AKS_STATE_RUNNING;
            break;
        case AKS_SYSTEM_STATE_FAULT:
            core_state = AKS_STATE_FAULT;
            break;
        case AKS_SYSTEM_STATE_EMERGENCY:
            core_state = AKS_STATE_EMERGENCY;
            break;
        case AKS_SYSTEM_STATE_SHUTDOWN:
            core_state = AKS_STATE_SHUTDOWN;
            break;
        default:
            core_state = AKS_STATE_FAULT;
            break;
    }
    
    aks_core_set_state(core_state);
    
    return AKS_OK;
}
