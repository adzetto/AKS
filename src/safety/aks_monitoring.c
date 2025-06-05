/**
 * @file aks_monitoring.c
 * @brief System monitoring and blink test implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_can.h"
#include <string.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Monitoring Constants */
#define AKS_MONITORING_CHECK_INTERVAL       1000    // ms
#define AKS_MONITORING_BLINK_INTERVAL       500     // ms
#define AKS_MONITORING_FAULT_TIMEOUT        5000    // ms
#define AKS_MONITORING_HEARTBEAT_TIMEOUT    2000    // ms
#define AKS_MONITORING_MAX_SYSTEMS          16      // Maximum monitored systems

/* System IDs for monitoring */
typedef enum {
    AKS_SYSTEM_MOTOR_CONTROLLER = 0,
    AKS_SYSTEM_BMS,
    AKS_SYSTEM_ISOLATION,
    AKS_SYSTEM_TELEMETRY,
    AKS_SYSTEM_GPS,
    AKS_SYSTEM_SAFETY,
    AKS_SYSTEM_CHARGER,
    AKS_SYSTEM_ADAS,
    AKS_SYSTEM_COUNT
} aks_system_id_t;

/* System Status */
typedef enum {
    AKS_SYSTEM_STATUS_UNKNOWN = 0,
    AKS_SYSTEM_STATUS_OK,
    AKS_SYSTEM_STATUS_WARNING,
    AKS_SYSTEM_STATUS_ERROR,
    AKS_SYSTEM_STATUS_OFFLINE
} aks_system_status_t;

/* LED States */
typedef enum {
    AKS_LED_STATE_OFF = 0,
    AKS_LED_STATE_ON,
    AKS_LED_STATE_BLINK_SLOW,
    AKS_LED_STATE_BLINK_FAST,
    AKS_LED_STATE_BLINK_PATTERN
} aks_led_state_t;

/* System Information */
typedef struct {
    aks_system_id_t id;
    const char* name;
    aks_system_status_t status;
    uint32_t last_heartbeat;
    uint32_t fault_count;
    uint16_t fault_code;
    bool enabled;
    bool critical;
} aks_system_info_t;

/* LED Control */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    aks_led_state_t state;
    uint32_t last_toggle;
    uint16_t blink_period;
    bool current_state;
} aks_led_control_t;

/* Monitoring Configuration */
typedef struct {
    uint32_t check_interval_ms;
    uint32_t heartbeat_timeout_ms;
    uint32_t fault_timeout_ms;
    bool enable_blink_test;
    bool enable_serial_monitor;
    bool enable_can_reporting;
    uint16_t blink_period_ms;
} aks_monitoring_config_t;

/* Monitoring Handle */
typedef struct {
    bool initialized;
    aks_monitoring_config_t config;
    aks_system_info_t systems[AKS_SYSTEM_COUNT];
    aks_led_control_t status_led;
    aks_led_control_t fault_led;
    aks_led_control_t system_leds[AKS_SYSTEM_COUNT];
    uint32_t last_check_time;
    uint32_t last_blink_time;
    bool all_systems_ok;
    bool blink_test_active;
    uint8_t blink_test_phase;
    uint32_t total_faults;
    uint32_t system_uptime;
} aks_monitoring_handle_t;

/* Private variables */
static aks_monitoring_handle_t g_monitoring_handle;

/* System names */
static const char* g_system_names[AKS_SYSTEM_COUNT] = {
    "Motor Controller",
    "BMS",
    "Isolation Monitor",
    "Telemetry",
    "GPS",
    "Safety System",
    "Charger",
    "ADAS"
};

/* Default configuration */
static const aks_monitoring_config_t g_default_config = {
    .check_interval_ms = AKS_MONITORING_CHECK_INTERVAL,
    .heartbeat_timeout_ms = AKS_MONITORING_HEARTBEAT_TIMEOUT,
    .fault_timeout_ms = AKS_MONITORING_FAULT_TIMEOUT,
    .enable_blink_test = true,
    .enable_serial_monitor = true,
    .enable_can_reporting = true,
    .blink_period_ms = AKS_MONITORING_BLINK_INTERVAL
};

/* Private function prototypes */
static aks_result_t aks_monitoring_check_systems(void);
static aks_result_t aks_monitoring_update_leds(void);
static aks_result_t aks_monitoring_blink_test(void);
static aks_result_t aks_monitoring_serial_report(void);
static aks_result_t aks_monitoring_can_report(void);
static void aks_monitoring_led_control(aks_led_control_t* led);
static void aks_monitoring_set_led_state(aks_led_control_t* led, aks_led_state_t state);
static aks_result_t aks_monitoring_check_heartbeat(aks_system_id_t system_id);
static void aks_monitoring_update_system_status(aks_system_id_t system_id, aks_system_status_t status, uint16_t fault_code);

/**
 * @brief Initialize monitoring system
 */
aks_result_t aks_monitoring_init(const aks_monitoring_config_t* config)
{
    if (g_monitoring_handle.initialized) {
        return AKS_ERROR_BUSY;
    }
    
    /* Use default config if none provided */
    if (config != NULL) {
        g_monitoring_handle.config = *config;
    } else {
        g_monitoring_handle.config = g_default_config;
    }
    
    /* Initialize systems */
    for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
        g_monitoring_handle.systems[i].id = (aks_system_id_t)i;
        g_monitoring_handle.systems[i].name = g_system_names[i];
        g_monitoring_handle.systems[i].status = AKS_SYSTEM_STATUS_UNKNOWN;
        g_monitoring_handle.systems[i].last_heartbeat = HAL_GetTick();
        g_monitoring_handle.systems[i].fault_count = 0;
        g_monitoring_handle.systems[i].fault_code = 0;
        g_monitoring_handle.systems[i].enabled = true;
        g_monitoring_handle.systems[i].critical = (i < 3); // Motor, BMS, Isolation are critical
    }
    
    /* Initialize LEDs - These would be configured with actual GPIO pins */
    g_monitoring_handle.status_led.port = GPIOA;
    g_monitoring_handle.status_led.pin = GPIO_PIN_0;
    g_monitoring_handle.status_led.state = AKS_LED_STATE_OFF;
    g_monitoring_handle.status_led.blink_period = g_monitoring_handle.config.blink_period_ms;
    
    g_monitoring_handle.fault_led.port = GPIOA;
    g_monitoring_handle.fault_led.pin = GPIO_PIN_1;
    g_monitoring_handle.fault_led.state = AKS_LED_STATE_OFF;
    g_monitoring_handle.fault_led.blink_period = g_monitoring_handle.config.blink_period_ms / 2;
    
    /* Initialize system LEDs */
    for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
        g_monitoring_handle.system_leds[i].port = GPIOB;
        g_monitoring_handle.system_leds[i].pin = GPIO_PIN_0 + i;
        g_monitoring_handle.system_leds[i].state = AKS_LED_STATE_OFF;
        g_monitoring_handle.system_leds[i].blink_period = g_monitoring_handle.config.blink_period_ms;
    }
    
    /* Initialize timestamps */
    g_monitoring_handle.last_check_time = HAL_GetTick();
    g_monitoring_handle.last_blink_time = HAL_GetTick();
    
    g_monitoring_handle.initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize monitoring system
 */
aks_result_t aks_monitoring_deinit(void)
{
    if (!g_monitoring_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Turn off all LEDs */
    for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
        aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_OFF);
    }
    aks_monitoring_set_led_state(&g_monitoring_handle.status_led, AKS_LED_STATE_OFF);
    aks_monitoring_set_led_state(&g_monitoring_handle.fault_led, AKS_LED_STATE_OFF);
    
    /* Reset handle */
    memset(&g_monitoring_handle, 0, sizeof(aks_monitoring_handle_t));
    
    return AKS_OK;
}

/**
 * @brief Start monitoring
 */
aks_result_t aks_monitoring_start(void)
{
    if (!g_monitoring_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Start blink test if enabled */
    if (g_monitoring_handle.config.enable_blink_test) {
        g_monitoring_handle.blink_test_active = true;
        g_monitoring_handle.blink_test_phase = 0;
    }
    
    return AKS_OK;
}

/**
 * @brief Stop monitoring
 */
aks_result_t aks_monitoring_stop(void)
{
    if (!g_monitoring_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_monitoring_handle.blink_test_active = false;
    
    return AKS_OK;
}

/**
 * @brief Update system heartbeat
 */
aks_result_t aks_monitoring_heartbeat(aks_system_id_t system_id)
{
    if (!g_monitoring_handle.initialized || system_id >= AKS_SYSTEM_COUNT) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    g_monitoring_handle.systems[system_id].last_heartbeat = HAL_GetTick();
    
    /* Update status to OK if it was offline */
    if (g_monitoring_handle.systems[system_id].status == AKS_SYSTEM_STATUS_OFFLINE) {
        g_monitoring_handle.systems[system_id].status = AKS_SYSTEM_STATUS_OK;
    }
    
    return AKS_OK;
}

/**
 * @brief Report system fault
 */
aks_result_t aks_monitoring_report_fault(aks_system_id_t system_id, uint16_t fault_code)
{
    if (!g_monitoring_handle.initialized || system_id >= AKS_SYSTEM_COUNT) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_monitoring_update_system_status(system_id, AKS_SYSTEM_STATUS_ERROR, fault_code);
    g_monitoring_handle.systems[system_id].fault_count++;
    g_monitoring_handle.total_faults++;
    
    return AKS_OK;
}

/**
 * @brief Clear system fault
 */
aks_result_t aks_monitoring_clear_fault(aks_system_id_t system_id)
{
    if (!g_monitoring_handle.initialized || system_id >= AKS_SYSTEM_COUNT) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_monitoring_update_system_status(system_id, AKS_SYSTEM_STATUS_OK, 0);
    
    return AKS_OK;
}

/**
 * @brief Get system status
 */
aks_result_t aks_monitoring_get_system_status(aks_system_id_t system_id, aks_system_info_t* info)
{
    if (!g_monitoring_handle.initialized || system_id >= AKS_SYSTEM_COUNT || info == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *info = g_monitoring_handle.systems[system_id];
    
    return AKS_OK;
}

/**
 * @brief Check if all systems are OK
 */
bool aks_monitoring_all_systems_ok(void)
{
    if (!g_monitoring_handle.initialized) {
        return false;
    }
    
    return g_monitoring_handle.all_systems_ok;
}

/**
 * @brief Get monitoring statistics
 */
aks_result_t aks_monitoring_get_stats(uint32_t* total_faults, uint32_t* uptime, bool* all_ok)
{
    if (!g_monitoring_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (total_faults) {
        *total_faults = g_monitoring_handle.total_faults;
    }
    
    if (uptime) {
        *uptime = g_monitoring_handle.system_uptime;
    }
    
    if (all_ok) {
        *all_ok = g_monitoring_handle.all_systems_ok;
    }
    
    return AKS_OK;
}

/**
 * @brief Monitoring periodic task
 */
aks_result_t aks_monitoring_task(void)
{
    if (!g_monitoring_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = HAL_GetTick();
    
    /* Update system uptime */
    g_monitoring_handle.system_uptime = current_time;
    
    /* Check systems periodically */
    if (current_time - g_monitoring_handle.last_check_time >= g_monitoring_handle.config.check_interval_ms) {
        g_monitoring_handle.last_check_time = current_time;
        
        aks_monitoring_check_systems();
        
        /* Serial monitoring report */
        if (g_monitoring_handle.config.enable_serial_monitor) {
            aks_monitoring_serial_report();
        }
        
        /* CAN monitoring report */
        if (g_monitoring_handle.config.enable_can_reporting) {
            aks_monitoring_can_report();
        }
    }
    
    /* Update LEDs */
    aks_monitoring_update_leds();
    
    /* Blink test */
    if (g_monitoring_handle.blink_test_active) {
        aks_monitoring_blink_test();
    }
    
    return AKS_OK;
}

/**
 * @brief Perform blink test
 */
aks_result_t aks_monitoring_perform_blink_test(void)
{
    if (!g_monitoring_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_monitoring_handle.blink_test_active = true;
    g_monitoring_handle.blink_test_phase = 0;
    
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Check all systems
 */
static aks_result_t aks_monitoring_check_systems(void)
{
    bool all_ok = true;
    uint32_t current_time = HAL_GetTick();
    
    for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
        if (!g_monitoring_handle.systems[i].enabled) {
            continue;
        }
        
        /* Check heartbeat timeout */
        if (current_time - g_monitoring_handle.systems[i].last_heartbeat > 
            g_monitoring_handle.config.heartbeat_timeout_ms) {
            
            aks_monitoring_update_system_status((aks_system_id_t)i, AKS_SYSTEM_STATUS_OFFLINE, 0);
        }
        
        /* Check if system is not OK */
        if (g_monitoring_handle.systems[i].status != AKS_SYSTEM_STATUS_OK) {
            all_ok = false;
            
            /* Critical system failure */
            if (g_monitoring_handle.systems[i].critical && 
                g_monitoring_handle.systems[i].status == AKS_SYSTEM_STATUS_ERROR) {
                all_ok = false;
            }
        }
    }
    
    g_monitoring_handle.all_systems_ok = all_ok;
    
    return AKS_OK;
}

/**
 * @brief Update LED states
 */
static aks_result_t aks_monitoring_update_leds(void)
{
    /* Status LED - Green when all systems OK */
    if (g_monitoring_handle.all_systems_ok) {
        aks_monitoring_set_led_state(&g_monitoring_handle.status_led, AKS_LED_STATE_ON);
        aks_monitoring_set_led_state(&g_monitoring_handle.fault_led, AKS_LED_STATE_OFF);
    } else {
        aks_monitoring_set_led_state(&g_monitoring_handle.status_led, AKS_LED_STATE_BLINK_SLOW);
        aks_monitoring_set_led_state(&g_monitoring_handle.fault_led, AKS_LED_STATE_BLINK_FAST);
    }
    
    /* Individual system LEDs */
    for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
        switch (g_monitoring_handle.systems[i].status) {
            case AKS_SYSTEM_STATUS_OK:
                aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_ON);
                break;
            case AKS_SYSTEM_STATUS_WARNING:
                aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_BLINK_SLOW);
                break;
            case AKS_SYSTEM_STATUS_ERROR:
                aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_BLINK_FAST);
                break;
            case AKS_SYSTEM_STATUS_OFFLINE:
                aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_OFF);
                break;
            default:
                aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_BLINK_SLOW);
                break;
        }
    }
    
    /* Control LED hardware */
    aks_monitoring_led_control(&g_monitoring_handle.status_led);
    aks_monitoring_led_control(&g_monitoring_handle.fault_led);
    
    for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
        aks_monitoring_led_control(&g_monitoring_handle.system_leds[i]);
    }
    
    return AKS_OK;
}

/**
 * @brief Perform blink test sequence
 */
static aks_result_t aks_monitoring_blink_test(void)
{
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - g_monitoring_handle.last_blink_time >= 1000) { // 1 second per phase
        g_monitoring_handle.last_blink_time = current_time;
        g_monitoring_handle.blink_test_phase++;
        
        if (g_monitoring_handle.blink_test_phase <= AKS_SYSTEM_COUNT) {
            /* Turn on LEDs sequentially */
            for (int i = 0; i < g_monitoring_handle.blink_test_phase; i++) {
                aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_ON);
            }
        } else if (g_monitoring_handle.blink_test_phase == AKS_SYSTEM_COUNT + 1) {
            /* All systems green - test complete */
            aks_monitoring_set_led_state(&g_monitoring_handle.status_led, AKS_LED_STATE_ON);
            
            /* Check if all systems are actually OK */
            if (g_monitoring_handle.all_systems_ok) {
                /* Keep all LEDs green for 2 seconds */
                for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
                    aks_monitoring_set_led_state(&g_monitoring_handle.system_leds[i], AKS_LED_STATE_ON);
                }
            }
        } else if (g_monitoring_handle.blink_test_phase >= AKS_SYSTEM_COUNT + 3) {
            /* End blink test */
            g_monitoring_handle.blink_test_active = false;
            g_monitoring_handle.blink_test_phase = 0;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Serial monitoring report
 */
static aks_result_t aks_monitoring_serial_report(void)
{
    printf("\n=== AKS System Monitor ===\n");
    printf("Uptime: %lu ms\n", g_monitoring_handle.system_uptime);
    printf("All Systems OK: %s\n", g_monitoring_handle.all_systems_ok ? "YES" : "NO");
    printf("Total Faults: %lu\n", g_monitoring_handle.total_faults);
    printf("\nSystem Status:\n");
    
    for (int i = 0; i < AKS_SYSTEM_COUNT; i++) {
        const char* status_str;
        switch (g_monitoring_handle.systems[i].status) {
            case AKS_SYSTEM_STATUS_OK: status_str = "OK"; break;
            case AKS_SYSTEM_STATUS_WARNING: status_str = "WARN"; break;
            case AKS_SYSTEM_STATUS_ERROR: status_str = "ERROR"; break;
            case AKS_SYSTEM_STATUS_OFFLINE: status_str = "OFFLINE"; break;
            default: status_str = "UNKNOWN"; break;
        }
        
        printf("  %s: %s", g_monitoring_handle.systems[i].name, status_str);
        if (g_monitoring_handle.systems[i].fault_code != 0) {
            printf(" (Fault: 0x%04X)", g_monitoring_handle.systems[i].fault_code);
        }
        printf("\n");
    }
    printf("========================\n\n");
    
    return AKS_OK;
}

/**
 * @brief CAN monitoring report
 */
static aks_result_t aks_monitoring_can_report(void)
{
    aks_can_frame_t frame;
    frame.id = 0x500; // Monitoring status CAN ID
    frame.length = 8;
    frame.is_extended = false;
    frame.is_remote = false;
    
    /* Pack monitoring data */
    frame.data[0] = g_monitoring_handle.all_systems_ok ? 1 : 0;
    frame.data[1] = (g_monitoring_handle.total_faults >> 8) & 0xFF;
    frame.data[2] = g_monitoring_handle.total_faults & 0xFF;
    
    /* Pack system status bits */
    uint16_t status_bits = 0;
    for (int i = 0; i < AKS_SYSTEM_COUNT && i < 16; i++) {
        if (g_monitoring_handle.systems[i].status == AKS_SYSTEM_STATUS_OK) {
            status_bits |= (1 << i);
        }
    }
    
    frame.data[3] = (status_bits >> 8) & 0xFF;
    frame.data[4] = status_bits & 0xFF;
    
    frame.data[5] = (g_monitoring_handle.system_uptime >> 16) & 0xFF;
    frame.data[6] = (g_monitoring_handle.system_uptime >> 8) & 0xFF;
    frame.data[7] = g_monitoring_handle.system_uptime & 0xFF;
    
    return aks_can_transmit(&frame, 100);
}

/**
 * @brief Control LED hardware
 */
static void aks_monitoring_led_control(aks_led_control_t* led)
{
    if (led == NULL) return;
    
    uint32_t current_time = HAL_GetTick();
    
    switch (led->state) {
        case AKS_LED_STATE_OFF:
            led->current_state = false;
            break;
            
        case AKS_LED_STATE_ON:
            led->current_state = true;
            break;
            
        case AKS_LED_STATE_BLINK_SLOW:
        case AKS_LED_STATE_BLINK_FAST:
            if (current_time - led->last_toggle >= led->blink_period) {
                led->current_state = !led->current_state;
                led->last_toggle = current_time;
            }
            break;
            
        default:
            break;
    }
    
    /* Update hardware */
#ifdef USE_HAL_DRIVER
    HAL_GPIO_WritePin(led->port, led->pin, led->current_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

/**
 * @brief Set LED state
 */
static void aks_monitoring_set_led_state(aks_led_control_t* led, aks_led_state_t state)
{
    if (led == NULL) return;
    
    led->state = state;
    
    /* Adjust blink period based on state */
    switch (state) {
        case AKS_LED_STATE_BLINK_SLOW:
            led->blink_period = g_monitoring_handle.config.blink_period_ms;
            break;
        case AKS_LED_STATE_BLINK_FAST:
            led->blink_period = g_monitoring_handle.config.blink_period_ms / 2;
            break;
        default:
            break;
    }
}

/**
 * @brief Update system status
 */
static void aks_monitoring_update_system_status(aks_system_id_t system_id, aks_system_status_t status, uint16_t fault_code)
{
    if (system_id >= AKS_SYSTEM_COUNT) return;
    
    g_monitoring_handle.systems[system_id].status = status;
    g_monitoring_handle.systems[system_id].fault_code = fault_code;
    g_monitoring_handle.systems[system_id].last_heartbeat = HAL_GetTick();
}
