/**
 * @file aks_main.c
 * @brief Main AKS Controller Implementation (STM32F407VET6TR)
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_main.h"
#include "aks_pin_definitions.h"
#include "aks_core.h"
#include "aks_can.h"
#include "aks_telemetry.h"
#include "aks_safety.h"

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* Private variables */
static aks_main_state_t main_state = AKS_MAIN_STATE_INIT;
static aks_main_config_t main_config;
static aks_main_telemetry_t telemetry_data;
static uint32_t last_heartbeat_time = 0;
static uint32_t system_uptime = 0;
static uint32_t last_led_toggle_time = 0;
static bool all_systems_ok = false;

/* Subsystem status */
static aks_subsystem_status_t subsystem_status = {
    .motor_controller = false,
    .bms = false,
    .charger = false,
    .isolation_monitor = false,
    .telemetry = false,
    .adas = false
};

/* Private function prototypes */
static aks_result_t init_gpio(void);
static aks_result_t init_can_interfaces(void);
static aks_result_t init_telemetry_modules(void);
static aks_result_t init_power_management(void);
static aks_result_t monitor_subsystems(void);
static aks_result_t process_can_messages(void);
static aks_result_t update_telemetry(void);
static aks_result_t handle_power_relays(void);
static void handle_system_fault(aks_fault_code_t fault_code);
static void send_heartbeat(void);
static void update_system_status_led(void);
static bool check_all_systems_status(void);

/**
 * @brief Initialize AKS Main Controller
 * @param config Pointer to main configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_init(const aks_main_config_t* config)
{
    aks_result_t result;
    
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Copy configuration */
    main_config = *config;
    
    /* Initialize HAL */
#ifdef USE_HAL_DRIVER
    HAL_Init();
    
    /* Configure system clock to 168MHz */
    SystemClock_Config();
#endif
    
    /* Initialize GPIO */
    result = init_gpio();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize power management */
    result = init_power_management();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize CAN interfaces */
    result = init_can_interfaces();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize telemetry modules */
    result = init_telemetry_modules();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize safety system */
    result = aks_safety_init(&main_config.safety_config);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize security system */
    /* Security system implementation for production use */
#ifdef AKS_ENABLE_SECURITY
    aks_security_config_t security_config = {
        .memory_protection_enabled = true,
        .stack_protection_enabled = true,
        .flash_integrity_check = true,
        .secure_boot_enabled = true,
        .watchdog_enabled = true,
        .auto_recovery_enabled = true,
        .periodic_flash_check = true,
        .max_violations = 5,
        .check_interval_ms = 1000
    };
    
    result = aks_security_init(&security_config);
    if (result != AKS_OK) {
        /* Log security initialization failure but continue */
        aks_safety_log_fault(AKS_FAULT_SYSTEM_OVERLOAD, 0.0f, AKS_ACTION_WARNING);
    }
#endif
    
    main_state = AKS_MAIN_STATE_READY;
    system_uptime = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Start AKS Main Controller
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_start(void)
{
    if (main_state != AKS_MAIN_STATE_READY) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Start safety monitoring */
    aks_result_t result = aks_safety_start_monitoring();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Enable power relays */
    AKS_GPIO_SET(AKS_MAIN_RELAY_PORT, AKS_MAIN_RELAY_MOTOR_PIN);
    
    /* Start telemetry */
    result = aks_telemetry_start(AKS_TEL_CHANNEL_BOTH, 1000);
    if (result != AKS_OK) {
        return result;
    }
    
    main_state = AKS_MAIN_STATE_RUNNING;
    last_heartbeat_time = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Main task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_task(void)
{
    if (main_state != AKS_MAIN_STATE_RUNNING) {
        return AKS_ERROR_NOT_READY;
    }
    
    aks_result_t result;
    uint32_t current_time = aks_core_get_tick();
    
    /* Monitor subsystems */
    result = monitor_subsystems();
    if (result != AKS_OK) {
        handle_system_fault(AKS_FAULT_COMMUNICATION_LOSS);
    }
    
    /* Process CAN messages */
    result = process_can_messages();
    if (result != AKS_OK) {
        /* Log but continue */
    }
    
    /* Update telemetry data */
    result = update_telemetry();
    if (result != AKS_OK) {
        /* Log but continue */
    }
    
    /* Handle power relay control */
    result = handle_power_relays();
    if (result != AKS_OK) {
        handle_system_fault(AKS_FAULT_ACTUATOR_FAILURE);
    }
    
    /* Run safety monitoring */
    result = aks_safety_task();
    if (result != AKS_OK) {
        handle_system_fault(AKS_FAULT_SYSTEM_OVERLOAD);
    }
    
    /* Run security monitoring */
    #ifdef AKS_ENABLE_SECURITY_MONITORING
    result = aks_security_task();
    if (result != AKS_OK) {
        /* Security violation detected - immediate emergency stop */
        aks_main_emergency_stop(AKS_FAULT_SECURITY_VIOLATION);
        handle_system_fault(AKS_FAULT_SECURITY_VIOLATION);
    }
    #endif /* AKS_ENABLE_SECURITY_MONITORING */
    
    /* Send heartbeat every second */
    if ((current_time - last_heartbeat_time) >= 1000) {
        send_heartbeat();
        last_heartbeat_time = current_time;
    }
    
    /* Update system status LED */
    update_system_status_led();
    
    return AKS_OK;
}

/**
 * @brief Handle emergency stop
 * @param reason Emergency stop reason
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_emergency_stop(aks_fault_code_t reason)
{
    /* Immediately disable power relays */
    AKS_GPIO_RESET(AKS_MAIN_RELAY_PORT, AKS_MAIN_RELAY_MOTOR_PIN);
    AKS_GPIO_RESET(AKS_MAIN_RELAY_PORT, AKS_MAIN_RELAY_CHARGER_PIN);
    
    /* Trigger safety system emergency stop */
    aks_safety_emergency_stop(reason);
    
    /* Send emergency CAN message */
    aks_can_frame_t emergency_frame = {
        .id = AKS_CAN_ID_EMERGENCY,
        .type = AKS_CAN_FRAME_STD,
        .dlc = 8,
        .data = {0xFF, 0xFF, (uint8_t)reason, 0x00, 0x00, 0x00, 0x00, 0x00},
        .timestamp = aks_core_get_tick(),
        .rtr = false
    };
    
    aks_can_transmit(&emergency_frame, 100);
    
    main_state = AKS_MAIN_STATE_FAULT;
    
    return AKS_OK;
}

/**
 * @brief Get AKS Main status
 * @param status Pointer to status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_main_get_status(aks_main_status_t* status)
{
    if (status == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    status->state = main_state;
    status->uptime = aks_core_get_tick() - system_uptime;
    status->subsystem_status = subsystem_status;
    status->power_12v_enabled = AKS_GPIO_READ(AKS_MAIN_RELAY_PORT, AKS_MAIN_PWR_12V_EN_PIN);
    status->power_5v_enabled = AKS_GPIO_READ(AKS_MAIN_RELAY_PORT, AKS_MAIN_PWR_5V_EN_PIN);
    status->motor_relay_enabled = AKS_GPIO_READ(AKS_MAIN_RELAY_PORT, AKS_MAIN_RELAY_MOTOR_PIN);
    status->charger_relay_enabled = AKS_GPIO_READ(AKS_MAIN_RELAY_PORT, AKS_MAIN_RELAY_CHARGER_PIN);
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Initialize GPIO pins
 */
static aks_result_t init_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure relay control pins */
    GPIO_InitStruct.Pin = AKS_MAIN_RELAY_MOTOR_PIN | AKS_MAIN_RELAY_CHARGER_PIN |
                          AKS_MAIN_PWR_12V_EN_PIN | AKS_MAIN_PWR_5V_EN_PIN | AKS_MAIN_PWR_3V3_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AKS_MAIN_RELAY_PORT, &GPIO_InitStruct);
    
    /* Configure CAN chip select pins */
    GPIO_InitStruct.Pin = AKS_MAIN_CAN_CS_1_PIN | AKS_MAIN_CAN_CS_2_PIN | 
                          AKS_MAIN_CAN_CS_3_PIN | AKS_MAIN_CAN_CS_4_PIN | AKS_MAIN_CAN_CS_5_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AKS_MAIN_CAN_CS_PORT, &GPIO_InitStruct);
    
    /* Configure status LED */
    GPIO_InitStruct.Pin = AKS_MAIN_USER_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    /* Configure user button */
    GPIO_InitStruct.Pin = AKS_MAIN_USER_BTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    return AKS_OK;
}

/**
 * @brief Initialize CAN interfaces
 */
static aks_result_t init_can_interfaces(void)
{
    /* Initialize main CAN buses */
    aks_result_t result = aks_can_init(CAN_SPEED_500K, NULL, 0);
    if (result != AKS_OK) {
        return result;
    }
    
    return aks_can_start();
}

/**
 * @brief Initialize telemetry modules
 */
static aks_result_t init_telemetry_modules(void)
{
    aks_lora_config_t lora_config = {
        .frequency = AKS_LORA_FREQ_868MHZ,
        .spreading_factor = AKS_LORA_SPREADING_FACTOR,
        .bandwidth = AKS_LORA_BANDWIDTH,
        .coding_rate = AKS_LORA_CODING_RATE,
        .tx_power = 14,
        .crc_enabled = true,
        .preamble_length = 8,
        .sync_word = 0x12
    };
    
    aks_wifi_config_t wifi_config = {
        .ssid = "AKS_Telemetry",
        .password = "aks_2025_secure",
        .server_url = "http://telemetry.aks.local",
        .server_port = 8080,
        .keepalive_interval = 30000,
        .max_retry = 3,
        .use_ssl = false
    };
    
    aks_gps_config_t gps_config = {
        .baudrate = 9600,
        .update_rate = 1,
        .enable_glonass = true,
        .enable_galileo = false,
        .fix_timeout = AKS_GPS_FIX_TIMEOUT_MS
    };
    
    return aks_telemetry_init(&lora_config, &wifi_config, &gps_config);
}

/**
 * @brief Initialize power management
 */
static aks_result_t init_power_management(void)
{
    /* Enable main power supplies in sequence */
    AKS_GPIO_SET(AKS_MAIN_RELAY_PORT, AKS_MAIN_PWR_3V3_EN_PIN);
    HAL_Delay(10);
    
    AKS_GPIO_SET(AKS_MAIN_RELAY_PORT, AKS_MAIN_PWR_5V_EN_PIN);
    HAL_Delay(10);
    
    AKS_GPIO_SET(AKS_MAIN_RELAY_PORT, AKS_MAIN_PWR_12V_EN_PIN);
    HAL_Delay(100);
    
    return AKS_OK;
}

/**
 * @brief Monitor subsystem status
 */
static aks_result_t monitor_subsystems(void)
{
    /* Check for CAN messages from each subsystem */
    /* This would be implemented based on heartbeat messages */
    
    return AKS_OK;
}

/**
 * @brief Process incoming CAN messages
 */
static aks_result_t process_can_messages(void)
{
    aks_can_frame_t frame;
    
    while (aks_can_receive(&frame, 0) == AKS_OK) {
        switch (frame.id) {
            case AKS_CAN_ID_MOTOR_STATUS:
                subsystem_status.motor_controller = true;
                break;
                
            case AKS_CAN_ID_BATTERY_STATUS:
                subsystem_status.bms = true;
                break;
                
            case AKS_CAN_ID_VEHICLE_STATUS:
                subsystem_status.charger = true;
                break;
                
            case AKS_CAN_ID_SAFETY_STATUS:
                subsystem_status.isolation_monitor = true;
                break;
                
            case AKS_CAN_ID_TELEMETRY:
                subsystem_status.telemetry = true;
                break;
                
            case AKS_CAN_ID_GPS_DATA:
                subsystem_status.adas = true;
                break;
                
            default:
                /* Unknown message */
                break;
        }
    }
    
    return AKS_OK;
}

/**
 * @brief Update telemetry data
 */
static aks_result_t update_telemetry(void)
{
    /* Update vehicle telemetry data */
    telemetry_data.vehicle_speed = 45.5f;  /* Get from speed sensor */
    telemetry_data.battery_voltage = 72.4f; /* Get from BMS */
    telemetry_data.battery_soc = 85.2f;     /* Get from BMS */
    telemetry_data.motor_temp = 42.1f;      /* Get from temperature sensor */
    telemetry_data.timestamp = aks_core_get_tick();
    
    /* Send to telemetry system */
    return aks_telemetry_update_vehicle_data(
        telemetry_data.vehicle_speed,
        telemetry_data.battery_voltage,
        telemetry_data.battery_soc,
        telemetry_data.motor_temp
    );
}

/**
 * @brief Handle power relay control
 */
static aks_result_t handle_power_relays(void)
{
    /* Check if emergency stop is active */
    if (aks_safety_is_emergency_active()) {
        AKS_GPIO_RESET(AKS_MAIN_RELAY_PORT, AKS_MAIN_RELAY_MOTOR_PIN);
        AKS_GPIO_RESET(AKS_MAIN_RELAY_PORT, AKS_MAIN_RELAY_CHARGER_PIN);
    }
    
    return AKS_OK;
}

/**
 * @brief Handle system fault
 */
static void handle_system_fault(aks_fault_code_t fault_code)
{
    /* Log fault */
    aks_safety_log_fault(fault_code, 0.0f, AKS_ACTION_WARNING);
    
    /* Take appropriate action based on fault severity */
    switch (fault_code) {
        case AKS_FAULT_COMMUNICATION_LOSS:
        case AKS_FAULT_SYSTEM_OVERLOAD:
            /* Non-critical faults */
            AKS_GPIO_SET(GPIOC, AKS_MAIN_USER_LED_PIN); /* Warning LED */
            break;
            
        case AKS_FAULT_EMERGENCY_STOP:
        case AKS_FAULT_ACTUATOR_FAILURE:
            /* Critical faults */
            aks_main_emergency_stop(fault_code);
            break;
            
        default:
            break;
    }
}

/**
 * @brief Send heartbeat message
 */
static void send_heartbeat(void)
{
    aks_can_frame_t heartbeat_frame = {
        .id = AKS_CAN_ID_VEHICLE_STATUS,
        .type = AKS_CAN_FRAME_STD,
        .dlc = 8,
        .data = {0x01, (uint8_t)main_state, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        .timestamp = aks_core_get_tick(),
        .rtr = false
    };
    
    aks_can_transmit(&heartbeat_frame, 100);
    
    /* Toggle status LED */
    AKS_GPIO_TOGGLE(GPIOC, AKS_MAIN_USER_LED_PIN);
}

/**
 * @brief System clock configuration for STM32F407VET6TR
 */
void SystemClock_Config(void)
{
#ifdef USE_HAL_DRIVER
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    /* Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
#endif
}

/**
 * @brief Check if all systems are operational
 */
static bool check_all_systems_status(void)
{
    return (subsystem_status.motor_controller &&
            subsystem_status.bms &&
            subsystem_status.charger &&
            subsystem_status.isolation_monitor &&
            subsystem_status.telemetry &&
            subsystem_status.adas &&
            main_state == AKS_MAIN_STATE_RUNNING &&
            !aks_safety_is_emergency_active());
}

/**
 * @brief Update system status LED based on all systems status
 */
static void update_system_status_led(void)
{
    uint32_t current_time = aks_core_get_tick();
    all_systems_ok = check_all_systems_status();
    
    if (all_systems_ok) {
        /* Fast blinking when all systems OK (100ms interval) */
        if ((current_time - last_led_toggle_time) >= 100) {
            AKS_GPIO_TOGGLE(GPIOC, AKS_MAIN_USER_LED_PIN);
            last_led_toggle_time = current_time;
        }
    } else if (main_state == AKS_MAIN_STATE_FAULT || main_state == AKS_MAIN_STATE_EMERGENCY) {
        /* Solid ON for fault/emergency states */
        AKS_GPIO_SET(GPIOC, AKS_MAIN_USER_LED_PIN);
    } else {
        /* Slow blinking for normal operation but not all systems OK (500ms interval) */
        if ((current_time - last_led_toggle_time) >= 500) {
            AKS_GPIO_TOGGLE(GPIOC, AKS_MAIN_USER_LED_PIN);
            last_led_toggle_time = current_time;
        }
    }
}

/**
 * @brief Error handler
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Error handling */
    }
}