/**
 * @file main.c
 * @brief Main application entry point for AKS (Vehicle Control System)
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include "aks_can.h"
#include "aks_telemetry.h"
#include "aks_safety.h"

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

/* System Configuration */
static aks_config_t system_config = {
    .system_frequency = 84000000,    /* 84 MHz system clock */
    .can_baudrate = 500000,          /* 500 kbps CAN */
    .uart_baudrate = 115200,         /* 115200 bps UART */
    .adc_resolution = 12,            /* 12-bit ADC */
    .node_id = 1,                    /* Node ID */
    .debug_enabled = true            /* Debug mode */
};

/* Telemetry Configuration */
static aks_lora_config_t lora_config = {
    .frequency = AKS_LORA_FREQ_868MHZ,
    .spreading_factor = AKS_LORA_SPREADING_FACTOR,
    .bandwidth = AKS_LORA_BANDWIDTH,
    .coding_rate = AKS_LORA_CODING_RATE,
    .tx_power = 14,
    .crc_enabled = true,
    .preamble_length = 8,
    .sync_word = 0x12
};

static aks_wifi_config_t wifi_config = {
    .ssid = "AKS_Telemetry",
    .password = "aks_2025_secure",
    .server_url = "http://telemetry.aks.local",
    .server_port = 8080,
    .keepalive_interval = 30000,
    .max_retry = 3,
    .use_ssl = false
};

static aks_gps_config_t gps_config = {
    .baudrate = 9600,
    .update_rate = 1,
    .enable_glonass = true,
    .enable_galileo = false,
    .fix_timeout = AKS_GPS_FIX_TIMEOUT_MS
};

/* Safety Configuration */
static aks_safety_config_t safety_config = {
    .limits = {
        .min_battery_voltage = 60.0f,
        .max_battery_voltage = 80.0f,
        .max_battery_current = 100.0f,
        .max_battery_temp = AKS_MAX_BATTERY_TEMP,
        .max_motor_temp = AKS_MAX_MOTOR_TEMP,
        .min_isolation_resistance = AKS_ISOLATION_MIN_RESISTANCE,
        .emergency_timeout = AKS_EMERGENCY_STOP_TIMEOUT
    },
    .isolation_monitoring_enabled = true,
    .temperature_monitoring_enabled = true,
    .voltage_monitoring_enabled = true,
    .current_monitoring_enabled = true,
    .monitoring_interval = 100,
    .fault_log_size = 32
};

/* Function Prototypes */
static void system_clock_config(void);
static void error_handler(void);
static void system_error_callback(aks_result_t error);
static aks_result_t initialize_peripherals(void);
static aks_result_t run_system_tasks(void);

/**
 * @brief Main application entry point
 * @return Should not return
 */
int main(void)
{
    aks_result_t result;

#ifdef USE_HAL_DRIVER
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    system_clock_config();
#endif

    /* Initialize AKS core system */
    result = aks_core_init(&system_config);
    if (result != AKS_OK) {
        error_handler();
    }

    /* Register error callback */
    aks_core_register_error_callback(system_error_callback);

    /* Initialize peripherals and subsystems */
    result = initialize_peripherals();
    if (result != AKS_OK) {
        error_handler();
    }

    /* Start the system */
    result = aks_core_start();
    if (result != AKS_OK) {
        error_handler();
    }

    /* Main application loop */
    while (1) {
        /* Run system tasks */
        result = run_system_tasks();
        if (result != AKS_OK) {
            /* Log error but continue operation */
            system_error_callback(result);
        }

        /* Refresh watchdog */
        aks_core_watchdog_refresh();

        /* Small delay to prevent CPU overload */
        aks_core_delay(1);
    }
}

/**
 * @brief Initialize all peripherals and subsystems
 * @return AKS_OK on success, error code otherwise
 */
static aks_result_t initialize_peripherals(void)
{
    aks_result_t result;

    /* Initialize CAN communication */
    result = aks_can_init(system_config.can_baudrate, NULL, 0);
    if (result != AKS_OK) {
        return result;
    }

    /* Initialize telemetry system */
    result = aks_telemetry_init(&lora_config, &wifi_config, &gps_config);
    if (result != AKS_OK) {
        return result;
    }

    /* Initialize safety monitoring */
    result = aks_safety_init(&safety_config);
    if (result != AKS_OK) {
        return result;
    }

    /* Start CAN communication */
    result = aks_can_start();
    if (result != AKS_OK) {
        return result;
    }

    /* Start telemetry transmission */
    result = aks_telemetry_start(AKS_TEL_CHANNEL_BOTH, 1000);
    if (result != AKS_OK) {
        return result;
    }

    /* Start safety monitoring */
    result = aks_safety_start_monitoring();
    if (result != AKS_OK) {
        return result;
    }

    /* Set vehicle ID for telemetry */
    aks_telemetry_set_vehicle_id(1);

    return AKS_OK;
}

/**
 * @brief Run periodic system tasks
 * @return AKS_OK on success, error code otherwise
 */
static aks_result_t run_system_tasks(void)
{
    aks_result_t result;

    /* Run core system task */
    result = aks_core_task();
    if (result != AKS_OK) {
        return result;
    }

    /* Run telemetry task */
    result = aks_telemetry_task();
    if (result != AKS_OK) {
        return result;
    }

    /* Run safety monitoring task */
    result = aks_safety_task();
    if (result != AKS_OK) {
        return result;
    }

    /* Update vehicle telemetry data */
    static uint32_t telemetry_counter = 0;
    if (++telemetry_counter >= 100) { /* Update every 100ms */
        telemetry_counter = 0;
        
        /* Get current vehicle data and update telemetry */
        float speed = 45.5f;           /* Example: Get from speed sensor */
        float battery_voltage = 72.4f; /* Example: Get from battery monitoring */
        float battery_soc = 85.2f;     /* Example: Get from BMS */
        float motor_temp = 42.1f;      /* Example: Get from temperature sensor */
        
        aks_telemetry_update_vehicle_data(speed, battery_voltage, battery_soc, motor_temp);
    }

    return AKS_OK;
}

/**
 * @brief System error callback
 * @param error Error code
 */
static void system_error_callback(aks_result_t error)
{
    /* Log error to safety system */
    aks_safety_log_fault(AKS_FAULT_SYSTEM_OVERLOAD, (float)error, AKS_ACTION_WARNING);
    
    /* Handle critical errors */
    if (error == AKS_ERROR_TIMEOUT || error == AKS_ERROR_CRC) {
        /* Trigger emergency procedures if needed */
        aks_safety_emergency_stop(AKS_FAULT_COMMUNICATION_LOSS);
    }
}

#ifdef USE_HAL_DRIVER
/**
 * @brief System Clock Configuration
 */
static void system_clock_config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the RCC Oscillators according to the specified parameters */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        error_handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        error_handler();
    }
}
#endif

/**
 * @brief Error handler
 */
static void error_handler(void)
{
    /* Disable interrupts */
    __disable_irq();
    
    /* Reset the system */
    aks_core_reset(0xFF);
    
    /* Infinite loop in case reset fails */
    while (1) {
        /* Do nothing */
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 *        where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    error_handler();
}
#endif /* USE_FULL_ASSERT */