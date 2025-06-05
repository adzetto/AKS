/**
 * @file aks_config.h
 * @brief Comprehensive configuration header for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_CONFIG_H
#define AKS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup AKS_Config System Configuration
 * @brief System-wide configuration parameters
 * @{
 */

/* =============================================================================
 * SYSTEM CONFIGURATION
 * ============================================================================= */

/* System Clock Configuration */
#define AKS_SYSTEM_CLOCK_HZ         84000000    /**< System clock frequency (84 MHz) */
#define AKS_SYSTICK_FREQ_HZ         1000        /**< SysTick frequency (1 kHz) */
#define AKS_WATCHDOG_TIMEOUT_MS     5000        /**< Watchdog timeout */

/* Hardware Platform */
#define AKS_MCU_STM32F407           1           /**< STM32F407VET6TR MCU */
#define AKS_FLASH_SIZE_KB           512         /**< Flash memory size */
#define AKS_RAM_SIZE_KB             128         /**< RAM memory size */

/* Debug Configuration */
#define AKS_DEBUG_ENABLED           1           /**< Enable debug features */
#define AKS_DEBUG_UART_INSTANCE     1           /**< Debug UART instance */
#define AKS_DEBUG_UART_BAUDRATE     115200      /**< Debug UART baudrate */

/* =============================================================================
 * CAN BUS CONFIGURATION
 * ============================================================================= */

/* CAN Bus Parameters */
#define AKS_CAN_BAUDRATE            500000      /**< CAN baudrate (500 kbps) */
#define AKS_CAN_INSTANCE            1           /**< CAN instance (CAN1) */
#define AKS_CAN_TX_MAILBOX_COUNT    3           /**< TX mailboxes */
#define AKS_CAN_RX_FIFO_SIZE        64          /**< RX FIFO size */

/* CAN Message IDs */
#define AKS_CAN_ID_MOTOR_CMD        0x101       /**< Motor control command */
#define AKS_CAN_ID_MOTOR_STATUS     0x102       /**< Motor status */
#define AKS_CAN_ID_MOTOR_TELEMETRY  0x103       /**< Motor telemetry */
#define AKS_CAN_ID_MOTOR_FAULT      0x104       /**< Motor fault */

#define AKS_CAN_ID_BMS_STATUS       0x200       /**< BMS status */
#define AKS_CAN_ID_BMS_CELLS        0x201       /**< BMS cell voltages */
#define AKS_CAN_ID_BMS_TEMPS        0x202       /**< BMS temperatures */
#define AKS_CAN_ID_BMS_FAULT        0x203       /**< BMS fault */

#define AKS_CAN_ID_ISOLATION_STATUS 0x400       /**< Isolation monitoring */
#define AKS_CAN_ID_ISOLATION_MEAS   0x401       /**< Isolation measurement */
#define AKS_CAN_ID_ISOLATION_FAULT  0x402       /**< Isolation fault */

#define AKS_CAN_ID_TELEMETRY_DATA   0x500       /**< Telemetry data */
#define AKS_CAN_ID_GPS_DATA         0x501       /**< GPS data */
#define AKS_CAN_ID_SYSTEM_STATUS    0x502       /**< System status */

#define AKS_CAN_ID_CHARGER_STATUS   0x600       /**< Charger status */
#define AKS_CAN_ID_CHARGER_CMD      0x601       /**< Charger command */

#define AKS_CAN_ID_ADAS_DATA        0x700       /**< ADAS data */
#define AKS_CAN_ID_ADAS_WARNING     0x701       /**< ADAS warning */

/* =============================================================================
 * UART CONFIGURATION
 * ============================================================================= */

/* UART Instances */
#define AKS_UART_DEBUG              0           /**< Debug UART */
#define AKS_UART_GPS                1           /**< GPS UART */
#define AKS_UART_TELEMETRY          2           /**< Telemetry UART */
#define AKS_UART_SPARE              3           /**< Spare UART */

/* UART Parameters */
#define AKS_UART_DEFAULT_BAUDRATE   115200      /**< Default UART baudrate */
#define AKS_UART_BUFFER_SIZE        256         /**< UART buffer size */
#define AKS_UART_TIMEOUT_MS         1000        /**< UART timeout */

/* GPS UART Configuration */
#define AKS_GPS_UART_BAUDRATE       9600        /**< GPS module baudrate */
#define AKS_GPS_UPDATE_RATE_HZ      1           /**< GPS update rate */

/* =============================================================================
 * SPI CONFIGURATION
 * ============================================================================= */

/* SPI Instances */
#define AKS_SPI_LORA                0           /**< LoRa module SPI */
#define AKS_SPI_SD_CARD             1           /**< SD card SPI */
#define AKS_SPI_EXTERNAL            2           /**< External devices SPI */

/* SPI Parameters */
#define AKS_SPI_DEFAULT_SPEED       1000000     /**< Default SPI speed (1 MHz) */
#define AKS_SPI_LORA_SPEED          8000000     /**< LoRa SPI speed (8 MHz) */
#define AKS_SPI_SD_SPEED            25000000    /**< SD card SPI speed (25 MHz) */

/* =============================================================================
 * ADC CONFIGURATION
 * ============================================================================= */

/* ADC Parameters */
#define AKS_ADC_RESOLUTION_BITS     12          /**< ADC resolution */
#define AKS_ADC_VREF_MV             3300        /**< ADC reference voltage */
#define AKS_ADC_SAMPLES_PER_CH      16          /**< Oversampling count */
#define AKS_ADC_CONVERSION_TIME_US  100         /**< ADC conversion time */

/* ADC Channel Mapping */
#define AKS_ADC_CH_BATTERY_VOLTAGE  0           /**< Battery voltage (0-80V) */
#define AKS_ADC_CH_BATTERY_CURRENT  1           /**< Battery current (±50A) */
#define AKS_ADC_CH_MOTOR_TEMP       2           /**< Motor temperature */
#define AKS_ADC_CH_CONTROLLER_TEMP  3           /**< Controller temperature */
#define AKS_ADC_CH_ISOLATION_POS    4           /**< Isolation positive */
#define AKS_ADC_CH_ISOLATION_NEG    5           /**< Isolation negative */
#define AKS_ADC_CH_BRAKE_PRESSURE   6           /**< Brake pressure */
#define AKS_ADC_CH_THROTTLE_POS     7           /**< Throttle position */
#define AKS_ADC_CH_BRAKE_PEDAL      8           /**< Brake pedal position 1 */
#define AKS_ADC_CH_BRAKE_FORCE      9           /**< Brake pedal force */
#define AKS_ADC_CH_STEERING_POS_1   10          /**< Steering position 1 */
#define AKS_ADC_CH_STEERING_POS_2   11          /**< Steering position 2 */

/* ADC Channel Bases for Arrays */
#define AKS_ADC_CHANNEL_WHEEL_SPEED_BASE    12  /**< Wheel speed sensors base */
#define AKS_ADC_CHANNEL_BRAKE_TEMP_BASE     16  /**< Brake temperature base */
#define AKS_ADC_CH_BATTERY_TEMP_BASE        20  /**< Battery temp sensors base */
#define AKS_ADC_CH_BATTERY_CURRENT_BASE     36  /**< Battery current sensors base */
#define AKS_ADC_CH_BMS_TEMP_BASE            40  /**< BMS temperature base */

/* Additional Constants */
#define AKS_BATTERY_VOLTAGE_SCALE_FACTOR    1.0f    /**< Battery voltage scaling */
#define AKS_ISOLATION_SETTLING_TIME_MS      50      /**< Isolation settling time */
#define AKS_BMS_FAULT_VOLTAGE_SENSOR        (1<<0)  /**< BMS voltage sensor fault */
#define AKS_BMS_FAULT_TEMP_SENSOR           (1<<1)  /**< BMS temp sensor fault */
#define AKS_BMS_FAULT_COMMUNICATION         (1<<2)  /**< BMS communication fault */

/* ADC Scaling Factors */
#define AKS_ADC_BATTERY_VOLTAGE_SCALE   0.0244f /**< Battery voltage scale (80V/3.3V) */
#define AKS_ADC_BATTERY_CURRENT_SCALE   0.0305f /**< Current scale (±50A/±1.65V) */
#define AKS_ADC_TEMPERATURE_SCALE       0.01f   /**< Temperature scale (°C/mV) */

/* =============================================================================
 * MOTOR CONTROL CONFIGURATION
 * ============================================================================= */

/* Motor Specifications */
#define AKS_MOTOR_MAX_TORQUE_NM     200.0f      /**< Maximum motor torque */
#define AKS_MOTOR_MAX_SPEED_RPM     8000        /**< Maximum motor speed */
#define AKS_MOTOR_MAX_CURRENT_A     300.0f      /**< Maximum motor current */
#define AKS_MOTOR_POLE_PAIRS        4           /**< Motor pole pairs */

/* Motor Control Parameters */
#define AKS_MOTOR_PWM_FREQ_HZ       20000       /**< PWM frequency */
#define AKS_MOTOR_CONTROL_FREQ_HZ   10000       /**< Control loop frequency */
#define AKS_MOTOR_RAMP_RATE_NM_S    100.0f      /**< Torque ramp rate */

/* PID Controller Gains */
#define AKS_MOTOR_TORQUE_KP         1.0f        /**< Torque P gain */
#define AKS_MOTOR_TORQUE_KI         0.1f        /**< Torque I gain */
#define AKS_MOTOR_TORQUE_KD         0.01f       /**< Torque D gain */

#define AKS_MOTOR_SPEED_KP          0.5f        /**< Speed P gain */
#define AKS_MOTOR_SPEED_KI          0.05f       /**< Speed I gain */
#define AKS_MOTOR_SPEED_KD          0.005f      /**< Speed D gain */

/* Regenerative Braking */
#define AKS_REGEN_MAX_POWER_KW      50.0f       /**< Max regen power */
#define AKS_REGEN_MIN_SPEED_RPM     100         /**< Min speed for regen */
#define AKS_REGEN_EFFICIENCY        0.85f       /**< Regen efficiency */

/* =============================================================================
 * BATTERY MANAGEMENT CONFIGURATION
 * ============================================================================= */

/* Battery Pack Specifications */
#define AKS_BATTERY_NOMINAL_VOLTAGE 72.0f       /**< Nominal voltage (V) */
#define AKS_BATTERY_MIN_VOLTAGE     60.0f       /**< Minimum voltage (V) */
#define AKS_BATTERY_MAX_VOLTAGE     84.0f       /**< Maximum voltage (V) */
#define AKS_BATTERY_CAPACITY_AH     100.0f      /**< Battery capacity (Ah) */
#define AKS_BATTERY_CELL_COUNT      20          /**< Number of cells in series */

/* Battery Limits */
#define AKS_BATTERY_MAX_CURRENT_A   100.0f      /**< Maximum discharge current */
#define AKS_BATTERY_MAX_CHARGE_A    50.0f       /**< Maximum charge current */
#define AKS_BATTERY_MAX_TEMP_C      60.0f       /**< Maximum temperature */
#define AKS_BATTERY_MIN_TEMP_C      -20.0f      /**< Minimum temperature */

/* Cell Monitoring */
#define AKS_CELL_MAX_VOLTAGE_V      4.2f        /**< Maximum cell voltage */
#define AKS_CELL_MIN_VOLTAGE_V      3.0f        /**< Minimum cell voltage */
#define AKS_CELL_BALANCE_THRESHOLD  0.01f       /**< Balance threshold (V) */

/* =============================================================================
 * SAFETY SYSTEM CONFIGURATION
 * ============================================================================= */

/* Isolation Monitoring */
#define AKS_ISOLATION_MIN_RESISTANCE    50000   /**< Minimum isolation (Ω) */
#define AKS_ISOLATION_WARNING_LEVEL     200000  /**< Warning level (Ω) */
#define AKS_ISOLATION_GOOD_LEVEL        1000000 /**< Good level (Ω) */
#define AKS_ISOLATION_MEAS_FREQ_HZ      10      /**< Measurement frequency */

/* Emergency Stop */
#define AKS_EMERGENCY_TIMEOUT_MS        5000    /**< Emergency timeout */
#define AKS_EMERGENCY_DEBOUNCE_MS       50      /**< Button debounce time */

/* Safety Monitoring Intervals */
#define AKS_SAFETY_MONITOR_FREQ_HZ      100     /**< Safety monitoring frequency */
#define AKS_HEARTBEAT_FREQ_HZ           1       /**< Heartbeat frequency */
#define AKS_FAULT_CHECK_FREQ_HZ         10      /**< Fault check frequency */

/* =============================================================================
 * TELEMETRY CONFIGURATION
 * ============================================================================= */

/* LoRa Configuration */
#define AKS_LORA_FREQUENCY_HZ       868000000   /**< LoRa frequency (868 MHz) */
#define AKS_LORA_BANDWIDTH_HZ       125000      /**< LoRa bandwidth (125 kHz) */
#define AKS_LORA_SPREADING_FACTOR   12          /**< LoRa spreading factor */
#define AKS_LORA_CODING_RATE        5           /**< LoRa coding rate (4/5) */
#define AKS_LORA_TX_POWER_DBM       14          /**< LoRa TX power */
#define AKS_LORA_MAX_PAYLOAD        64          /**< LoRa max payload */

/* WiFi Configuration */
#define AKS_WIFI_SSID               "AKS_Vehicle"    /**< WiFi SSID */
#define AKS_WIFI_PASSWORD           "aks2025!"       /**< WiFi password */
#define AKS_WIFI_CHANNEL            6               /**< WiFi channel */
#define AKS_WIFI_MAX_CONNECTIONS    4               /**< Max WiFi connections */

/* Telemetry Data Rates */
#define AKS_TELEMETRY_FAST_RATE_HZ  10          /**< Fast telemetry rate */
#define AKS_TELEMETRY_SLOW_RATE_HZ  1           /**< Slow telemetry rate */
#define AKS_GPS_LOG_RATE_HZ         1           /**< GPS logging rate */

/* Data Storage */
#define AKS_LOG_BUFFER_SIZE         4096        /**< Log buffer size */
#define AKS_LOG_FILE_MAX_SIZE_MB    100         /**< Max log file size */
#define AKS_LOG_RETENTION_DAYS      7           /**< Log retention period */

/* =============================================================================
 * TIMING CONFIGURATION
 * ============================================================================= */

/* Task Frequencies */
#define AKS_TASK_MAIN_FREQ_HZ       100         /**< Main task frequency */
#define AKS_TASK_MOTOR_FREQ_HZ      1000        /**< Motor control task */
#define AKS_TASK_SAFETY_FREQ_HZ     100         /**< Safety task */
#define AKS_TASK_TELEMETRY_FREQ_HZ  10          /**< Telemetry task */
#define AKS_TASK_MONITORING_FREQ_HZ 1           /**< Monitoring task */

/* Timeout Values */
#define AKS_CAN_TIMEOUT_MS          100         /**< CAN timeout */
#define AKS_UART_TIMEOUT_MS         1000        /**< UART timeout */
#define AKS_SPI_TIMEOUT_MS          1000        /**< SPI timeout */
#define AKS_SYSTEM_INIT_TIMEOUT_MS  5000        /**< System init timeout */

/* =============================================================================
 * MEMORY CONFIGURATION
 * ============================================================================= */

/* Stack Sizes */
#define AKS_MAIN_STACK_SIZE         2048        /**< Main stack size */
#define AKS_IRQ_STACK_SIZE          512         /**< IRQ stack size */

/* Buffer Sizes */
#define AKS_CAN_TX_BUFFER_SIZE      32          /**< CAN TX buffer */
#define AKS_CAN_RX_BUFFER_SIZE      64          /**< CAN RX buffer */
#define AKS_UART_TX_BUFFER_SIZE     256         /**< UART TX buffer */
#define AKS_UART_RX_BUFFER_SIZE     256         /**< UART RX buffer */
#define AKS_TELEMETRY_BUFFER_SIZE   1024        /**< Telemetry buffer */

/* Queue Sizes */
#define AKS_EVENT_QUEUE_SIZE        16          /**< Event queue size */
#define AKS_MESSAGE_QUEUE_SIZE      32          /**< Message queue size */
#define AKS_LOG_QUEUE_SIZE          64          /**< Log queue size */

/* =============================================================================
 * FEATURE ENABLES
 * ============================================================================= */

/* Feature Flags */
#define AKS_ENABLE_MOTOR_CONTROL    1           /**< Enable motor control */
#define AKS_ENABLE_BMS              1           /**< Enable BMS */
#define AKS_ENABLE_ISOLATION        1           /**< Enable isolation monitoring */
#define AKS_ENABLE_TELEMETRY        1           /**< Enable telemetry */
#define AKS_ENABLE_GPS              1           /**< Enable GPS */
#define AKS_ENABLE_ADAS             1           /**< Enable ADAS */
#define AKS_ENABLE_CHARGER          1           /**< Enable onboard charger */
#define AKS_ENABLE_LOGGING          1           /**< Enable data logging */
#define AKS_ENABLE_DIAGNOSTICS      1           /**< Enable diagnostics */
#define AKS_ENABLE_CALIBRATION      1           /**< Enable calibration */

/* Debug Features */
#define AKS_ENABLE_DEBUG_UART       1           /**< Enable debug UART */
#define AKS_ENABLE_DEBUG_LED        1           /**< Enable debug LEDs */
#define AKS_ENABLE_PROFILING        1           /**< Enable profiling */
#define AKS_ENABLE_ASSERT           1           /**< Enable assertions */

/* =============================================================================
 * CALIBRATION VALUES
 * ============================================================================= */

/* ADC Calibration */
#define AKS_ADC_OFFSET_CORRECTION   0           /**< ADC offset correction */
#define AKS_ADC_GAIN_CORRECTION     1.0f        /**< ADC gain correction */

/* Current Sensor Calibration */
#define AKS_CURRENT_SENSOR_OFFSET   2048        /**< Current sensor zero offset */
#define AKS_CURRENT_SENSOR_SCALE    0.0244f     /**< Current sensor scale (A/count) */

/* Temperature Sensor Calibration */
#define AKS_TEMP_SENSOR_OFFSET      0.5f        /**< Temperature offset (°C) */
#define AKS_TEMP_SENSOR_SCALE       0.01f       /**< Temperature scale (°C/mV) */

/* =============================================================================
 * COMPETITION SPECIFIC SETTINGS
 * ============================================================================= */

/* 2025 Efficiency Challenge Settings */
#define AKS_COMPETITION_MODE        1           /**< Competition mode enable */
#define AKS_MAX_VEHICLE_SPEED_KMH   45          /**< Maximum vehicle speed */
#define AKS_EFFICIENCY_TARGET       150         /**< Target efficiency (km/kWh) */
#define AKS_TRACK_LENGTH_KM         10          /**< Track length */

/* Competition Requirements */
#define AKS_REQUIRED_ISOLATION_OHM  50000       /**< Required isolation resistance */
#define AKS_REQUIRED_EMERGENCY_STOP 1           /**< Emergency stop required */
#define AKS_REQUIRED_MONITORING     1           /**< System monitoring required */
#define AKS_REQUIRED_TELEMETRY      1           /**< Telemetry required */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_CONFIG_H */ 