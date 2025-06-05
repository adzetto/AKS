/**
 * @file aks_pin_definitions.h
 * @brief Comprehensive pin definitions for AKS Vehicle Control System
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 * 
 * This file contains all pin definitions for the complete AKS system including:
 * - Main AKS Controller (STM32F407VET6TR)
 * - Motor Controller (STM32F103C8T6)
 * - Battery Management System (ESP32 + LTC6811-1)
 * - Onboard Charger (STM32F103C6T6)
 * - Isolation Monitoring (STM32F103)
 * - Telemetry System (ESP8266, LoRa, GPS)
 * - ADAS System (Raspberry Pi 5)
 */

#ifndef AKS_PIN_DEFINITIONS_H
#define AKS_PIN_DEFINITIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*==============================================================================
 * AKS MAIN CONTROLLER (STM32F407VET6TR)
 * Main Vehicle Control System - Central ECU
 *============================================================================*/

/* CAN Bus Interface */
#define AKS_MAIN_CAN1_TX_PIN            GPIO_PIN_9    /* PA9 - CAN1 TX */
#define AKS_MAIN_CAN1_TX_PORT           GPIOA
#define AKS_MAIN_CAN1_RX_PIN            GPIO_PIN_8    /* PA8 - CAN1 RX */
#define AKS_MAIN_CAN1_RX_PORT           GPIOA

#define AKS_MAIN_CAN2_TX_PIN            GPIO_PIN_13   /* PB13 - CAN2 TX */
#define AKS_MAIN_CAN2_TX_PORT           GPIOB
#define AKS_MAIN_CAN2_RX_PIN            GPIO_PIN_12   /* PB12 - CAN2 RX */
#define AKS_MAIN_CAN2_RX_PORT           GPIOB

/* D-Sub Connectors (5x CAN interfaces) */
#define AKS_MAIN_CAN_CS_1_PIN           GPIO_PIN_0    /* PC0 - CAN Chip Select 1 */
#define AKS_MAIN_CAN_CS_2_PIN           GPIO_PIN_1    /* PC1 - CAN Chip Select 2 */
#define AKS_MAIN_CAN_CS_3_PIN           GPIO_PIN_2    /* PC2 - CAN Chip Select 3 */
#define AKS_MAIN_CAN_CS_4_PIN           GPIO_PIN_3    /* PC3 - CAN Chip Select 4 */
#define AKS_MAIN_CAN_CS_5_PIN           GPIO_PIN_4    /* PC4 - CAN Chip Select 5 */
#define AKS_MAIN_CAN_CS_PORT            GPIOC

/* Power Relays (2x 12V 120A) */
#define AKS_MAIN_RELAY_MOTOR_PIN        GPIO_PIN_5    /* PC5 - Motor Relay Control */
#define AKS_MAIN_RELAY_CHARGER_PIN      GPIO_PIN_6    /* PC6 - Charger Relay Control */
#define AKS_MAIN_RELAY_PORT             GPIOC

/* Telemetry Module Interfaces */
#define AKS_MAIN_ESP8266_TX_PIN         GPIO_PIN_2    /* PA2 - ESP8266 UART TX */
#define AKS_MAIN_ESP8266_RX_PIN         GPIO_PIN_3    /* PA3 - ESP8266 UART RX */
#define AKS_MAIN_ESP8266_RST_PIN        GPIO_PIN_7    /* PC7 - ESP8266 Reset */
#define AKS_MAIN_ESP8266_EN_PIN         GPIO_PIN_8    /* PC8 - ESP8266 Enable */

/* LoRa Module (SPI) */
#define AKS_MAIN_LORA_SPI_SCK_PIN       GPIO_PIN_5    /* PA5 - LoRa SPI SCK */
#define AKS_MAIN_LORA_SPI_MISO_PIN      GPIO_PIN_6    /* PA6 - LoRa SPI MISO */
#define AKS_MAIN_LORA_SPI_MOSI_PIN      GPIO_PIN_7    /* PA7 - LoRa SPI MOSI */
#define AKS_MAIN_LORA_CS_PIN            GPIO_PIN_9    /* PC9 - LoRa Chip Select */
#define AKS_MAIN_LORA_RST_PIN           GPIO_PIN_10   /* PC10 - LoRa Reset */
#define AKS_MAIN_LORA_DIO0_PIN          GPIO_PIN_11   /* PC11 - LoRa DIO0 */

/* GPS Module (UART) */
#define AKS_MAIN_GPS_TX_PIN             GPIO_PIN_10   /* PA10 - GPS UART TX */
#define AKS_MAIN_GPS_RX_PIN             GPIO_PIN_11   /* PA11 - GPS UART RX */
#define AKS_MAIN_GPS_PPS_PIN            GPIO_PIN_12   /* PC12 - GPS PPS Signal */

/* RTC Module (I2C) */
#define AKS_MAIN_RTC_SCL_PIN            GPIO_PIN_6    /* PB6 - RTC I2C SCL */
#define AKS_MAIN_RTC_SDA_PIN            GPIO_PIN_7    /* PB7 - RTC I2C SDA */
#define AKS_MAIN_RTC_INT_PIN            GPIO_PIN_13   /* PC13 - RTC Interrupt */

/* Temperature Sensors */
#define AKS_MAIN_TEMP_1_PIN             GPIO_PIN_0    /* PA0 - Temperature Sensor 1 (ADC) */
#define AKS_MAIN_TEMP_2_PIN             GPIO_PIN_1    /* PA1 - Temperature Sensor 2 (ADC) */

/* User Interface */
#define AKS_MAIN_USER_LED_PIN           GPIO_PIN_14   /* PC14 - User LED */
#define AKS_MAIN_USER_BTN_PIN           GPIO_PIN_15   /* PC15 - User Button */

/* Power Management */
#define AKS_MAIN_PWR_12V_EN_PIN         GPIO_PIN_0    /* PB0 - 12V Power Enable */
#define AKS_MAIN_PWR_5V_EN_PIN          GPIO_PIN_1    /* PB1 - 5V Power Enable */
#define AKS_MAIN_PWR_3V3_EN_PIN         GPIO_PIN_2    /* PB2 - 3.3V Power Enable */

/*==============================================================================
 * MOTOR CONTROLLER (STM32F103C8T6)
 * BLDC Motor Driver with FOC Control
 *============================================================================*/

/* MOSFET PWM Outputs (3-Phase Inverter) */
#define MOTOR_PWM_UH_PIN                GPIO_PIN_8    /* PA8 - Phase U High-side PWM */
#define MOTOR_PWM_UL_PIN                GPIO_PIN_7    /* PA7 - Phase U Low-side PWM */
#define MOTOR_PWM_VH_PIN                GPIO_PIN_9    /* PA9 - Phase V High-side PWM */
#define MOTOR_PWM_VL_PIN                GPIO_PIN_0    /* PB0 - Phase V Low-side PWM */
#define MOTOR_PWM_WH_PIN                GPIO_PIN_10   /* PA10 - Phase W High-side PWM */
#define MOTOR_PWM_WL_PIN                GPIO_PIN_1    /* PB1 - Phase W Low-side PWM */

/* MOSFET Gate Drivers (IR2110) */
#define MOTOR_GATE_UH_PIN               GPIO_PIN_6    /* PA6 - Phase U High Gate */
#define MOTOR_GATE_UL_PIN               GPIO_PIN_5    /* PA5 - Phase U Low Gate */
#define MOTOR_GATE_VH_PIN               GPIO_PIN_4    /* PA4 - Phase V High Gate */
#define MOTOR_GATE_VL_PIN               GPIO_PIN_3    /* PA3 - Phase V Low Gate */
#define MOTOR_GATE_WH_PIN               GPIO_PIN_2    /* PA2 - Phase W High Gate */
#define MOTOR_GATE_WL_PIN               GPIO_PIN_1    /* PA1 - Phase W Low Gate */

/* Current Sensors (ACS758ECB-200B-PFF-T) */
#define MOTOR_CURRENT_U_PIN             GPIO_PIN_0    /* PA0 - Phase U Current (ADC) */
#define MOTOR_CURRENT_V_PIN             GPIO_PIN_1    /* PA1 - Phase V Current (ADC) */
#define MOTOR_CURRENT_W_PIN             GPIO_PIN_2    /* PA2 - Phase W Current (ADC) */

/* Hall Sensors */
#define MOTOR_HALL_A_PIN                GPIO_PIN_3    /* PA3 - Hall Sensor A */
#define MOTOR_HALL_B_PIN                GPIO_PIN_4    /* PA4 - Hall Sensor B */
#define MOTOR_HALL_C_PIN                GPIO_PIN_5    /* PA5 - Hall Sensor C */

/* Motor Temperature Sensor */
#define MOTOR_TEMP_PIN                  GPIO_PIN_6    /* PA6 - Motor Temperature (ADC) */

/* CAN Communication */
#define MOTOR_CAN_TX_PIN                GPIO_PIN_12   /* PA12 - CAN TX */
#define MOTOR_CAN_RX_PIN                GPIO_PIN_11   /* PA11 - CAN RX */

/* Protection and Status */
#define MOTOR_FAULT_PIN                 GPIO_PIN_15   /* PA15 - Fault Output */
#define MOTOR_ENABLE_PIN                GPIO_PIN_14   /* PA14 - Motor Enable */
#define MOTOR_BRAKE_PIN                 GPIO_PIN_13   /* PA13 - Motor Brake */

/* Photocoupler Isolation (IS314W) */
#define MOTOR_ISO_TX_PIN                GPIO_PIN_9    /* PB9 - Isolated TX */
#define MOTOR_ISO_RX_PIN                GPIO_PIN_8    /* PB8 - Isolated RX */

/*==============================================================================
 * BATTERY MANAGEMENT SYSTEM (ESP32 + LTC6811-1)
 * Master: ESP32, Slave: LTC6811-1 (2x units)
 *============================================================================*/

/* ESP32 Master Controller Pins */
#define BMS_ESP32_SPI_SCK_PIN           18            /* SPI Clock */
#define BMS_ESP32_SPI_MOSI_PIN          23            /* SPI MOSI */
#define BMS_ESP32_SPI_MISO_PIN          19            /* SPI MISO */
#define BMS_ESP32_SPI_CS_PIN            5             /* SPI Chip Select */

/* CAN Communication */
#define BMS_ESP32_CAN_TX_PIN            21            /* CAN TX */
#define BMS_ESP32_CAN_RX_PIN            22            /* CAN RX */

/* Temperature Sensors (LM35DZ) */
#define BMS_ESP32_TEMP1_PIN             32            /* Temperature Sensor 1 (ADC) */
#define BMS_ESP32_TEMP2_PIN             33            /* Temperature Sensor 2 (ADC) */
#define BMS_ESP32_TEMP3_PIN             34            /* Temperature Sensor 3 (ADC) */

/* Fan Control */
#define BMS_ESP32_FAN_TOP_PIN           25            /* Top Fan Control */
#define BMS_ESP32_FAN_SIDE1_PIN         26            /* Side Fan 1 Control */
#define BMS_ESP32_FAN_SIDE2_PIN         27            /* Side Fan 2 Control */

/* Buzzer */
#define BMS_ESP32_BUZZER_PIN            4             /* Buzzer Control */

/* Status LEDs */
#define BMS_ESP32_LED_STATUS_PIN        2             /* Status LED */
#define BMS_ESP32_LED_FAULT_PIN         16            /* Fault LED */

/* Power Control */
#define BMS_ESP32_PWR_EN_PIN            17            /* Power Enable */

/* LTC6811-1 Slave Board Connections */
#define BMS_LTC_CELL_COUNT              19            /* Cells per LTC6811-1 */
#define BMS_LTC_BOARDS                  2             /* Number of LTC6811-1 boards */

/* Current Sensor (ACS758ECB) */
#define BMS_CURRENT_SENSE_PIN           35            /* Current Sensor (ADC) */

/*==============================================================================
 * ONBOARD CHARGER (STM32F103C6T6)
 * LLC Resonant Converter Topology
 *============================================================================*/

/* MOSFET PWM Outputs (Full Bridge) */
#define CHARGER_PWM_Q1_PIN              GPIO_PIN_8    /* PA8 - MOSFET Q1 PWM */
#define CHARGER_PWM_Q2_PIN              GPIO_PIN_9    /* PA9 - MOSFET Q2 PWM */
#define CHARGER_PWM_Q3_PIN              GPIO_PIN_10   /* PA10 - MOSFET Q3 PWM */
#define CHARGER_PWM_Q4_PIN              GPIO_PIN_11   /* PA11 - MOSFET Q4 PWM */

/* Gate Driver Controls */
#define CHARGER_GATE_DRV1_PIN           GPIO_PIN_0    /* PA0 - Gate Driver 1 */
#define CHARGER_GATE_DRV2_PIN           GPIO_PIN_1    /* PA1 - Gate Driver 2 */

/* Feedback and Sensing */
#define CHARGER_VOUT_SENSE_PIN          GPIO_PIN_2    /* PA2 - Output Voltage (ADC) */
#define CHARGER_IOUT_SENSE_PIN          GPIO_PIN_3    /* PA3 - Output Current (ADC) */
#define CHARGER_VIN_SENSE_PIN           GPIO_PIN_4    /* PA4 - Input Voltage (ADC) */
#define CHARGER_TEMP_SENSE_PIN          GPIO_PIN_5    /* PA5 - Temperature (ADC) */

/* AC Input Detection */
#define CHARGER_AC_DETECT_PIN           GPIO_PIN_6    /* PA6 - AC Presence Detection */

/* Transformer and Resonant Tank */
#define CHARGER_LLC_FREQ_PIN            GPIO_PIN_7    /* PA7 - LLC Frequency Control */

/* Protection and Control */
#define CHARGER_ENABLE_PIN              GPIO_PIN_12   /* PA12 - Charger Enable */
#define CHARGER_FAULT_PIN               GPIO_PIN_13   /* PA13 - Fault Output */

/* Communication with AKS */
#define CHARGER_CAN_TX_PIN              GPIO_PIN_15   /* PA15 - CAN TX */
#define CHARGER_CAN_RX_PIN              GPIO_PIN_14   /* PA14 - CAN RX */

/* Status Indicators */
#define CHARGER_LED_POWER_PIN           GPIO_PIN_0    /* PB0 - Power LED */
#define CHARGER_LED_CHARGING_PIN        GPIO_PIN_1    /* PB1 - Charging LED */
#define CHARGER_LED_FAULT_PIN           GPIO_PIN_2    /* PB2 - Fault LED */

/*==============================================================================
 * ISOLATION MONITORING (STM32F103)
 * Electrical Safety System
 *============================================================================*/

/* Differential Amplifier Inputs */
#define ISO_MON_VPOS_PIN                GPIO_PIN_0    /* PA0 - Positive Voltage (ADC) */
#define ISO_MON_VNEG_PIN                GPIO_PIN_1    /* PA1 - Negative Voltage (ADC) */

/* Reference and Control */
#define ISO_MON_VREF_PIN                GPIO_PIN_2    /* PA2 - Reference Voltage */
#define ISO_MON_ENABLE_PIN              GPIO_PIN_3    /* PA3 - Monitoring Enable */

/* Test Signal Generation */
#define ISO_MON_TEST_SIG_PIN            GPIO_PIN_4    /* PA4 - Test Signal Output */

/* Alert and Status */
#define ISO_MON_ALERT_PIN               GPIO_PIN_5    /* PA5 - Alert Output */
#define ISO_MON_STATUS_PIN              GPIO_PIN_6    /* PA6 - Status LED */

/* Communication */
#define ISO_MON_CAN_TX_PIN              GPIO_PIN_12   /* PA12 - CAN TX */
#define ISO_MON_CAN_RX_PIN              GPIO_PIN_11   /* PA11 - CAN RX */

/* Buzzer for Audio Alert */
#define ISO_MON_BUZZER_PIN              GPIO_PIN_8    /* PA8 - Buzzer Control */

/*==============================================================================
 * TELEMETRY SYSTEM
 * Multi-platform: ESP8266, LoRa (SX1276), GPS (NEO-6M)
 *============================================================================*/

/* ESP8266 WiFi Module */
#define TEL_ESP8266_TX_PIN              1             /* GPIO1 - UART TX */
#define TEL_ESP8266_RX_PIN              3             /* GPIO3 - UART RX */
#define TEL_ESP8266_RST_PIN             0             /* GPIO0 - Reset */
#define TEL_ESP8266_EN_PIN              2             /* GPIO2 - Enable */

/* LoRa Module (SX1276) - SPI Interface */
#define TEL_LORA_SPI_SCK_PIN            14            /* GPIO14 - SPI Clock */
#define TEL_LORA_SPI_MISO_PIN           12            /* GPIO12 - SPI MISO */
#define TEL_LORA_SPI_MOSI_PIN           13            /* GPIO13 - SPI MOSI */
#define TEL_LORA_CS_PIN                 15            /* GPIO15 - Chip Select */
#define TEL_LORA_RST_PIN                16            /* GPIO16 - Reset */
#define TEL_LORA_DIO0_PIN               4             /* GPIO4 - DIO0 Interrupt */
#define TEL_LORA_DIO1_PIN               5             /* GPIO5 - DIO1 */

/* GPS Module (NEO-6M) - UART Interface */
#define TEL_GPS_TX_PIN                  9             /* GPIO9 - GPS UART TX */
#define TEL_GPS_RX_PIN                  10            /* GPIO10 - GPS UART RX */
#define TEL_GPS_PPS_PIN                 6             /* GPIO6 - PPS Signal */

/* Data Storage (SD Card) */
#define TEL_SD_CS_PIN                   7             /* GPIO7 - SD Card CS */
#define TEL_SD_CD_PIN                   8             /* GPIO8 - SD Card Detect */

/* Status LEDs */
#define TEL_LED_WIFI_PIN                11            /* GPIO11 - WiFi Status */
#define TEL_LED_LORA_PIN                17            /* GPIO17 - LoRa Status */
#define TEL_LED_GPS_PIN                 18            /* GPIO18 - GPS Status */

/*==============================================================================
 * ADAS SYSTEM (Raspberry Pi 5)
 * Advanced Driver Assistance Systems
 *============================================================================*/

/* Camera Interfaces (USB) */
#define ADAS_FRONT_CAMERA_PORT          "/dev/video0" /* Front Camera */
#define ADAS_REAR_CAMERA_PORT           "/dev/video1" /* Rear Camera */

/* Light Sensor (BH1750) - I2C */
#define ADAS_LIGHT_SENSOR_I2C_BUS       1             /* I2C Bus 1 */
#define ADAS_LIGHT_SENSOR_ADDR          0x23          /* I2C Address */

/* Radar Modules (RD-03D) - UART */
#define ADAS_RADAR_LEFT_PORT            "/dev/ttyUSB0" /* Left Radar */
#define ADAS_RADAR_RIGHT_PORT           "/dev/ttyUSB1" /* Right Radar */

/* Display Interface */
#define ADAS_DISPLAY_HDMI_PORT          0             /* HDMI Port 0 */

/* GPIO Pins (Raspberry Pi 5) */
#define ADAS_GPIO_STATUS_LED            18            /* Status LED */
#define ADAS_GPIO_FAULT_LED             19            /* Fault LED */
#define ADAS_GPIO_BUZZER                20            /* Buzzer */
#define ADAS_GPIO_USER_BTN              21            /* User Button */

/* Communication with AKS */
#define ADAS_UART_AKS_PORT              "/dev/ttyAMA0" /* UART to AKS */
#define ADAS_CAN_INTERFACE              "can0"        /* CAN Interface */

/*==============================================================================
 * SYSTEM-WIDE DEFINITIONS
 *============================================================================*/

/* CAN Bus IDs */
#define CAN_ID_AKS_STATUS               0x100
#define CAN_ID_MOTOR_STATUS             0x200
#define CAN_ID_MOTOR_CONTROL            0x210
#define CAN_ID_BMS_STATUS               0x300
#define CAN_ID_BMS_CELL_DATA            0x310
#define CAN_ID_CHARGER_STATUS           0x400
#define CAN_ID_ISO_MONITOR              0x500
#define CAN_ID_TELEMETRY                0x600
#define CAN_ID_ADAS_STATUS              0x700
#define CAN_ID_ADAS_WARNINGS            0x720

/* Power Supply Voltages */
#define SYSTEM_VOLTAGE_12V              12.0f
#define SYSTEM_VOLTAGE_5V               5.0f
#define SYSTEM_VOLTAGE_3V3              3.3f
#define BATTERY_NOMINAL_VOLTAGE         68.97f
#define BATTERY_MAX_VOLTAGE             79.8f

/* Safety Thresholds */
#define ISOLATION_MIN_RESISTANCE        50000         /* 50kΩ */
#define ISOLATION_WARNING_RESISTANCE    200000        /* 200kΩ */
#define ISOLATION_GOOD_RESISTANCE       1000000       /* 1MΩ */
#define MAX_BATTERY_TEMP                60.0f         /* °C */
#define MAX_MOTOR_TEMP                  85.0f         /* °C */

/* Communication Speeds */
#define CAN_SPEED_500K                  500000
#define UART_SPEED_115200               115200
#define UART_SPEED_9600                 9600
#define SPI_SPEED_1MHZ                  1000000
#define I2C_SPEED_100KHZ                100000

/* ADC Resolution */
#define ADC_RESOLUTION_12BIT            4096
#define ADC_RESOLUTION_10BIT            1024

/* PWM Settings */
#define PWM_FREQUENCY_20KHZ             20000
#define PWM_FREQUENCY_25KHZ             25000
#define PWM_RESOLUTION_10BIT            1024

/*==============================================================================
 * FUNCTION MACROS
 *============================================================================*/

/* GPIO Macros */
#define AKS_GPIO_SET(port, pin)         HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define AKS_GPIO_RESET(port, pin)       HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define AKS_GPIO_READ(port, pin)        HAL_GPIO_ReadPin(port, pin)
#define AKS_GPIO_TOGGLE(port, pin)      HAL_GPIO_TogglePin(port, pin)

/* ADC Conversion Macros */
#define ADC_TO_VOLTAGE(adc_val)         ((adc_val * 3.3f) / ADC_RESOLUTION_12BIT)
#define VOLTAGE_TO_TEMP_LM35(voltage)   (voltage * 100.0f)  /* LM35: 10mV/°C */

/* CAN Macros */
#define CAN_STD_ID(id)                  ((id) & 0x7FF)
#define CAN_EXT_ID(id)                  ((id) & 0x1FFFFFFF)

#ifdef __cplusplus
}
#endif

#endif /* AKS_PIN_DEFINITIONS_H */