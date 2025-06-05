/**
 * @file aks_types.h
 * @brief Core type definitions for AKS (Vehicle Control System)
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_TYPES_H
#define AKS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @defgroup AKS_Types Core Types
 * @brief Core type definitions and constants
 * @{
 */

/* Version Information */
#define AKS_VERSION_MAJOR   1
#define AKS_VERSION_MINOR   0
#define AKS_VERSION_PATCH   0
#define AKS_VERSION_STRING  "1.0.0"

/* System Configuration */
#define AKS_MAX_NODES       16
#define AKS_MAX_SENSORS     32
#define AKS_MAX_ACTUATORS   16
#define AKS_BUFFER_SIZE     256
#define AKS_QUEUE_SIZE      64

/* Return Codes */
typedef enum {
    AKS_OK                  = 0x00,
    AKS_ERROR              = 0x01,
    AKS_ERROR_INVALID_PARAM = 0x02,
    AKS_ERROR_TIMEOUT      = 0x03,
    AKS_ERROR_NOT_READY    = 0x04,
    AKS_ERROR_BUSY         = 0x05,
    AKS_ERROR_NO_MEMORY    = 0x06,
    AKS_ERROR_CRC          = 0x07,
    AKS_ERROR_OVERFLOW     = 0x08,
    AKS_ERROR_UNDERFLOW    = 0x09,
    AKS_ERROR_UNSUPPORTED  = 0x0A
} aks_result_t;

/* System States */
typedef enum {
    AKS_STATE_INIT         = 0x00,
    AKS_STATE_IDLE         = 0x01,
    AKS_STATE_READY        = 0x02,
    AKS_STATE_RUNNING      = 0x03,
    AKS_STATE_FAULT        = 0x04,
    AKS_STATE_EMERGENCY    = 0x05,
    AKS_STATE_SHUTDOWN     = 0x06
} aks_state_t;

/* Priority Levels */
typedef enum {
    AKS_PRIORITY_LOW       = 0,
    AKS_PRIORITY_NORMAL    = 1,
    AKS_PRIORITY_HIGH      = 2,
    AKS_PRIORITY_CRITICAL  = 3
} aks_priority_t;

/* Vehicle Control Commands */
typedef enum {
    AKS_CMD_STOP           = 0x00,
    AKS_CMD_START          = 0x01,
    AKS_CMD_ACCELERATE     = 0x02,
    AKS_CMD_BRAKE          = 0x03,
    AKS_CMD_TURN_LEFT      = 0x04,
    AKS_CMD_TURN_RIGHT     = 0x05,
    AKS_CMD_EMERGENCY_STOP = 0x06,
    AKS_CMD_REGEN_BRAKE    = 0x07
} aks_command_t;

/* Motor Control */
typedef struct {
    float torque_request;    /**< Requested torque in Nm */
    float speed_request;     /**< Requested speed in RPM */
    float current_limit;     /**< Current limit in A */
    uint8_t direction;       /**< 0: Forward, 1: Reverse */
    bool enable;            /**< Motor enable flag */
} aks_motor_control_t;

/* Battery Status */
typedef struct {
    float voltage;          /**< Battery voltage in V */
    float current;          /**< Battery current in A */
    float temperature;      /**< Battery temperature in °C */
    float soc;             /**< State of charge in % */
    float soh;             /**< State of health in % */
    uint16_t cell_count;   /**< Number of cells */
    bool fault_detected;   /**< Fault flag */
} aks_battery_status_t;

/* Vehicle Telemetry */
typedef struct {
    float speed;           /**< Vehicle speed in km/h */
    float acceleration;    /**< Acceleration in m/s² */
    float steering_angle;  /**< Steering angle in degrees */
    float brake_pressure;  /**< Brake pressure in bar */
    uint32_t odometer;     /**< Total distance in meters */
    uint32_t timestamp;    /**< Unix timestamp */
} aks_vehicle_telemetry_t;

/* GPS Data */
typedef struct {
    double latitude;       /**< Latitude in degrees */
    double longitude;      /**< Longitude in degrees */
    float altitude;        /**< Altitude in meters */
    float speed;          /**< GPS speed in km/h */
    float heading;        /**< Heading in degrees */
    uint8_t satellites;   /**< Number of satellites */
    bool fix_valid;       /**< GPS fix validity */
    uint32_t timestamp;   /**< GPS timestamp */
} aks_gps_data_t;

/* Safety Status */
typedef struct {
    bool emergency_stop;   /**< Emergency stop activated */
    bool door_open;       /**< Door status */
    bool seatbelt;        /**< Seatbelt status */
    bool isolation_fault; /**< Isolation monitoring fault */
    float isolation_resistance; /**< Isolation resistance in kΩ */
    uint8_t fault_code;   /**< Fault code */
} aks_safety_status_t;

/* Message Structure */
typedef struct {
    uint16_t id;          /**< Message ID */
    uint8_t length;       /**< Data length */
    uint8_t data[8];      /**< Message data */
    uint32_t timestamp;   /**< Message timestamp */
    aks_priority_t priority; /**< Message priority */
} aks_message_t;

/* Configuration Structure */
typedef struct {
    uint32_t system_frequency;     /**< System clock frequency */
    uint32_t can_baudrate;        /**< CAN bus baudrate */
    uint32_t uart_baudrate;       /**< UART baudrate */
    uint16_t adc_resolution;      /**< ADC resolution in bits */
    uint8_t node_id;             /**< Node ID for communication */
    bool debug_enabled;          /**< Debug mode flag */
} aks_config_t;

/* Function Pointer Types */
typedef void (*aks_callback_t)(void);
typedef void (*aks_error_callback_t)(aks_result_t error);
typedef void (*aks_message_callback_t)(const aks_message_t* message);

/* Utility Macros */
#define AKS_ARRAY_SIZE(arr)     (sizeof(arr) / sizeof((arr)[0]))
#define AKS_MIN(a, b)          ((a) < (b) ? (a) : (b))
#define AKS_MAX(a, b)          ((a) > (b) ? (a) : (b))
#define AKS_CLAMP(val, min, max) (AKS_MAX((min), AKS_MIN((val), (max))))

/* Bit Manipulation Macros */
#define AKS_SET_BIT(reg, bit)   ((reg) |= (1U << (bit)))
#define AKS_CLEAR_BIT(reg, bit) ((reg) &= ~(1U << (bit)))
#define AKS_TOGGLE_BIT(reg, bit) ((reg) ^= (1U << (bit)))
#define AKS_READ_BIT(reg, bit)  (((reg) >> (bit)) & 1U)

/* Conversion Macros */
#define AKS_MS_TO_TICKS(ms)     ((ms) * 1000)
#define AKS_TICKS_TO_MS(ticks)  ((ticks) / 1000)
#define AKS_DEG_TO_RAD(deg)     ((deg) * 0.017453292519943295)
#define AKS_RAD_TO_DEG(rad)     ((rad) * 57.295779513082320876)

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_TYPES_H */