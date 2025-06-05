/**
 * @file aks_can.h
 * @brief CAN bus communication for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_CAN_H
#define AKS_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"

/**
 * @defgroup AKS_CAN CAN Communication
 * @brief CAN bus communication interface
 * @{
 */

/* CAN Configuration */
#define AKS_CAN_STD_ID_MASK     0x7FF
#define AKS_CAN_EXT_ID_MASK     0x1FFFFFFF
#define AKS_CAN_MAX_DLC         8

/* Standard CAN IDs for AKS System */
#define AKS_CAN_ID_MOTOR_CTRL        0x100
#define AKS_CAN_ID_MOTOR_STATUS      0x101
#define AKS_CAN_ID_BATTERY_STATUS    0x200
#define AKS_CAN_ID_BATTERY_CTRL      0x201
#define AKS_CAN_ID_VEHICLE_STATUS    0x300
#define AKS_CAN_ID_SAFETY_STATUS     0x400
#define AKS_CAN_ID_TELEMETRY         0x500
#define AKS_CAN_ID_GPS_DATA          0x600
#define AKS_CAN_ID_EMERGENCY         0x700
#define AKS_CAN_ID_DIAGNOSTIC        0x7DF

/* CAN Frame Types */
typedef enum {
    AKS_CAN_FRAME_STD = 0,
    AKS_CAN_FRAME_EXT = 1
} aks_can_frame_type_t;

/* CAN Frame Structure */
typedef struct {
    uint32_t id;                    /**< CAN identifier */
    aks_can_frame_type_t type;      /**< Frame type (STD/EXT) */
    uint8_t dlc;                    /**< Data length code */
    uint8_t data[AKS_CAN_MAX_DLC];  /**< Data bytes */
    uint32_t timestamp;             /**< Reception timestamp */
    bool rtr;                       /**< Remote transmission request */
} aks_can_frame_t;

/* CAN Filter Configuration */
typedef struct {
    uint32_t id;            /**< Filter ID */
    uint32_t mask;          /**< Filter mask */
    bool extended;          /**< Extended ID filter */
    bool enabled;           /**< Filter enabled */
} aks_can_filter_t;

/* CAN Statistics */
typedef struct {
    uint32_t tx_frames;     /**< Transmitted frames */
    uint32_t rx_frames;     /**< Received frames */
    uint32_t tx_errors;     /**< Transmission errors */
    uint32_t rx_errors;     /**< Reception errors */
    uint32_t bus_off_count; /**< Bus-off events */
    uint32_t last_error;    /**< Last error code */
} aks_can_stats_t;

/**
 * @brief Initialize CAN interface
 * @param baudrate CAN bus baudrate
 * @param filters Pointer to filter configuration array
 * @param filter_count Number of filters
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_init(uint32_t baudrate, const aks_can_filter_t* filters, uint8_t filter_count);

/**
 * @brief Deinitialize CAN interface
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_deinit(void);

/**
 * @brief Start CAN communication
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_start(void);

/**
 * @brief Stop CAN communication
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_stop(void);

/**
 * @brief Transmit CAN frame
 * @param frame Pointer to CAN frame
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_transmit(const aks_can_frame_t* frame, uint32_t timeout_ms);

/**
 * @brief Receive CAN frame
 * @param frame Pointer to receive buffer
 * @param timeout_ms Timeout in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_receive(aks_can_frame_t* frame, uint32_t timeout_ms);

/**
 * @brief Check if CAN frame is available
 * @return true if frame is available, false otherwise
 */
bool aks_can_is_frame_available(void);

/**
 * @brief Get number of pending frames in RX buffer
 * @return Number of pending frames
 */
uint32_t aks_can_get_rx_pending_count(void);

/**
 * @brief Register message callback for specific CAN ID
 * @param id CAN identifier
 * @param callback Callback function
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_register_callback(uint32_t id, aks_message_callback_t callback);

/**
 * @brief Unregister message callback
 * @param id CAN identifier
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_unregister_callback(uint32_t id);

/**
 * @brief Get CAN bus error state
 * @return Current error state
 */
uint32_t aks_can_get_error_state(void);

/**
 * @brief Get CAN statistics
 * @param stats Pointer to statistics structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_get_statistics(aks_can_stats_t* stats);

/**
 * @brief Clear CAN statistics
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_clear_statistics(void);

/**
 * @brief Set CAN bus baudrate
 * @param baudrate New baudrate
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_set_baudrate(uint32_t baudrate);

/**
 * @brief Enable/disable loopback mode
 * @param enable Loopback enable flag
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_set_loopback(bool enable);

/**
 * @brief Enable/disable silent mode
 * @param enable Silent mode enable flag
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_set_silent(bool enable);

/**
 * @brief Send motor control command via CAN
 * @param motor_ctrl Pointer to motor control structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_send_motor_control(const aks_motor_control_t* motor_ctrl);

/**
 * @brief Send battery status via CAN
 * @param battery_status Pointer to battery status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_send_battery_status(const aks_battery_status_t* battery_status);

/**
 * @brief Send vehicle telemetry via CAN
 * @param telemetry Pointer to telemetry structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_send_telemetry(const aks_vehicle_telemetry_t* telemetry);

/**
 * @brief Send safety status via CAN
 * @param safety_status Pointer to safety status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_send_safety_status(const aks_safety_status_t* safety_status);

/**
 * @brief Send emergency message via CAN
 * @param emergency_code Emergency code
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_can_send_emergency(uint8_t emergency_code);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_CAN_H */