/**
 * @file aks_telemetry.h
 * @brief Telemetry system for AKS - LoRa, WiFi, GPS integration
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#ifndef AKS_TELEMETRY_H
#define AKS_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aks_types.h"

/**
 * @defgroup AKS_Telemetry Telemetry System
 * @brief Telemetry data collection and transmission
 * @{
 */

/* LoRa Configuration */
#define AKS_LORA_FREQ_868MHZ        868000000
#define AKS_LORA_FREQ_915MHZ        915000000
#define AKS_LORA_MAX_PAYLOAD        64
#define AKS_LORA_SPREADING_FACTOR   12
#define AKS_LORA_BANDWIDTH          125000
#define AKS_LORA_CODING_RATE        5

/* WiFi Configuration */
#define AKS_WIFI_SSID_MAX_LEN       32
#define AKS_WIFI_PASS_MAX_LEN       64
#define AKS_WIFI_MAX_RETRY          5

/* GPS Configuration */
#define AKS_GPS_NMEA_BUFFER_SIZE    256
#define AKS_GPS_MIN_SATELLITES      4
#define AKS_GPS_FIX_TIMEOUT_MS      30000

/* Telemetry Data Formats */
typedef enum {
    AKS_TEL_FORMAT_JSON = 0,
    AKS_TEL_FORMAT_BINARY = 1,
    AKS_TEL_FORMAT_CSV = 2
} aks_telemetry_format_t;

/* Communication Channels */
typedef enum {
    AKS_TEL_CHANNEL_LORA = 0x01,
    AKS_TEL_CHANNEL_WIFI = 0x02,
    AKS_TEL_CHANNEL_BOTH = 0x03
} aks_telemetry_channel_t;

/* LoRa Configuration */
typedef struct {
    uint32_t frequency;         /**< Operating frequency in Hz */
    uint8_t spreading_factor;   /**< Spreading factor (6-12) */
    uint32_t bandwidth;         /**< Bandwidth in Hz */
    uint8_t coding_rate;        /**< Coding rate (5-8) */
    int8_t tx_power;           /**< TX power in dBm */
    bool crc_enabled;          /**< CRC enable flag */
    uint16_t preamble_length;  /**< Preamble length */
    uint8_t sync_word;         /**< Sync word */
} aks_lora_config_t;

/* WiFi Configuration */
typedef struct {
    char ssid[AKS_WIFI_SSID_MAX_LEN];       /**< WiFi SSID */
    char password[AKS_WIFI_PASS_MAX_LEN];   /**< WiFi password */
    char server_url[128];                   /**< Server URL */
    uint16_t server_port;                   /**< Server port */
    uint32_t keepalive_interval;            /**< Keep-alive interval in ms */
    uint8_t max_retry;                      /**< Maximum retry attempts */
    bool use_ssl;                           /**< SSL/TLS enable flag */
} aks_wifi_config_t;

/* GPS Configuration */
typedef struct {
    uint32_t baudrate;          /**< UART baudrate */
    uint8_t update_rate;        /**< Update rate in Hz */
    bool enable_glonass;        /**< GLONASS enable */
    bool enable_galileo;        /**< Galileo enable */
    uint32_t fix_timeout;       /**< Fix timeout in ms */
} aks_gps_config_t;

/* Telemetry Packet */
typedef struct {
    uint8_t car_id;                         /**< Vehicle ID */
    uint32_t timestamp;                     /**< Unix timestamp */
    float speed;                            /**< Speed in km/h */
    float battery_voltage;                  /**< Battery voltage in V */
    float battery_soc;                      /**< State of charge in % */
    float motor_temperature;                /**< Motor temperature in °C */
    aks_gps_data_t gps;                    /**< GPS data */
    uint16_t crc;                          /**< Data CRC */
} aks_telemetry_packet_t;

/* Telemetry Statistics */
typedef struct {
    uint32_t packets_sent;      /**< Total packets sent */
    uint32_t packets_acked;     /**< Acknowledged packets */
    uint32_t packets_lost;      /**< Lost packets */
    uint32_t transmission_errors; /**< Transmission errors */
    float packet_loss_rate;     /**< Packet loss rate in % */
    uint32_t last_tx_time;      /**< Last transmission time */
} aks_telemetry_stats_t;

/* LoRa Status */
typedef struct {
    bool initialized;           /**< Initialization status */
    bool connected;            /**< Connection status */
    int8_t rssi;              /**< Received signal strength */
    float snr;                /**< Signal-to-noise ratio */
    uint32_t frequency_error; /**< Frequency error */
    uint8_t spreading_factor; /**< Current spreading factor */
} aks_lora_status_t;

/* WiFi Status */
typedef struct {
    bool initialized;          /**< Initialization status */
    bool connected;           /**< Connection status */
    int8_t rssi;             /**< WiFi signal strength */
    char ip_address[16];     /**< Assigned IP address */
    char mac_address[18];    /**< MAC address */
    uint32_t connection_time; /**< Connection uptime */
} aks_wifi_status_t;

/**
 * @brief Initialize telemetry system
 * @param lora_config LoRa configuration
 * @param wifi_config WiFi configuration
 * @param gps_config GPS configuration
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_init(const aks_lora_config_t* lora_config,
                                const aks_wifi_config_t* wifi_config,
                                const aks_gps_config_t* gps_config);

/**
 * @brief Deinitialize telemetry system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_deinit(void);

/**
 * @brief Start telemetry transmission
 * @param channels Communication channels to use
 * @param interval_ms Transmission interval in milliseconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_start(aks_telemetry_channel_t channels, uint32_t interval_ms);

/**
 * @brief Stop telemetry transmission
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_stop(void);

/**
 * @brief Send telemetry packet
 * @param packet Pointer to telemetry packet
 * @param channel Communication channel
 * @param format Data format
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_send_packet(const aks_telemetry_packet_t* packet,
                                       aks_telemetry_channel_t channel,
                                       aks_telemetry_format_t format);

/**
 * @brief Update vehicle data for telemetry
 * @param speed Vehicle speed in km/h
 * @param battery_voltage Battery voltage in V
 * @param battery_soc State of charge in %
 * @param motor_temp Motor temperature in °C
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_update_vehicle_data(float speed, float battery_voltage,
                                               float battery_soc, float motor_temp);

/**
 * @brief Get current GPS data
 * @param gps_data Pointer to GPS data structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_get_gps_data(aks_gps_data_t* gps_data);

/**
 * @brief Get LoRa status
 * @param status Pointer to LoRa status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_get_lora_status(aks_lora_status_t* status);

/**
 * @brief Get WiFi status
 * @param status Pointer to WiFi status structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_get_wifi_status(aks_wifi_status_t* status);

/**
 * @brief Get telemetry statistics
 * @param stats Pointer to statistics structure
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_get_statistics(aks_telemetry_stats_t* stats);

/**
 * @brief Clear telemetry statistics
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_clear_statistics(void);

/**
 * @brief Set vehicle ID
 * @param car_id Vehicle identifier
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_set_vehicle_id(uint8_t car_id);

/**
 * @brief Enable/disable local data logging
 * @param enable Logging enable flag
 * @param filename Log filename (if NULL, auto-generated)
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_set_logging(bool enable, const char* filename);

/**
 * @brief Handle connection loss and reconnection
 * @param max_retry_time Maximum retry time in seconds
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_handle_connection_loss(uint32_t max_retry_time);

/**
 * @brief Test communication channels
 * @param channels Channels to test
 * @return AKS_OK if all channels working, error code otherwise
 */
aks_result_t aks_telemetry_test_channels(aks_telemetry_channel_t channels);

/**
 * @brief Configure AES encryption for data transmission
 * @param key 128-bit encryption key
 * @param enable Encryption enable flag
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_set_encryption(const uint8_t* key, bool enable);

/**
 * @brief Telemetry task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_task(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AKS_TELEMETRY_H */