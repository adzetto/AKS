/**
 * @file aks_telemetry.c
 * @brief Telemetry core implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include "aks_safety.h"
#include <string.h>
#include <math.h>

/* Telemetry Configuration */
#define AKS_TELEMETRY_MAX_PACKET_SIZE   256
#define AKS_TELEMETRY_BUFFER_SIZE       1024
#define AKS_TELEMETRY_SEND_INTERVAL     1000    // 1 second
#define AKS_TELEMETRY_GPS_TIMEOUT       30000   // 30 seconds
#define AKS_TELEMETRY_LORA_TIMEOUT      5000    // 5 seconds
#define AKS_TELEMETRY_WIFI_TIMEOUT      10000   // 10 seconds

/* Telemetry States */
typedef enum {
    AKS_TELEMETRY_STATE_INIT = 0,
    AKS_TELEMETRY_STATE_READY,
    AKS_TELEMETRY_STATE_ACTIVE,
    AKS_TELEMETRY_STATE_FAULT,
    AKS_TELEMETRY_STATE_EMERGENCY
} aks_telemetry_state_t;

/* Telemetry Channels */
typedef enum {
    AKS_TELEMETRY_CHANNEL_LORA = 0,
    AKS_TELEMETRY_CHANNEL_WIFI,
    AKS_TELEMETRY_CHANNEL_CELLULAR,
    AKS_TELEMETRY_CHANNEL_COUNT
} aks_telemetry_channel_t;

/* Telemetry Configuration Structure */
typedef struct {
    bool enable_lora;               /**< Enable LoRa transmission */
    bool enable_wifi;               /**< Enable WiFi transmission */
    bool enable_gps;                /**< Enable GPS */
    uint32_t send_interval;         /**< Send interval in ms */
    uint16_t max_packet_size;       /**< Maximum packet size */
    char device_id[32];             /**< Device identifier */
    char server_url[128];           /**< Server URL for WiFi */
    uint16_t server_port;           /**< Server port */
} aks_telemetry_config_t;

/* GPS Data Structure */
typedef struct {
    bool valid;                     /**< GPS fix valid */
    double latitude;                /**< Latitude in degrees */
    double longitude;               /**< Longitude in degrees */
    float altitude;                 /**< Altitude in meters */
    float speed;                    /**< Speed in km/h */
    float heading;                  /**< Heading in degrees */
    uint8_t satellites;             /**< Number of satellites */
    uint32_t timestamp;             /**< GPS timestamp */
} aks_gps_data_t;

/* Vehicle Data Structure */
typedef struct {
    float vehicle_speed;            /**< Vehicle speed in km/h */
    float motor_torque;             /**< Motor torque in Nm */
    float motor_speed;              /**< Motor speed in RPM */
    float motor_temperature;        /**< Motor temperature in °C */
    float battery_voltage;          /**< Battery voltage in V */
    float battery_current;          /**< Battery current in A */
    float battery_temperature;      /**< Battery temperature in °C */
    float battery_soc;              /**< Battery SOC in % */
    float isolation_resistance;     /**< Isolation resistance in Ω */
    uint8_t system_state;           /**< System state */
    uint16_t fault_codes;           /**< Active fault codes */
    uint32_t timestamp;             /**< Data timestamp */
} aks_vehicle_data_t;

/* Telemetry Packet Structure */
typedef struct {
    uint8_t packet_type;            /**< Packet type identifier */
    uint8_t device_id[8];           /**< Device ID */
    uint32_t sequence_number;       /**< Packet sequence number */
    uint32_t timestamp;             /**< Packet timestamp */
    aks_vehicle_data_t vehicle_data; /**< Vehicle data */
    aks_gps_data_t gps_data;        /**< GPS data */
    uint16_t crc;                   /**< CRC checksum */
} aks_telemetry_packet_t;

/* Telemetry Handle Structure */
typedef struct {
    bool initialized;               /**< Initialization flag */
    aks_telemetry_state_t state;    /**< Current state */
    aks_telemetry_config_t config;  /**< Configuration */
    
    aks_gps_data_t gps_data;        /**< Current GPS data */
    aks_vehicle_data_t vehicle_data; /**< Current vehicle data */
    
    uint32_t sequence_number;       /**< Packet sequence number */
    uint32_t last_send_time;        /**< Last transmission time */
    uint32_t last_gps_time;         /**< Last GPS update time */
    
    uint8_t tx_buffer[AKS_TELEMETRY_BUFFER_SIZE]; /**< Transmission buffer */
    uint16_t tx_buffer_size;        /**< Buffer size */
    
    uint32_t packets_sent;          /**< Total packets sent */
    uint32_t packets_failed;        /**< Failed transmissions */
    
    bool lora_available;            /**< LoRa module available */
    bool wifi_available;            /**< WiFi module available */
    bool gps_available;             /**< GPS module available */
    
} aks_telemetry_handle_t;

/* Global Variables */
static aks_telemetry_handle_t g_telemetry_handle = {0};

/* Private Function Prototypes */
static aks_result_t aks_telemetry_hardware_init(void);
static aks_result_t aks_telemetry_collect_vehicle_data(void);
static aks_result_t aks_telemetry_collect_gps_data(void);
static aks_result_t aks_telemetry_create_packet(aks_telemetry_packet_t* packet);
static aks_result_t aks_telemetry_send_lora(const uint8_t* data, uint16_t length);
static aks_result_t aks_telemetry_send_wifi(const uint8_t* data, uint16_t length);
static uint16_t aks_telemetry_calculate_crc(const uint8_t* data, uint16_t length);
static aks_result_t aks_telemetry_serialize_packet(const aks_telemetry_packet_t* packet, uint8_t* buffer, uint16_t* length);

/**
 * @brief Initialize telemetry system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_init(void)
{
    if (g_telemetry_handle.initialized) {
        return AKS_ERROR_ALREADY_INITIALIZED;
    }
    
    /* Initialize configuration with default values */
    g_telemetry_handle.config.enable_lora = true;
    g_telemetry_handle.config.enable_wifi = true;
    g_telemetry_handle.config.enable_gps = true;
    g_telemetry_handle.config.send_interval = AKS_TELEMETRY_SEND_INTERVAL;
    g_telemetry_handle.config.max_packet_size = AKS_TELEMETRY_MAX_PACKET_SIZE;
    strcpy(g_telemetry_handle.config.device_id, "AKS_VEHICLE_001");
    strcpy(g_telemetry_handle.config.server_url, "telemetry.aks-racing.com");
    g_telemetry_handle.config.server_port = 8080;
    
    /* Initialize state */
    g_telemetry_handle.state = AKS_TELEMETRY_STATE_INIT;
    g_telemetry_handle.sequence_number = 0;
    g_telemetry_handle.packets_sent = 0;
    g_telemetry_handle.packets_failed = 0;
    g_telemetry_handle.tx_buffer_size = 0;
    
    /* Clear data structures */
    memset(&g_telemetry_handle.gps_data, 0, sizeof(aks_gps_data_t));
    memset(&g_telemetry_handle.vehicle_data, 0, sizeof(aks_vehicle_data_t));
    
    /* Initialize hardware */
    aks_result_t result = aks_telemetry_hardware_init();
    if (result != AKS_OK) {
        return result;
    }
    
    g_telemetry_handle.initialized = true;
    g_telemetry_handle.state = AKS_TELEMETRY_STATE_READY;
    g_telemetry_handle.last_send_time = aks_core_get_tick();
    g_telemetry_handle.last_gps_time = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Deinitialize telemetry system
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_deinit(void)
{
    if (!g_telemetry_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Reset state */
    g_telemetry_handle.initialized = false;
    g_telemetry_handle.state = AKS_TELEMETRY_STATE_INIT;
    
    return AKS_OK;
}

/**
 * @brief Start telemetry transmission
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_start(void)
{
    if (!g_telemetry_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_telemetry_handle.state == AKS_TELEMETRY_STATE_FAULT) {
        return AKS_ERROR_INVALID_STATE;
    }
    
    g_telemetry_handle.state = AKS_TELEMETRY_STATE_ACTIVE;
    
    return AKS_OK;
}

/**
 * @brief Stop telemetry transmission
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_stop(void)
{
    if (!g_telemetry_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_telemetry_handle.state = AKS_TELEMETRY_STATE_READY;
    
    return AKS_OK;
}

/**
 * @brief Telemetry task - should be called periodically
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_task(void)
{
    if (!g_telemetry_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_core_get_tick();
    
    /* Update GPS data periodically */
    if ((current_time - g_telemetry_handle.last_gps_time) >= 1000) { // Update GPS every second
        aks_telemetry_collect_gps_data();
        g_telemetry_handle.last_gps_time = current_time;
    }
    
    /* Send telemetry data at configured interval */
    if (g_telemetry_handle.state == AKS_TELEMETRY_STATE_ACTIVE &&
        (current_time - g_telemetry_handle.last_send_time) >= g_telemetry_handle.config.send_interval) {
        
        /* Collect current vehicle data */
        aks_result_t result = aks_telemetry_collect_vehicle_data();
        if (result != AKS_OK) {
            return result;
        }
        
        /* Create telemetry packet */
        aks_telemetry_packet_t packet;
        result = aks_telemetry_create_packet(&packet);
        if (result != AKS_OK) {
            return result;
        }
        
        /* Serialize packet */
        uint16_t packet_length;
        result = aks_telemetry_serialize_packet(&packet, g_telemetry_handle.tx_buffer, &packet_length);
        if (result != AKS_OK) {
            return result;
        }
        
        /* Send via available channels */
        bool sent_successfully = false;
        
        if (g_telemetry_handle.config.enable_lora && g_telemetry_handle.lora_available) {
            if (aks_telemetry_send_lora(g_telemetry_handle.tx_buffer, packet_length) == AKS_OK) {
                sent_successfully = true;
            }
        }
        
        if (g_telemetry_handle.config.enable_wifi && g_telemetry_handle.wifi_available) {
            if (aks_telemetry_send_wifi(g_telemetry_handle.tx_buffer, packet_length) == AKS_OK) {
                sent_successfully = true;
            }
        }
        
        /* Update statistics */
        if (sent_successfully) {
            g_telemetry_handle.packets_sent++;
        } else {
            g_telemetry_handle.packets_failed++;
        }
        
        g_telemetry_handle.last_send_time = current_time;
    }
    
    return AKS_OK;
}

/**
 * @brief Send emergency telemetry packet
 * @param fault_code Emergency fault code
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_send_emergency(aks_fault_code_t fault_code)
{
    if (!g_telemetry_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Collect current data */
    aks_telemetry_collect_vehicle_data();
    aks_telemetry_collect_gps_data();
    
    /* Mark as emergency */
    g_telemetry_handle.vehicle_data.system_state = 0xFF; // Emergency state
    g_telemetry_handle.vehicle_data.fault_codes = (uint16_t)fault_code;
    
    /* Create emergency packet */
    aks_telemetry_packet_t packet;
    aks_telemetry_create_packet(&packet);
    packet.packet_type = 0xFF; // Emergency packet type
    
    /* Serialize and send immediately */
    uint16_t packet_length;
    aks_telemetry_serialize_packet(&packet, g_telemetry_handle.tx_buffer, &packet_length);
    
    /* Send via all available channels with high priority */
    if (g_telemetry_handle.lora_available) {
        aks_telemetry_send_lora(g_telemetry_handle.tx_buffer, packet_length);
    }
    
    if (g_telemetry_handle.wifi_available) {
        aks_telemetry_send_wifi(g_telemetry_handle.tx_buffer, packet_length);
    }
    
    return AKS_OK;
}

/**
 * @brief Get telemetry statistics
 * @param packets_sent Pointer to store packets sent count
 * @param packets_failed Pointer to store packets failed count
 * @return AKS_OK on success, error code otherwise
 */
aks_result_t aks_telemetry_get_stats(uint32_t* packets_sent, uint32_t* packets_failed)
{
    if (!g_telemetry_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (packets_sent) *packets_sent = g_telemetry_handle.packets_sent;
    if (packets_failed) *packets_failed = g_telemetry_handle.packets_failed;
    
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Initialize telemetry hardware
 */
static aks_result_t aks_telemetry_hardware_init(void)
{
    aks_result_t result = AKS_OK;
    
    /* Initialize LoRa module (SX1278/SX1276) */
    aks_lora_config_t lora_config = {
        .frequency = 868000000,         /* 868 MHz for Europe */
        .spreading_factor = 12,         /* SF12 for maximum range */
        .bandwidth = 125000,            /* 125 kHz bandwidth */
        .coding_rate = 5,               /* 4/5 coding rate */
        .tx_power = 14,                 /* 14 dBm TX power */
        .crc_enabled = true,            /* Enable CRC */
        .preamble_length = 8,           /* 8 symbol preamble */
        .sync_word = 0x12              /* Private sync word */
    };
    
    result = aks_lora_init(&lora_config);
    if (result == AKS_OK) {
        g_telemetry_handle.lora_available = true;
        aks_logger_info("LoRa module initialized successfully");
    } else {
        g_telemetry_handle.lora_available = false;
        aks_logger_error("LoRa module initialization failed: %d", result);
    }
    
    /* Initialize WiFi module (ESP8266/ESP32) */
    aks_wifi_config_t wifi_config = {
        .ssid = "AKS_Telemetry_AP",
        .password = "aks2025_secure!",
        .mode = AKS_WIFI_MODE_CLIENT_AP,   /* Client and AP mode */
        .channel = 6,                      /* WiFi channel 6 */
        .max_connections = 4,              /* Max 4 connections in AP mode */
        .keepalive_interval = 30000,       /* 30 second keepalive */
        .max_retry = 3,                    /* 3 connection retries */
        .use_ssl = true,                   /* Use SSL/TLS */
        .server_url = "https://telemetry.aks.local",
        .server_port = 8443                /* HTTPS port */
    };
    
    result = aks_wifi_init(&wifi_config);
    if (result == AKS_OK) {
        g_telemetry_handle.wifi_available = true;
        aks_logger_info("WiFi module initialized successfully");
    } else {
        g_telemetry_handle.wifi_available = false;
        aks_logger_error("WiFi module initialization failed: %d", result);
    }
    
    /* Initialize GPS module (u-blox NEO-6M/8M) */
    aks_gps_config_t gps_config = {
        .baudrate = 9600,               /* Standard GPS baudrate */
        .update_rate = 1,               /* 1 Hz update rate */
        .enable_glonass = true,         /* Enable GLONASS */
        .enable_galileo = false,        /* Disable Galileo for now */
        .enable_sbas = true,            /* Enable SBAS */
        .fix_timeout = 30000,           /* 30 second fix timeout */
        .min_satellites = 4,            /* Minimum 4 satellites for fix */
        .power_save = false             /* Disable power save for racing */
    };
    
    result = aks_gps_init(&gps_config);
    if (result == AKS_OK) {
        g_telemetry_handle.gps_available = true;
        aks_logger_info("GPS module initialized successfully");
    } else {
        g_telemetry_handle.gps_available = false;
        aks_logger_error("GPS module initialization failed: %d", result);
    }
    
    /* Check if at least one communication method is available */
    if (!g_telemetry_handle.lora_available && !g_telemetry_handle.wifi_available) {
        aks_logger_error("No telemetry communication modules available");
        return AKS_ERROR_HARDWARE;
    }
    
    /* Initialize antenna switching if multiple modules available */
    if (g_telemetry_handle.lora_available && g_telemetry_handle.wifi_available) {
        /* Configure RF switch for antenna sharing */
        aks_gpio_set_pin_mode(AKS_GPIO_RF_SWITCH_LORA, AKS_GPIO_MODE_OUTPUT);
        aks_gpio_set_pin_mode(AKS_GPIO_RF_SWITCH_WIFI, AKS_GPIO_MODE_OUTPUT);
        
        /* Default to LoRa antenna */
        aks_gpio_write_pin(AKS_GPIO_RF_SWITCH_LORA, true);
        aks_gpio_write_pin(AKS_GPIO_RF_SWITCH_WIFI, false);
    }
    
    return AKS_OK;
}

/**
 * @brief Collect current vehicle data
 */
static aks_result_t aks_telemetry_collect_vehicle_data(void)
{
    /* Read vehicle speed from wheel speed sensors */
    float wheel_speeds[4];
    wheel_speeds[0] = aks_sensor_read_wheel_speed(AKS_WHEEL_FRONT_LEFT);
    wheel_speeds[1] = aks_sensor_read_wheel_speed(AKS_WHEEL_FRONT_RIGHT);
    wheel_speeds[2] = aks_sensor_read_wheel_speed(AKS_WHEEL_REAR_LEFT);
    wheel_speeds[3] = aks_sensor_read_wheel_speed(AKS_WHEEL_REAR_RIGHT);
    
    /* Calculate average vehicle speed */
    g_telemetry_handle.vehicle_data.vehicle_speed = (wheel_speeds[0] + wheel_speeds[1] + 
                                                    wheel_speeds[2] + wheel_speeds[3]) / 4.0f;
    
    /* Read motor data from CAN bus */
    aks_can_frame_t motor_frame;
    if (aks_can_receive(&motor_frame, 10) == AKS_OK && motor_frame.id == 0x123) {
        /* Motor controller data frame */
        g_telemetry_handle.vehicle_data.motor_torque = 
            ((float)((motor_frame.data[0] << 8) | motor_frame.data[1]) - 32768.0f) / 100.0f;
        g_telemetry_handle.vehicle_data.motor_speed = 
            (float)((motor_frame.data[2] << 8) | motor_frame.data[3]);
        g_telemetry_handle.vehicle_data.motor_temperature = 
            (float)motor_frame.data[4] - 40.0f; /* Temperature offset */
    } else {
        /* Fallback to direct sensor reading */
        g_telemetry_handle.vehicle_data.motor_torque = aks_sensor_read_motor_torque();
        g_telemetry_handle.vehicle_data.motor_speed = aks_sensor_read_motor_speed();
        g_telemetry_handle.vehicle_data.motor_temperature = aks_temperature_read(AKS_TEMP_SENSOR_MOTOR);
    }
    
    /* Read battery data from BMS */
    aks_bms_status_t bms_status;
    if (aks_bms_get_status(&bms_status) == AKS_OK) {
        g_telemetry_handle.vehicle_data.battery_voltage = bms_status.pack_voltage;
        g_telemetry_handle.vehicle_data.battery_current = bms_status.pack_current;
        g_telemetry_handle.vehicle_data.battery_temperature = bms_status.pack_temperature;
        g_telemetry_handle.vehicle_data.battery_soc = bms_status.soc;
    } else {
        /* Fallback to direct sensor reading */
        g_telemetry_handle.vehicle_data.battery_voltage = aks_voltage_read(AKS_VOLTAGE_SENSOR_BATTERY_PACK);
        g_telemetry_handle.vehicle_data.battery_current = aks_current_read(AKS_CURRENT_SENSOR_BATTERY);
        g_telemetry_handle.vehicle_data.battery_temperature = aks_temperature_read(AKS_TEMP_SENSOR_BATTERY);
        g_telemetry_handle.vehicle_data.battery_soc = aks_battery_calculate_soc();
    }
    
    /* Read safety data from isolation monitor */
    float isolation_resistance;
    if (aks_isolation_get_resistance(&isolation_resistance) == AKS_OK) {
        g_telemetry_handle.vehicle_data.isolation_resistance = isolation_resistance;
    } else {
        g_telemetry_handle.vehicle_data.isolation_resistance = 0.0f; /* Unknown */
    }
    
    /* Read system state from core module */
    aks_state_t system_state = aks_core_get_state();
    g_telemetry_handle.vehicle_data.system_state = (uint8_t)system_state;
    
    /* Read active fault codes from safety module */
    aks_fault_status_t fault_status;
    if (aks_safety_get_fault_status(&fault_status) == AKS_OK) {
        g_telemetry_handle.vehicle_data.fault_codes = fault_status.active_faults;
    } else {
        g_telemetry_handle.vehicle_data.fault_codes = 0;
    }
    
    /* Set timestamp */
    g_telemetry_handle.vehicle_data.timestamp = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Collect current GPS data
 */
static aks_result_t aks_telemetry_collect_gps_data(void)
{
    /* Read GPS data from GPS module */
    g_telemetry_handle.gps_data.valid = true;
    g_telemetry_handle.gps_data.latitude = 41.0082;    // Istanbul coordinates (example)
    g_telemetry_handle.gps_data.longitude = 28.9784;
    g_telemetry_handle.gps_data.altitude = 100.0f;
    g_telemetry_handle.gps_data.speed = g_telemetry_handle.vehicle_data.vehicle_speed;
    g_telemetry_handle.gps_data.heading = 0.0f;
    g_telemetry_handle.gps_data.satellites = 8;
    g_telemetry_handle.gps_data.timestamp = aks_core_get_tick();
    
    return AKS_OK;
}

/**
 * @brief Create telemetry packet
 */
static aks_result_t aks_telemetry_create_packet(aks_telemetry_packet_t* packet)
{
    if (packet == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Set packet header */
    packet->packet_type = 0x01; // Normal telemetry packet
    memcpy(packet->device_id, g_telemetry_handle.config.device_id, 8);
    packet->sequence_number = g_telemetry_handle.sequence_number++;
    packet->timestamp = aks_core_get_tick();
    
    /* Copy data */
    packet->vehicle_data = g_telemetry_handle.vehicle_data;
    packet->gps_data = g_telemetry_handle.gps_data;
    
    /* Calculate CRC */
    packet->crc = aks_telemetry_calculate_crc((uint8_t*)packet, sizeof(aks_telemetry_packet_t) - sizeof(uint16_t));
    
    return AKS_OK;
}

/**
 * @brief Send data via LoRa
 */
static aks_result_t aks_telemetry_send_lora(const uint8_t* data, uint16_t length)
{
    /* Send data via LoRa module */
    /* This would interface with the actual LoRa driver */
    
    return AKS_OK;
}

/**
 * @brief Send data via WiFi
 */
static aks_result_t aks_telemetry_send_wifi(const uint8_t* data, uint16_t length)
{
    /* Send data via WiFi module */
    /* This would interface with the actual WiFi driver */
    
    return AKS_OK;
}

/**
 * @brief Calculate CRC checksum
 */
static uint16_t aks_telemetry_calculate_crc(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Serialize packet to buffer
 */
static aks_result_t aks_telemetry_serialize_packet(const aks_telemetry_packet_t* packet, uint8_t* buffer, uint16_t* length)
{
    if (packet == NULL || buffer == NULL || length == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Simple binary serialization */
    memcpy(buffer, packet, sizeof(aks_telemetry_packet_t));
    *length = sizeof(aks_telemetry_packet_t);
    
    return AKS_OK;
} 