/**
 * @file aks_gps.c
 * @brief GPS sensor implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include "aks_uart.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* GPS Configuration */
#define AKS_GPS_UART_INSTANCE       1           /**< GPS UART instance */
#define AKS_GPS_UART_BAUDRATE       9600        /**< GPS baudrate */
#define AKS_GPS_BUFFER_SIZE         256         /**< GPS buffer size */
#define AKS_GPS_TIMEOUT_MS          2000        /**< GPS timeout */
#define AKS_GPS_NMEA_MAX_FIELDS     20          /**< Max NMEA fields */

/* NMEA Message Types */
#define AKS_GPS_NMEA_GGA            "GPGGA"     /**< Global Positioning System Fix Data */
#define AKS_GPS_NMEA_RMC            "GPRMC"     /**< Recommended Minimum */
#define AKS_GPS_NMEA_GSV            "GPGSV"     /**< GPS Satellites in View */
#define AKS_GPS_NMEA_GSA            "GPGSA"     /**< GPS DOP and active satellites */

/* GPS State */
typedef enum {
    AKS_GPS_STATE_UNINIT = 0,
    AKS_GPS_STATE_INIT,
    AKS_GPS_STATE_SEARCHING,
    AKS_GPS_STATE_FIXED,
    AKS_GPS_STATE_ERROR
} aks_gps_state_t;

/* GPS Data Structure */
static struct {
    aks_gps_state_t state;
    aks_gps_data_t last_data;
    char rx_buffer[AKS_GPS_BUFFER_SIZE];
    uint16_t rx_index;
    uint32_t last_update_time;
    uint32_t fix_count;
    uint32_t error_count;
    bool initialized;
} g_gps = {0};

/* Private function prototypes */
static aks_result_t aks_gps_parse_nmea(const char* sentence);
static aks_result_t aks_gps_parse_gga(const char* sentence);
static aks_result_t aks_gps_parse_rmc(const char* sentence);
static bool aks_gps_validate_checksum(const char* sentence);
static uint8_t aks_gps_calculate_checksum(const char* sentence);
static float aks_gps_convert_coordinate(const char* coord_str, const char* direction);
static uint32_t aks_gps_get_time_ms(void);
static void aks_gps_split_fields(char* sentence, char* fields[], uint8_t max_fields, uint8_t* field_count);

/**
 * @brief Initialize GPS module
 */
aks_result_t aks_gps_init(void)
{
    if (g_gps.initialized) {
        return AKS_OK;
    }
    
    /* Initialize UART for GPS */
    aks_uart_config_t uart_config = {
        .baudrate = AKS_GPS_UART_BAUDRATE,
        .data_bits = AKS_UART_DATA_8BIT,
        .stop_bits = AKS_UART_STOP_1BIT,
        .parity = AKS_UART_PARITY_NONE,
        .flow_control = AKS_UART_FLOW_NONE,
        .enable_dma = false
    };
    
    aks_result_t result = aks_uart_init(AKS_GPS_UART_INSTANCE, &uart_config);
    if (result != AKS_OK) {
        return result;
    }
    
    /* Initialize GPS state */
    memset(&g_gps, 0, sizeof(g_gps));
    g_gps.state = AKS_GPS_STATE_INIT;
    g_gps.initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize GPS module
 */
aks_result_t aks_gps_deinit(void)
{
    if (!g_gps.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    aks_uart_deinit(AKS_GPS_UART_INSTANCE);
    
    memset(&g_gps, 0, sizeof(g_gps));
    
    return AKS_OK;
}

/**
 * @brief Update GPS data (call this periodically)
 */
aks_result_t aks_gps_update(void)
{
    if (!g_gps.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint8_t byte;
    aks_result_t result;
    
    /* Read available data from UART */
    while ((result = aks_uart_receive_byte(AKS_GPS_UART_INSTANCE, &byte, 0)) == AKS_OK) {
        
        /* Check for buffer overflow */
        if (g_gps.rx_index >= AKS_GPS_BUFFER_SIZE - 1) {
            g_gps.rx_index = 0; /* Reset buffer */
            g_gps.error_count++;
            continue;
        }
        
        /* Add byte to buffer */
        g_gps.rx_buffer[g_gps.rx_index] = byte;
        
        /* Check for end of NMEA sentence */
        if (byte == '\n') {
            g_gps.rx_buffer[g_gps.rx_index] = '\0'; /* Null terminate */
            
            /* Parse NMEA sentence */
            if (g_gps.rx_index > 5) { /* Minimum NMEA sentence length */
                aks_gps_parse_nmea(g_gps.rx_buffer);
            }
            
            g_gps.rx_index = 0; /* Reset for next sentence */
        } else if (byte != '\r') { /* Ignore carriage return */
            g_gps.rx_index++;
        }
    }
    
    /* Check for timeout */
    uint32_t current_time = aks_gps_get_time_ms();
    if (g_gps.state == AKS_GPS_STATE_FIXED && 
        (current_time - g_gps.last_update_time) > AKS_GPS_TIMEOUT_MS) {
        g_gps.state = AKS_GPS_STATE_SEARCHING;
        g_gps.last_data.fix_valid = false;
    }
    
    return AKS_OK;
}

/**
 * @brief Get GPS data
 */
aks_result_t aks_gps_get_data(aks_gps_data_t* data)
{
    if (!g_gps.initialized || data == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    *data = g_gps.last_data;
    
    return AKS_OK;
}

/**
 * @brief Check if GPS has valid fix
 */
bool aks_gps_has_fix(void)
{
    return (g_gps.initialized && g_gps.last_data.fix_valid);
}

/**
 * @brief Get GPS status
 */
aks_gps_state_t aks_gps_get_state(void)
{
    if (!g_gps.initialized) {
        return AKS_GPS_STATE_UNINIT;
    }
    
    return g_gps.state;
}

/**
 * @brief Get GPS statistics
 */
aks_result_t aks_gps_get_stats(uint32_t* fix_count, uint32_t* error_count)
{
    if (!g_gps.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (fix_count != NULL) {
        *fix_count = g_gps.fix_count;
    }
    
    if (error_count != NULL) {
        *error_count = g_gps.error_count;
    }
    
    return AKS_OK;
}

/**
 * @brief Calculate distance between two GPS coordinates
 */
float aks_gps_calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    const double earth_radius = 6371000.0; /* Earth radius in meters */
    
    /* Convert to radians */
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double delta_lat = (lat2 - lat1) * M_PI / 180.0;
    double delta_lon = (lon2 - lon1) * M_PI / 180.0;
    
    /* Haversine formula */
    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return (float)(earth_radius * c);
}

/**
 * @brief Calculate bearing between two GPS coordinates
 */
float aks_gps_calculate_bearing(double lat1, double lon1, double lat2, double lon2)
{
    /* Convert to radians */
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double delta_lon = (lon2 - lon1) * M_PI / 180.0;
    
    double y = sin(delta_lon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
    
    double bearing_rad = atan2(y, x);
    
    /* Convert to degrees and normalize to 0-360 */
    double bearing_deg = bearing_rad * 180.0 / M_PI;
    bearing_deg = fmod(bearing_deg + 360.0, 360.0);
    
    return (float)bearing_deg;
}

/* Private function implementations */

/**
 * @brief Parse NMEA sentence
 */
static aks_result_t aks_gps_parse_nmea(const char* sentence)
{
    if (sentence == NULL || sentence[0] != '$') {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Validate checksum */
    if (!aks_gps_validate_checksum(sentence)) {
        g_gps.error_count++;
        return AKS_ERROR_CRC;
    }
    
    /* Determine message type and parse */
    if (strncmp(sentence + 1, AKS_GPS_NMEA_GGA, 5) == 0) {
        return aks_gps_parse_gga(sentence);
    } else if (strncmp(sentence + 1, AKS_GPS_NMEA_RMC, 5) == 0) {
        return aks_gps_parse_rmc(sentence);
    }
    
    return AKS_OK; /* Unsupported message type */
}

/**
 * @brief Parse GGA (Global Positioning System Fix Data) sentence
 */
static aks_result_t aks_gps_parse_gga(const char* sentence)
{
    char sentence_copy[AKS_GPS_BUFFER_SIZE];
    char* fields[AKS_GPS_NMEA_MAX_FIELDS];
    uint8_t field_count;
    
    /* Make a copy for parsing */
    strncpy(sentence_copy, sentence, AKS_GPS_BUFFER_SIZE - 1);
    sentence_copy[AKS_GPS_BUFFER_SIZE - 1] = '\0';
    
    /* Split into fields */
    aks_gps_split_fields(sentence_copy, fields, AKS_GPS_NMEA_MAX_FIELDS, &field_count);
    
    if (field_count < 14) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Parse quality indicator */
    int quality = atoi(fields[6]);
    if (quality == 0) {
        g_gps.last_data.fix_valid = false;
        g_gps.state = AKS_GPS_STATE_SEARCHING;
        return AKS_OK;
    }
    
    /* Parse latitude */
    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
        g_gps.last_data.latitude = aks_gps_convert_coordinate(fields[2], fields[3]);
    }
    
    /* Parse longitude */
    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
        g_gps.last_data.longitude = aks_gps_convert_coordinate(fields[4], fields[5]);
    }
    
    /* Parse altitude */
    if (strlen(fields[9]) > 0) {
        g_gps.last_data.altitude = atof(fields[9]);
    }
    
    /* Parse number of satellites */
    if (strlen(fields[7]) > 0) {
        g_gps.last_data.satellites = atoi(fields[7]);
    }
    
    /* Update status */
    g_gps.last_data.fix_valid = true;
    g_gps.last_data.timestamp = aks_gps_get_time_ms();
    g_gps.last_update_time = g_gps.last_data.timestamp;
    g_gps.state = AKS_GPS_STATE_FIXED;
    g_gps.fix_count++;
    
    return AKS_OK;
}

/**
 * @brief Parse RMC (Recommended Minimum) sentence
 */
static aks_result_t aks_gps_parse_rmc(const char* sentence)
{
    char sentence_copy[AKS_GPS_BUFFER_SIZE];
    char* fields[AKS_GPS_NMEA_MAX_FIELDS];
    uint8_t field_count;
    
    /* Make a copy for parsing */
    strncpy(sentence_copy, sentence, AKS_GPS_BUFFER_SIZE - 1);
    sentence_copy[AKS_GPS_BUFFER_SIZE - 1] = '\0';
    
    /* Split into fields */
    aks_gps_split_fields(sentence_copy, fields, AKS_GPS_NMEA_MAX_FIELDS, &field_count);
    
    if (field_count < 12) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Check validity */
    if (fields[2][0] != 'A') { /* 'A' = valid, 'V' = invalid */
        g_gps.last_data.fix_valid = false;
        g_gps.state = AKS_GPS_STATE_SEARCHING;
        return AKS_OK;
    }
    
    /* Parse speed (knots to km/h) */
    if (strlen(fields[7]) > 0) {
        float speed_knots = atof(fields[7]);
        g_gps.last_data.speed = speed_knots * 1.852f; /* Convert to km/h */
    }
    
    /* Parse heading */
    if (strlen(fields[8]) > 0) {
        g_gps.last_data.heading = atof(fields[8]);
    }
    
    return AKS_OK;
}

/**
 * @brief Validate NMEA checksum
 */
static bool aks_gps_validate_checksum(const char* sentence)
{
    const char* checksum_start = strrchr(sentence, '*');
    if (checksum_start == NULL) {
        return false; /* No checksum found */
    }
    
    uint8_t received_checksum = strtol(checksum_start + 1, NULL, 16);
    uint8_t calculated_checksum = aks_gps_calculate_checksum(sentence);
    
    return (received_checksum == calculated_checksum);
}

/**
 * @brief Calculate NMEA checksum
 */
static uint8_t aks_gps_calculate_checksum(const char* sentence)
{
    uint8_t checksum = 0;
    
    /* Start after '$' and stop before '*' */
    for (const char* p = sentence + 1; *p && *p != '*'; p++) {
        checksum ^= *p;
    }
    
    return checksum;
}

/**
 * @brief Convert coordinate string to decimal degrees
 */
static float aks_gps_convert_coordinate(const char* coord_str, const char* direction)
{
    if (coord_str == NULL || direction == NULL || strlen(coord_str) < 4) {
        return 0.0f;
    }
    
    /* Parse degrees and minutes */
    char degrees_str[4] = {0};
    strncpy(degrees_str, coord_str, (strlen(coord_str) > 4) ? 2 : 1);
    
    float degrees = atof(degrees_str);
    float minutes = atof(coord_str + strlen(degrees_str));
    
    /* Convert to decimal degrees */
    float decimal_degrees = degrees + minutes / 60.0f;
    
    /* Apply direction */
    if (direction[0] == 'S' || direction[0] == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    
    return decimal_degrees;
}

/**
 * @brief Get current time in milliseconds
 */
static uint32_t aks_gps_get_time_ms(void)
{
#ifdef USE_HAL_DRIVER
    return HAL_GetTick();
#elif defined(USE_FREERTOS)
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
#elif defined(__unix__) || defined(__APPLE__)
    /* Unix/Linux timestamp for testing */
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    }
    return 0;
#else
    /* Real-time counter for embedded systems without HAL */
    static uint32_t system_time_ms = 0;
    static uint32_t last_call_time = 0;
    
    /* Simple increment based on assumed 1ms tick */
    uint32_t current_systick = aks_core_get_tick();
    if (current_systick != last_call_time) {
        system_time_ms += (current_systick - last_call_time);
        last_call_time = current_systick;
    }
    
    return system_time_ms;
#endif
}

/**
 * @brief Split NMEA sentence into fields
 */
static void aks_gps_split_fields(char* sentence, char* fields[], uint8_t max_fields, uint8_t* field_count)
{
    *field_count = 0;
    char* token = strtok(sentence, ",*");
    
    while (token != NULL && *field_count < max_fields) {
        fields[*field_count] = token;
        (*field_count)++;
        token = strtok(NULL, ",*");
    }
}
