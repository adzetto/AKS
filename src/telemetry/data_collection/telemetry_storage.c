/**
 * @file telemetry_storage.c
 * @brief Telemetry data storage implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "telemetry_storage.h"
#include "aks_core.h"
#include <string.h>
#include <stdio.h>

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#include "fatfs.h"  // For SD card file system
#endif

/* Private variables */
static bool storage_initialized = false;
static FIL log_file;
static FATFS sd_fatfs;
static char current_log_filename[64];
static uint32_t log_entry_count = 0;
static aks_telemetry_buffer_t memory_buffer;
static uint16_t buffer_index = 0;

/* Private function prototypes */
static aks_result_t init_sd_card(void);
static aks_result_t create_log_file(void);
static aks_result_t write_to_sd_card(const char* data);
static void generate_log_filename(void);

/**
 * @brief Initialize telemetry storage system
 */
aks_result_t aks_telemetry_storage_init(void)
{
    if (storage_initialized) {
        return AKS_ERROR_BUSY;
    }
    
    /* Initialize memory buffer */
    memset(&memory_buffer, 0, sizeof(memory_buffer));
    buffer_index = 0;
    log_entry_count = 0;
    
    /* Initialize SD card */
    aks_result_t result = init_sd_card();
    if (result != AKS_OK) {
        return result;
    }
    
    /* Create new log file */
    result = create_log_file();
    if (result != AKS_OK) {
        return result;
    }
    
    storage_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Store telemetry data to memory and SD card
 */
aks_result_t aks_telemetry_store_data(const aks_telemetry_packet_t* packet)
{
    if (!storage_initialized || packet == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Store in memory buffer (circular buffer) */
    memory_buffer.entries[buffer_index] = *packet;
    buffer_index = (buffer_index + 1) % AKS_TELEMETRY_BUFFER_SIZE;
    if (buffer_index == 0) {
        memory_buffer.buffer_full = true;
    }
    
    /* Format data for SD card storage */
    char log_entry[256];
    snprintf(log_entry, sizeof(log_entry),
        "%lu,%u,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%u,%s\r\n",
        packet->timestamp,
        packet->car_id,
        packet->speed,
        packet->battery_voltage,
        packet->battery_soc,
        packet->motor_temperature,
        packet->gps.latitude,
        packet->gps.longitude,
        packet->gps.altitude,
        packet->gps.speed,
        packet->gps.heading,
        packet->gps.satellites,
        packet->gps.fix_valid ? "1" : "0"
    );
    
    /* Write to SD card */
    aks_result_t result = write_to_sd_card(log_entry);
    if (result == AKS_OK) {
        log_entry_count++;
    }
    
    return result;
}

/**
 * @brief Get stored data from memory buffer
 */
aks_result_t aks_telemetry_get_stored_data(aks_telemetry_packet_t* packets, 
                                          uint16_t max_count, 
                                          uint16_t* actual_count)
{
    if (!storage_initialized || packets == NULL || actual_count == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    uint16_t available_entries = memory_buffer.buffer_full ? 
                                AKS_TELEMETRY_BUFFER_SIZE : buffer_index;
    
    *actual_count = (max_count < available_entries) ? max_count : available_entries;
    
    for (uint16_t i = 0; i < *actual_count; i++) {
        uint16_t idx = memory_buffer.buffer_full ? 
                      (buffer_index + i) % AKS_TELEMETRY_BUFFER_SIZE : i;
        packets[i] = memory_buffer.entries[idx];
    }
    
    return AKS_OK;
}

/**
 * @brief Handle connection loss - store data locally
 */
aks_result_t aks_telemetry_store_offline_data(const aks_telemetry_packet_t* packet)
{
    if (!storage_initialized || packet == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Mark packet as offline data */
    aks_telemetry_packet_t offline_packet = *packet;
    offline_packet.car_id |= 0x80;  // Set MSB to indicate offline data
    
    return aks_telemetry_store_data(&offline_packet);
}

/**
 * @brief Get current log file information
 */
aks_result_t aks_telemetry_get_log_info(aks_telemetry_log_info_t* info)
{
    if (!storage_initialized || info == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    strcpy(info->filename, current_log_filename);
    info->entry_count = log_entry_count;
    info->buffer_entries = memory_buffer.buffer_full ? 
                          AKS_TELEMETRY_BUFFER_SIZE : buffer_index;
    info->sd_card_available = true;  // Check SD card status
    
    return AKS_OK;
}

/* Private function implementations */

/**
 * @brief Initialize SD card
 */
static aks_result_t init_sd_card(void)
{
#ifdef USE_HAL_DRIVER
    /* Mount SD card filesystem */
    FRESULT fres = f_mount(&sd_fatfs, "", 1);
    if (fres != FR_OK) {
        return AKS_ERROR;
    }
    
    /* Create telemetry directory if it doesn't exist */
    f_mkdir("telemetry");
    
    return AKS_OK;
#else
    return AKS_OK;  // Simulation mode
#endif
}

/**
 * @brief Create new log file
 */
static aks_result_t create_log_file(void)
{
    generate_log_filename();
    
#ifdef USE_HAL_DRIVER
    FRESULT fres = f_open(&log_file, current_log_filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
        return AKS_ERROR;
    }
    
    /* Write CSV header */
    const char* header = "timestamp,car_id,speed,battery_voltage,battery_soc,"
                        "motor_temperature,latitude,longitude,altitude,"
                        "gps_speed,heading,satellites,fix_valid\r\n";
    
    UINT bytes_written;
    fres = f_write(&log_file, header, strlen(header), &bytes_written);
    if (fres != FR_OK) {
        f_close(&log_file);
        return AKS_ERROR;
    }
    
    f_sync(&log_file);
    
    return AKS_OK;
#else
    return AKS_OK;  // Simulation mode
#endif
}

/**
 * @brief Write data to SD card
 */
static aks_result_t write_to_sd_card(const char* data)
{
#ifdef USE_HAL_DRIVER
    UINT bytes_written;
    FRESULT fres = f_write(&log_file, data, strlen(data), &bytes_written);
    if (fres != FR_OK) {
        return AKS_ERROR;
    }
    
    /* Sync every 10 entries for data integrity */
    if (log_entry_count % 10 == 0) {
        f_sync(&log_file);
    }
    
    return AKS_OK;
#else
    return AKS_OK;  // Simulation mode
#endif
}

/**
 * @brief Generate unique log filename
 */
static void generate_log_filename(void)
{
    uint32_t timestamp = aks_core_get_tick();
    snprintf(current_log_filename, sizeof(current_log_filename),
             "telemetry/AKS_LOG_%08lX.csv", timestamp);
} 