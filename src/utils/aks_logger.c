/**
 * @file aks_logger.c
 * @brief Data logging and storage implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_types.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#ifdef USE_FATFS
#include "ff.h"
#endif

/* Logger Configuration */
#define AKS_LOGGER_BUFFER_SIZE          4096        // Circular buffer size
#define AKS_LOGGER_MAX_LOG_LENGTH       256         // Maximum log entry length
#define AKS_LOGGER_MAX_FILENAME_LENGTH  64          // Maximum filename length
#define AKS_LOGGER_FLUSH_INTERVAL       5000        // Auto-flush interval in ms
#define AKS_LOGGER_MAX_FILE_SIZE        (10*1024*1024) // 10MB max file size
#define AKS_LOGGER_BACKUP_COUNT         5           // Number of backup files

/* Log Levels */
typedef enum {
    AKS_LOG_LEVEL_ERROR = 0,
    AKS_LOG_LEVEL_WARNING,
    AKS_LOG_LEVEL_INFO,
    AKS_LOG_LEVEL_DEBUG,
    AKS_LOG_LEVEL_TELEMETRY
} aks_log_level_t;

/* Log Entry Structure */
typedef struct {
    uint32_t timestamp;
    aks_log_level_t level;
    char message[AKS_LOGGER_MAX_LOG_LENGTH];
    uint16_t length;
} aks_log_entry_t;

/* Circular Buffer for Logs */
typedef struct {
    aks_log_entry_t entries[AKS_LOGGER_BUFFER_SIZE / sizeof(aks_log_entry_t)];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    bool overflow;
} aks_log_buffer_t;

/* Logger Configuration */
typedef struct {
    aks_log_level_t min_level;          /**< Minimum log level to record */
    bool enable_console;                /**< Enable console output */
    bool enable_file;                   /**< Enable file logging */
    bool enable_can;                    /**< Enable CAN logging */
    bool enable_telemetry;              /**< Enable telemetry logging */
    uint32_t flush_interval_ms;         /**< Auto-flush interval */
    uint32_t max_file_size;             /**< Maximum file size */
    char log_directory[64];             /**< Log directory path */
    char filename_prefix[32];           /**< Log filename prefix */
} aks_logger_config_t;

/* Telemetry Data Structure */
typedef struct {
    uint32_t timestamp;
    float vehicle_speed;
    float motor_torque;
    float motor_speed;
    float battery_voltage;
    float battery_current;
    float battery_temperature;
    float battery_soc;
    double gps_latitude;
    double gps_longitude;
    float gps_speed;
    float isolation_resistance;
    uint8_t system_state;
    uint16_t fault_codes;
} aks_telemetry_data_t;

/* Logger Handle */
typedef struct {
    bool initialized;
    aks_logger_config_t config;
    aks_log_buffer_t buffer;
    uint32_t last_flush_time;
    uint32_t total_entries;
    uint32_t dropped_entries;
    char current_log_file[AKS_LOGGER_MAX_FILENAME_LENGTH];
    char current_telemetry_file[AKS_LOGGER_MAX_FILENAME_LENGTH];
    bool file_open;
    bool telemetry_file_open;
#ifdef USE_FATFS
    FIL log_file;
    FIL telemetry_file;
    FATFS fs;
#endif
} aks_logger_handle_t;

/* Private variables */
static aks_logger_handle_t g_logger_handle;

/* Default configuration */
static const aks_logger_config_t g_default_config = {
    .min_level = AKS_LOG_LEVEL_INFO,
    .enable_console = true,
    .enable_file = true,
    .enable_can = false,
    .enable_telemetry = true,
    .flush_interval_ms = AKS_LOGGER_FLUSH_INTERVAL,
    .max_file_size = AKS_LOGGER_MAX_FILE_SIZE,
    .log_directory = "logs",
    .filename_prefix = "aks"
};

/* Log level strings */
static const char* g_log_level_strings[] = {
    "ERROR", "WARN", "INFO", "DEBUG", "TELEM"
};

/* Private function prototypes */
static aks_result_t aks_logger_buffer_put(const aks_log_entry_t* entry);
static aks_result_t aks_logger_buffer_get(aks_log_entry_t* entry);
static bool aks_logger_buffer_is_empty(void);
static aks_result_t aks_logger_flush_to_file(void);
static aks_result_t aks_logger_create_log_file(void);
static aks_result_t aks_logger_create_telemetry_file(void);
static aks_result_t aks_logger_rotate_files(void);
static void aks_logger_generate_filename(char* filename, const char* prefix, const char* extension);
static uint32_t aks_logger_get_timestamp(void);

/**
 * @brief Initialize logger system
 */
aks_result_t aks_logger_init(const aks_logger_config_t* config)
{
    if (g_logger_handle.initialized) {
        return AKS_ERROR_BUSY;
    }
    
    /* Use default config if none provided */
    if (config != NULL) {
        g_logger_handle.config = *config;
    } else {
        g_logger_handle.config = g_default_config;
    }
    
    /* Initialize buffer */
    memset(&g_logger_handle.buffer, 0, sizeof(aks_log_buffer_t));
    
    /* Initialize file system if enabled */
#ifdef USE_FATFS
    if (g_logger_handle.config.enable_file || g_logger_handle.config.enable_telemetry) {
        FRESULT fr = f_mount(&g_logger_handle.fs, "", 1);
        if (fr != FR_OK) {
            return AKS_ERROR;
        }
        
        /* Create log directory if it doesn't exist */
        f_mkdir(g_logger_handle.config.log_directory);
    }
#endif
    
    /* Initialize timestamps */
    g_logger_handle.last_flush_time = aks_logger_get_timestamp();
    
    g_logger_handle.initialized = true;
    
    /* Log initialization */
    aks_logger_info("Logger initialized successfully");
    
    return AKS_OK;
}

/**
 * @brief Deinitialize logger system
 */
aks_result_t aks_logger_deinit(void)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    /* Flush remaining logs */
    aks_logger_flush();
    
    /* Close files */
#ifdef USE_FATFS
    if (g_logger_handle.file_open) {
        f_close(&g_logger_handle.log_file);
    }
    if (g_logger_handle.telemetry_file_open) {
        f_close(&g_logger_handle.telemetry_file);
    }
    f_unmount("");
#endif
    
    /* Reset handle */
    memset(&g_logger_handle, 0, sizeof(aks_logger_handle_t));
    
    return AKS_OK;
}

/**
 * @brief Log error message
 */
aks_result_t aks_logger_error(const char* format, ...)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_logger_handle.config.min_level > AKS_LOG_LEVEL_ERROR) {
        return AKS_OK;
    }
    
    va_list args;
    va_start(args, format);
    
    aks_log_entry_t entry;
    entry.timestamp = aks_logger_get_timestamp();
    entry.level = AKS_LOG_LEVEL_ERROR;
    entry.length = vsnprintf(entry.message, sizeof(entry.message), format, args);
    
    va_end(args);
    
    return aks_logger_buffer_put(&entry);
}

/**
 * @brief Log warning message
 */
aks_result_t aks_logger_warning(const char* format, ...)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_logger_handle.config.min_level > AKS_LOG_LEVEL_WARNING) {
        return AKS_OK;
    }
    
    va_list args;
    va_start(args, format);
    
    aks_log_entry_t entry;
    entry.timestamp = aks_logger_get_timestamp();
    entry.level = AKS_LOG_LEVEL_WARNING;
    entry.length = vsnprintf(entry.message, sizeof(entry.message), format, args);
    
    va_end(args);
    
    return aks_logger_buffer_put(&entry);
}

/**
 * @brief Log info message
 */
aks_result_t aks_logger_info(const char* format, ...)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_logger_handle.config.min_level > AKS_LOG_LEVEL_INFO) {
        return AKS_OK;
    }
    
    va_list args;
    va_start(args, format);
    
    aks_log_entry_t entry;
    entry.timestamp = aks_logger_get_timestamp();
    entry.level = AKS_LOG_LEVEL_INFO;
    entry.length = vsnprintf(entry.message, sizeof(entry.message), format, args);
    
    va_end(args);
    
    return aks_logger_buffer_put(&entry);
}

/**
 * @brief Log debug message
 */
aks_result_t aks_logger_debug(const char* format, ...)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (g_logger_handle.config.min_level > AKS_LOG_LEVEL_DEBUG) {
        return AKS_OK;
    }
    
    va_list args;
    va_start(args, format);
    
    aks_log_entry_t entry;
    entry.timestamp = aks_logger_get_timestamp();
    entry.level = AKS_LOG_LEVEL_DEBUG;
    entry.length = vsnprintf(entry.message, sizeof(entry.message), format, args);
    
    va_end(args);
    
    return aks_logger_buffer_put(&entry);
}

/**
 * @brief Log telemetry data
 */
aks_result_t aks_logger_telemetry(const aks_telemetry_data_t* data)
{
    if (!g_logger_handle.initialized || data == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (!g_logger_handle.config.enable_telemetry) {
        return AKS_OK;
    }
    
#ifdef USE_FATFS
    /* Create telemetry file if not open */
    if (!g_logger_handle.telemetry_file_open) {
        aks_result_t result = aks_logger_create_telemetry_file();
        if (result != AKS_OK) {
            return result;
        }
    }
    
    /* Format telemetry data as CSV */
    char csv_line[512];
    int length = snprintf(csv_line, sizeof(csv_line),
        "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%u,%u\n",
        data->timestamp,
        data->vehicle_speed,
        data->motor_torque,
        data->motor_speed,
        data->battery_voltage,
        data->battery_current,
        data->battery_temperature,
        data->battery_soc,
        data->gps_latitude,
        data->gps_longitude,
        data->gps_speed,
        data->isolation_resistance,
        data->system_state,
        data->fault_codes
    );
    
    /* Write to file */
    UINT bytes_written;
    FRESULT fr = f_write(&g_logger_handle.telemetry_file, csv_line, length, &bytes_written);
    
    if (fr == FR_OK && bytes_written == length) {
        f_sync(&g_logger_handle.telemetry_file);
        return AKS_OK;
    }
#endif
    
    return AKS_ERROR;
}

/**
 * @brief Flush logger buffer to storage
 */
aks_result_t aks_logger_flush(void)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    return aks_logger_flush_to_file();
}

/**
 * @brief Logger periodic task
 */
aks_result_t aks_logger_task(void)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    uint32_t current_time = aks_logger_get_timestamp();
    
    /* Auto-flush if interval elapsed */
    if (current_time - g_logger_handle.last_flush_time >= g_logger_handle.config.flush_interval_ms) {
        aks_logger_flush();
        g_logger_handle.last_flush_time = current_time;
    }
    
    return AKS_OK;
}

/**
 * @brief Get logger statistics
 */
aks_result_t aks_logger_get_stats(uint32_t* total_entries, uint32_t* dropped_entries, uint16_t* buffer_count)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (total_entries) {
        *total_entries = g_logger_handle.total_entries;
    }
    
    if (dropped_entries) {
        *dropped_entries = g_logger_handle.dropped_entries;
    }
    
    if (buffer_count) {
        *buffer_count = g_logger_handle.buffer.count;
    }
    
    return AKS_OK;
}

/**
 * @brief Set log level
 */
aks_result_t aks_logger_set_level(aks_log_level_t level)
{
    if (!g_logger_handle.initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_logger_handle.config.min_level = level;
    return AKS_OK;
}

/* Private Functions */

/**
 * @brief Put log entry into circular buffer
 */
static aks_result_t aks_logger_buffer_put(const aks_log_entry_t* entry)
{
    if (entry == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    /* Check if buffer is full */
    uint16_t max_entries = sizeof(g_logger_handle.buffer.entries) / sizeof(aks_log_entry_t);
    
    if (g_logger_handle.buffer.count >= max_entries) {
        /* Buffer overflow - drop oldest entry */
        g_logger_handle.buffer.tail = (g_logger_handle.buffer.tail + 1) % max_entries;
        g_logger_handle.buffer.overflow = true;
        g_logger_handle.dropped_entries++;
    } else {
        g_logger_handle.buffer.count++;
    }
    
    /* Add new entry */
    g_logger_handle.buffer.entries[g_logger_handle.buffer.head] = *entry;
    g_logger_handle.buffer.head = (g_logger_handle.buffer.head + 1) % max_entries;
    g_logger_handle.total_entries++;
    
    /* Console output if enabled */
    if (g_logger_handle.config.enable_console) {
        printf("[%lu] %s: %s\n", entry->timestamp, g_log_level_strings[entry->level], entry->message);
    }
    
    return AKS_OK;
}

/**
 * @brief Get log entry from circular buffer
 */
static aks_result_t aks_logger_buffer_get(aks_log_entry_t* entry)
{
    if (entry == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (aks_logger_buffer_is_empty()) {
        return AKS_ERROR_NO_MEMORY;
    }
    
    uint16_t max_entries = sizeof(g_logger_handle.buffer.entries) / sizeof(aks_log_entry_t);
    
    *entry = g_logger_handle.buffer.entries[g_logger_handle.buffer.tail];
    g_logger_handle.buffer.tail = (g_logger_handle.buffer.tail + 1) % max_entries;
    g_logger_handle.buffer.count--;
    
    return AKS_OK;
}

/**
 * @brief Check if buffer is empty
 */
static bool aks_logger_buffer_is_empty(void)
{
    return (g_logger_handle.buffer.count == 0);
}

/**
 * @brief Flush buffer to file
 */
static aks_result_t aks_logger_flush_to_file(void)
{
#ifdef USE_FATFS
    if (!g_logger_handle.config.enable_file) {
        return AKS_OK;
    }
    
    /* Create log file if not open */
    if (!g_logger_handle.file_open) {
        aks_result_t result = aks_logger_create_log_file();
        if (result != AKS_OK) {
            return result;
        }
    }
    
    /* Write all buffered entries */
    aks_log_entry_t entry;
    while (!aks_logger_buffer_is_empty()) {
        if (aks_logger_buffer_get(&entry) == AKS_OK) {
            char log_line[AKS_LOGGER_MAX_LOG_LENGTH + 64];
            int length = snprintf(log_line, sizeof(log_line),
                "[%lu] %s: %s\n",
                entry.timestamp,
                g_log_level_strings[entry.level],
                entry.message
            );
            
            UINT bytes_written;
            FRESULT fr = f_write(&g_logger_handle.log_file, log_line, length, &bytes_written);
            
            if (fr != FR_OK || bytes_written != length) {
                return AKS_ERROR;
            }
        }
    }
    
    /* Sync file */
    f_sync(&g_logger_handle.log_file);
    
    return AKS_OK;
#else
    return AKS_OK;
#endif
}

/**
 * @brief Create log file
 */
static aks_result_t aks_logger_create_log_file(void)
{
#ifdef USE_FATFS
    aks_logger_generate_filename(g_logger_handle.current_log_file, 
                                g_logger_handle.config.filename_prefix, "log");
    
    FRESULT fr = f_open(&g_logger_handle.log_file, g_logger_handle.current_log_file, 
                       FA_CREATE_ALWAYS | FA_WRITE);
    
    if (fr == FR_OK) {
        g_logger_handle.file_open = true;
        
        /* Write header */
        const char* header = "# AKS System Log\n# Timestamp, Level, Message\n";
        UINT bytes_written;
        f_write(&g_logger_handle.log_file, header, strlen(header), &bytes_written);
        f_sync(&g_logger_handle.log_file);
        
        return AKS_OK;
    }
#endif
    
    return AKS_ERROR;
}

/**
 * @brief Create telemetry file
 */
static aks_result_t aks_logger_create_telemetry_file(void)
{
#ifdef USE_FATFS
    aks_logger_generate_filename(g_logger_handle.current_telemetry_file,
                                g_logger_handle.config.filename_prefix, "csv");
    
    FRESULT fr = f_open(&g_logger_handle.telemetry_file, g_logger_handle.current_telemetry_file,
                       FA_CREATE_ALWAYS | FA_WRITE);
    
    if (fr == FR_OK) {
        g_logger_handle.telemetry_file_open = true;
        
        /* Write CSV header */
        const char* header = "timestamp,vehicle_speed,motor_torque,motor_speed,battery_voltage,"
                           "battery_current,battery_temperature,battery_soc,gps_latitude,"
                           "gps_longitude,gps_speed,isolation_resistance,system_state,fault_codes\n";
        UINT bytes_written;
        f_write(&g_logger_handle.telemetry_file, header, strlen(header), &bytes_written);
        f_sync(&g_logger_handle.telemetry_file);
        
        return AKS_OK;
    }
#endif
    
    return AKS_ERROR;
}

/**
 * @brief Rotate log files
 */
static aks_result_t aks_logger_rotate_files(void)
{
#ifdef USE_FATFS
    /* Check if current log file exceeds maximum size */
    FILINFO file_info;
    FRESULT fr = f_stat(g_logger_handle.current_log_file, &file_info);
    
    if (fr == FR_OK && file_info.fsize >= g_logger_handle.config.max_file_size) {
        /* Close current log file */
        if (g_logger_handle.file_open) {
            f_close(&g_logger_handle.log_file);
            g_logger_handle.file_open = false;
        }
        
        /* Rename current file with timestamp */
        char archived_filename[AKS_LOGGER_MAX_FILENAME_LENGTH];
        uint32_t timestamp = aks_logger_get_timestamp();
        snprintf(archived_filename, sizeof(archived_filename),
                "%s/%s_archived_%lu.log",
                g_logger_handle.config.log_directory,
                g_logger_handle.config.filename_prefix,
                timestamp);
        
        /* Rename the file */
        f_rename(g_logger_handle.current_log_file, archived_filename);
        
        /* Create new log file */
        aks_result_t result = aks_logger_create_log_file();
        if (result != AKS_OK) {
            return result;
        }
    }
    
    /* Check telemetry file size */
    fr = f_stat(g_logger_handle.current_telemetry_file, &file_info);
    
    if (fr == FR_OK && file_info.fsize >= g_logger_handle.config.max_file_size) {
        /* Close current telemetry file */
        if (g_logger_handle.telemetry_file_open) {
            f_close(&g_logger_handle.telemetry_file);
            g_logger_handle.telemetry_file_open = false;
        }
        
        /* Rename current telemetry file */
        char archived_telemetry[AKS_LOGGER_MAX_FILENAME_LENGTH];
        uint32_t timestamp = aks_logger_get_timestamp();
        snprintf(archived_telemetry, sizeof(archived_telemetry),
                "%s/%s_telemetry_archived_%lu.csv",
                g_logger_handle.config.log_directory,
                g_logger_handle.config.filename_prefix,
                timestamp);
        
        /* Rename the file */
        f_rename(g_logger_handle.current_telemetry_file, archived_telemetry);
        
        /* Create new telemetry file */
        aks_result_t result = aks_logger_create_telemetry_file();
        if (result != AKS_OK) {
            return result;
        }
    }
    
    /* Clean up old files if directory has too many files */
    DIR dir;
    FILINFO fno;
    uint16_t file_count = 0;
    
    fr = f_opendir(&dir, g_logger_handle.config.log_directory);
    if (fr == FR_OK) {
        while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
            if (!(fno.fattrib & AM_DIR)) {
                file_count++;
            }
        }
        f_closedir(&dir);
        
        /* If too many files, delete oldest archived files */
        if (file_count > 50) { // Keep maximum 50 files
            // Implementation to delete oldest files would go here
            // This would involve sorting files by timestamp and deleting oldest
        }
    }
    
    return AKS_OK;
#else
    return AKS_OK;
#endif
}

/**
 * @brief Generate filename with timestamp
 */
static void aks_logger_generate_filename(char* filename, const char* prefix, const char* extension)
{
    uint32_t timestamp = aks_logger_get_timestamp();
    snprintf(filename, AKS_LOGGER_MAX_FILENAME_LENGTH,
            "%s/%s_%lu.%s",
            g_logger_handle.config.log_directory,
            prefix,
            timestamp,
            extension);
}

/**
 * @brief Get current timestamp
 */
static uint32_t aks_logger_get_timestamp(void)
{
#ifdef USE_HAL_DRIVER
    return HAL_GetTick();
#else
    return 0; // Placeholder for testing
#endif
}
