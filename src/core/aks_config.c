/**
 * @file aks_config.c
 * @brief System configuration implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_core.h"
#include <string.h>

/* Default configuration values */
static const aks_config_t g_default_config = {
    .system_frequency = 84000000,    // 84 MHz
    .can_baudrate = 500000,          // 500 kbps
    .uart_baudrate = 115200,         // 115200 bps
    .adc_resolution = 12,            // 12-bit
    .node_id = 1,                    // Default node ID
    .debug_enabled = false           // Debug disabled by default
};

/* Current active configuration */
static aks_config_t g_current_config;
static bool g_config_loaded = false;

/**
 * @brief Load default configuration
 */
aks_result_t aks_config_load_default(void)
{
    memcpy(&g_current_config, &g_default_config, sizeof(aks_config_t));
    g_config_loaded = true;
    return AKS_OK;
}

/**
 * @brief Load configuration from memory/EEPROM
 */
aks_result_t aks_config_load(void)
{
#ifdef USE_HAL_DRIVER
    /* EEPROM/Flash implementation for STM32 */
    uint32_t flash_address = 0x08060000; // Last sector of flash for config
    uint32_t config_magic = 0xAKS12345;  // Magic number to verify config
    
    /* Read magic number first */
    uint32_t stored_magic = *((uint32_t*)flash_address);
    
    if (stored_magic == config_magic) {
        /* Valid configuration found, read it */
        uint8_t* flash_data = (uint8_t*)(flash_address + sizeof(uint32_t));
        memcpy(&g_current_config, flash_data, sizeof(aks_config_t));
        
        /* Validate loaded configuration */
        if (g_current_config.system_frequency >= 1000000 && 
            g_current_config.system_frequency <= 168000000 &&
            g_current_config.can_baudrate >= 10000 && 
            g_current_config.can_baudrate <= 1000000 &&
            g_current_config.uart_baudrate >= 9600 && 
            g_current_config.uart_baudrate <= 921600 &&
            g_current_config.adc_resolution >= 8 && 
            g_current_config.adc_resolution <= 16 &&
            g_current_config.node_id <= 255) {
            
            g_config_loaded = true;
            return AKS_OK;
        }
    }
#endif
    
    /* If no valid config found or validation failed, load defaults */
    return aks_config_load_default();
}

/**
 * @brief Save configuration to memory/EEPROM
 */
aks_result_t aks_config_save(void)
{
    if (!g_config_loaded) {
        return AKS_ERROR_NOT_READY;
    }
    
#ifdef USE_HAL_DRIVER
    /* Flash implementation for STM32 */
    uint32_t flash_address = 0x08060000; // Last sector of flash for config
    uint32_t config_magic = 0xAKS12345;  // Magic number to verify config
    
    /* Unlock flash for writing */
    HAL_FLASH_Unlock();
    
    /* Erase the sector first */
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = FLASH_SECTOR_7; // Last sector
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    uint32_t sector_error;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    
    if (status == HAL_OK) {
        /* Write magic number first */
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, config_magic);
        
        if (status == HAL_OK) {
            /* Write configuration data */
            uint32_t* config_data = (uint32_t*)&g_current_config;
            uint32_t config_words = (sizeof(aks_config_t) + 3) / 4; // Round up to word boundary
            
            for (uint32_t i = 0; i < config_words && status == HAL_OK; i++) {
                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
                                         flash_address + sizeof(uint32_t) + (i * 4), 
                                         config_data[i]);
            }
        }
    }
    
    /* Lock flash */
    HAL_FLASH_Lock();
    
    return (status == HAL_OK) ? AKS_OK : AKS_ERROR;
#else
    /* For testing without HAL */
    return AKS_OK;
#endif
}

/**
 * @brief Get current configuration
 */
aks_result_t aks_config_get(aks_config_t* config)
{
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (!g_config_loaded) {
        return AKS_ERROR_NOT_READY;
    }
    
    memcpy(config, &g_current_config, sizeof(aks_config_t));
    return AKS_OK;
}

/**
 * @brief Set configuration
 */
aks_result_t aks_config_set(const aks_config_t* config)
{
    if (config == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    // Validate configuration parameters
    if (config->system_frequency < 1000000 || config->system_frequency > 168000000) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (config->can_baudrate < 10000 || config->can_baudrate > 1000000) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (config->uart_baudrate < 9600 || config->uart_baudrate > 921600) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (config->adc_resolution < 8 || config->adc_resolution > 16) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (config->node_id > 255) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    memcpy(&g_current_config, config, sizeof(aks_config_t));
    g_config_loaded = true;
    
    return AKS_OK;
}

/**
 * @brief Reset configuration to defaults
 */
aks_result_t aks_config_reset(void)
{
    return aks_config_load_default();
}

/**
 * @brief Get system frequency
 */
uint32_t aks_config_get_system_frequency(void)
{
    return g_config_loaded ? g_current_config.system_frequency : g_default_config.system_frequency;
}

/**
 * @brief Get CAN baudrate
 */
uint32_t aks_config_get_can_baudrate(void)
{
    return g_config_loaded ? g_current_config.can_baudrate : g_default_config.can_baudrate;
}

/**
 * @brief Get UART baudrate
 */
uint32_t aks_config_get_uart_baudrate(void)
{
    return g_config_loaded ? g_current_config.uart_baudrate : g_default_config.uart_baudrate;
}

/**
 * @brief Get ADC resolution
 */
uint16_t aks_config_get_adc_resolution(void)
{
    return g_config_loaded ? g_current_config.adc_resolution : g_default_config.adc_resolution;
}

/**
 * @brief Get node ID
 */
uint8_t aks_config_get_node_id(void)
{
    return g_config_loaded ? g_current_config.node_id : g_default_config.node_id;
}

/**
 * @brief Check if debug is enabled
 */
bool aks_config_is_debug_enabled(void)
{
    return g_config_loaded ? g_current_config.debug_enabled : g_default_config.debug_enabled;
}

/**
 * @brief Set debug mode
 */
aks_result_t aks_config_set_debug(bool enable)
{
    if (!g_config_loaded) {
        aks_config_load_default();
    }
    
    g_current_config.debug_enabled = enable;
    return AKS_OK;
}
